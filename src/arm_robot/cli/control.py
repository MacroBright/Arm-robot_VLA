#!/usr/bin/env python3
"""ZDT 电机交互式调试器 — CAN 通讯学习/调试/标定用 (支持六关节).

用法:
  1. sudo ip link set can0 type can bitrate 500000 && sudo ip link set can0 up
  2. python zdt_interactive.py                    # 默认 addr=0x02, scheme=pc
  3. python zdt_interactive.py --addr 0x03        # 指定电机地址
  4. python zdt_interactive.py --scheme firmware  # 关节映射改固件侧 (J1=0x01..J6=0x06)

连接后输入 help 查看所有命令. 六关节操作:
  j1..j6 切换关节, scan 探测链路, all status/en/dis/stop 批量操作.
"""
import argparse
import time

import can

from arm_robot.controller.config import (
    CALIB, CHECKSUM, F_ENABLE, F_READ_CUR, F_READ_POS, F_STOP,
    FIRMWARE_JOINT_ADDRS, JOINT_ADDRS, DEFAULT_REDUCTION_RATIOS,
)
from arm_robot.driver.frames import (
    add_checksum, encode_frame, CanFrame, decode_pos4,
)

# ── 原始 CAN 收发 ──────────────────────────────────────────

def send_payload(bus: can.Bus, addr: int, payload: bytes):
    """发送完整负载 (自动按 CAN DLC≤8 拆成多帧, 帧序号由 ID 低字节编码)."""
    for frame in encode_frame(addr, payload):
        msg = can.Message(arbitration_id=frame.arbitration_id,
                          is_extended_id=True, data=frame.data)
        bus.send(msg)


def raw_send_recv(bus: can.Bus, addr: int, payload: bytes, func: int,
                  timeout_s: float = 0.5):
    """发送完整负载(多帧) 并等待 (addr, func) 匹配回帧.

    不匹配的帧会被丢弃并继续读 (避免排队帧污染), 超时返回 None.
    """
    send_payload(bus, addr, payload)
    t0 = time.monotonic()
    deadline = t0 + timeout_s
    while time.monotonic() < deadline:
        resp = bus.recv(timeout=max(0, deadline - time.monotonic()))
        if resp is None:
            continue
        r_addr = resp.arbitration_id >> 8
        if r_addr == addr and len(resp.data) > 0 and resp.data[0] == func:
            elapsed_ms = (time.monotonic() - t0) * 1000
            return bytes(resp.data), elapsed_ms
    return None, 0


def send_no_reply(bus: can.Bus, addr: int, data: bytes):
    """发送无回帧命令 (单帧)."""
    ext_id = (addr << 8) | 0x00
    msg = can.Message(arbitration_id=ext_id, is_extended_id=True, data=data)
    bus.send(msg)


# ── ZDT 命令封装 ───────────────────────────────────────────

def cmd_read_pos(bus, addr):
    data, ms = raw_send_recv(bus, addr, bytes([F_READ_POS, CHECKSUM]), F_READ_POS)
    if data is None:
        print(f"  超时 ({ms:.0f}ms)")
        return
    sign = -1 if data[1] == 0x01 else 1   # 符号字节 0x00=正 0x01=负 (说明书 §7.4.4)
    motor_deg = decode_pos4(data[2:6], sign)
    out_deg = motor_to_output(motor_deg)
    if GEAR_RATIO != 1.0:
        print(f"  输出轴: {out_deg:+.1f}  (电机轴: {motor_deg:+.1f} ÷{GEAR_RATIO:.3f}  "
              f"符号=0x{data[1]:02X} raw={data[1:5].hex()}  {ms:.1f}ms)")
    else:
        print(f"  位置: {motor_deg:+.1f}  (符号=0x{data[1]:02X} raw={data[1:5].hex()}  {ms:.1f}ms)")


def cmd_read_current(bus, addr):
    data, ms = raw_send_recv(bus, addr, bytes([F_READ_CUR, CHECKSUM]), F_READ_CUR)
    if data is None:
        print(f"  超时 ({ms:.0f}ms)")
        return
    ma = (data[1] << 8) | data[2]
    print(f"  电流: {ma}mA  (raw={data[1:4].hex()}  {ms:.1f}ms)")


def cmd_read_flag(bus, addr):
    data, ms = raw_send_recv(bus, addr, bytes([0x3A, CHECKSUM]), 0x3A)
    if data is None:
        print(f"  超时 ({ms:.0f}ms)")
        return
    flag = data[1]
    bits = []
    if flag & 0x01: bits.append("使能")
    if flag & 0x02: bits.append("到位")
    if flag & 0x04: bits.append("堵转")
    if flag & 0x08: bits.append("堵转保护")
    print(f"  状态: 0x{flag:02X}  [{', '.join(bits) if bits else '无标志'}]  ({ms:.1f}ms)")


def cmd_enable(bus, addr, state: bool):
    # 固件 Emm_V5_En_Control: [F3, AB, 状态, 多机同步标志(0), 6B]
    body = bytes([F_ENABLE, 0xAB, 1 if state else 0, 0x00, CHECKSUM])
    send_no_reply(bus, addr, body)
    print(f"  {'使能' if state else '失能'} 已发送")


def cmd_stop(bus, addr):
    # 固件 Emm_V5_Stop_Now: [FE, 98, 同步标志(0), 6B]
    body = bytes([F_STOP, 0x98, 0x00, CHECKSUM])
    send_no_reply(bus, addr, body)
    print(f"  停止已发送")


def cmd_stop_all(bus):
    # 广播急停: addr=0x00, [FE, 98, 0, 6B]
    body = bytes([F_STOP, 0x98, 0x00, CHECKSUM])
    send_no_reply(bus, 0x00, body)
    print(f"  广播急停已发送 (addr=0x00)")


# ── 六关节批量操作 (链路确认 / 标定用) ───────────────────────

def _addr_label(addr: int, joint_addrs: list[int]) -> str:
    """当前地址显示名: 命中关节映射显示 J#+addr, 否则仅 hex."""
    for i, a in enumerate(joint_addrs):
        if a == addr:
            return f"J{i+1} 0x{a:02X}"
    return f"0x{addr:02X}"


def _flag_bits(flags: int) -> str:
    """状态字节 → 可读文本 (使能/到位/堵转/堵转保护)."""
    bits = []
    if flags & 0x01: bits.append("使能")
    if flags & 0x02: bits.append("到位")
    if flags & 0x04: bits.append("堵转")
    if flags & 0x08: bits.append("堵转保护")
    return f"0x{flags:02X}[{','.join(bits) if bits else '无'}]"


def _read_motor_state(bus, addr: int):
    """读一台电机: 版本/标志/电流/位置(0x01 符号约定, 说明书 §7.4.4). 离线返回 None."""
    ver, _ = raw_send_recv(bus, addr, bytes([0x1F, CHECKSUM]), 0x1F, timeout_s=0.15)
    if ver is None:
        return None
    flag_data, _ = raw_send_recv(bus, addr, bytes([0x3A, CHECKSUM]), 0x3A, timeout_s=0.15)
    cur_data, _ = raw_send_recv(bus, addr, bytes([F_READ_CUR, CHECKSUM]), F_READ_CUR, timeout_s=0.15)
    pos_data, _ = raw_send_recv(bus, addr, bytes([F_READ_POS, CHECKSUM]), F_READ_POS, timeout_s=0.15)
    flags = flag_data[1] if flag_data is not None and len(flag_data) > 1 else 0
    cur = ((cur_data[1] << 8) | cur_data[2]) if cur_data is not None and len(cur_data) > 2 else None
    pos = None
    if pos_data is not None and len(pos_data) > 4:
        sign = -1 if pos_data[1] == 0x01 else 1   # 0x00=正 0x01=负
        pos = decode_pos4(pos_data[2:6], sign)
    return {"fw": (ver[1] if len(ver) > 2 else 0, ver[2] if len(ver) > 3 else 0),
            "flags": flags, "cur_ma": cur, "pos_deg": pos}


def cmd_scan(bus, joint_addrs: list[int]):
    """探测全部关节地址, 显示在线/离线 + 位置 + 标志 (一次性确认链路)."""
    print(f"== 关节扫描 ({len(joint_addrs)} 个地址) ==")
    online = 0
    for i, a in enumerate(joint_addrs):
        st = _read_motor_state(bus, a)
        if st is None:
            print(f"  J{i+1} 0x{a:02X}: 离线")
            continue
        online += 1
        fw = f"{st['fw'][0]:#x}/{st['fw'][1]:#x}"
        pos = f"{st['pos_deg']:+.1f}" if st['pos_deg'] is not None else "-"
        print(f"  J{i+1} 0x{a:02X}: 在线 FW/HW={fw} 位置={pos}°(电机轴)  {_flag_bits(st['flags'])}")
    if online == len(joint_addrs):
        print(f"  全部在线 ({online}/{len(joint_addrs)}) ✓")
    else:
        print(f"  ⚠ 在线 {online}/{len(joint_addrs)} — 检查供电/重复ID/波特率/Response设置")


def cmd_all_status(bus, joint_addrs: list[int]):
    """读全部关节 pos+anchor+cur+flag 一张表 (标定/链路确认).

    anchor = 真实输出角度 (0x36 电机轴 pos 经 CALIB(k,b) 换算, 同命令坐下
    `anchor`/zdt_anchor.py 结论). 未标定 (CALIB[slot] 为 None) 退化纯减速比换算.
    """
    print("J#  ID     电机轴°  anchor°  电流mA 标志")
    for i, a in enumerate(joint_addrs):
        st = _read_motor_state(bus, a)
        if st is None:
            print(f"  J{i+1} 0x{a:02X}  离线")
            continue
        pos_deg = st['pos_deg']
        anchor = None
        if pos_deg is not None:
            kb = CALIB[i] if i < len(CALIB) else None
            if kb is not None:
                k, b = kb
                anchor = (pos_deg - b) / k if abs(k) > 1e-9 else pos_deg
            else:
                anchor = pos_deg / DEFAULT_REDUCTION_RATIOS[i]
        pos = f"{pos_deg:+8.1f}" if pos_deg is not None else "       -"
        anc = f"{anchor:+8.2f}" if anchor is not None else "       -"
        cur = f"{st['cur_ma']:5d}" if st['cur_ma'] is not None else "    -"
        print(f"  J{i+1} 0x{a:02X}  {pos}  {anc}  {cur}  {_flag_bits(st['flags'])}")


def cmd_all_enable(bus, joint_addrs: list[int]):
    body = bytes([F_ENABLE, 0xAB, 0x01, 0x00, CHECKSUM])
    for a in joint_addrs:
        send_no_reply(bus, a, body)
    print(f"  批量使能已发送 ({len(joint_addrs)} 台)")


def cmd_all_disable(bus, joint_addrs: list[int]):
    body = bytes([F_ENABLE, 0xAB, 0x00, 0x00, CHECKSUM])
    for a in joint_addrs:
        send_no_reply(bus, a, body)
    print(f"  批量失能已发送 ({len(joint_addrs)} 台)")
    print(f"  ⚠⚠ J2(肩抬)/J3(肘) 为重力关节, 已失去保持力矩 — 手臂必须有人支撑!")


def cmd_all_stop(bus):
    cmd_stop_all(bus)   # 广播 FE 98 00 6B, 断命令保力矩


# ── 固件兼容位置命令 (0xFD 脉冲计数, 与 robot.c 一致) ─────────

# 命令缩放比: 决定 fdrel 的脉冲数 (由 setmed/fixup 标定, 或手动 ratio 设置)
# 计数约定: 3200 脉冲 = 电机轴 360°, 故脉冲数 = 输出度 × ratio × 3200/360
# 初始值参考固件实测标定表 (robot.c g_joints_init): 50/50.89/50.89/51/27/51
# 单机调试勿盲信 — 换电机/减速器必须 setmed 精确标定
# fdrel 默认按当前关节查 DEFAULT_REDUCTION_RATIOS, ratio 命令可手动 override
CURRENT_RATIO = None  # None = 自动按关节查表; 非 None = 手动 override (影响所有关节)
# 显示齿轮比: 驱动板 OLED / 读回值 = 电机轴角度;
# 装了 51:1 行星减速器后, 真实输出轴角度 = 电机轴角度 ÷ GEAR_RATIO
GEAR_RATIO = 1.0
PULSES_PER_REV = 3200   # 16 细分下 3200 脉冲 = 电机轴转一圈


def _ratio_for_addr(addr: int, joint_addrs: list[int]) -> float:
    """按当前关节地址查减速比. 命中关节映射返回 DEFAULT_REDUCTION_RATIOS[slot],
    未命中 (如 addr 命令切到非关节地址) 退化为 51.0 (最常见的减速比)."""
    slot = next((i for i, a in enumerate(joint_addrs) if a == addr), None)
    if slot is not None and slot < len(DEFAULT_REDUCTION_RATIOS):
        return DEFAULT_REDUCTION_RATIOS[slot]
    return 51.0


def fd_pulses_for_deg(deg: float, ratio: float) -> int:
    """输出轴角度 → 电机轴脉冲. 固件约定: 3200 脉冲 = 360° × 减速比."""
    return max(1, int(round(abs(deg) * ratio * PULSES_PER_REV / 360)))


def motor_to_output(motor_deg: float) -> float:
    """电机轴角度 → 输出轴角度 (÷ 显示齿轮比)."""
    return motor_deg / GEAR_RATIO


def cmd_fd_move_rel(bus, addr, delta_deg, speed_rpm=30.0, acc=20, ratio=51.0):
    """固件兼容相对运动 (0xFD). 布局同 Emm_V5_Pos_Control:
    数据 = [FD, dir, vel_RPM, acc, 脉冲4B, raF=0(相对), snF=0, 6B]."""
    steps = fd_pulses_for_deg(delta_deg, ratio)
    d = 0 if delta_deg >= 0 else 1       # dir: 0=CW (正), 其他=CCW (如固件 dir 参数)
    vel = int(max(1, round(abs(speed_rpm)))) & 0xFFFF      # 0xFD 速度字段 = RPM 直传
    body = (bytes([0xFD, d, (vel >> 8) & 0xFF, vel & 0xFF, acc,
                   (steps >> 24) & 0xFF, (steps >> 16) & 0xFF,
                   (steps >> 8) & 0xFF, steps & 0xFF,
                   0x00, 0x00]))          # raF=0 相对, snF=0 不启用同步
    payload = add_checksum(body)
    frames = encode_frame(addr, payload)
    for f in frames:
        print(f"  TX 帧: ID=0x{f.arbitration_id:04X} data={f.data.hex()}")
    send_payload(bus, addr, payload)
    print(f"  -> {delta_deg:+.1f} ({steps} 脉冲) @ {speed_rpm}rpm  [{len(frames)}帧]")


def cmd_fd_pulse(bus, addr, n_pulses, dir_is_cw=True, speed_rpm=30.0, acc=20):
    """原始脉冲数相对运动 (0xFD). dir 仅 0=CW / 非0=CCW."""
    d = 0 if dir_is_cw else 1
    vel = int(max(1, round(abs(speed_rpm)))) & 0xFFFF      # 0xFD 速度字段 = RPM 直传
    body = (bytes([0xFD, d, (vel >> 8) & 0xFF, vel & 0xFF, acc,
                   (n_pulses >> 24) & 0xFF, (n_pulses >> 16) & 0xFF,
                   (n_pulses >> 8) & 0xFF, n_pulses & 0xFF,
                   0x00, 0x00]))
    payload = add_checksum(body)
    frames = encode_frame(addr, payload)
    for f in frames:
        print(f"  TX 帧: ID=0x{f.arbitration_id:04X} data={f.data.hex()}")
    send_payload(bus, addr, payload)
    print(f"  -> {n_pulses} 脉冲 {'CW' if dir_is_cw else 'CCW'} @ {speed_rpm}rpm  [{len(frames)}帧]")


def cmd_raw(bus, addr, hex_str):
    """发送原始 hex 字节 (自动加校验, 支持多帧拆分)."""
    raw = bytes.fromhex(hex_str)
    payload = add_checksum(raw)
    frames = encode_frame(addr, payload)
    for frame in frames:
        print(f"  TX 帧: ID=0x{frame.arbitration_id:04X} data={frame.data.hex()}")
    func = raw[0]
    data, ms = raw_send_recv(bus, addr, payload, func)
    if data is None:
        print(f"  RX: (超时 {ms:.0f}ms) [{len(frames)}帧]")
    else:
        print(f"  RX: {data.hex()}  ({ms:.1f}ms)")


def cmd_monitor(bus, duration_s=3.0):
    """监听总线上所有帧."""
    print(f"  监听中... ({duration_s}s, Ctrl+C 提前退出)")
    deadline = time.monotonic() + duration_s
    count = 0
    try:
        while time.monotonic() < deadline:
            msg = bus.recv(timeout=0.5)
            if msg is None:
                continue
            count += 1
            addr = msg.arbitration_id >> 8
            seq = msg.arbitration_id & 0xFF
            print(f"  [{count:3d}] addr=0x{addr:02X} seq={seq} dlc={msg.dlc} data={msg.data.hex()}")
    except KeyboardInterrupt:
        pass
    print(f"  共收到 {count} 帧")


# ── 生产驱动测试: 直接包 ZdtController (MassageRobot transport=can 同路径) ──

class _BusAdapter:
    """把脚本的 can.Bus 包装成 zdt.CanTransport 接口, 注入 ZdtController.

    避免走 controller.connect() 再开一个 SocketCAN 实例 (同一通道双 bus
    会竞争收帧); 脚本与 controller 共享同一总线实例, 命令严格串行.
    """

    def __init__(self, bus: "can.Bus"):
        self._bus = bus

    def open(self) -> None:
        pass

    def close(self) -> None:
        pass

    def send(self, frame: CanFrame) -> None:
        self._bus.send(can.Message(
            arbitration_id=frame.arbitration_id,
            is_extended_id=frame.is_extended_id,
            data=frame.data))

    def recv(self, timeout_s: float):
        msg = self._bus.recv(timeout=timeout_s)
        if msg is None:
            return None
        return CanFrame(arbitration_id=msg.arbitration_id, data=bytes(msg.data))


def _make_controller(bus, addr, joint_addrs):
    """单轴 ZdtController, 减速比按当前关节查 DEFAULT_REDUCTION_RATIOS.

    限位同步裁剪为当前关节 (FIRMWARE_JOINT_LIMITS[slot]) — 否则单轴模式下
    set_joints_safe/check_limits_real 会用 limits[0]=J1, 对 J2+ 限位失效.
    """
    from arm_robot.controller.config import (
        FIRMWARE_JOINT_LIMITS, ZdtConfig,
    )
    from arm_robot.controller.controller import ZdtController
    ratio = CURRENT_RATIO if CURRENT_RATIO is not None else _ratio_for_addr(addr, joint_addrs)
    slot = joint_addrs.index(addr) if addr in joint_addrs else 0
    cfg = ZdtConfig(
        channel="<interactive>",      # 不走 connect(), 仅占位
        timeout_s=0.3, retries=2,
        joint_addrs=[addr],
        reduction_ratios=[ratio],
        limits=[FIRMWARE_JOINT_LIMITS[slot]],
        speed_rpm=30.0,
        watchdog_s=5.0,
    )
    return ZdtController(config=cfg, transport=_BusAdapter(bus))


def _make_controller_all(bus, joint_addrs):
    """整机 (6 轴) ZdtController, 用 DEFAULT_REDUCTION_RATIOS/限位/CALIB 全表.

    ready/soft_reset 等全轴命令用. 速度/标定默认从 config 取 (ready 内部用
    READY_SPEED_RPM 覆盖, 不改全局).
    """
    from arm_robot.controller.config import ZdtConfig
    from arm_robot.controller.controller import ZdtController
    cfg = ZdtConfig(
        channel="<interactive>",      # 不走 connect(), 仅占位
        timeout_s=0.3, retries=2,
        joint_addrs=list(joint_addrs),
        speed_rpm=30.0,
        watchdog_s=5.0,
    )
    return ZdtController(config=cfg, transport=_BusAdapter(bus))


def cmd_prod_status(bus, addr, joint_addrs):
    """生产 get_state: 读角度/电流, 返回输出轴角度."""
    ctrl = _make_controller(bus, addr, joint_addrs)
    try:
        angles, vels, loads = ctrl.get_state()
        print(f"  [ZdtController.get_state] 输出轴: {angles[0]:+.1f}°  电流: {int(loads[0])}mA")
    except Exception as exc:  # noqa: BLE001 — ZdtDriverError 等
        print(f"  [get_state] 失败: {type(exc).__name__}: {exc}")


def cmd_prod_move(bus, addr, deg, joint_addrs):
    """生产 set_joints: clamp → 读当前 → 最短路径相对运动."""
    ctrl = _make_controller(bus, addr, joint_addrs)
    try:
        ctrl.set_joints([deg])
        print(f"  [ZdtController.set_joints] 目标输出轴 {deg:+.1f}° 已发 (读当前→相对 0xFD)")
    except Exception as exc:  # noqa: BLE001
        print(f"  [set_joints] 失败: {type(exc).__name__}: {exc}")


def cmd_prod_rel(bus, addr, deg, joint_addrs):
    """生产 rel_rotate: 输出轴相对角度."""
    ctrl = _make_controller(bus, addr, joint_addrs)
    try:
        ctrl.rel_rotate(1, deg)
        print(f"  [ZdtController.rel_rotate] {deg:+.1f}° 已发")
    except Exception as exc:  # noqa: BLE001
        print(f"  [rel_rotate] 失败: {type(exc).__name__}: {exc}")


def cmd_prod_reset(bus, addr, joint_addrs):
    """生产 soft_reset: 读当前→回 INIT_POSE (轴1 → 90°)."""
    ctrl = _make_controller(bus, addr, joint_addrs)
    try:
        ctrl.soft_reset()
        print("  [ZdtController.soft_reset] 回 INIT_POSE 90° (读当前→相对运动)")
    except Exception as exc:  # noqa: BLE001
        print(f"  [soft_reset] 失败: {type(exc).__name__}: {exc}")


def cmd_prod_ready(bus, joint_addrs):
    """生产 ready: 6 关节同步慢速运动至按摩准备姿态 READY_POSE_DEG.

    用整机控制器 + CALIB(k,b) 真实位置限位; 速度 READY_SPEED_RPM (仅本条, 快慢
    不影响全局 speed_rpm). 每轴 0xFD(snF=1) → multi_sync 广播同时启动.
    """
    from arm_robot.controller.config import (
        CALIB, READY_POSE_DEG, READY_SPEED_RPM,
    )
    ctrl = _make_controller_all(bus, joint_addrs)
    try:
        targets = ctrl.ready(use_kb=True, calib_kb=CALIB)
        print(f"  [ready] 目标 {READY_POSE_DEG}  (同步 {len([t for t in targets if t != 0])} 轴"
              f" @ {READY_SPEED_RPM} RPM)")
        print(f"  [ready] clamp 后 {targets}")
    except Exception as exc:  # noqa: BLE001
        print(f"  [ready] 失败: {type(exc).__name__}: {exc}")


def cmd_safe_move(bus, addr, deg, joint_addrs):
    """安全运动 (真机验证限位): 目标越界 → stop+告警不下发; 界内 → 运动.

    单轴模式: controller 的 slot0 = 当前关节 (limits/CALIB 均按 slot 裁剪为当前
    关节), high_freq={0} 强制检查当前关节. 顺序 = 预检 → 通过才 set_joints_safe 运动.
    基准 = 0x36 真实位置 (经 CALIB (k,b) 换算), 不依赖命令积分.
    """
    ctrl = _make_controller(bus, addr, joint_addrs)
    slot = joint_addrs.index(addr) if addr in joint_addrs else 0
    calib_kb_slot = [CALIB[slot]]   # 单轴控制器索引恒为 0, 必须裁到当前关节行
    try:
        alarms = ctrl.check_limits_real([deg], use_kb=True, calib_kb=calib_kb_slot,
                                        high_freq={0})
        if alarms:
            for a in alarms:
                print(f"  ⚠ 限位告警: target={a['target']}° real={a['real']}° "
                      f"limit={a['limit']} → 已 stop, 不下发运动")
            return
        print(f"  ✓ 目标 {deg:+.1f}° 在限位内")
        targets = ctrl.set_joints_safe([deg], use_kb=True, calib_kb=calib_kb_slot)
        print(f"  [set_joints_safe] 已下发 → {targets[0]:+.1f}°")
    except Exception as exc:  # noqa: BLE001
        print(f"  [safe mv] 失败: {type(exc).__name__}: {exc}")


# ── 物理标定修正 ─────────────────────────────────────────────

def cmd_fixup(commanded_deg: float, measured_deg: float,
              addr: int, joint_addrs: list[int]):
    """按物理实测角度修正当前关节减速比.

    new_ratio = old_ratio × commanded / measured
    用法: 输出轴贴上标记, 发 fdrel <commanded>, 实测转了 <measured>,
          然后 fixup <commanded> <measured>. 不依赖驱动板位置读数.
    标定结果写入 DEFAULT_REDUCTION_RATIOS 对应槽位 (持久影响该关节).
    """
    if measured_deg <= 0:
        print(f"  fixup 失败: measured 必须 > 0, got {measured_deg}")
        return
    slot = next((i for i, a in enumerate(joint_addrs) if a == addr), None)
    if slot is None:
        print(f"  当前地址 0x{addr:02X} 不在关节映射中, 无法标定")
        return
    old_ratio = DEFAULT_REDUCTION_RATIOS[slot]
    factor = commanded_deg / measured_deg
    new_ratio = old_ratio * factor
    print(f"  fixup J{slot+1}: ratio {old_ratio:.2f} × ({commanded_deg}/{measured_deg}={factor:.4f})")
    print(f"       → {new_ratio:.2f}")
    DEFAULT_REDUCTION_RATIOS[slot] = new_ratio
    print(f"  已更新 DEFAULT_REDUCTION_RATIOS[{slot}] = {new_ratio:.2f}  (J{slot+1})")
    print(f"  验证: 下次 fdrel {commanded_deg:.1f} 应恰好转 {commanded_deg:.1f}°")
    print(f"  ⚠ 仅内存生效, 需手动同步到 config.py 持久化")


def cmd_setmed(n_pulses: int, measured_deg: float,
               addr: int, joint_addrs: list[int]):
    """由原始脉冲数与实测输出角度直接精确反推当前关节减速比.

    ratio = n_pulses×360 / (measured_deg×3200)
    精确标定法: 用 fdp <P> 发已知脉冲, 数输出轴实际转的整圈+尾数得 M,
    再 setmed <P> <M> 直接定 ratio (不依赖驱动板位置读数).
    测量误差被大角度摊薄: 多圈数圈比量角度准.
    标定结果写入 DEFAULT_REDUCTION_RATIOS 对应槽位 (持久影响该关节).
    """
    if measured_deg <= 0:
        print(f"  setmed 失败: measured 必须 > 0, got {measured_deg}")
        return
    slot = next((i for i, a in enumerate(joint_addrs) if a == addr), None)
    if slot is None:
        print(f"  当前地址 0x{addr:02X} 不在关节映射中, 无法标定")
        return
    ratio = n_pulses * 360.0 / (measured_deg * PULSES_PER_REV)
    DEFAULT_REDUCTION_RATIOS[slot] = ratio
    model_deg = n_pulses * 360.0 / PULSES_PER_REV
    print(f"  setmed J{slot+1}: {n_pulses} 脉冲 / 实测 {measured_deg:.1f}°")
    print(f"  (命令模型角度 = {model_deg:.1f}° @3200脉冲/圈)")
    print(f"  已更新 DEFAULT_REDUCTION_RATIOS[{slot}] = {ratio:.2f}  (J{slot+1})")
    print(f"  验证: fdrel 90 → 输出轴应精确转 90° (数圈复核)")
    print(f"  ⚠ 仅内存生效, 需手动同步到 config.py 持久化")


# ── 0x36 刻度标定 (pos 读数 ↔ 实际输出角度) ─────────────────

def least_squares_fit(points: list[tuple[float, float]]):
    """最小二乘拟合 pos = k×角度 + b. 点<2 或角度过集中返回 None.

    返回 (k, b, r2, max_err): k=斜率(pos/°), b=截距(pos@0°),
    r2=决定系数, max_err=用拟合线反推每个点的最大角度偏差(°) — 拟合质量.
    """
    n = len(points)
    if n < 2:
        return None
    sum_x = sum(p[0] for p in points)
    sum_y = sum(p[1] for p in points)
    sum_xx = sum(p[0] * p[0] for p in points)
    sum_xy = sum(p[0] * p[1] for p in points)
    denom = n * sum_xx - sum_x * sum_x
    if abs(denom) < 1e-12:               # 角度都一样 → 无法拟合斜率
        return None
    k = (n * sum_xy - sum_x * sum_y) / denom
    b = (sum_y - k * sum_x) / n
    ybar = sum_y / n
    ss_tot = sum((p[1] - ybar) ** 2 for p in points)
    ss_res = sum((p[1] - (k * p[0] + b)) ** 2 for p in points)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 1.0
    max_err = max(abs((p[1] - b) / k - p[0]) for p in points) if k != 0 else 0.0
    return k, b, r2, max_err


def cmd_cal(cal_points: dict[int, list[tuple[float, float]]],
            addr: int, joint_addrs: list[int], args: list[str]):
    """0x36 刻度标定: 收集 (实际角度, pos读数) 对, 拟合 pos=k×角+b.

    0x36 返回带符号真实位置 (说明书 §7.4.4), 不是"V型绝对值" — 旧注释已修正.
    pos 带符号, 拟合 pos = k × 实际角度 + b (线性, 不取绝对值).
    标定点按电机地址分开存放: 切 jN 自动切到该关节自己的点, 各关节不串数据.
    用法:
      cal <角°> <pos>  添加标定点并实时拟合
      cal              显示当前关节的点与拟合
      cal del          删除当前关节最后一个点
      cal clear        清空当前关节所有点
    """
    points = cal_points.setdefault(addr, [])
    label = _addr_label(addr, joint_addrs)
    if args and args[0] == "clear":
        points.clear()
        print(f"  [{label}] 标定点已清空")
        return
    if args and args[0] == "del":
        if points:
            a, p = points.pop()
            print(f"  [{label}] 已删除标定点 ({a:+.1f}°, pos={p:+.1f})")
        else:
            print(f"  [{label}] 无标定点可删")
    elif len(args) >= 2:
        try:
            angle, pos = float(args[0]), float(args[1])
        except ValueError:
            print("  用法: cal <实际角度°> <pos读数>  (如: cal 45 200)")
            return
        points.append((angle, pos))
        print(f"  [{label}] 已添加标定点 ({angle:+.1f}°, pos={pos:+.1f})")
    elif args:
        print("  用法: cal <实际角度°> <pos> | cal | cal del | cal clear")
        return

    print(f"  [{label}] 当前 {len(points)} 个标定点:")
    for i, (a, p) in enumerate(points):
        print(f"    {i+1}. 实际 {a:+.1f}°  ↔  pos {p:+.1f}")
    if len(points) < 2:
        print("  (需 ≥2 个点才能拟合, 且点与点角度差要拉开)")
        return
    fit = least_squares_fit(points)
    if fit is None:
        print("  角度过于集中, 无法拟合 — 请拉开点与点之间的角度差")
        return
    k, b, r2, max_err = fit
    sgn = "+" if b < 0 else "-"
    print(f"  拟合: pos = {k:.4f} × 角度 {sgn} {abs(b):.2f}")
    print(f"  换算: 输出角度 ≈ (pos {sgn} {abs(b):.2f}) / {k:.4f}")
    print(f"  质量: R²={r2:.4f}  逐点最大角度偏差 {max_err:.2f}°")
    print(f"  记录: 把 (k={k:.4f}, b={b:.2f}) 填入 config.py 的 CALIB 对应关节槽, 即可用 anchor 锚定")
    print(f"  ⚠ (k,b) 是这台电机独有, 勿跨关节套用")


def cmd_anchor(bus, addr: int, joint_addrs: list[int], args: list[str]):
    """上电锚定真实角度: 读 0x36 带符号位置 → 标定换算 → 输出真实角度.

    0x36 返回带符号真实位置 (说明书 §7.4.4 + 固件 robot.c:1045 确认),
    不是"V型绝对值" — 旧版要用户指定符号是错误认知, 已修正.

    两种标定路线 (互斥):
      CALIB_OFFSETS[slot] 非空: 真实 = (0x36/减速比) - offset
      CALIB[slot] (k,b) 非空:   真实 = (0x36 - b) / k
      都空: 退化为纯减速比换算 (未标定, 仅参考)

    用法:
      anchor              按当前关节标定表锚定
      anchor <k> <b>      临时用给定 k,b 锚定 (未标定时验证用)
      anchor <offset>     临时用给定 offset 锚定 (CALIB_OFFSETS 路线)
    """
    slot = next((i for i, a in enumerate(joint_addrs) if a == addr), None)
    if slot is None:
        print(f"  0x{addr:02X} 不在关节映射中 — 先切到 j1..j6")
        return

    # 解析参数: 临时 k,b 或 offset
    tmp_kb = None
    tmp_offset = None
    if len(args) >= 2:
        try:
            tmp_kb = (float(args[0]), float(args[1]))
        except ValueError:
            print("  用法: anchor | anchor <k> <b> | anchor <offset>")
            return
    elif len(args) == 1:
        try:
            tmp_offset = float(args[0])
        except ValueError:
            print("  用法: anchor | anchor <k> <b> | anchor <offset>")
            return

    # 确定标定参数 (优先临时参数, 其次 CALIB_OFFSETS, 其次 CALIB)
    offset = tmp_offset
    kb = tmp_kb
    if offset is None and kb is None:
        from arm_robot.controller.config import CALIB_OFFSETS
        if CALIB_OFFSETS[slot] is not None:
            offset = CALIB_OFFSETS[slot]
        elif CALIB[slot] is not None:
            kb = CALIB[slot]

    data, _ms = raw_send_recv(bus, addr, bytes([F_READ_POS, CHECKSUM]),
                              F_READ_POS, timeout_s=0.2)
    if data is None:
        print("  读 pos 超时 — 电机离线或 Response=None?")
        return
    sbyte = -1 if data[1] == 0x01 else 1   # 0x00=正 0x01=负
    pos = decode_pos4(data[2:6], sbyte)

    # 换算真实角度
    if kb is not None:
        k, b = kb
        if abs(k) < 1e-9:
            print("  k≈0 无法换算")
            return
        real = (pos - b) / k
        calib_desc = f"CALIB(k={k:.4f}, b={b:.2f})"
    else:
        out_deg = pos / GEAR_RATIO
        real = out_deg - (offset if offset is not None else 0.0)
        calib_desc = (f"CALIB_OFFSETS(offset={offset:.2f})" if offset is not None
                      else "未标定(纯减速比换算)")
    print(f"  锚定 J{slot+1}(0x{addr:02X}): pos={pos:+.1f}  {calib_desc}")
    print(f"  →  真实输出角度 ≈ {real:+.1f}°")
    print(f"  此值即面板 tracked_deg 的初始化基准 (上电/外力搬动后重新 anchor)")


def cmd_clrpos(bus, addr: int, joint_addrs: list[int], args: list[str]):
    """清零当前位置 (0x0A 6D): 让 0x36 立刻读出 0.

    A 任务标定方案 b 主命令 (固件 robot.c:231/242 + robot_cmd.c:131 验证可用).
    与 setzero(0x93 88 01) 区别: 0x0A 6D 改 0x36 当前读数, 不影响回零目标;
    0x93 88 01 设回零目标但不改 0x36 当前读数.

    用法:
      clrpos         提示后需输 clrpos CONFIRM 才执行
      clrpos CONFIRM 直接执行
    """
    label = _addr_label(addr, joint_addrs)
    if len(args) < 1 or args[0] != "CONFIRM":
        print(f"  [{label}] 清零当前位置 (0x0A 6D) — 让 0x36 立刻读出 0")
        print(f"  确认后执行: clrpos CONFIRM")
        return
    send_no_reply(bus, addr, add_checksum(bytes([0x0A, 0x6D])))
    print(f"  [{label}] 清零命令已发送 (0x0A 6D)")
    time.sleep(0.3)
    data, _ = raw_send_recv(bus, addr, bytes([F_READ_POS, CHECKSUM]),
                            F_READ_POS, timeout_s=0.2)
    if data is None:
        print("  验证 pos 超时 — 清零可能已生效, 用 pos 手动确认")
        return
    pos = decode_pos4(data[2:6], -1 if data[1] == 0x01 else 1)
    if abs(pos) < 1.0:
        print(f"  ✅ [{label}] pos≈{pos:+.1f} — 当前位置已清零, 0x36 此后读相对偏移")
        print("  下一步: anchor 锚定真实角度, 把 offset 存入 config.CALIB_OFFSETS")
    else:
        print(f"  ⚠ [{label}] pos={pos:+.1f} 不为 0 — 清零可能失败, 检查后重试")


def cmd_setzero(bus, addr: int, joint_addrs: list[int], args: list[str]):
    """设单圈回零零点 (0x93 88 01, 存储型): 把当前机械位置设为回零目标.

    ⚠ 这是"单圈回零零点设置", 不是"清零当前位置". 设完后触发 home(0x9A) 会回到
    这个位置, 但 0x36 当前读数不变. 要让 0x36 立刻读出 0 用 clrpos(0x0A 6D).
    持久写入驱动器, 不可逆. 设零后须从新 0 点重新标定并更新 CALIB.
    用法:
      setzero         提示后需输 setzero CONFIRM 才执行
      setzero CONFIRM 直接执行
    """
    label = _addr_label(addr, joint_addrs)
    if len(args) < 1 or args[0] != "CONFIRM":
        print(f"  [{label}] ⚠ 设单圈回零零点是持久写入驱动器 (0x93 88 01), 不可逆!")
        print(f"  先把关节摆到期望回零位置, 确认后执行: setzero CONFIRM")
        print(f"  (若想让 0x36 立刻读出 0, 用 clrpos 命令 — 0x0A 6D 清零当前位置)")
        return
    send_no_reply(bus, addr, add_checksum(bytes([0x93, 0x88, 0x01])))
    print(f"  [{label}] 设零命令已发送 (0x93 88 01)")
    time.sleep(0.3)
    data, _ = raw_send_recv(bus, addr, bytes([F_READ_POS, CHECKSUM]),
                            F_READ_POS, timeout_s=0.2)
    if data is None:
        print("  验证 pos 超时 — 设零可能已生效, 用 pos 手动确认")
        return
    pos = decode_pos4(data[2:6], -1 if data[1] == 0x01 else 1)
    print(f"  [{label}] pos={pos:+.1f} (0x36 读数不变是正常 — 0x93 设回零目标, 不清零当前)")
    print(f"  触发回零用 home 命令 (0x9A 00 00), 电机会回到刚设的零点位置")


# ── 回零闭环: setzero 设零点 → origin set 配置 → home 触发/上电自动回零 ──
#   (说明书 §7.4.2/§8.1: 0x93 设零点 + 0x4C 0xAE 配参数 + 0x9A 触发)
#   ⚠ 驱动器 Flash 记忆零点与回零参数, 使能上电自动回零后, 每次上电电机
#     自动转到设过的零点 → pos=0 即零点 → PC 侧 anchor 基准有效.

HOME_MODES = {0: "Nearest(单圈就近)", 1: "Dir(单圈方向)",
              2: "Senseless(多圈无限位)", 3: "EndStop(多圈有限位)"}


def _cmd_status_to_str(status_byte: int) -> str:
    """回零命令状态字节 → 人类可读 (说明书 §7.4.2)."""
    if status_byte == 0x02:
        return "OK(命令已接受)"
    if status_byte == 0xE2:
        return "FAIL(条件不满足: 堵转保护/零点无效/正在回零)"
    return f"0x{status_byte:02X}"


def cmd_home(bus, addr: int, joint_addrs: list[int], args: list[str]):
    """触发回零 (0x9A <mode> 00): 电机回到 setzero 存的零点.

    ⚠ 会真实运动电机 (可能转大角度), 需 CONFIRM. 0x9A 仅触发, 回零模式参数
    由 origin set (0x4C 0xAE) 预先配置; 此处 mode 为触发时的模式字节.
    用法:
      home               提示后需输 home CONFIRM 才执行
      home CONFIRM [mode] 触发回零, 默认 mode=0(Nearest)
      home stop          强制中断并退出回零 (0x9C 48)
    """
    label = _addr_label(addr, joint_addrs)
    if len(args) >= 1 and args[0] == "stop":
        send_no_reply(bus, addr, add_checksum(bytes([0x9C, 0x48])))
        print(f"  [{label}] 已发送中断回零命令 (0x9C 48)")
        return
    mode = 0
    if len(args) >= 2:
        try:
            mode = int(args[1])
        except ValueError:
            print("  用法: home CONFIRM [mode] | home stop")
            return
    if len(args) < 1 or args[0] != "CONFIRM":
        print(f"  [{label}] ⚠ 触发回零会让电机回到 setzero 设的零点 (可能转大角度)!")
        print(f"  回零模式: {HOME_MODES.get(mode, '未知')}")
        print(f"  确认后执行: home CONFIRM [{mode}]   | 中断: home stop")
        return
    # 0x9A <mode> <snF>: 触发回零, 期待回帧 [9A, 状态, 6B]
    data, ms = raw_send_recv(bus, addr, bytes([0x9A, mode, 0x00]),
                             0x9A, timeout_s=1.0)
    if data is None or len(data) < 2:
        print(f"  [{label}] 回零触发超时 ({ms:.0f}ms) — 固件可能不识别 0x9A, 用 origin status 确认")
        return
    print(f"  [{label}] 触发回零 (0x9A 模式={mode} {HOME_MODES.get(mode,'?')}) → {_cmd_status_to_str(data[1])}")
    print(f"  回零需数秒, 用 origin status 查状态, 或 loop flag 看是否到位")


def cmd_origin_status(bus, addr: int, joint_addrs: list[int], args: list[str]):
    """读回零状态标志 (0x3B): 就绪/正在回零/回零失败."""
    label = _addr_label(addr, joint_addrs)
    data, ms = raw_send_recv(bus, addr, bytes([0x3B, CHECKSUM]), 0x3B, timeout_s=0.3)
    if data is None or len(data) < 2:
        print(f"  [{label}] 读回零状态超时 — 固件可能不识别 0x3B")
        return
    st = data[1]
    parts = []
    parts.append(f"编码器就绪: {bool(st & 0x01)}")
    parts.append(f"校准表就绪: {bool(st & 0x02)}")
    parts.append(f"正在回零: {bool(st & 0x04)}")
    parts.append(f"回零失败: {bool(st & 0x08)}")
    print(f"  [{label}] 回零状态 0x{st:02X}: " + " | ".join(parts))
    if st & 0x08:
        print("  ⚠ 回零失败 — 触发堵转/零点无效/超时, 检查机械结构或重新 setzero")


def cmd_origin(bus, addr: int, joint_addrs: list[int], args: list[str]):
    """读原点回零参数 (0x22) 并解析显示."""
    label = _addr_label(addr, joint_addrs)
    data, ms = raw_send_recv(bus, addr, bytes([0x22, CHECKSUM]), 0x22, timeout_s=0.3)
    if data is None or len(data) < 2:
        print(f"  [{label}] 读回零参数超时 — 固件可能不识别 0x22")
        return
    d = data
    mode = d[1] if len(d) > 1 else 0
    direction = d[2] if len(d) > 2 else 0
    vel = ((d[3] << 8) | d[4]) if len(d) > 4 else 0
    timeout = ((d[5] << 24) | (d[6] << 16) | (d[7] << 8) | d[8]) if len(d) > 8 else 0
    col_speed = ((d[9] << 8) | d[10]) if len(d) > 10 else 0
    col_cur = ((d[11] << 8) | d[12]) if len(d) > 12 else 0
    col_time = ((d[13] << 8) | d[14]) if len(d) > 14 else 0
    pot_en = d[15] if len(d) > 15 else 0
    print(f"  [{label}] 原点回零参数 (0x22):")
    print(f"    回零模式   : {HOME_MODES.get(mode, mode)}")
    print(f"    回零方向   : {'CW' if direction == 0 else 'CCW'}")
    print(f"    回零速度   : {vel} RPM")
    print(f"    回零超时   : {timeout} ms")
    print(f"    碰撞检测转速: {col_speed} RPM | 电流 {col_cur} mA | 时间 {col_time} ms")
    print(f"    上电自动回零: {'使能 Enable' if pot_en else '不使能 Disable'}")


def cmd_origin_set(bus, addr: int, joint_addrs: list[int], args: list[str]):
    """修改并存储原点回零参数 (0x4C 0xAE 01 ...).

    核心: 使能"上电自动触发回零"后, 每次上电电机自动回零到 setzero 存的零点,
    实现断电记忆定位 (该位置即 pos=0, PC 侧 anchor 基准).
    ⚠ 持久写入驱动器 Flash, 不可逆, 需 CONFIRM.

    用法 (参数均可选, 默认=说明书示例):
      origin set                         提示后需输 CONFIRM
      origin set CONFIRM [mode] [dir] [vel] [poweron]
        mode    : 0=Nearest 1=Dir 2=Senseless 3=EndStop (默认0)
        dir     : 0=CW 1=CCW (默认0)
        vel     : 回零转速 RPM (默认30)
        poweron : 0=不使能上电自动回零 1=使能 (默认0)
    """
    label = _addr_label(addr, joint_addrs)
    if len(args) < 1 or args[0] != "CONFIRM":
        print(f"  [{label}] ⚠ 修改原点回零参数是持久写入驱动器 Flash, 不可逆!")
        print(f"  先完成 setzero 设零点, 再执行: origin set CONFIRM [mode] [dir] [vel] [poweron]")
        print(f"  (poweron=1 → 让每次上电自动回零到零点, 即断电记忆定位)")
        return
    mode = int(args[1]) if len(args) > 1 else 0
    direction = int(args[2]) if len(args) > 2 else 0
    vel = int(args[3]) if len(args) > 3 else 30
    poweron = int(args[4]) if len(args) > 4 else 0
    timeout_ms = 10_000
    col_speed, col_cur, col_time = 4000, 800, 60
    # 参数体 16 字节 (说明书 §7.4.2 标准布局): svF,模式,方向,速度,超时,碰速,碰流,碰时,POT
    params = bytes([
        0x01,                      # 存储标志 svF=1
        mode & 0xFF,               # 回零模式
        direction & 0xFF,          # 回零方向
        (vel >> 8) & 0xFF, vel & 0xFF,                    # 回零速度 (16bit)
        (timeout_ms >> 24) & 0xFF, (timeout_ms >> 16) & 0xFF,
        (timeout_ms >> 8) & 0xFF, timeout_ms & 0xFF,      # 回零超时 (32bit)
        (col_speed >> 8) & 0xFF, col_speed & 0xFF,         # 碰撞转速
        (col_cur >> 8) & 0xFF, col_cur & 0xFF,             # 碰撞电流
        (col_time >> 8) & 0xFF, col_time & 0xFF,           # 碰撞时间
        poweron & 0xFF,             # 上电自动回零使能
    ])
    payload = add_checksum(bytes([0x4C, 0xAE]) + params)
    data, ms = raw_send_recv(bus, addr, payload, 0x4C, timeout_s=0.5)
    if data is None or len(data) < 2:
        print(f"  [{label}] 修改回零参数超时 ({ms:.0f}ms) — 固件可能不识别 0x4C 0xAE")
        return
    print(f"  [{label}] 已存储回零参数: {HOME_MODES.get(mode, mode)} 方向={'CW' if direction==0 else 'CCW'} "
          f"速度={vel}RPM 超时={timeout_ms}ms → {_cmd_status_to_str(data[1])}")
    print(f"  上电自动回零: {'使能 Enable (每次上电自动回零, 断电记忆定位)' if poweron else '不使能 Disable'}")
    if poweron:
        print(f"  ⚠ 已使能上电自动回零 — 下次上电机械臂会自行运动到零点, 请确保周围无遮挡!")


# ── 帮助信息 ────────────────────────────────────────────────

HELP = """
=== ZDT 电机交互式调试器 (本机已验证指令集) ===

  读命令 (期待回帧):
    pos              读驱动器位置 (仅作参考, 刻度与物理输出不成固定比例)
    cur              读相电流 (mA)
    flag             读状态标志位 (使能/到位/堵转/堵转保护)

  写命令 (无回帧):
    en               使能电机
    dis              失能电机
    stop             停止当前电机
    estop            广播急停 (所有电机)

  运动命令 (0xFD 固件兼容格式, 与 STM32 固件 Emm_V5_Pos_Control 一致, 本机已验证):
    fdrel <deg> [spd] [acc]  相对运动 (输出角度; 换算 = deg×ratio×3200/360)
    fdp <n> [dir] [spd] [acc] 原始脉冲数相对运动 (标定用)

  标定与换算:
    ratio [n]        显示/设置 命令缩放比. 默认按当前关节自动查 DEFAULT_REDUCTION_RATIOS;
                     ratio <n> 手动 override (影响所有关节); ratio 0 清除 override
    setmed <P> <M>  精确标定当前关节: fdp <P> 发已知脉冲, 数输出轴实际转 M 度
                     (整圈×360+尾数), 直接定 ratio = P×360/(M×3200), 写入对应关节槽
    fixup <cmd> <meas>  迭代修正当前关节: 发 fdrel <cmd> 实测转了 <meas> °,
                     new = ratio × cmd/meas, 写入对应关节槽
    cal <角°> <pos>  0x36 刻度标定: 拟合 pos=k×角+b (pos 带符号, 说明书 §7.4.4)
                     (点按关节分存: 切 jN 自动切换; cal del/clear 管理点位)
    anchor           上电锚定真实角度: 读 0x36 带符号位置 + CALIB_OFFSETS/CALIB 换算
                     → 输出真实角度 (限位基准; anchor <k> <b> 或 anchor <offset> 临时指定)
    clrpos [CONFIRM] 清零当前位置 (0x0A 6D): 让 0x36 立刻读出 0 (A 任务标定方案 b 主命令)
                     (固件 robot.c 验证可用; 与 setzero 区别: 0x0A 6D 改 0x36 读数)
    setzero [CONFIRM] 设单圈回零零点 (0x93 88 01, 存储, 不可逆): 当前位置设为回零目标
                     (0x36 读数不变; 触发回零用 home; 让 0x36 读 0 用 clrpos)

  回零闭环 (驱动器断电台记忆: 零点+回零参数存 Flash, 使能后上电自动回零):
    home CONFIRM [mode] 触发回零 (0x9A mode 00): 电机回 setzero 存的零点
                     mode: 0=Nearest 1=Dir 2=Senseless 3=EndStop (默认0)
    home stop      强制中断并退出回零 (0x9C 48)
    origin         读原点回零参数 (0x22) 并解析显示
    origin status  读回零状态 (0x3B): 就绪/正在回零/回零失败
    origin set CONFIRM [mode] [dir] [vel] [poweron]
                     修改并存储回零参数 (0x4C 0xAE 01, 持久写入, 不可逆):
                     poweron=1 → 使能上电自动回零, 每次上电电机自动回零点
                     (实现断电记忆定位: pos=0 即零点, PC 侧 anchor 基准有效)

  生产驱动测试 (直接调 ZdtController, = MassageRobot transport=can 同路径):
    prod status      读输出轴角度(软件跟踪)/电流 (get_state)
    prod mv <deg>    运动到目标输出角度 (set_joints: clamp→相对0xFD)
    prod rel <deg>   输出轴相对运动 (rel_rotate)
    prod reset       回 INIT_POSE (soft_reset)

  安全运动 (带 0x36 真实位置限位守卫, 验证/使用限位):
    safe mv <deg>    目标越出限位 → stop+告警 不下发; 界内 → set_joints_safe 运动
                     (当前关节限位, 基准 0x36 真实位置经 CALIB 换算; 若 CALIB 未标
                      定该关节, 退化为纯减速比换算 — 需要精确限位前先 cal/anchor)

  原始帧与监控:
    raw <hex>        发送原始 hex (自动加校验, 支持多帧)
    send <id> <hex>  发送指定 CAN ID + 原始 hex
    mon [sec]        监听总线帧 (默认 3 秒)

  六关节操作 (链路确认 / 标定):
    j1..j6          切换当前关节 (pc: J1=0x02..J6=0x07; firmware: 0x01..0x06)
    scan            探测全部关节地址, 显示在线/离线+位置+标志
    all status      读全部关节 pos+cur+flag 一张表
    all en          批量使能全部关节
    all dis         批量失能全部关节 (⚠ 重力 J2/J3 失去力矩, 需 CONFIRM)
    all stop        广播停止全部关节 (= estop, 断命令保力矩)

  系统:
    addr             显示/切换电机地址
    poll [N]         连续读位置 N 次 (默认无限, Ctrl+C 停)
    status           一次读 pos+cur+flag
    help             显示本帮助
    quit / q         退出

  推荐流程 (精确标定, 依据固件 Emm_V5.c 的 pulses=deg×ratio×3200/360):
    en               →  先使能
    fdp 32000 0 60   →  发恰好 32000 脉冲 (模型 10 圈), 等到位
                        (输出轴绑指针+固定参考点)
    [数指针转过的整圈×360 + 尾数°] → 得实测 M
    setmed 32000 M   →  ratio = 3600/M, 直接精确标定
    fdrel 90         →  复核: 指针应精确转 90°
    prod mv 90       →  验证生产 set_joints 同步 OK
    [有偏差再 fixup 一次收敛]
"""


# ── 主循环 ──────────────────────────────────────────────────

def main():
    global CURRENT_RATIO
    p = argparse.ArgumentParser(description="ZDT 电机交互式调试器 (六关节)")
    p.add_argument("--iface", default="can0", help="SocketCAN 接口 (默认 can0)")
    p.add_argument("--addr", type=lambda x: int(x, 0), default=0x02,
                   help="起始电机 CAN 地址 (默认 0x02)")
    p.add_argument("--scheme", choices=["pc", "firmware"], default="pc",
                   help="关节→地址映射: pc=J1..J6→0x02..0x07 (默认), firmware=→0x01..0x06")
    args = p.parse_args()

    joint_addrs = list(JOINT_ADDRS if args.scheme == "pc" else FIRMWARE_JOINT_ADDRS)
    addr = args.addr
    cal_points: dict[int, list[tuple[float, float]]] = {}   # cal 标定点, 按电机地址分存
    bus = can.Bus(interface="socketcan", channel=args.iface, bitrate=500_000)
    print(f"SocketCAN {args.iface} @ 500kbps  |  scheme={args.scheme}  |  "
          f"当前: {_addr_label(addr, joint_addrs)}")
    print(f"输入 help 查看命令; j1..j6 切换关节, scan 探测链路\n")

    try:
        while True:
            try:
                line = input(f"[{_addr_label(addr, joint_addrs)}] > ").strip()
            except EOFError:
                break
            if not line:
                continue

            parts = line.split()
            cmd = parts[0].lower()

            try:
                if cmd in ("quit", "q", "exit"):
                    break

                elif cmd == "help":
                    print(HELP)

                elif cmd == "pos":
                    cmd_read_pos(bus, addr)

                elif cmd == "cur":
                    cmd_read_current(bus, addr)

                elif cmd == "flag":
                    cmd_read_flag(bus, addr)

                elif cmd == "status":
                    print(f"  --- 位置 ---")
                    cmd_read_pos(bus, addr)
                    print(f"  --- 电流 ---")
                    cmd_read_current(bus, addr)
                    print(f"  --- 状态 ---")
                    cmd_read_flag(bus, addr)

                elif cmd == "en":
                    cmd_enable(bus, addr, True)

                elif cmd == "dis":
                    cmd_enable(bus, addr, False)

                elif cmd == "stop":
                    cmd_stop(bus, addr)

                elif cmd == "estop":
                    cmd_stop_all(bus)

                elif cmd in ("j1", "j2", "j3", "j4", "j5", "j6"):
                    slot = int(cmd[1]) - 1
                    addr = joint_addrs[slot]
                    n = len(cal_points.get(addr, []))
                    print(f"  切换到 J{slot+1} (0x{addr:02X})"
                          + (f"  [已有 {n} 个标定点]" if n else "  [无标定点]"))

                elif cmd == "scan":
                    cmd_scan(bus, joint_addrs)

                elif cmd == "all":
                    sub = parts[1].lower() if len(parts) > 1 else ""
                    if sub == "status":
                        cmd_all_status(bus, joint_addrs)
                    elif sub == "en":
                        cmd_all_enable(bus, joint_addrs)
                    elif sub == "dis":
                        if len(parts) < 3 or parts[2] != "CONFIRM":
                            print("  ⚠ 批量失能会让 J2/J3 重力关节失去力矩, 手臂必须有人支撑!")
                            print("  确认执行请输入: all dis CONFIRM")
                        else:
                            cmd_all_disable(bus, joint_addrs)
                    elif sub == "stop":
                        cmd_all_stop(bus)
                    else:
                        print("  用法: all status | en | dis | stop")

                elif cmd == "fdrel":
                    deg = float(parts[1])
                    spd = float(parts[2]) if len(parts) > 2 else 30.0
                    acc = int(parts[3]) if len(parts) > 3 else 20
                    ratio = CURRENT_RATIO if CURRENT_RATIO is not None else _ratio_for_addr(addr, joint_addrs)
                    cmd_fd_move_rel(bus, addr, deg, spd, acc, ratio=ratio)

                elif cmd == "fdp":
                    n_pulses = int(parts[1])
                    dir_is_cw = (int(parts[2]) if len(parts) > 2 else 0) == 0
                    spd = float(parts[3]) if len(parts) > 3 else 30.0
                    acc = int(parts[4]) if len(parts) > 4 else 20
                    cmd_fd_pulse(bus, addr, n_pulses, dir_is_cw, spd, acc)

                elif cmd == "prod":
                    sub = parts[1].lower() if len(parts) > 1 else ""
                    if sub == "status":
                        cmd_prod_status(bus, addr, joint_addrs)
                    elif sub == "mv":
                        cmd_prod_move(bus, addr, float(parts[2]), joint_addrs)
                    elif sub == "rel":
                        cmd_prod_rel(bus, addr, float(parts[2]), joint_addrs)
                    elif sub == "reset":
                        cmd_prod_reset(bus, addr, joint_addrs)
                    elif sub == "ready":
                        cmd_prod_ready(bus, joint_addrs)
                    else:
                        print("  用法: prod status | mv <deg> | rel <deg> | reset | ready")

                elif cmd == "safe":
                    # 带 0x36 真实位置限位守卫的单关节安全运动 (验证限位)
                    sub = parts[1].lower() if len(parts) > 1 else ""
                    if sub == "mv":
                        cmd_safe_move(bus, addr, float(parts[2]), joint_addrs)
                    else:
                        print("  用法: safe mv <deg>  (目标越界→stop+告警, 界内→运动)")

                elif cmd == "ratio":
                    if len(parts) > 1:
                        val = float(parts[1])
                        if val == 0:
                            CURRENT_RATIO = None
                            print(f"  已清除 override, 恢复自动按关节查表")
                        else:
                            CURRENT_RATIO = val
                            print(f"  命令缩放比 override -> {CURRENT_RATIO}  (影响所有关节的 fdrel)")
                    else:
                        if CURRENT_RATIO is not None:
                            print(f"  当前命令缩放比: {CURRENT_RATIO}  (override, 影响所有关节)")
                        else:
                            cur = _ratio_for_addr(addr, joint_addrs)
                            slot = joint_addrs.index(addr) if addr in joint_addrs else -1
                            jlabel = f"J{slot+1}" if slot >= 0 else f"0x{addr:02X}"
                            print(f"  当前命令缩放比: {cur}  (自动按关节查表, 当前 {jlabel})")
                            print(f"  全表: {DEFAULT_REDUCTION_RATIOS}")
                            print(f"  ratio <n> 手动 override; ratio 0 清除 override 恢复自动")

                elif cmd == "fixup":
                    # 物理实测修正: 我要求 X° 实际转了 Y°
                    if len(parts) < 3:
                        print("  用法: fixup <命令角度> <实测角度>  (如 fixup 10 120)")
                    else:
                        cmd_fixup(float(parts[1]), float(parts[2]), addr, joint_addrs)

                elif cmd == "setmed":
                    # 精确标定: 发过 P 脉冲, 实测输出轴转 M 度 → 直接定 ratio
                    if len(parts) < 3:
                        print("  用法: setmed <脉冲数> <实测角度>  (如 setmed 32000 59.2)")
                    else:
                        cmd_setmed(int(parts[1]), float(parts[2]), addr, joint_addrs)

                elif cmd == "cal":
                    cmd_cal(cal_points, addr, joint_addrs, parts[1:])

                elif cmd == "anchor":
                    cmd_anchor(bus, addr, joint_addrs, parts[1:])

                elif cmd == "setzero":
                    cmd_setzero(bus, addr, joint_addrs, parts[1:])

                elif cmd == "home":
                    cmd_home(bus, addr, joint_addrs, parts[1:])

                elif cmd == "origin":
                    sub = parts[1].lower() if len(parts) > 1 else ""
                    if sub == "set":
                        cmd_origin_set(bus, addr, joint_addrs, parts[2:])
                    elif sub == "status":
                        cmd_origin_status(bus, addr, joint_addrs, parts[2:])
                    else:
                        cmd_origin(bus, addr, joint_addrs, parts[1:])

                elif cmd == "clrpos":
                    cmd_clrpos(bus, addr, joint_addrs, parts[1:])

                elif cmd == "raw":
                    cmd_raw(bus, addr, parts[1])

                elif cmd == "send":
                    target = int(parts[1], 0)
                    cmd_raw(bus, target, parts[2])

                elif cmd == "mon":
                    dur = float(parts[1]) if len(parts) > 1 else 3.0
                    cmd_monitor(bus, dur)

                elif cmd == "poll":
                    n = int(parts[1]) if len(parts) > 1 else None
                    print("  Ctrl+C 停止")
                    count = 0
                    while n is None or count < n:
                        try:
                            cmd_read_pos(bus, addr)
                            count += 1
                            time.sleep(0.1)
                        except KeyboardInterrupt:
                            print(f"\n  已采 {count} 次")
                            break

                elif cmd == "addr":
                    if len(parts) > 1:
                        addr = int(parts[1], 0)
                        print(f"  电机地址 -> 0x{addr:02X}")
                    else:
                        print(f"  当前地址: 0x{addr:02X}")

                else:
                    print(f"  未知命令: {cmd}  (输入 help 查看)")

            except IndexError:
                print("  参数不足 (输入 help 查看)")
            except ValueError as e:
                print(f"  参数错误: {e}")
            except can.CanOperationError as e:
                print(f"  CAN 发送失败: {e}")

    except KeyboardInterrupt:
        print()
    finally:
        bus.shutdown()
        print("已断开")


if __name__ == "__main__":
    main()
