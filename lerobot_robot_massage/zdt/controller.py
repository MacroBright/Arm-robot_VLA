"""ZdtController — 高层控制 + 安全层 (spec §3/§4.1).

API 与 SerialProtocol 对齐, 使 MassageRobot 可直接换协议对象 (Task 7).
安全: 位置 clamp 出口 · e_stop 广播 · 看门狗 (tick) · 电流力控 (tick 预留).
"""
import logging
import time
from typing import Optional

from .can_transport import CanTransport
from .config import JOINT_INIT_ANGLE_DEG, READY_POSE_DEG, READY_SPEED_RPM, ZdtConfig
from .safety import MotorState, RobotPhase, RobotStateMachine, SafetyError, verify_enumeration
from .scan import scan_via_driver
from .zdt_driver import (
    CommunicationError, ZdtDriver, ZdtDriverError,
)

logger = logging.getLogger(__name__)


class ZdtController:
    def __init__(self, config: Optional[ZdtConfig] = None,
                 transport: Optional[CanTransport] = None):
        self.config = config or ZdtConfig()
        self._transport = transport          # None → connect() 构造 SocketCanTransport
        self._driver: Optional[ZdtDriver] = None
        if self._transport is not None:
            # 注入 transport (测试/Fake) → 立即构造 driver, 免 connect
            self._driver = ZdtDriver(self._transport, timeout_s=self.config.timeout_s,
                                     retries=self.config.retries)
        self._connected = False
        self._last_io_s = 0.0                # 看门狗依据
        # 软件位置跟踪 (输出轴角度): 命令积分维护, 不依赖驱动器位置寄存器
        # (本机寄存器刻度与物理输出不成固定比例, 回读标定已证明不可靠;
        #  固件 robot.c 亦用 g_robot.joints.current_angle 命令积分跟踪)
        self._tracked_angles: list[float] = [0.0] * len(self.config.joint_addrs)
        self.robot = RobotStateMachine()
        self._last_scan = None
        self._last_real_q: Optional[list[float]] = None
        self._last_real_ts: Optional[float] = None
        self._vel = [0.0] * len(self.config.joint_addrs)

    # ── 连接生命周期 ─────────────────────────────────────

    def connect(self) -> None:
        if self._transport is None:
            from .can_transport import SocketCanTransport
            self._transport = SocketCanTransport(self.config.channel,
                                                 self.config.bitrate)
        try:
            self._transport.open()
            self._driver = ZdtDriver(self._transport,
                                     timeout_s=self.config.timeout_s,
                                     retries=self.config.retries)
            self.robot = RobotStateMachine()          # 新连接 = 新生命周期
            self.robot.on_connected()
            self._scan_and_verify()                   # 枚举 + 硬不变式
            self.sync()                               # 读 0x36 对齐 tracked
            self.robot.on_safe_idle()
        except Exception:
            # 失败清理: 已使能的轴先急停 (best-effort), 再关总线; _connected 保持 False
            self._connected = False
            if self._driver is not None:
                try:
                    self.e_stop()
                except ZdtDriverError:
                    logger.warning("connect 失败后 e_stop 发送失败", exc_info=True)
            try:
                self._transport.close()
            except Exception:  # noqa: BLE001
                logger.warning("connect 失败后 transport close 失败", exc_info=True)
            raise
        self._connected = True
        self._last_io_s = time.monotonic()
        logger.info("ZDT CAN connected: %s (6 drives verified)", self.config.channel)

    def _scan_and_verify(self) -> None:
        """扫描 + 枚举硬不变式 (修订 #6). 任一违例 → fault 闩锁 + 抛 SafetyError."""
        scan = scan_via_driver(self._driver, timeout_s=self.config.timeout_s,
                               retries=0)
        self._last_scan = scan
        problems = verify_enumeration(scan.found)
        if problems:
            self.robot.fault("枚举失败: " + "; ".join(problems))
            raise SafetyError("枚举失败, 进入 FAULT: " + "; ".join(problems))
        self.robot.on_enumerated(scan.found)
        # 采纳实际发现的寻址方案 (firmware 1..6 / pc 2..7), 后续 IO 用真实地址
        slot_addrs = [None] * 6
        for cid, m in scan.found.items():
            if m.joint_slot is not None:
                slot_addrs[m.joint_slot] = cid
        if any(a is None for a in slot_addrs):
            raise SafetyError("枚举后关节槽地址不完整")   # 理论不可达 (verify 已拦)
        self.config.joint_addrs = list(slot_addrs)

    def sync(self) -> None:
        """从硬件寄存器重新对齐跟踪值 (真实输出角度 = 0x36 pos 经 CALIB (k,b) 换算).

        与 read_real_angles 同一换算 (开机姿态 pos=0 → 真实 ≈0), 供命令积分
        观测 (get_state) 作初始对齐. 未标定轴退化为 pos/减速比.
        """
        for i, addr in enumerate(self.config.joint_addrs):
            kb = self._kb(i)
            pos_raw = self._driver.read_pos(addr)
            self._tracked_angles[i] = ((pos_raw - kb[1]) / kb[0]
                                       if kb is not None and abs(kb[0]) > 1e-9
                                       else pos_raw / self.config.reduction_ratios[i])
        self._last_io_s = time.monotonic()

    def arm(self, gravity_confirmed: bool = False) -> None:
        """SAFE_IDLE → 使能扭矩 → ARMED. 重力关节 J2/J3 需二次确认."""
        self.robot.arm(gravity_confirmed)             # 门禁 + 重力确认 (无 IO)
        try:
            self.set_torque(True)
        except Exception:
            self.robot.disarm()                       # 使能失败回滚到 SAFE_IDLE
            raise

    def disarm(self) -> None:
        try:
            self.set_torque(False)
        finally:
            self.robot.disarm()

    def enter_teleop(self) -> None:
        self.robot.enter_teleop()

    def exit_teleop(self) -> None:
        self.robot.exit_teleop()

    def fault(self, reason: str) -> None:
        self.robot.fault(reason)

    def re_arm(self, confirmed: bool = False) -> None:
        """STOPPED/FAULT → 重枚举验证 → SAFE_IDLE. 需显式确认."""
        scan = scan_via_driver(self._driver, timeout_s=self.config.timeout_s,
                               retries=0)
        problems = verify_enumeration(scan.found)
        if problems:
            self.robot.fault("重枚举失败: " + "; ".join(problems))
            raise SafetyError("重枚举失败: " + "; ".join(problems))
        self.robot.re_arm(confirmed)
        slot_addrs = [None] * 6
        for cid, m in scan.found.items():
            if m.joint_slot is not None:
                slot_addrs[m.joint_slot] = cid
        self.config.joint_addrs = list(slot_addrs)
        self.sync()
        self.robot.on_safe_idle()

    def disconnect(self) -> None:
        if self._transport is not None:
            try:
                self._transport.close()
            except Exception:  # noqa: BLE001 — 关总线失败不应遮蔽主流程
                logger.warning("transport close 失败", exc_info=True)
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    # ── SerialProtocol 兼容接口 ──────────────────────────

    def get_state(self) -> tuple[list[float], list[float], list[float]]:
        """输出轴角度°(软件跟踪值)/速度(占位0)/电流mA(实时读).

        角度不再读驱动器位置寄存器 (本机寄存器刻度与物理输出不成固定比例,
        回读会使观测与命令漂移); 改为由命令积分维护的跟踪值 — 观测/动作
        自洽, 物理精度由 0xFD 脉冲保证, 与固件 robot.c 命令积分一致.
        电流仍实时读 CAN, 用于力监测.

        电流读失败:
          - 一个电流都没读到 (总失败) → 抛 CommunicationError (loud fail);
          - 部分读到 → logger.warning 并返回 (跟踪角度, 零速, 空电流)
            (调用方按空电流列表降级, 角度仍可用).
        """
        angles = list(self._tracked_angles)
        loads: list[float] = []
        try:
            # 增量 append: 中途抛异常时 loads 保留已读部分 → 判定"部分读取"
            for addr in self.config.joint_addrs:
                loads.append(self._driver.read_current(addr))
        except ZdtDriverError as exc:
            if not loads:
                raise CommunicationError("total CAN current read failure") from exc
            logger.warning("partial current read failure (got %d/6); "
                           "returning tracked angles with empty loads",
                           len(loads), exc_info=True)
            return angles, [0.0] * len(angles), []
        self._last_io_s = time.monotonic()
        return angles, [0.0] * len(angles), loads

    def get_real_state(self) -> dict:
        """0x36 真实观测 (spec §5.3): {q, velocity, current, flags, status}.

        q = 真实输出角 (anchor, use_kb=True); velocity = 低通有限差分 dq
        (vel_filter_alpha, 驱动器 0x35 实时速度留待后续); current/flags 逐轴.
        """
        q = self.read_real_angles(use_kb=True)
        now = time.monotonic()
        dq = [0.0] * len(q)
        if self._last_real_q is not None and self._last_real_ts is not None:
            dt = max(1e-3, now - self._last_real_ts)
            raw = [(q[i] - self._last_real_q[i]) / dt for i in range(len(q))]
            alpha = self.config.vel_filter_alpha
            self._vel = [alpha * raw[i] + (1.0 - alpha) * self._vel[i]
                         for i in range(len(q))]
            dq = list(self._vel)
        else:
            self._vel = [0.0] * len(q)
        self._last_real_q = list(q)
        self._last_real_ts = now
        currents = [self._driver.read_current(addr) for addr in self.config.joint_addrs]
        flags = [self._driver.read_flag(addr) for addr in self.config.joint_addrs]
        return {"q": list(q), "velocity": dq, "current": currents,
                "flags": flags, "status": self.robot.phase.name}

    def set_joints(self, angles: list[float]) -> None:
        """目标输出角度 → clamp → 相对跟踪值最短路径 → 0xFD 运动.

        用软件跟踪值 (命令积分) 而非寄存器回读计算增量, 避免观测/命令漂移.
        支持 joint_addrs 少于 6 的配置 (单电机调试).
        """
        n = len(self.config.joint_addrs)
        if len(angles) < n:
            raise ValueError(f"需 {n} 个角度, 实际 {len(angles)}")
        targets = [
            max(self.config.limits[i][0], min(self.config.limits[i][1],
                                              float(angles[i])))
            for i in range(n)
        ]
        for i in range(n):
            self._drive_rel(i, self._shortest_delta(targets[i] - self._tracked_angles[i]))
            self._tracked_angles[i] = targets[i]
        self._last_io_s = time.monotonic()

    def set_torque(self, enable: bool) -> None:
        for addr in self.config.joint_addrs:
            self._driver.enable(addr, enable)
        self._last_io_s = time.monotonic()

    def e_stop(self) -> None:
        self._driver.stop_all()
        self.robot.e_stop()                            # 闩锁 STOPPED
        self._last_io_s = time.monotonic()
        logger.warning("EMERGENCY STOP broadcast")

    def zero(self) -> None:
        """逐轴设当前为机械零位 (0x93 88 01 存储), 并重置跟踪起点为 0.

        调用前把输出轴摆到期望机械零位 — 这是软件跟踪的正式对齐步骤.
        """
        for addr in self.config.joint_addrs:
            self._driver.set_zero(addr)
        self._tracked_angles = [0.0] * len(self.config.joint_addrs)
        self._last_io_s = time.monotonic()

    # ── A 任务: 0x0A 6D 标定路线 (方案 b: 人工摆姿态 → 清零 → 偏置) ──

    def reset_position(self, addr: int) -> None:
        """单轴清零当前位置 (0x0A 6D): 让 0x36 立刻读出 0.

        转发到 driver.reset_position. 不改 _tracked_angles (清零是驱动器寄存器层,
        软件跟踪值由 anchor_pose 统一更新).
        """
        self._driver.reset_position(addr)
        self._last_io_s = time.monotonic()

    def anchor_pose(self, expected_angles: list[float],
                    calib_offsets: Optional[list] = None,
                    clear_position: bool = True) -> list[float]:
        """把当前机械位置标定为 expected_angles, 返回每轴 offset.

        A 任务方案 b 主流程 (0x0A 6D 路线):
          1. 人工把机械臂摆到已知姿态 (如 JOINT_INIT_ANGLE_DEG)
          2. 对每轴: 读 0x36 → 换算输出角度 → offset = 输出角度 - expected
          3. (可选) 发 0x0A 6D 清零驱动器寄存器
          4. 更新 _tracked_angles = expected (软件跟踪对齐)
          5. 返回 offsets, 调用方写入 config.CALIB_OFFSETS

        参数:
          expected_angles: 当前姿态的期望输出角度 (6 个)
          calib_offsets: 已有 offset 表 (用于修正读数); None=不做 offset 修正 (首次标定)
          clear_position: 是否发 0x0A 6D 清零 (True=标定流程, False=只读不写)

        返回: 每轴 offset (输出角度 - expected). 调用方应存入 CALIB_OFFSETS.
        """
        n = len(self.config.joint_addrs)
        if len(expected_angles) < n:
            raise ValueError(f"需 {n} 个期望角度, 实际 {len(expected_angles)}")
        offsets = [0.0] * n
        for i, addr in enumerate(self.config.joint_addrs):
            pos_raw = self._driver.read_pos(addr)              # 电机轴度数 (带符号)
            out_deg = pos_raw / self.config.reduction_ratios[i]  # 输出轴度数
            if calib_offsets is not None and calib_offsets[i] is not None:
                out_deg -= calib_offsets[i]                    # 减去已有偏置
            offsets[i] = out_deg - expected_angles[i]
            if clear_position:
                self._driver.reset_position(addr)
            self._tracked_angles[i] = expected_angles[i]
        self._last_io_s = time.monotonic()
        logger.info("anchor_pose → expected=%s offsets=%s clear=%s",
                    expected_angles, offsets, clear_position)
        return offsets

    def read_real_angles(self, calib_offsets: Optional[list] = None,
                         use_kb: bool = False,
                         calib_kb: Optional[list] = None) -> list[float]:
        """从 0x36 读带符号真实位置 → 换算输出轴角度 (B 任务限位基准).

        0x36 返回电机轴带符号位置 (说明书 §7.4.4 + 固件 robot.c:1045 确认).
        两种标定路线 (互斥):
          use_kb=False: 用 CALIB_OFFSETS 简单偏置 → 真实 = (0x36/减速比) - offset
          use_kb=True:  用 CALIB (k,b) 精确换算 → 真实 = (0x36 - b) / k

        未标定时 (offset=None / kb=None) 退化为纯减速比换算 (无偏置修正).
        """
        n = len(self.config.joint_addrs)
        angles = [0.0] * n
        for i, addr in enumerate(self.config.joint_addrs):
            pos_raw = self._driver.read_pos(addr)
            if use_kb:
                kb = (calib_kb[i] if calib_kb is not None else None) or self._kb(i)
                if kb is not None:
                    k, b = kb
                    angles[i] = (pos_raw - b) / k if abs(k) > 1e-9 else 0.0
                else:
                    angles[i] = pos_raw / self.config.reduction_ratios[i]
            else:
                off = calib_offsets[i] if calib_offsets is not None else None
                out_deg = pos_raw / self.config.reduction_ratios[i]
                angles[i] = out_deg - (off if off is not None else 0.0)
        self._last_io_s = time.monotonic()
        return angles

    # ── B 任务: 基于 0x36 真实位置的软限位 + 漂移守卫 ──

    def set_joints_safe(self, angles: list[float],
                        calib_offsets: Optional[list] = None,
                        use_kb: bool = False,
                        calib_kb: Optional[list] = None,
                        real_angles: Optional[list[float]] = None) -> list[float]:
        """B 任务: 用 0x36 真实位置做软限位 (替代 set_joints 的命令积分限位).

        流程: 读 0x36 真实位置 (或复用传入 real_angles) → clamp 到限位 → 相对真实位置最短路径 → 0xFD
              → 同步更新 _tracked_angles (命令积分与真实位置对齐).

        与 set_joints 区别: 限位基准从 tracked_deg 换成 0x36 真实位置,
        外力搬动/失步后仍准确. 代价: 若未传入 real_angles 则每次多 6 次 0x36 读 (CAN 往返).

        返回: 实际下发的目标角度 (clamp 后). 调用方可对比入参判断是否被限位.
        """
        n = len(self.config.joint_addrs)
        if len(angles) < n:
            raise ValueError(f"需 {n} 个角度, 实际 {len(angles)}")
        if real_angles is None:
            real_angles = self.read_real_angles(calib_offsets, use_kb, calib_kb)
        targets = [0.0] * n
        for i in range(n):
            lo, hi = self.config.limits[i]
            targets[i] = max(lo, min(hi, float(angles[i])))
            delta = self._shortest_delta(targets[i] - real_angles[i])
            self._drive_rel(i, delta)
            # 同步跟踪值到真实位置 + delta (消除漂移)
            self._tracked_angles[i] = real_angles[i] + delta
        self._last_io_s = time.monotonic()
        return targets

    def check_drift(self, calib_offsets: Optional[list] = None,
                    use_kb: bool = False,
                    calib_kb: Optional[list] = None,
                    threshold_deg: float = 2.0) -> list[tuple[bool, float]]:
        """B 任务: 命令积分 vs 0x36 真实位置 漂移守卫.

        返回每轴 (ok, drift). ok=False 时调用方应告警/重锚/急停.
        重力关节 (J2/J3) 建议用更严的 threshold (如 1°).
        """
        real_angles = self.read_real_angles(calib_offsets, use_kb, calib_kb)
        results = []
        for i in range(len(self.config.joint_addrs)):
            drift = abs(self._tracked_angles[i] - real_angles[i])
            results.append((drift < threshold_deg, drift))
        self._last_io_s = time.monotonic()
        return results

    # ── C 任务: 基于 0x36 真实位置的实时限位守卫 (IK 遥操监控) ──

    def check_limits_real(self, targets: list[float],
                          calib_offsets: Optional[list] = None,
                          use_kb: bool = False,
                          calib_kb: Optional[list] = None,
                          high_freq: frozenset = frozenset({1, 2, 3, 4}),
                          real_angles: Optional[list[float]] = None) -> list[dict]:
        """实时限位守卫: 目标/真实角度越出限位边界 → 单轴 stop + 告警.

        用于 IK 笛卡尔遥操数据采集的控制循环, 每帧调用. 目的: 防关节摆动过大
        导致力矩过载损坏电机.

        基准 = 0x36 真实位置 (经 (k,b) 或 offset 换算输出轴角度), 不依赖命令积分
        (tracked_deg 会漂移; IK 遥操失步/外力搬动后仍可信). 判定以指令目标为准:
          - 目标 target 越出 [lo, hi]        → 往外走 / 卡在界外 → 停 + 告警
          - 目标在界内 (即使真实角度已越界)  → 往回走 → 允许 (不卡死回退)
        真实角度 real 越界但目标在界内 = 正在回退, 仅提示不停止.

        high_freq: 实时检查的关节槽集合 (默认 {1,2,3,4} = J2/J3/J4/J5).
        J4 腕滚有界 [-90,90] 须实时检查; 仅 360° 旋转关节 (J1/J6) 不检查.
        停: 对该轴发单轴 stop (保持力矩可回退), 不停全局 e_stop.

        返回: 告警项列表 [{"slot","real","target","limit"}], 空 = 全安全.
        """
        n = len(self.config.joint_addrs)
        if len(targets) < n:
            raise ValueError(f"需 {n} 个角度, 实际 {len(targets)}")
        if real_angles is None:
            real_angles = self.read_real_angles(calib_offsets, use_kb, calib_kb)
        alarms = []
        for i in range(n):
            if i not in high_freq:
                continue  # 360° 旋转关节不实时检查
            lo, hi = self.config.limits[i]
            tgt = float(targets[i])
            real = real_angles[i]
            if tgt < lo or tgt > hi:
                # 目标越界 → 往外走 → 停 + 告警
                self._stop_axis(i)
                alarms.append({"slot": i, "real": round(real, 2),
                               "target": round(tgt, 2), "limit": (lo, hi)})
                logger.warning("限位守卫: J%d 目标越界 target=%.2f limit=%s real=%.2f",
                               i + 1, tgt, (lo, hi), real)
            elif real < lo or real > hi:
                # 真实角度越界但目标界内 (正在回退) → 仅提示, 不停止
                alarms.append({"slot": i, "real": round(real, 2),
                               "target": round(tgt, 2), "limit": (lo, hi)})
                logger.info("限位守卫: J%d 真实角度越界 real=%.2f, 目标界内回退中",
                            i + 1, real)
        self._last_io_s = time.monotonic()
        return alarms

    def _stop_axis(self, joint_idx: int) -> None:
        """单轴停止 (保持力矩可回退), 不触发全局 e_stop."""
        self._driver.stop(self.config.joint_addrs[joint_idx])

    # ── ZDT 扩展 ─────────────────────────────────────────

    def rel_rotate(self, joint_id: int, delta_deg: float) -> None:
        """关节相对旋转 (输出轴角度). joint_id: 1-based (1=关节1), 合法范围 1-6."""
        if not 1 <= joint_id <= 6:
            raise ValueError(f"joint_id 需 1-6, got {joint_id}")
        idx = joint_id - 1
        self._drive_rel(idx, delta_deg)
        self._tracked_angles[idx] = self._normalize_angle(
            self._tracked_angles[idx] + delta_deg)
        self._last_io_s = time.monotonic()

    def soft_reset(self) -> None:
        """回到固件初始位姿 (JOINT_INIT_ANGLE_DEG, 输出角度).

        相对当前跟踪值最短路径运动, 不 clamp — INIT_POSE 为固件定义复位位.
        """
        n = len(self.config.joint_addrs)
        for i in range(n):
            self._drive_rel(i, self._shortest_delta(JOINT_INIT_ANGLE_DEG[i] - self._tracked_angles[i]))
            self._tracked_angles[i] = JOINT_INIT_ANGLE_DEG[i]
        self._last_io_s = time.monotonic()
        logger.info("soft_reset → %s", JOINT_INIT_ANGLE_DEG)

    def _move_pose_safe(self, pose: list[float], speed_rpm: float, label: str,
                        calib_offsets: Optional[list] = None,
                        use_kb: bool = True,
                        calib_kb: Optional[list] = None) -> list[float]:
        """安全同步运动至目标姿态 — ready()/home() 共用.

        流程 (与 set_joints_safe 同基准):
          1. 读 0x36 真实位置 → 每轴 (pos-b)/k 换真实角
          2. 目标 clamp 到限位 → 相对真实位置最短路径 → 脉冲
          3. 每轴发 0xFD(raF=相对, snF=1) 只设不转, speed=speed_rpm (仅本条生效)
          4. multi_sync() 广播 → 6 轴同时启动
          5. 已到位轴 (delta=0) 不发; 全部到位则不广播

        Returns:
            实际目标角度 (clamp 后; 与入参 pose 对比可判断是否被限位).
        """
        n = len(self.config.joint_addrs)
        real_angles = self.read_real_angles(calib_offsets, use_kb, calib_kb)
        targets = [0.0] * n
        pending = 0
        for i in range(n):
            lo, hi = self.config.limits[i]
            targets[i] = max(lo, min(hi, float(pose[i])))
            delta = self._shortest_delta(targets[i] - real_angles[i])
            if abs(delta) < 1e-9:
                continue                                   # 已到位, 不发
            if not self._drive_rel(i, delta, snF=True, speed_rpm=speed_rpm):
                continue                                   # 凑整 0 脉冲, 不算运动
            pending += 1
            self._tracked_angles[i] = real_angles[i] + delta
        if pending:
            self._driver.multi_sync()
            logger.info("%s → %s (%d 轴同步, %.1f RPM)", label, targets,
                        pending, speed_rpm)
        else:
            logger.info("%s: 已在目标姿态 %s", label, targets)
        self._last_io_s = time.monotonic()
        return targets

    def ready(self, calib_offsets: Optional[list] = None,
              use_kb: bool = True,
              calib_kb: Optional[list] = None) -> list[float]:
        """安全运动至按摩准备姿态 (READY_POSE_DEG), 6 关节同步安全速度."""
        return self._move_pose_safe(READY_POSE_DEG, READY_SPEED_RPM, "ready",
                                    calib_offsets, use_kb, calib_kb)

    def home(self, calib_offsets: Optional[list] = None,
             use_kb: bool = True,
             calib_kb: Optional[list] = None) -> list[float]:
        """安全运动回上电初始姿态 (JOINT_INIT_ANGLE_DEG 全 0), 6 关节同步安全速度.

        注意与 driver.home (0x9A 单圈回零) 不同: 本方法用 0xFD 相对运动回上电姿态.
        """
        return self._move_pose_safe(JOINT_INIT_ANGLE_DEG, READY_SPEED_RPM, "home",
                                    calib_offsets, use_kb, calib_kb)

    # ── 角度/脉冲换算 (输出轴角度 ↔ 0xFD 脉冲) ────────────

    def _kb(self, joint_idx: int) -> Optional[tuple[float, float]]:
        """返回某关节的 CALIB (k, b), 未标定 (config.calib 无此槽/None) 时 None."""
        calib = getattr(self.config, "calib", None)
        if calib is None or joint_idx >= len(calib):
            return None
        return calib[joint_idx]

    @staticmethod
    def _shortest_delta(delta: float) -> float:
        """归一到 [-180, 180) 的最短旋转方向 (0/360 连续关节绕圈)."""
        return ((delta + 180.0) % 360.0) - 180.0

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """归一到 [0, 360)."""
        return angle % 360.0

    def _drive_rel(self, joint_idx: int, delta_deg: float,
                   snF: bool = False,
                   speed_rpm: Optional[float] = None) -> bool:
        """按输出轴角度差发 0xFD 相对脉冲命令 (delta=0 不发).

        脉冲 = |Δ输出角度| × reduction_ratios[joint_idx] × pulses_per_rev / 360.
        snF=True: 多机同步标志 (收到命令不立即转, 等 multi_sync 广播).
        speed_rpm: 覆盖本次命令速度 (None=用 config.speed_rpm); 仅本条生效.

        返回: 是否实际发送了脉冲 (delta 凑整为 0 脉冲 → False, 调用方勿计运动).
        """
        ratio = self.config.reduction_ratios[joint_idx]
        n = int(round(abs(delta_deg) * ratio * self.config.pulses_per_rev / 360))
        if n == 0:
            return False
        spd = self.config.speed_rpm if speed_rpm is None else speed_rpm
        self._driver.move_pulse(
            self.config.joint_addrs[joint_idx], n, dir_cw=(delta_deg >= 0),
            speed_rpm=spd, acc=self.config.position_acc, snF=snF)
        return True

    # ── 安全: 看门狗 + 力控 (调用方循环 tick) ─────────────

    def tick(self) -> None:
        """看门狗: >watchdog_s 无成功 IO → e_stop. 力控阈值在后续任务接入."""
        if self._connected and time.monotonic() - self._last_io_s > self.config.watchdog_s:
            logger.error("watchdog: no CAN IO for %.1fs → e_stop", self.config.watchdog_s)
            try:
                self.e_stop()
            except ZdtDriverError:
                # TransportError 是 ZdtDriverError 子类 → 总线死时也被捕住, 留痕
                logger.exception("watchdog e_stop 发送失败")
