"""ZdtController 单测 (ZdtDriver over FakeTransport, 注入回帧)."""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from lerobot_robot_massage.zdt.config import CHECKSUM, F_READ_CUR, F_READ_POS, ZdtConfig
from lerobot_robot_massage.zdt.controller import ZdtController
from lerobot_robot_massage.zdt.fakes import (
    FailingRecvAfterNTransport, FailingRecvTransport, FailingSendTransport,
    FakeTransport,
)
from lerobot_robot_massage.zdt.testutil import run_all
from lerobot_robot_massage.zdt.safety import (
    MotorState, RobotPhase, SafetyError,
)
from lerobot_robot_massage.zdt.zdt_driver import (
    CommunicationError, ZdtDriverError,
)


def _mk(cfg=None):
    t = FakeTransport()
    # 默认 1:1 减速比 + 1:1 标定 (k=1,b=0), 让测试里的"度数 = 电机轴读数"直观可算
    cfg = cfg or ZdtConfig(timeout_s=0.001, retries=0,
                           reduction_ratios=[1.0] * 6,
                           calib=[(1.0, 0.0)] * 6)
    ctrl = ZdtController(config=cfg, transport=t)
    return ctrl, t


def _inject_all_currents(t: FakeTransport):
    """注入 6 轴电流回帧: 50..55 mA."""
    for i, addr in enumerate([0x02, 0x03, 0x04, 0x05, 0x06, 0x07]):
        t.inject(addr, F_READ_CUR, b"\x00" + bytes([50 + i]) + b"\x6b")


def test_get_state_returns_tracked_angles_and_currents():
    ctrl, t = _mk()
    # 角度来自软件跟踪值 (命令积分), 不读寄存器
    ctrl._tracked_angles = [100.0 + i / 10 for i in range(6)]
    _inject_all_currents(t)
    angles, vels, loads = ctrl.get_state()
    assert len(angles) == 6
    assert abs(angles[0] - 100.0) < 0.01
    assert abs(angles[5] - 100.5) < 0.01
    assert abs(loads[5] - 55.0) < 0.01


def test_get_state_raises_on_total_current_failure():
    ctrl, t = _mk()   # 无注入 → read_current 超时, 一个电流都没读到
    try:
        ctrl.get_state()
        raise AssertionError("应抛 CommunicationError")
    except CommunicationError:
        pass
    except ZdtDriverError as exc:
        raise AssertionError(f"抛了非 CommunicationError: {type(exc).__name__}")
    # 异常应可被 ZdtDriverError 边界捕住 (非裸 RuntimeError)
    try:
        ctrl.get_state()
        raise AssertionError("应抛 ZdtDriverError")
    except ZdtDriverError:
        pass


def test_get_state_partial_current_failure_keeps_angles():
    # 角度始终可用 (跟踪值), 电流部分读到 → 返回 (跟踪角度, 零速, 空电流)
    ctrl, t = _mk()
    ctrl._tracked_angles = [10.0] * 6
    t.inject(0x02, F_READ_CUR, b"\x00\x7b\x6b")   # 只注入 1 个电流, 第 2 个超时
    angles, vels, loads = ctrl.get_state()
    assert angles == [10.0] * 6
    assert loads == []


def test_set_joints_clamps_and_drives_rel():
    ctrl, t = _mk()
    # 跟踪值从 0° 起 (软件跟踪, 无需注入当前角度)
    # J5 目标 720 越界 → clamp 180 (新真机实测限位); J6 目标 0 = 当前 0 → 不发
    ctrl.set_joints([90.0, 45.0, 90.0, 90.0, 720.0, 0.0])
    moves = [f for f in t.sent if f.data and f.data[0] == 0xFD]
    assert len(moves) == 10   # 5 关节 × 2 帧
    j1 = [f for f in moves if f.arbitration_id >> 8 == 0x02]
    j5 = [f for f in moves if f.arbitration_id >> 8 == 0x06]
    assert j1 and j1[0].data[1] == 0x00            # dir CW (正方向)
    # J1: 90° 目标, 跟踪 0 → Δ90 → 90×1×3200/360 = 800 = 0x320
    assert j1[0].data[5:8] == bytes([0x00, 0x00, 0x03])
    assert j1[1].data[1] == 0x20
    # J5: clamp 后的 180° 是运动目标 (证明 clamp 生效); 180 最短路径归一向负侧
    assert j5 and j5[0].data[1] == 0x01
    # 跟踪值更新为 clamp 后的目标 (J5 720° → 180°; J2 45° 在新下限 0° 内合法不 clamp)
    assert ctrl._tracked_angles == [90.0, 45.0, 90.0, 90.0, 180.0, 0.0]


def test_set_torque_sends_six_enables():
    ctrl, t = _mk()
    ctrl.set_torque(True)
    assert len(t.sent) == 6
    assert t.sent[0].data[:3] == bytes([0xF3, 0xAB, 0x01])


def test_e_stop_broadcasts():
    ctrl, t = _mk()
    ctrl.e_stop()
    assert t.sent[-1].arbitration_id == 0x0000
    assert t.sent[-1].data == bytes([0xFE, 0x98, 0x00, CHECKSUM])


def test_soft_reset_sends_init_pose():
    ctrl, t = _mk()
    # 开机姿态即全零期望位 JOINT_INIT_ANGLE_DEG = [0,0,0,0,0,0]:
    # tracked 已到位 (全 0) → 不运动.
    ctrl.soft_reset()
    moves = [f for f in t.sent if f.data and f.data[0] == 0xFD]
    assert len(moves) == 0
    assert ctrl._tracked_angles == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def test_soft_reset_drives_back_to_zero():
    # 重置非零跟踪点 → soft_reset 相对最短路径回全零开机姿态.
    ctrl, t = _mk()
    ctrl._tracked_angles = [90.0, 45.0, -30.0, 0.0, 90.0, 180.0]
    ctrl.soft_reset()
    moves = [f for f in t.sent if f.data and f.data[0] == 0xFD]
    assert len(moves) == 10   # 5 个运动关节 × 2 帧 (J4 delta=0 不发)
    # J1: delta=-90 → dir CCW (J1 postive_direction=1)
    j1 = [f for f in moves if f.arbitration_id >> 8 == 0x02]
    assert j1 and j1[0].data[1] == 0x01
    # J2: delta=-45 → dir CCW; 45°×1×3200/360=400=0x190
    j2 = [f for f in moves if f.arbitration_id >> 8 == 0x03]
    assert j2 and j2[0].data[1] == 0x01
    assert j2[0].data[5:8] == bytes([0x00, 0x00, 0x01])
    assert j2[1].data[1] == 0x90
    # J4 (wrist_roll) 已在 0 → 不发; 其余回归 0
    assert not [f for f in moves if f.arbitration_id >> 8 == 0x05]
    assert ctrl._tracked_angles == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def test_rel_rotate_one_joint():
    ctrl, t = _mk()
    ctrl.rel_rotate(1, 5.0)   # joint_id 1-based, 输出轴 +5°
    assert t.sent[0].arbitration_id == 0x0200
    assert t.sent[0].data[0] == 0xFD
    assert t.sent[0].data[1] == 0x00      # dir CW (正方向)
    # 5°×1×3200/360 = 44 = 0x2C → 首帧脉冲高3B 全 0, 尾帧第4B=0x2C
    assert t.sent[0].data[5:8] == bytes([0x00, 0x00, 0x00])
    assert t.sent[1].data[1] == 0x2C
    # 跟踪值更新
    assert ctrl._tracked_angles[0] == 5.0


def test_rel_rotate_negative_updates_tracked():
    ctrl, t = _mk()
    ctrl.rel_rotate(1, -5.0)
    assert ctrl._tracked_angles[0] == 355.0   # 归一化到 [0,360)
    # 反向: dir 位 = 1 (CCW)
    mv = [f for f in t.sent if f.data and f.data[0] == 0xFD]
    assert mv[0].data[1] == 0x01


def test_rel_rotate_invalid_joint_rejected():
    # joint_id 越界 (0/-1/7) 必须显式报错, 不能静默寻到 J5/J6
    for bad in (0, -1, 7, 99):
        ctrl, t = _mk()
        try:
            ctrl.rel_rotate(bad, 5.0)
            raise AssertionError(f"joint_id={bad} 应抛 ValueError")
        except ValueError:
            pass
        assert t.sent == []   # 未发送任何帧


def test_get_state_partial_current_transport_death_keeps_angles():
    # 读电流时总线死 (部分读到) → 返回跟踪角度, 不泄漏, 电流降级为空
    t = FailingRecvAfterNTransport(fail_on_recv_n=3)
    t.inject(0x02, F_READ_CUR, b"\x00\x7b\x6b")   # 第 1 个电流读到, 第 2 个总线死
    ctrl = ZdtController(config=ZdtConfig(timeout_s=0.001, retries=0), transport=t)
    ctrl._tracked_angles = [20.0] * 6
    angles, vels, loads = ctrl.get_state()
    assert angles == [20.0] * 6
    assert loads == []


def test_get_state_total_current_transport_death_raises_communication_error():
    # 总线死且一个电流都没读到 → CommunicationError (ZdtDriverError), 非裸 CanTransportError
    t = FailingRecvTransport()
    ctrl = ZdtController(config=ZdtConfig(timeout_s=0.001, retries=0), transport=t)
    try:
        ctrl.get_state()
        raise AssertionError("应抛 CommunicationError")
    except CommunicationError:
        pass
    except ZdtDriverError as exc:
        raise AssertionError(f"抛了非 CommunicationError: {type(exc).__name__}: {exc}")


def test_tick_swallows_transport_error():
    # 总线死时 tick 的 e_stop 发送失败 (TransportError) 必须被吞掉留痕, 不传播
    t = FailingSendTransport()
    ctrl = ZdtController(config=ZdtConfig(timeout_s=0.001, retries=0, watchdog_s=0.05),
                         transport=t)
    ctrl._connected = True
    ctrl._last_io_s = time.monotonic() - 1.0   # 陈旧 → 触发 e_stop 路径
    ctrl.tick()   # 不应抛任何异常


def test_connect_failure_estop_and_close():
    # 第 4 次 read_pos 的 recv 抛 CanTransportError → connect 失败:
    #   已使能轴先 e_stop 广播 (addr=0x00) + transport 已 close + _connected=False
    t = FailingRecvAfterNTransport(fail_on_recv_n=4)
    for i, addr in enumerate([0x02, 0x03, 0x04]):
        v = 1000 + i
        t.inject(addr, F_READ_POS,
                 b"\x01" + bytes([v >> 16, (v >> 8) & 0xFF, v & 0xFF]) + b"\x6b")
    ctrl = ZdtController(config=ZdtConfig(timeout_s=0.001, retries=0), transport=t)
    try:
        ctrl.connect()
        raise AssertionError("connect 应抛错")
    except (ZdtDriverError, SafetyError):
        pass
    # e_stop 广播帧 (addr=0x00, 停止命令)
    assert t.sent[-1].arbitration_id == 0x0000
    assert t.sent[-1].data == bytes([0xFE, 0x98, 0x00, CHECKSUM])
    assert t.closed is True
    assert ctrl._connected is False


def test_tick_triggers_estop_when_stale():
    cfg = ZdtConfig(timeout_s=0.001, retries=0, watchdog_s=0.05)
    ctrl, t = _mk(cfg)
    ctrl._connected = True
    ctrl._last_io_s = time.monotonic() - 1.0   # 陈旧
    ctrl.tick()
    assert t.sent and t.sent[-1].arbitration_id == 0x0000


def test_tick_noop_when_fresh():
    cfg = ZdtConfig(timeout_s=0.001, retries=0, watchdog_s=0.05)
    ctrl, t = _mk(cfg)
    ctrl._connected = True
    ctrl._last_io_s = time.monotonic()
    ctrl.tick()
    assert t.sent == []


# ── A 任务: anchor_pose / reset_position / read_real_angles ──────────────

def _inject_pos_frame(t, addr: int, deg: float):
    """注入一帧 0x36 回帧: 电机轴角度 deg (Emm42 V5.0 4字节×360/65536 格式)."""
    v = int(round(abs(deg) * 65536.0 / 360.0)) & 0xFFFFFFFF
    sign_byte = 0x01 if deg < 0 else 0x00
    t.inject(addr, F_READ_POS,
             bytes([sign_byte,
                    (v >> 24) & 0xFF, (v >> 16) & 0xFF,
                    (v >> 8) & 0xFF, v & 0xFF])
             + b"\x6b")


def test_reset_position_sends_0x0A_6D():
    """reset_position 发 0x0A 6D 清零命令 (A 任务核心命令)."""
    ctrl, t = _mk()
    ctrl.reset_position(0x03)
    # 帧 ID = (0x03 << 8) | 0x00, data = [0x0A, 0x6D, 0x6B]
    assert t.sent[-1].arbitration_id == (0x03 << 8)
    assert t.sent[-1].data == bytes([0x0A, 0x6D, 0x6B])


def test_anchor_pose_returns_offsets_and_clears():
    """anchor_pose: 读 0x36 → 算 offset → 发 0x0A 6D 清零 → 更新跟踪.

    场景: 减速比 1:1, 人工摆到 [90,90,-90,0,90,0], 但 0x36 读出 [95,90,-85,5,90,2]
         → offsets 应为 [5,0,5,5,0,2], 清零命令发 6 次, 跟踪值=expected.
    """
    ctrl, t = _mk()
    expected = [90.0, 90.0, -90.0, 0.0, 90.0, 0.0]
    actual_pos = [95.0, 90.0, -85.0, 5.0, 90.0, 2.0]
    for addr, deg in zip([0x02, 0x03, 0x04, 0x05, 0x06, 0x07], actual_pos):
        _inject_pos_frame(t, addr, deg)
    offsets = ctrl.anchor_pose(expected, clear_position=True)
    assert all(abs(o - e) < 0.01 for o, e in zip(offsets, [5.0, 0.0, 5.0, 5.0, 0.0, 2.0]))
    # 6 个 0x0A 6D 清零命令
    clear_frames = [f for f in t.sent if f.data[:2] == bytes([0x0A, 0x6D])]
    assert len(clear_frames) == 6
    # 跟踪值对齐到 expected
    assert ctrl._tracked_angles == expected


def test_anchor_pose_no_clear_skips_0x0A():
    """clear_position=False: 只读不写, 不发 0x0A 6D."""
    ctrl, t = _mk()
    expected = [0.0] * 6
    for addr in [0x02, 0x03, 0x04, 0x05, 0x06, 0x07]:
        _inject_pos_frame(t, addr, 10.0)
    offsets = ctrl.anchor_pose(expected, clear_position=False)
    assert all(abs(o - 10.0) < 0.01 for o in offsets)
    clear_frames = [f for f in t.sent if f.data[:2] == bytes([0x0A, 0x6D])]
    assert len(clear_frames) == 0


def test_anchor_pose_with_reduction_ratio():
    """减速比 50:1: 0x36 读 500° (电机轴) → 输出 10° → offset=10-0=10."""
    cfg = ZdtConfig(timeout_s=0.001, retries=0,
                    reduction_ratios=[50.0] * 6)
    t = FakeTransport()
    ctrl = ZdtController(config=cfg, transport=t)
    for addr in [0x02, 0x03, 0x04, 0x05, 0x06, 0x07]:
        _inject_pos_frame(t, addr, 500.0)   # 电机轴 500°
    offsets = ctrl.anchor_pose([0.0] * 6, clear_position=False)
    assert all(abs(o - 10.0) < 0.001 for o in offsets)


def test_anchor_pose_applies_existing_offset():
    """已有 offset 修正: 0x36 读 95, 已有 offset=5 → 真实 90 → 新 offset=90-90=0."""
    ctrl, t = _mk()
    # 6 轴: J1 测 offset 修正, 其余读 0 + offset 0 → offset 0
    for addr in [0x02, 0x03, 0x04, 0x05, 0x06, 0x07]:
        _inject_pos_frame(t, addr, 95.0 if addr == 0x02 else 0.0)
    offsets = ctrl.anchor_pose(
        [90.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        calib_offsets=[5.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        clear_position=False)
    assert abs(offsets[0]) < 0.01


def test_read_real_angles_with_offset():
    """read_real_angles: 0x36 读 100, offset=10 → 真实 90."""
    ctrl, t = _mk()
    for addr in [0x02, 0x03, 0x04, 0x05, 0x06, 0x07]:
        _inject_pos_frame(t, addr, 100.0)
    angles = ctrl.read_real_angles(calib_offsets=[10.0] * 6)
    assert all(abs(a - 90.0) < 0.01 for a in angles)


def test_read_real_angles_with_kb():
    """read_real_angles use_kb: 0x36 读 200, (k=2,b=20) → (200-20)/2=90."""
    ctrl, t = _mk()
    for addr in [0x02, 0x03, 0x04, 0x05, 0x06, 0x07]:
        _inject_pos_frame(t, addr, 200.0)
    angles = ctrl.read_real_angles(use_kb=True, calib_kb=[(2.0, 20.0)] * 6)
    assert all(abs(a - 90.0) < 0.01 for a in angles)


def test_read_real_angles_uncalibrated_fallback():
    """未标定时退化为纯减速比换算 (无偏置)."""
    ctrl, t = _mk()
    for addr in [0x02, 0x03, 0x04, 0x05, 0x06, 0x07]:
        _inject_pos_frame(t, addr, 50.0)
    angles = ctrl.read_real_angles()   # calib_offsets=None
    assert all(abs(a - 50.0) < 0.01 for a in angles)


def test_read_real_angles_negative_position():
    """0x36 负位置 (符号字节 0x01) 正确解析为负角度 (B 任务限位关键)."""
    ctrl, t = _mk()
    for addr in [0x02, 0x03, 0x04, 0x05, 0x06, 0x07]:
        _inject_pos_frame(t, addr, -45.0)
    angles = ctrl.read_real_angles()
    assert all(abs(a - (-45.0)) < 0.001 for a in angles)


# ── B 任务: set_joints_safe / check_drift ────────────────────────────────

def test_set_joints_safe_uses_real_position():
    """set_joints_safe: 0x36 读 50, 目标 90 → delta=40, 跟踪同步到 90."""
    ctrl, t = _mk()
    for addr in [0x02, 0x03, 0x04, 0x05, 0x06, 0x07]:
        _inject_pos_frame(t, addr, 50.0)
    targets = ctrl.set_joints_safe([90.0] * 6)
    assert all(abs(tgt - 90.0) < 0.001 for tgt in targets)
    # 跟踪值同步到真实位置 + delta = 50 + 40 = 90
    assert all(abs(tr - 90.0) < 0.001 for tr in ctrl._tracked_angles)


def test_set_joints_safe_clamps_to_limits():
    """set_joints_safe: 限位 J1(0,360) 目标 400 → clamp 到 360."""
    from lerobot_robot_massage.zdt.config import FIRMWARE_JOINT_LIMITS
    ctrl, t = _mk()
    for addr in [0x02, 0x03, 0x04, 0x05, 0x06, 0x07]:
        _inject_pos_frame(t, addr, 0.0)
    # 每轴目标都超过 hi, 期望 clamp 到 hi
    targets = ctrl.set_joints_safe([hi + 40 for _, hi in FIRMWARE_JOINT_LIMITS])
    for i, (_, hi) in enumerate(FIRMWARE_JOINT_LIMITS):
        assert abs(targets[i] - hi) < 0.001, f"J{i+1} 期望 clamp 到 {hi}, 实际 {targets[i]}"


def test_set_joints_safe_corrects_drift():
    """set_joints_safe 消除漂移: tracked=0, 0x36 读 10 (外力搬了 10°)
    → delta=目标80-真实10=70, 跟踪同步到 10+70=80 (不是 0+80=80, 但基准对了).
    用 J1(0,360) 确保目标 80 不被 clamp."""
    ctrl, t = _mk()
    ctrl._tracked_angles = [0.0] * 6   # 命令积分以为在 0
    for addr in [0x02, 0x03, 0x04, 0x05, 0x06, 0x07]:
        _inject_pos_frame(t, addr, 10.0)   # 真实在 10 (外力搬动)
    # J1 目标 80 (在限位 0-360 内, 不被 clamp)
    from lerobot_robot_massage.zdt.config import FIRMWARE_JOINT_LIMITS
    targets_in = [80.0] * 6
    # 确保目标都在限位内 (J2 限位 90-180, 80 会被 clamp 到 90, 用 100 替代)
    targets_in = [max(lo + 10, min(hi - 10, 80.0)) for lo, hi in FIRMWARE_JOINT_LIMITS]
    ctrl.set_joints_safe(targets_in)
    # 跟踪值应同步到真实+delta = 10+(target-10) = target (消除漂移)
    for i, tgt in enumerate(targets_in):
        assert abs(ctrl._tracked_angles[i] - tgt) < 0.001, \
            f"J{i+1} 期望跟踪 {tgt}, 实际 {ctrl._tracked_angles[i]}"


# ── 生产命令: ready (按摩准备姿态, 6 轴同步慢速) ──────────────

def test_ready_moves_all_axes_with_multisync():
    """ready: 全零位 → 目标 READY_POSE_DEG [0,60,50,0,120,0]. 运动轴下 0xFD(snF=1), 末尾广播同步.

    1:1 标定 (calib=[(1,0)]*6) 下注入 pos=0 → 真实 0:
      J2 delta=60° 脉冲=60×1×3200/360≈533=0x215, @100RPM→vel=100=0x64
      J3 delta=50° 脉冲≈444; J5 delta=120° 脉冲≈1067
      J1/J4/J6 delta=0 → 不发. 多机同步: 每轴 snF=1 + 1 条广播 00 FF 66 6B.
    """
    ctrl, t = _mk()
    for addr in [0x02, 0x03, 0x04, 0x05, 0x06, 0x07]:
        _inject_pos_frame(t, addr, 0.0)
    targets = ctrl.ready()
    # 目标即 READY_POSE, 未被限位改动
    from lerobot_robot_massage.zdt.config import READY_POSE_DEG
    for i, tgt in enumerate(targets):
        assert abs(tgt - READY_POSE_DEG[i]) < 1e-9, f"J{i+1} 期望 {READY_POSE_DEG[i]}, 实际 {tgt}"
    # 三相 (J2/J3/J5) 各发 2 帧 0xFD = 6 帧 + 1 条广播
    fd_frames = [f for f in t.sent if f.data and f.data[0] == 0xFD]
    assert len(fd_frames) == 6, f"期望 6 帧 0xFD (3 轴×2), 实际 {len(fd_frames)}"
    # snF=1: 每个运动的轴, 第二帧 [FD, p0, raF, snF, 6B] 中 data[3]==0x01
    for addr in [0x03, 0x04, 0x06]:        # J2/J3/J5
        frames = [f for f in fd_frames if f.arbitration_id >> 8 == addr]
        assert len(frames) == 2, f"0x{addr:02X} 期望 2 帧, 实际 {len(frames)}"
        assert frames[1].data[3] == 0x01, f"0x{addr:02X} snF 应为 1"
    # 速度 100 RPM → vel=100=0x64: 第一帧 [FD,d,vel_hi,vel_lo,...] data[2..3]
    j2 = [f for f in fd_frames if f.arbitration_id >> 8 == 0x03][0]
    assert j2.data[2:4] == bytes([0x00, 0x64]), f"J2 速度应 100RPM(0x0064), 实际 {j2.data[2:4].hex()}"
    # J1/J4/J6 无 0xFD (delta=0)
    for addr in [0x02, 0x05, 0x07]:
        assert not [f for f in fd_frames if f.arbitration_id >> 8 == addr], \
            f"0x{addr:02X} delta=0 不应发 0xFD"
    # 广播多机同步: ID=0x0000, data=[FF,66,6B]
    sync = [f for f in t.sent if f.arbitration_id == 0x0000]
    assert len(sync) == 1, f"期望 1 条多机同步广播, 实际 {len(sync)}"
    assert sync[0].data == bytes([0xFF, 0x66, 0x6B])
    # 跟踪值同步到目标
    for i, tgt in enumerate(READY_POSE_DEG):
        assert abs(ctrl._tracked_angles[i] - tgt) < 1e-9


def test_home_returns_to_init_pose():
    """home: 从 ready 位 → 回上电姿态全 0 (JOINT_INIT_ANGLE_DEG). 安全速度同步."""
    ctrl, t = _mk()
    from lerobot_robot_massage.zdt.config import JOINT_INIT_ANGLE_DEG, READY_POSE_DEG
    # 注入 ready 位 (calib 1:1): 真实 = READY_POSE_DEG
    for addr, deg in zip([0x02, 0x03, 0x04, 0x05, 0x06, 0x07], READY_POSE_DEG):
        _inject_pos_frame(t, addr, deg)
    targets = ctrl.home()
    # 目标 = 上电姿态全 0, 未被限位改动
    for i, tgt in enumerate(targets):
        assert abs(tgt - JOINT_INIT_ANGLE_DEG[i]) < 1e-9, \
            f"J{i+1} 期望 {JOINT_INIT_ANGLE_DEG[i]}, 实际 {tgt}"
    # J2/J3/J5 从 ready 回 0 → 各 2 帧 0xFD; J1/J4/J6 已在 0 不发
    fd_frames = [f for f in t.sent if f.data and f.data[0] == 0xFD]
    assert len(fd_frames) == 6, f"期望 6 帧 0xFD (3 轴×2), 实际 {len(fd_frames)}"
    for addr in [0x03, 0x04, 0x06]:
        frames = [f for f in fd_frames if f.arbitration_id >> 8 == addr]
        assert len(frames) == 2, f"0x{addr:02X} 期望 2 帧"
        assert frames[1].data[3] == 0x01, f"0x{addr:02X} snF 应为 1"
    # 安全速度 = READY_SPEED_RPM=100 → vel=100=0x64
    j2 = [f for f in fd_frames if f.arbitration_id >> 8 == 0x03][0]
    assert j2.data[2:4] == bytes([0x00, 0x64])
    # 多机同步广播
    sync = [f for f in t.sent if f.arbitration_id == 0x0000]
    assert len(sync) == 1
    assert sync[0].data == bytes([0xFF, 0x66, 0x6B])
    # 跟踪值同步到 0
    for i in range(6):
        assert abs(ctrl._tracked_angles[i]) < 1e-9


def test_ready_already_in_pose_no_motion():
    """ready: 已在准备姿态 (真实角==READY_POSE) → 不发 0xFD 也不广播."""
    ctrl, t = _mk()
    from lerobot_robot_massage.zdt.config import READY_POSE_DEG
    for addr, deg in zip([0x02, 0x03, 0x04, 0x05, 0x06, 0x07], READY_POSE_DEG):
        _inject_pos_frame(t, addr, deg)
    targets = ctrl.ready()
    fd_frames = [f for f in t.sent if f.data and f.data[0] == 0xFD]
    sync = [f for f in t.sent if f.arbitration_id == 0x0000]
    assert not fd_frames
    assert not sync
    for i, tgt in enumerate(targets):
        assert abs(tgt - READY_POSE_DEG[i]) < 1e-9


def test_check_drift_no_drift():
    """check_drift: tracked == real → ok=True, drift=0."""
    ctrl, t = _mk()
    ctrl._tracked_angles = [50.0] * 6
    for addr in [0x02, 0x03, 0x04, 0x05, 0x06, 0x07]:
        _inject_pos_frame(t, addr, 50.0)
    results = ctrl.check_drift()
    assert all(ok and abs(d) < 0.01 for ok, d in results)


def test_check_drift_detects_drift():
    """check_drift: tracked=50, real=55 → drift=5 > 2° → ok=False."""
    ctrl, t = _mk()
    ctrl._tracked_angles = [50.0] * 6
    for addr in [0x02, 0x03, 0x04, 0x05, 0x06, 0x07]:
        _inject_pos_frame(t, addr, 55.0)
    results = ctrl.check_drift(threshold_deg=2.0)
    assert all(not ok and abs(d - 5.0) < 0.01 for ok, d in results)


def test_check_drift_custom_threshold():
    """check_drift: drift=1.5, threshold=1 → 超限; threshold=2 → 未超限.
    需注入两次回帧 (check_drift 调两次, 每次读 6 轴 0x36)."""
    ctrl, t = _mk()
    ctrl._tracked_angles = [50.0] * 6
    for _ in range(2):   # 两次 check_drift, 每次需要 6 轴回帧
        for addr in [0x02, 0x03, 0x04, 0x05, 0x06, 0x07]:
            _inject_pos_frame(t, addr, 51.5)
    strict = ctrl.check_drift(threshold_deg=1.0)
    loose = ctrl.check_drift(threshold_deg=2.0)
    assert all(not ok for ok, _ in strict)
    assert all(ok for ok, _ in loose)


# ── C 任务: check_limits_real 实时限位守卫 ───────────────────────
# _mk() reduction_ratios=[1.0]*6 → 注入 pos 度数 = read_real_angles 输出角度.
# 默认 high_freq={1,2,3,4} → 检查 J2/J3/J4/J5; J1/J6 (360° 旋转) 跳过.

def _inject_all_pos(t, real_deg: list[float]):
    """按 joint_addrs 顺序注入 6 轴 0x36 位置回帧."""
    for addr, deg in zip([0x02, 0x03, 0x04, 0x05, 0x06, 0x07], real_deg):
        _inject_pos_frame(t, addr, deg)


def _sent_stop(t, addr):
    """addr 是否收到单轴 stop (0xFE) 帧."""
    return any(f.arbitration_id >> 8 == addr and f.data
               and f.data[0] == 0xFE for f in t.sent)


def test_check_limits_real_target_oob_stops():
    """J2 (slot1, 0x03) 限位 (-1,150), 目标 160 越界 → stop + 告警."""
    ctrl, t = _mk()
    _inject_all_pos(t, [0.0, 10.0, 0.0, 0.0, 0.0, 0.0])   # J2 real=10
    alarms = ctrl.check_limits_real([0.0, 160.0, 0.0, 0.0, 0.0, 0.0])
    assert _sent_stop(t, 0x03)
    assert any(a["slot"] == 1 for a in alarms)


def test_check_limits_real_real_oob_return_allow():
    """J2 real=160 (越界) 但目标 10 (界内回退) → 只提示不 stop."""
    ctrl, t = _mk()
    _inject_all_pos(t, [0.0, 160.0, 0.0, 0.0, 0.0, 0.0])   # J2 real=160 越界
    alarms = ctrl.check_limits_real([0.0, 10.0, 0.0, 0.0, 0.0, 0.0])
    assert not _sent_stop(t, 0x03)                         # 回退允许, 不停
    assert any(a["slot"] == 1 for a in alarms)             # 但提示真实越界


def test_check_limits_real_360_joint_skipped():
    """J1 (slot0, 0x02) 是 360° 关节不在 high_freq, 目标 500 越界 → 不检查不停."""
    ctrl, t = _mk()
    _inject_all_pos(t, [0.0] * 6)
    alarms = ctrl.check_limits_real([500.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert not _sent_stop(t, 0x02)
    assert not any(a["slot"] == 0 for a in alarms)


def test_check_limits_real_j4_oob_stops():
    """J4 (slot3, 0x05) 有界 [-90,90], 目标 100 越界 → stop + 告警."""
    ctrl, t = _mk()
    _inject_all_pos(t, [0.0] * 6)
    alarms = ctrl.check_limits_real([0.0, 0.0, 0.0, 100.0, 0.0, 0.0])
    assert _sent_stop(t, 0x05)
    assert any(a["slot"] == 3 for a in alarms)


def test_check_limits_real_j6_360_skipped():
    """J6 (slot5, 0x07) 360° 关节不在 high_freq, 目标 500 → 不检查."""
    ctrl, t = _mk()
    _inject_all_pos(t, [0.0] * 6)
    alarms = ctrl.check_limits_real([0.0, 0.0, 0.0, 0.0, 0.0, 500.0])
    assert not _sent_stop(t, 0x07)
    assert not any(a["slot"] == 5 for a in alarms)


def test_check_limits_real_all_within_no_alarm():
    """全部高频关节目标在界内 → 无告警无 stop."""
    ctrl, t = _mk()
    _inject_all_pos(t, [0.0] * 6)
    alarms = ctrl.check_limits_real([10.0, 10.0, 10.0, 10.0, 10.0, 10.0])
    assert alarms == []
    assert not _sent_stop(t, 0x03)


def test_check_limits_real_use_kb_conversion():
    """use_kb=True 用 (k,b) 换算真实角度判限位.
    J2 k=50.8886 b=2.02: 注入 pos=510.906 → real=(510.906-2.02)/50.8886≈10 (界内);
    目标 160 越界 → stop. (2026-08-21 更新为新解码下的标定值, k≈减速比)."""
    ctrl, t = _mk()
    kb = [(50.0014, 0.03), (50.8886, 2.02), (50.8992, 0.02),
          (51.0041, -1.55), (27.0, 0.0), (51.009, 0.01)]
    _inject_all_pos(t, [0.0, 510.906, 0.0, 0.0, 0.0, 0.0])  # J2 pos→real≈10
    alarms = ctrl.check_limits_real([0.0, 160.0, 0.0, 0.0, 0.0, 0.0],
                                    use_kb=True, calib_kb=kb)
    assert _sent_stop(t, 0x03)
    assert any(a["slot"] == 1 for a in alarms)


# ── 2026-08-23: connect 生命周期 (scan/verify + 状态机) ─────

def _mk_armed(ctrl):
    """把注入式 ZdtController 的状态机直接推进到 ARMED (纯状态, 无 CAN)."""
    ctrl.robot.on_connected()
    motors = {a: MotorState(can_id=a, online=True, joint_slot=i)
              for i, a in enumerate(ctrl.config.joint_addrs)}
    ctrl.robot.on_enumerated(motors)
    ctrl.robot.on_safe_idle()
    ctrl.robot.arm(gravity_confirmed=True)
    return ctrl


def test_connect_scan_verify_reaches_safe_idle():
    t = FakeTransport()
    cfg = ZdtConfig(timeout_s=0.001, retries=0, reduction_ratios=[1.0] * 6,
                    calib=[(1.0, 0.0)] * 6)
    ctrl = ZdtController(config=cfg, transport=t)
    for addr in range(0x01, 0x07):                       # firmware scheme
        t.inject(addr, 0x1F, bytes([0x00, 0x01, 0x01]) + b"\x6b")
    for addr in range(0x01, 0x07):                       # sync 读 0x36
        t.inject(addr, F_READ_POS, b"\x00\x00\x00\x00\x00" + b"\x6b")
    ctrl.connect()
    assert ctrl.robot.phase == RobotPhase.SAFE_IDLE
    assert ctrl._connected is True
    assert ctrl.config.joint_addrs == [0x01, 0x02, 0x03, 0x04, 0x05, 0x06]


def test_connect_enumeration_failure_latches_fault():
    # 只探测到 5 台 → 缺 J6 → 硬不变式 → FAULT + 抛 SafetyError + 不使能扭矩
    t = FakeTransport()
    cfg = ZdtConfig(timeout_s=0.001, retries=0, reduction_ratios=[1.0] * 6,
                    calib=[(1.0, 0.0)] * 6)
    ctrl = ZdtController(config=cfg, transport=t)
    for addr in range(0x01, 0x06):
        t.inject(addr, 0x1F, bytes([0x00, 0x01, 0x01]) + b"\x6b")
    try:
        ctrl.connect()
        raise AssertionError("枚举失败应抛 SafetyError")
    except SafetyError:
        pass
    assert ctrl.robot.phase == RobotPhase.FAULT
    assert ctrl.robot.fault_reason
    assert ctrl._connected is False
    assert not any(f.data and f.data[0] == 0xF3 for f in t.sent)   # 未使能扭矩


def test_connect_no_longer_enables_torque():
    # 修订: connect 不再 set_torque(True); arm() 才使能
    t = FakeTransport()
    cfg = ZdtConfig(timeout_s=0.001, retries=0, reduction_ratios=[1.0] * 6,
                    calib=[(1.0, 0.0)] * 6)
    ctrl = ZdtController(config=cfg, transport=t)
    for addr in range(0x01, 0x07):
        t.inject(addr, 0x1F, bytes([0x00, 0x01, 0x01]) + b"\x6b")
    for addr in range(0x01, 0x07):
        t.inject(addr, F_READ_POS, b"\x00\x00\x00\x00\x00" + b"\x6b")
    ctrl.connect()
    assert not any(f.data and f.data[0] == 0xF3 for f in t.sent)
    ctrl.arm(gravity_confirmed=True)
    assert ctrl.robot.phase == RobotPhase.ARMED
    assert any(f.data and f.data[0] == 0xF3 for f in t.sent)


def test_arm_requires_safe_idle():
    ctrl, t = _mk()
    try:
        ctrl.arm(gravity_confirmed=True)
        raise AssertionError("DISCONNECTED 不应能 arm")
    except SafetyError:
        pass


def test_disarm_disables_torque_and_returns_safe_idle():
    ctrl, t = _mk()
    _mk_armed(ctrl)
    ctrl.disarm()
    assert ctrl.robot.phase == RobotPhase.SAFE_IDLE
    assert any(f.data and f.data[0] == 0xF3 and len(f.data) > 2 and f.data[2] == 0x00
               for f in t.sent)


def test_connect_enumeration_failure_cleans_up():
    # 无任何回帧 → 扫描全空 → 硬不变式失败 → SafetyError + FAULT + 清理
    # (read_version 吞掉 TransportError, 总线失败表现为"枚举失败"; e_stop 广播 + close)
    t = FakeTransport()
    cfg = ZdtConfig(timeout_s=0.001, retries=0, reduction_ratios=[1.0] * 6,
                    calib=[(1.0, 0.0)] * 6)
    ctrl = ZdtController(config=cfg, transport=t)
    try:
        ctrl.connect()
        raise AssertionError("connect 应抛 SafetyError")
    except SafetyError:
        pass
    assert t.sent[-1].arbitration_id == 0x0000        # e_stop 广播
    assert t.closed is True
    assert ctrl._connected is False
    assert ctrl.robot.phase == RobotPhase.FAULT        # 闩锁 FAULT, 禁止 arm


def test_get_real_state_fields():
    ctrl, t = _mk()
    _mk_armed(ctrl)
    # 注入顺序必须匹配读取顺序: 全 q (0x36) → 全 current (0x27) → 全 flags (0x3A)
    for addr in ctrl.config.joint_addrs:
        t.inject(addr, F_READ_POS, b"\x00\x00\x00\x00\x00" + b"\x6b")
    for addr in ctrl.config.joint_addrs:
        t.inject(addr, 0x27, b"\x00" + bytes([50]) + b"\x6b")
    for addr in ctrl.config.joint_addrs:
        t.inject(addr, 0x3A, b"\x01\x6b")
    st = ctrl.get_real_state()
    assert set(st) == {"q", "velocity", "current", "flags", "status"}
    assert len(st["q"]) == 6 and len(st["velocity"]) == 6
    assert len(st["current"]) == 6 and len(st["flags"]) == 6
    assert st["status"] == "ARMED"


def test_get_real_state_velocity_filters():
    # 两次读取不同真实角 → 滤波有限差分 dq 非零
    ctrl, t = _mk()
    _mk_armed(ctrl)
    for addr in ctrl.config.joint_addrs:
        t.inject(addr, F_READ_POS, b"\x00\x00\x00\x00\x00" + b"\x6b")
    for addr in ctrl.config.joint_addrs:
        t.inject(addr, 0x27, b"\x00" + bytes([50]) + b"\x6b")
    for addr in ctrl.config.joint_addrs:
        t.inject(addr, 0x3A, b"\x01\x6b")
    st0 = ctrl.get_real_state()
    assert all(v == 0.0 for v in st0["velocity"])   # 首帧无差分
    # 第二帧: q 前进 1°
    for addr, deg in zip(ctrl.config.joint_addrs, [1.0] * 6):
        v = int(round(abs(deg) * 65536.0 / 360.0)) & 0xFFFFFFFF
        t.inject(addr, F_READ_POS, bytes([0x00, (v >> 24) & 0xFF,
                                          (v >> 16) & 0xFF, (v >> 8) & 0xFF,
                                          v & 0xFF]) + b"\x6b")
    for addr in ctrl.config.joint_addrs:
        t.inject(addr, 0x27, b"\x00" + bytes([50]) + b"\x6b")
    for addr in ctrl.config.joint_addrs:
        t.inject(addr, 0x3A, b"\x01\x6b")
    st1 = ctrl.get_real_state()
    assert any(abs(v) > 0.0 for v in st1["velocity"])   # 差分后非零


if __name__ == "__main__":
    run_all(globals())
