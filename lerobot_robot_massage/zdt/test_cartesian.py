import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from lerobot_robot_massage.zdt.cartesian import CartesianController  # noqa: E402
from lerobot_robot_massage.zdt.config import F_READ_POS, ZdtConfig  # noqa: E402
from lerobot_robot_massage.zdt.controller import ZdtController  # noqa: E402
from lerobot_robot_massage.zdt.fakes import FakeTransport  # noqa: E402
from lerobot_robot_massage.zdt.kinematics import (  # noqa: E402
    T_0_6_RESET, anchor_to_source, fk_mdh,
)
from lerobot_robot_massage.zdt.safety import MotorState, RobotPhase  # noqa: E402
from lerobot_robot_massage.zdt.testutil import FakeClock  # noqa: E402

ADDRS = [0x02, 0x03, 0x04, 0x05, 0x06, 0x07]

# 按摩准备姿态 (anchor 帧) — 数值 IK 的起始点 (ready), 2026-08-23 真机调整
READY_ANCHOR = [0.0, 60.0, 50.0, 0.0, 120.0, 0.0]


def _mk(calib=None):
    t = FakeTransport()
    cfg = ZdtConfig(timeout_s=0.1, retries=0,
                    reduction_ratios=[1.0] * 6,
                    calib=calib or [(1.0, 0.0)] * 6)
    ctrl = ZdtController(config=cfg, transport=t)
    return ctrl, t


def _arm_robot(ctrl):
    ctrl.robot.on_connected()
    motors = {a: MotorState(can_id=a, online=True, joint_slot=i)
              for i, a in enumerate(ADDRS)}
    ctrl.robot.on_enumerated(motors)
    ctrl.robot.on_safe_idle()
    ctrl.robot.arm(gravity_confirmed=True)
    return ctrl


def _mk_armed(calib=None):
    ctrl, t = _mk(calib)
    _arm_robot(ctrl)
    return ctrl, t


def _ready_cart(loop_hz=20.0, max_vel=20.0, clock=None, stale_cmd_max_s=0.25):
    ctrl, t = _mk_armed()
    cart = CartesianController(ctrl, loop_hz=loop_hz, max_vel_mm_s=max_vel,
                               clock=clock or FakeClock(),
                               stale_cmd_max_s=stale_cmd_max_s)
    return ctrl, t, cart


def _inject_anchor_zero(t, n=1):
    """注入 n 轮 0x36 回帧: 6 轴 anchor 全 0 (开机姿态)."""
    for _ in range(n):
        for addr in ADDRS:
            t.inject(addr, F_READ_POS, b"\x00\x00\x00\x00\x00" + b"\x6b")


def _inject_anchor_pose(t, q_anchor, n=1):
    """注入 n 轮指定 anchor 关节角 (度) 的 0x36 回帧."""
    for _ in range(n):
        for addr, deg in zip(ADDRS, q_anchor):
            v = int(round(abs(deg) * 65536.0 / 360.0)) & 0xFFFFFFFF
            sign = 0x01 if deg < 0 else 0x00
            t.inject(addr, F_READ_POS,
                     bytes([sign, (v >> 24) & 0xFF, (v >> 16) & 0xFF,
                            (v >> 8) & 0xFF, v & 0xFF]) + b"\x6b")


def test_step_zero_velocity_target_is_current_ee():
    """step(v=0): 目标 = 当前末端 (开机姿态 FK 位置)."""
    ctrl, t = _mk_armed()
    _inject_anchor_zero(t, n=1)
    cart = CartesianController(ctrl, loop_hz=20.0)
    res = cart.step(0.0, 0.0, 0.0)
    assert res["moved"] is True
    # 开机姿态 (anchor 0 → source reset) FK 位置 = T_0_6_RESET 平移
    for i in range(3):
        assert abs(res["target_xyz"][i] - T_0_6_RESET[i, 3]) < 1.0, \
            f"target {res['target_xyz']} vs reset {T_0_6_RESET[:3,3]}"


def test_step_positive_vx_moves_along_x():
    """step(vx>0): 目标 x 增大, 发送 0xFD 运动帧."""
    ctrl, t = _mk_armed()
    _inject_anchor_zero(t, n=1)
    cart = CartesianController(ctrl, loop_hz=20.0, max_vel_mm_s=20.0)
    res = cart.step(10.0, 0.0, 0.0)   # 10 mm/s × 0.05s = 0.5mm
    assert res["moved"] is True
    assert res["target_xyz"][0] > T_0_6_RESET[0, 3] + 0.1  # x 增大
    # 发送了 0xFD 帧 (set_joints_safe)
    fd = [f for f in t.sent if f.data and f.data[0] == 0xFD]
    assert len(fd) > 0


def test_step_velocity_clamped():
    """速度超上限 (100mm/s > 20) → 钳制到 20mm/s, 步进 ≤ 20*0.05=1mm."""
    ctrl, t = _mk_armed()
    _inject_anchor_zero(t, n=1)
    cart = CartesianController(ctrl, loop_hz=20.0, max_vel_mm_s=20.0)
    res = cart.step(100.0, 0.0, 0.0)
    assert res["moved"] is True
    step_mm = res["target_xyz"][0] - T_0_6_RESET[0, 3]
    assert step_mm <= 1.0 + 1e-6, f"step {step_mm}mm 应 ≤1mm (20mm/s @20Hz)"


def test_step_ready_pose_tracks_position():
    """ready 起点 50 步 +x: 末态 FK 位置 ≈ 起点 + v·(N·dt) (数值 IK 可用)."""
    clock = FakeClock()
    ctrl, t, cart = _ready_cart(clock=clock)
    q = list(READY_ANCHOR)
    N, vx = 50, 10.0
    for _ in range(N):
        _inject_anchor_pose(t, q, n=1)
        res = cart.step(vx, 0.0, 0.0)
        assert res["moved"] is True, f"step 未运动: {res}"
        q = list(ctrl._tracked_angles)   # 完美伺服: 命令角 = 下帧真实角
        clock.tick(0.05)
    ee0 = fk_mdh(anchor_to_source(READY_ANCHOR))[:3, 3]
    eeN = fk_mdh(anchor_to_source(q))[:3, 3]
    expect = ee0[0] + vx * N / 20.0
    assert abs(eeN[0] - expect) < 1.5, \
        f"x 末态 {eeN[0]:.2f} vs 期望 {expect:.2f}"


def test_step_ready_no_wrist_flip():
    """100 步 +x: J5 保持 ~165 (不折腕), J4 不进翻腕带 [150,210] (回归关键)."""
    ctrl, t, cart = _ready_cart()
    q = list(READY_ANCHOR)
    j5_lo, j5_hi, j4_flip = 999.0, -999.0, False
    for _ in range(100):
        _inject_anchor_pose(t, q, n=1)
        res = cart.step(10.0, 0.0, 0.0)
        assert res["moved"] is True, f"step 未运动: {res}"
        q = list(ctrl._tracked_angles)
        j5_lo, j5_hi = min(j5_lo, q[4]), max(j5_hi, q[4])
        if 150.0 <= q[3] <= 210.0:
            j4_flip = True
    assert j5_lo >= 117.0 and j5_hi <= 123.0, \
        f"J5 范围 [{j5_lo:.1f},{j5_hi:.1f}] 应≈120 (不翻腕)"
    assert not j4_flip, "J4 进入翻腕带 [150,210]"


def test_step_unreachable_target_hits_limit_alarm():
    """边界处试图越出限位 → limit_alarm 停车, 不发 0xFD."""
    ctrl, t, cart = _ready_cart()
    # J2 处于极限边界 -0.99 (限位 -1.0), 试图向上运动将越界
    q = [0.0, -0.99, 50.0, 0.0, 120.0, 0.0]
    fd_before = sum(1 for f in t.sent if f.data and f.data[0] == 0xFD)
    _inject_anchor_pose(t, q, n=1)
    res = cart.step(0.0, 0.0, 20.0)
    assert not res["moved"]
    assert res["reason"] == "limit_alarm"
    assert len(res["alarms"]) > 0
    # 告警帧不发 0xFD (step 在 set_joints_safe 前返回)
    fd_after = sum(1 for f in t.sent if f.data and f.data[0] == 0xFD)
    assert fd_after == fd_before


def test_step_zero_velocity_sends_no_pulses():
    """ready 位 v=0: moved=True 且不发任何 0xFD 脉冲."""
    ctrl, t, cart = _ready_cart()
    _inject_anchor_pose(t, READY_ANCHOR, n=1)
    res = cart.step(0.0, 0.0, 0.0)
    assert res["moved"] is True
    assert not any(f.data and f.data[0] == 0xFD for f in t.sent)


def test_step_zero_velocity_never_alarms():
    """零速度在开机姿态不应触发限位告警."""
    ctrl2, t2 = _mk_armed()
    _inject_anchor_zero(t2, n=1)
    cart2 = CartesianController(ctrl2, loop_hz=0.05, max_vel_mm_s=20.0)
    res = cart2.step(0.0, 0.0, 0.0)
    assert res["moved"] is True


def test_get_ee_xyz_reports_reset_position():
    """get_ee_xyz: 开机姿态 → FK 位置 ≈ (0, -47.63, 15.5)."""
    ctrl, t = _mk()
    _inject_anchor_zero(t, n=1)
    cart = CartesianController(ctrl)
    xyz = cart.get_ee_xyz()
    assert abs(xyz[0] - T_0_6_RESET[0, 3]) < 1.0
    assert abs(xyz[1] - T_0_6_RESET[1, 3]) < 1.0
    assert abs(xyz[2] - T_0_6_RESET[2, 3]) < 1.0


# ── 新增测试 (6DOF / dt / 陈旧命令 / 安全链 / step_pose) ───────

def test_step_6dof_passes_angular_velocity():
    """6DOF: 角速度进入 twist, 遥测含 λ/scale/condition."""
    ctrl, t, cart = _ready_cart(max_vel=20.0)
    _inject_anchor_pose(t, READY_ANCHOR, n=1)
    res = cart.step(0.0, 0.0, 0.0, wx=0.5, wy=0.0, wz=0.0)
    assert res["moved"] is True
    assert "lambda" in res and "scale" in res and "condition" in res


def test_step_angular_velocity_clamped():
    ctrl, t, cart = _ready_cart(max_vel=20.0)
    _inject_anchor_pose(t, READY_ANCHOR, n=1)
    res = cart.step(0.0, 0.0, 0.0, wx=100.0, wy=0.0, wz=0.0)   # 100 rad/s >> 1
    assert res["moved"] is True


def test_step_requires_armed():
    ctrl, t = _mk()                     # 未 arm 的状态机 (DISCONNECTED)
    cart = CartesianController(ctrl)
    _inject_anchor_zero(t, n=1)
    res = cart.step(0.0, 0.0, 0.0)
    assert res["moved"] is False
    assert res["reason"].startswith("not_armed")


def test_step_measured_dt_uses_frame_gap():
    clock = FakeClock()
    ctrl, t, cart = _ready_cart(clock=clock)
    q = list(READY_ANCHOR)
    _inject_anchor_pose(t, q, n=1)
    cart.step(10.0, 0.0, 0.0)              # 首帧 dt=dt_default=0.05
    q = list(ctrl._tracked_angles)
    clock.tick(0.04)
    _inject_anchor_pose(t, q, n=1)
    res = cart.step(10.0, 0.0, 0.0)        # 第二帧 dt=0.04
    x0 = fk_mdh(anchor_to_source(q))[0, 3]
    assert abs(res["target_xyz"][0] - x0 - 10.0 * 0.04) < 0.1


def test_step_dt_bounded_by_dt_max():
    clock = FakeClock()
    ctrl, t, cart = _ready_cart(clock=clock)
    q = list(READY_ANCHOR)
    _inject_anchor_pose(t, q, n=1)
    cart.step(10.0, 0.0, 0.0)
    q = list(ctrl._tracked_angles)
    clock.tick(5.0)                         # 挂起 5s → dt 钳到 dt_max=3*0.05=0.15
    _inject_anchor_pose(t, q, n=1)
    res = cart.step(10.0, 0.0, 0.0)
    x0 = fk_mdh(anchor_to_source(q))[0, 3]
    assert abs(res["target_xyz"][0] - x0 - 10.0 * 0.15) < 0.1


def test_step_dt_bounded_by_dt_min():
    clock = FakeClock()
    ctrl, t, cart = _ready_cart(clock=clock)
    q = list(READY_ANCHOR)
    _inject_anchor_pose(t, q, n=1)
    cart.step(10.0, 0.0, 0.0)
    q = list(ctrl._tracked_angles)
    clock.tick(1e-9)                        # 时钟未推进 → dt 钳到 dt_min=0.5*0.05
    _inject_anchor_pose(t, q, n=1)
    res = cart.step(10.0, 0.0, 0.0)
    x0 = fk_mdh(anchor_to_source(q))[0, 3]
    assert abs(res["target_xyz"][0] - x0 - 10.0 * 0.025) < 0.1


def test_step_stale_command_refused():
    """陈旧命令看门狗 (控制层权威, 单调期限)."""
    clock = FakeClock()
    ctrl, t, cart = _ready_cart(clock=clock, stale_cmd_max_s=0.25)
    _inject_anchor_pose(t, READY_ANCHOR, n=1)
    clock.tick(0.3)                         # cmd_ts=0.0 距今 0.3s > 0.25s
    res = cart.step(10.0, 0.0, 0.0, cmd_ts=0.0)
    assert res["moved"] is False
    assert res["reason"] == "stale_command"
    assert not any(f.data and f.data[0] == 0xFD for f in t.sent)


def test_step_fresh_command_accepted():
    clock = FakeClock()
    ctrl, t, cart = _ready_cart(clock=clock, stale_cmd_max_s=0.25)
    _inject_anchor_pose(t, READY_ANCHOR, n=1)
    res = cart.step(10.0, 0.0, 0.0, cmd_ts=clock.t)   # 新鲜
    assert res["moved"] is True


def test_step_workspace_blocks_outside_box():
    from lerobot_robot_massage.zdt.workspace import BoxWorkspace
    ctrl, t, cart = _ready_cart()
    T = fk_mdh(anchor_to_source(READY_ANCHOR))
    x0 = T[0, 3]
    # 盒 x∈(x0, x0+10] 严格排除当前位 (已越界且静止) → workspace_blocked
    box = BoxWorkspace([x0 + 0.01, -500.0, -500.0], [x0 + 10.0, 500.0, 500.0])
    cart.workspace = box
    cart._limiter.workspace = box
    _inject_anchor_pose(t, READY_ANCHOR, n=1)
    res = cart.step(0.0, 0.0, 0.0)
    assert res["moved"] is False and res["reason"] == "workspace_blocked"


def test_scale_toward_limits_progressive():
    """两层不变式 (评审 §六): 预测限位只渐进减速, 硬拒绝交给 check_limits_real."""
    ctrl, t, cart = _ready_cart()
    q_anchor = np.array([0.0, 148.0, 50.0, 0.0, 120.0, 0.0])   # J2 靠近上限 150
    q_src = np.array(anchor_to_source(q_anchor))
    dq = np.zeros(6)
    dq[1] = math.radians(1.0)                     # 下一帧 J2=149, margin 内 → 渐进缩小
    scaled = cart._scale_toward_limits(q_src, dq)
    assert 0.0 < scaled[1] < dq[1]
    dq[1] = math.radians(3.0)                     # 下一帧 J2=151 > 150 → 缩到 0
    scaled2 = cart._scale_toward_limits(q_src, dq)
    assert scaled2[1] == 0.0


def test_step_singular_band_refuses_motion():
    """奇异度实际参与: 强制 sing_ratio 覆盖常态 → SINGULAR 拒动 (非仅 telemetry)."""
    ctrl, t, cart = _ready_cart()
    cart.sing_ratio = 1.5                # ratio ≤ 1 恒成立 → 必入 SINGULAR
    cart.near_ratio = 2.0
    _inject_anchor_pose(t, READY_ANCHOR, n=1)
    res = cart.step(10.0, 0.0, 0.0)
    assert res["moved"] is False and res["reason"] == "singular"
    assert res["scale"] == 0.0 and res["lambda"] > 10.0


def test_step_near_singular_scales_twist():
    """NEAR 带: scale<1 且 λ>base → 实际参与速度缩放 (按当前位实测 ratio 设带)."""
    from lerobot_robot_massage.zdt.kinematics import (
        anchor_to_source, jacobian, singularity_metrics,
    )
    ctrl, t, cart = _ready_cart()
    m = singularity_metrics(jacobian(anchor_to_source(READY_ANCHOR)))
    ratio = m["sigma_min"] / max(m["sigma_max"], 1e-12)
    cart.sing_ratio = ratio / 2.0        # 实际 ratio 落 NEAR 带 (确定)
    cart.near_ratio = ratio * 1.5
    _inject_anchor_pose(t, READY_ANCHOR, n=1)
    res = cart.step(10.0, 0.0, 0.0)
    assert res["moved"] is True
    assert 0.0 < res["scale"] < 1.0
    assert res["lambda"] > 10.0


def test_step_pose_reaches_target():
    """step_pose: SE(3) 误差 → 位置+姿态环 → step. 姿态误差经 log_so3 (无 Euler 累加)."""
    clock = FakeClock()
    ctrl, t, cart = _ready_cart(clock=clock)
    q = list(READY_ANCHOR)
    T0 = fk_mdh(anchor_to_source(q))
    p_des = T0[:3, 3] + np.array([5.0, 0.0, 0.0])
    for _ in range(3):
        _inject_anchor_pose(t, q, n=1)
        res = cart.step_pose(p_des, T0[:3, :3])
        assert res["moved"] is True
        q = list(ctrl._tracked_angles)
        clock.tick(0.05)
    ee = fk_mdh(anchor_to_source(q))[:3, 3]
    assert ee[0] > T0[:3, 3][0] + 1.0        # 沿 +x 移动


def test_step_pose_rpy_safety_clamps_orientation():
    """可选 RPY 安全约束 (相对 anchor): 大姿态误差被 clamp 到界内, 内部仍 SO(3)."""
    ctrl, t, cart = _ready_cart()
    _inject_anchor_pose(t, READY_ANCHOR, n=1)
    T = fk_mdh(anchor_to_source(READY_ANCHOR))
    roll = np.array([[1, 0, 0], [0, np.cos(0.4), -np.sin(0.4)],
                     [0, np.sin(0.4), np.cos(0.4)]])
    R_des = T[:3, :3] @ roll
    res = cart.step_pose(T[:3, 3], R_des,
                         rpy_anchor=T[:3, :3],
                         rpy_limits=(np.array([-0.1, -0.1, -0.1]),
                                     np.array([0.1, 0.1, 0.1])))
    assert res["moved"] is True             # clamp 后仍在安全范围, 不拒绝


if __name__ == "__main__":
    failed = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS {name}")
            except Exception as exc:  # noqa: BLE001
                failed += 1
                print(f"FAIL {name}: {exc}")
    print("ALL PASS" if failed == 0 else f"{failed} FAILED")
