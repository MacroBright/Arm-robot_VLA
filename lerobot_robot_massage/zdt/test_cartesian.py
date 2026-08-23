"""cartesian 笛卡尔控制器测试 — FK 反馈 → IK → 安全运动闭环 (P1)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from lerobot_robot_massage.zdt.cartesian import CartesianController  # noqa: E402
from lerobot_robot_massage.zdt.config import F_READ_POS, ZdtConfig  # noqa: E402
from lerobot_robot_massage.zdt.controller import ZdtController  # noqa: E402
from lerobot_robot_massage.zdt.fakes import FakeTransport  # noqa: E402
from lerobot_robot_massage.zdt.kinematics import (  # noqa: E402
    T_0_6_RESET, anchor_to_source, fk_mdh,
)

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
    ctrl, t = _mk()
    _inject_anchor_zero(t, n=5)   # FK反馈 + check_limits + set_joints_safe 多次读取
    cart = CartesianController(ctrl, loop_hz=20.0)
    res = cart.step(0.0, 0.0, 0.0)
    assert res["moved"] is True
    # 开机姿态 (anchor 0 → source reset) FK 位置 = T_0_6_RESET 平移
    for i in range(3):
        assert abs(res["target_xyz"][i] - T_0_6_RESET[i, 3]) < 1.0, \
            f"target {res['target_xyz']} vs reset {T_0_6_RESET[:3,3]}"


def test_step_positive_vx_moves_along_x():
    """step(vx>0): 目标 x 增大, 发送 0xFD 运动帧."""
    ctrl, t = _mk()
    _inject_anchor_zero(t, n=6)
    cart = CartesianController(ctrl, loop_hz=20.0, max_vel_mm_s=20.0)
    res = cart.step(10.0, 0.0, 0.0)   # 10 mm/s × 0.05s = 0.5mm
    assert res["moved"] is True
    assert res["target_xyz"][0] > T_0_6_RESET[0, 3] + 0.1  # x 增大
    # 发送了 0xFD 帧 (set_joints_safe)
    fd = [f for f in t.sent if f.data and f.data[0] == 0xFD]
    assert len(fd) > 0


def test_step_velocity_clamped():
    """速度超上限 (100mm/s > 20) → 钳制到 20mm/s, 步进 ≤ 20*0.05=1mm."""
    ctrl, t = _mk()
    _inject_anchor_zero(t, n=6)
    cart = CartesianController(ctrl, loop_hz=20.0, max_vel_mm_s=20.0)
    res = cart.step(100.0, 0.0, 0.0)
    assert res["moved"] is True
    step_mm = res["target_xyz"][0] - T_0_6_RESET[0, 3]
    assert step_mm <= 1.0 + 1e-6, f"step {step_mm}mm 应 ≤1mm (20mm/s @20Hz)"


def _ready_cart(loop_hz=20.0, max_vel=20.0):
    """ready 姿态起点的控制器 + FakeTransport."""
    ctrl, t = _mk()
    cart = CartesianController(ctrl, loop_hz=loop_hz, max_vel_mm_s=max_vel)
    return ctrl, t, cart


def test_step_ready_pose_tracks_position():
    """ready 起点 50 步 +x: 末态 FK 位置 ≈ 起点 + v·(N·dt) (数值 IK 可用)."""
    ctrl, t, cart = _ready_cart()
    q = list(READY_ANCHOR)
    N, vx = 50, 10.0
    for _ in range(N):
        _inject_anchor_pose(t, q, n=3)   # FK + guard + set_joints_safe 三轮回读
        res = cart.step(vx, 0.0, 0.0)
        assert res["moved"] is True, f"step 未运动: {res}"
        q = list(ctrl._tracked_angles)   # 完美伺服: 命令角 = 下帧真实角
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
        _inject_anchor_pose(t, q, n=3)
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
    """持续 +z 推高 → 某关节目标越出限位 → limit_alarm 停车, 不发 0xFD."""
    ctrl, t, cart = _ready_cart()
    q = list(READY_ANCHOR)
    hit, fd_before = None, 0
    for _ in range(1000):
        fd_before = sum(1 for f in t.sent if f.data and f.data[0] == 0xFD)
        _inject_anchor_pose(t, q, n=3)
        res = cart.step(0.0, 0.0, 10.0)
        if not res["moved"]:
            hit = res
            break
        q = list(ctrl._tracked_angles)
    assert hit is not None, "1000 步内未触发限位"
    assert hit["reason"] == "limit_alarm"
    assert len(hit["alarms"]) > 0
    # 告警帧不发 0xFD (step 在 set_joints_safe 前返回)
    fd_after = sum(1 for f in t.sent if f.data and f.data[0] == 0xFD)
    assert fd_after == fd_before


def test_step_zero_velocity_sends_no_pulses():
    """ready 位 v=0: moved=True 且不发任何 0xFD 脉冲."""
    ctrl, t, cart = _ready_cart()
    _inject_anchor_pose(t, READY_ANCHOR, n=3)
    res = cart.step(0.0, 0.0, 0.0)
    assert res["moved"] is True
    assert not any(f.data and f.data[0] == 0xFD for f in t.sent)


def test_step_zero_velocity_never_alarms():
    """零速度在开机姿态不应触发限位告警."""
    ctrl2, t2 = _mk()
    _inject_anchor_zero(t2, n=6)
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
