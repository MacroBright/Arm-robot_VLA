"""workspace 盒 + 速度限幅器测试 (spec TASK-13)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import numpy as np

from lerobot_robot_massage.zdt.workspace import (  # noqa: E402
    BoxWorkspace, CartesianVelocityLimiter,
)


def _box():
    return BoxWorkspace([-200.0, -200.0, 0.0], [200.0, 200.0, 300.0])


def test_contains_inside_outside_boundary():
    b = _box()
    assert b.contains([0.0, 0.0, 150.0])
    assert not b.contains([500.0, 0.0, 150.0])
    assert b.contains([200.0, -200.0, 300.0])   # 边界含


def test_clamp_components():
    b = _box()
    got = b.clamp([500.0, -500.0, 150.0])
    np.testing.assert_allclose(got, [200.0, -200.0, 150.0])


def test_scale_velocity_inside_high_velocity_unchanged():
    b = _box()
    v, clamped = b.scale_velocity(np.array([0.0, 0.0, 300.0]),
                                  np.array([0.0, 0.0, 100.0]), dt=0.1)
    np.testing.assert_allclose(v, [0.0, 0.0, 300.0])
    assert clamped == []


def test_scale_velocity_heading_out_stops_at_wall():
    b = _box()
    # x=190, 200mm/s 沿 +x, dt=0.1 → 目标 210 > 200 → 缩到刚好停在 200
    v, clamped = b.scale_velocity(np.array([200.0, 0.0, 0.0]),
                                  np.array([190.0, 0.0, 150.0]), dt=0.1)
    np.testing.assert_allclose(v, [100.0, 0.0, 0.0])   # (200-190)/0.1 = 100
    assert clamped == [0]


def test_scale_velocity_already_out_blocked_axis():
    b = _box()
    # 已越界 (x=250>200) 再往外走 → 该轴速度置 0 (blocked)
    v, clamped = b.scale_velocity(np.array([100.0, 0.0, 0.0]),
                                  np.array([250.0, 0.0, 150.0]), dt=0.1)
    np.testing.assert_allclose(v, [0.0, 0.0, 0.0])
    assert clamped == [0]


def test_scale_velocity_moving_back_allowed():
    b = _box()
    v, clamped = b.scale_velocity(np.array([-100.0, 0.0, 0.0]),
                                  np.array([250.0, 0.0, 150.0]), dt=0.1)
    np.testing.assert_allclose(v, [-100.0, 0.0, 0.0])
    assert clamped == []


def test_scale_velocity_zero_velocity_inside_unchanged():
    b = _box()
    v, clamped = b.scale_velocity(np.array([0.0, 0.0, 0.0]),
                                  np.array([0.0, 0.0, 150.0]), dt=0.1)
    np.testing.assert_allclose(v, [0.0, 0.0, 0.0])
    assert clamped == []


def test_limiter_clamps_max_speed():
    lim = CartesianVelocityLimiter(max_vel_mm_s=20.0)
    v, clamped = lim(np.array([100.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]), dt=0.1)
    assert abs(float(np.linalg.norm(v)) - 20.0) < 1e-9
    assert clamped == []


def test_limiter_applies_box():
    box = _box()
    lim = CartesianVelocityLimiter(max_vel_mm_s=200.0, workspace=box)
    # z=290, 300mm/s 向上, dt=0.1 → 目标 320 > 300 → 缩到 (300-290)/0.1=100
    v, clamped = lim(np.array([0.0, 0.0, 300.0]), np.array([0.0, 0.0, 290.0]), dt=0.1)
    np.testing.assert_allclose(v, [0.0, 0.0, 100.0])
    assert clamped == [2]


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
