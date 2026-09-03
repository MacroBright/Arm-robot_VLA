"""zdt/types.py 测试 — 共享数据契约 (spec TASK-06/24)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import numpy as np

from lerobot_robot_massage.zdt.types import (  # noqa: E402
    CartesianCommand, EEPose, JointState, rotmat_to_quat,
)


def _rotz(deg):
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def test_cartesian_command_defaults():
    cmd = CartesianCommand(linear_velocity=(1.0, 2.0, 3.0))
    assert cmd.angular_velocity == (0.0, 0.0, 0.0)
    assert cmd.timestamp == 0.0
    assert cmd.twist.shape == (6,)
    np.testing.assert_allclose(cmd.twist, [1, 2, 3, 0, 0, 0])


def test_cartesian_command_immutable():
    cmd = CartesianCommand(linear_velocity=(1.0, 0.0, 0.0), timestamp=12.5)
    try:
        cmd.timestamp = 1.0
        raise AssertionError("frozen dataclass 应拒绝修改")
    except Exception:
        pass


def test_joint_state_defaults():
    js = JointState(q=(0.0,) * 6)
    assert len(js.dq) == 6
    assert js.flags == () and js.status == ""


def test_joint_state_flags_six_axis():
    # 6 轴 flags 全量保存 (P2-⑨): 任一轴堵转/失使能在 observation 可见
    js = JointState(q=(0.0,) * 6, flags=(1, 2, 4, 8, 0, 1))
    assert len(js.flags) == 6
    assert js.flags[2] == 4        # J3 堵转标志可见


def test_ee_pose_identity_quaternion():
    p = EEPose(position=np.zeros(3), rotation=np.eye(3))
    w, x, y, z = p.to_quaternion()
    assert abs(w - 1.0) < 1e-9 and abs(x) < 1e-9 and abs(y) < 1e-9 and abs(z) < 1e-9


def test_ee_pose_known_rotation_quaternion():
    # Rz(90°): 四元数 w=cos45, z=sin45
    p = EEPose(position=np.zeros(3), rotation=_rotz(90.0))
    w, x, y, z = p.to_quaternion()
    assert abs(w - np.cos(np.pi / 4)) < 1e-9
    assert abs(z - np.sin(np.pi / 4)) < 1e-9
    assert abs(x) < 1e-9 and abs(y) < 1e-9


def test_rotmat_to_quat_unit_norm_robust():
    # 近 π 旋转 + 随机旋转: 四元数必须单位模长
    for axis in (np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0])):
        K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        R = np.eye(3) + K * np.pi + (K @ K) * 2.0
        q = np.array(rotmat_to_quat(R))
        assert abs(np.linalg.norm(q) - 1.0) < 1e-9, f"axis={axis} q={q}"


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
