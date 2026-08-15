"""config 常量与配置单测 (直接运行: python lerobot_robot_massage/zdt/test_config.py)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from lerobot_robot_massage.zdt.config import (
    CHECKSUM, DEFAULT_LIMITS, INIT_POSE_DEG, JOINT_ADDRS, POS_SCALE,
    VEL_SCALE, ZdtConfig, F_ENABLE, F_POS, F_READ_POS, F_READ_CUR,
    F_STOP, F_VEL, F_ARRIVED,
)


def test_joint_addrs_mapping():
    assert JOINT_ADDRS == [0x02, 0x03, 0x04, 0x05, 0x06, 0x07]


def test_checksum_and_funcs():
    assert CHECKSUM == 0x6B
    assert (F_ENABLE, F_STOP, F_POS, F_VEL, F_READ_POS, F_READ_CUR, F_ARRIVED) == (
        0xF3, 0xFE, 0xFB, 0xF6, 0x36, 0x27, 0xFD)


def test_scales():
    assert POS_SCALE == 10.0 and VEL_SCALE == 10.0


def test_limits_len_and_order():
    assert len(DEFAULT_LIMITS) == 6
    assert DEFAULT_LIMITS[0] == (0.0, 360.0)   # J1 shoulder_pan
    assert DEFAULT_LIMITS[2] == (-90.0, 90.0)  # J3 elbow_flex


def test_init_pose():
    assert INIT_POSE_DEG == [90.0, 45.0, 90.0, 90.0, 0.0, 0.0]


def test_zdtconfig_defaults():
    c = ZdtConfig()
    assert c.channel == "can0"
    assert c.bitrate == 500_000
    assert c.joint_addrs == JOINT_ADDRS
    assert c.limits == DEFAULT_LIMITS
    assert c.speed_rpm == 60.0 and c.watchdog_s == 0.5


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
    sys.exit(1 if failed else 0)
