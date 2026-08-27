"""config 常量与配置单测 (直接运行: python lerobot_robot_massage/zdt/test_config.py)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from lerobot_robot_massage.zdt.config import (
    CHECKSUM, FIRMWARE_JOINT_LIMITS, JOINT_INIT_ANGLE_DEG, JOINT_ADDRS, POS_SCALE,
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
    # 2026-08-23: 0xFD/0xF6 速度字段 = RPM 直传 (修复 ×10 bug), VEL_SCALE=1.
    assert POS_SCALE == 10.0 and VEL_SCALE == 1.0


def test_limits_len_and_order():
    # 真机 anchor 实测限位 (2026-08-20 更新); 下界 -1.0 = 开机姿态锚定余量
    # (pos=0 经 CALIB 换算出 -b/k 小负角, 如 J2 b=2.02 → -0.040°).
    assert len(FIRMWARE_JOINT_LIMITS) == 6
    assert FIRMWARE_JOINT_LIMITS[0] == (-1.0, 360.0)    # J1 shoulder_pan
    assert FIRMWARE_JOINT_LIMITS[1] == (-1.0, 150.0)    # J2 shoulder_lift (上限实测↑150)
    assert FIRMWARE_JOINT_LIMITS[2] == (-1.0, 120.0)    # J3 elbow_flex (真机实测)
    assert FIRMWARE_JOINT_LIMITS[3] == (-90.0, 90.0)    # J4 wrist_roll
    assert FIRMWARE_JOINT_LIMITS[4] == (-1.0, 180.0)    # J5 wrist_flex (真机实测)
    assert FIRMWARE_JOINT_LIMITS[5] == (-1.0, 360.0)    # J6 gripper


def test_init_pose():
    # 开机姿态即全零期望位: 手动摆固定姿态 → 上电 pos=0 → anchor 真实≈0.
    # (旧固件出厂角 [90,90,-90,0,90,0] 与本方案冲突已弃用)
    assert JOINT_INIT_ANGLE_DEG == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def test_zdtconfig_defaults():
    c = ZdtConfig()
    assert c.channel == "can0"
    assert c.bitrate == 500_000
    assert c.joint_addrs == JOINT_ADDRS
    assert c.limits == FIRMWARE_JOINT_LIMITS
    assert c.speed_rpm == 1800.0 and c.watchdog_s == 1.5 and c.timeout_s == 0.03


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
