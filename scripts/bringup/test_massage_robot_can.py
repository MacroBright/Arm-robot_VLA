"""MassageRobot CAN transport 冒烟 (无 lerobot 依赖时跳过)."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

try:
    from lerobot_robot_massage.config_massage_robot import MassageRobotConfig
    _HAS = True
except ImportError:
    _HAS = False


def test_config_defaults_serial():
    if not _HAS:
        return
    c = MassageRobotConfig()
    assert c.transport == "serial"


def test_config_can_fields():
    if not _HAS:
        return
    c = MassageRobotConfig(transport="can", channel="can1", can_bitrate=500_000)
    assert c.transport == "can" and c.channel == "can1"


def test_can_transport_selects_zdt_controller():
    # transport="can" → _protocol 必须是 ZdtController (空 cameras 避免硬件依赖)
    if not _HAS:
        return
    from lerobot_robot_massage.massage_robot import MassageRobot
    from lerobot_robot_massage.zdt.controller import ZdtController
    cfg = MassageRobotConfig(transport="can", cameras={})
    robot = MassageRobot(cfg)
    assert isinstance(robot._protocol, ZdtController)


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
