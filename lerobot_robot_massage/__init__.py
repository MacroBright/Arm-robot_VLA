"""LeRobot plugin for STM32-mediated Emm_V5 CAN bus massage robotic arm.

Package name follows the lerobot_robot_ prefix convention for auto-discovery
by LeRobot CLI tools (lerobot-record, lerobot-teleop, etc.).
"""

# 无 lerobot 依赖的轻量导入 (手柄遥控、仿真用)
from .serial_protocol import EmergencyStopError, SerialProtocol, SerialProtocolError

# ZDT 直连 CAN 控制 (无 lerobot 依赖)
from .zdt.controller import ZdtController          # noqa: E402
from .zdt.config import ZdtConfig                  # noqa: E402

# 需要 lerobot 核心库的重型导入 (数据采集、训练用)
try:
    from .config_massage_robot import MassageRobotConfig
    from .massage_robot import MassageRobot
    _HAS_LEROBOT = True
except ImportError:
    MassageRobotConfig = None   # type: ignore
    MassageRobot = None         # type: ignore
    _HAS_LEROBOT = False

__all__ = [
    "MassageRobotConfig",
    "MassageRobot",
    "SerialProtocol",
    "SerialProtocolError",
    "EmergencyStopError",
    "ZdtController",
    "ZdtConfig",
]
