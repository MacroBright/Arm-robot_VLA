"""HuggingFace LeRobot integration for 6-DOF massage robotic arm."""

from .config_massage_robot import MassageRobotConfig
from .massage_robot import MassageRobot

__all__ = [
    "MassageRobot",
    "MassageRobotConfig",
]
