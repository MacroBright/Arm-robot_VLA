"""Robot configuration dataclass for MassageRobot.

Registered with LeRobot's configuration system via @RobotConfig.register_subclass.
"""

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots import RobotConfig


@RobotConfig.register_subclass("massage_robot")
@dataclass
class MassageRobotConfig(RobotConfig):
    """Configuration for the massage robotic arm (STM32-mediated).

    Attributes:
        port: Serial port for STM32 communication (e.g., "COM3" on Windows,
            "/dev/ttyUSB0" on Linux).
        baudrate: Serial baud rate. Default 115200 (matches zero-robotic-arm firmware).
        joint_names: Human-readable names for each joint, in motor ID order (1-6).
        num_joints: Number of arm joints (default 6, expandable for dexterous hand).
        cameras: Camera configurations. Default: Intel RealSense D455 RGB stream
            at 640x480@30fps (verified 2026-07-16; USB 2.1 link caps 720p at 15fps).
    """

    port: str = "COM3"
    baudrate: int = 115200

    joint_names: list[str] = field(default_factory=lambda: [
        "shoulder_pan",    # Joint 1 — base rotation
        "shoulder_lift",   # Joint 2 — shoulder lift
        "elbow_flex",      # Joint 3 — elbow
        "wrist_flex",      # Joint 4 — wrist flex
        "wrist_roll",      # Joint 5 — wrist roll
        "gripper",         # Joint 6 — gripper/end-effector
    ])

    num_joints: int = 6

    cameras: dict[str, CameraConfig] = field(default_factory=lambda: {
        # Intel RealSense D455, S/N 135122252036 (2026-07-16 实测):
        #   Windows 下 index 0 = Depth 流 (OpenCV 无法抓帧, 勿用), index 1 = RGB 流
        #   当前线缆只协商到 USB 2.1 → RGB 稳定模式为 640x480@30 (实测 29.9fps)
        #   1280x720 在 USB 2 下最高 15fps (DSHOW 后端会谎报 30fps, 勿信)
        #   换 USB 3.x 线/口 (设备管理器确认协商 3.x) 后可改回 1280x720@30
        # 注意: 抓到的帧 shape 为 (H, W, C) = (480, 640, 3),
        #   numpy 约定"高"在前 — 不是 width/height 写反。
        # 若将来需要深度流/按序列号绑定设备, 改用 lerobot 的 RealSenseCameraConfig。
        "cam_top": OpenCVCameraConfig(
            index_or_path=1,
            fps=30,
            width=640,
            height=480,
        ),
    })

    # Future: dexterous hand joints (AmazingHand: 8-DOF, IDs 7-14)
    # dexterous_hand_enabled: bool = False
    # hand_joint_names: list[str] = field(default_factory=list)
