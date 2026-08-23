"""Robot configuration dataclass for MassageRobot.

Registered with LeRobot's configuration system via @RobotConfig.register_subclass.
"""

from dataclasses import dataclass, field
from typing import Literal

from lerobot.cameras import CameraConfig
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots import RobotConfig


@RobotConfig.register_subclass("massage_robot")
@dataclass
class MassageRobotConfig(RobotConfig):
    """Configuration for the massage robotic arm (STM32-mediated).

    Attributes:
        port: Serial port for STM32 communication (e.g., "COM3" on Windows,
            "/dev/ttyUSB0" on Linux). Only used when transport == "serial".
        baudrate: Serial baud rate. Default 115200 (matches zero-robotic-arm
            firmware). Only used when transport == "serial".
        transport: Protocol backend selection, "serial" (STM32 gateway via
            SerialProtocol) or "can" (PC 直连 CAN via ZdtController).
        channel: Linux SocketCAN interface name (e.g., "can0"), used when
            transport == "can".
        can_bitrate: CAN bus bitrate in bits/s, used when transport == "can".
        joint_names: Human-readable names for each joint, in motor ID order (1-6).
        num_joints: Number of arm joints (default 6, expandable for dexterous hand).
        cameras: Camera configurations. Default: Intel RealSense D455 RGB stream
            at 640x480@30fps (verified 2026-07-16; USB 2.1 link caps 720p at 15fps).
    """

    port: str = "COM3"
    baudrate: int = 115200

    transport: Literal["serial", "can"] = "serial"  # "serial" | "can"
    channel: str = "can0"           # transport=="can" 时用
    can_bitrate: int = 500_000
    # 0xFD 位置命令参数 (transport=="can" 时):
    #   各关节减速比 (输出轴 1° = 电机轴 N°). 空 → 用 zdt 默认 (固件整臂值).
    #   装外置减速器 (如 51:1) 用 calib 标定后填写 6 个值.
    reduction_ratios: list[float] = field(default_factory=list)
    # connect() 完成后自动运动到按摩准备姿态 (READY_POSE_DEG, 慢速同步).
    # 仅 transport=="can" 支持. 推理/评估部署开 True; 手动示教采集保持 False
    # (采集前人工摆位, 力矩关闭).
    move_to_ready_on_connect: bool = False

    joint_names: list[str] = field(default_factory=lambda: [
        "shoulder_pan",    # Joint 1 — base rotation
        "shoulder_lift",   # Joint 2 — shoulder lift
        "elbow_flex",      # Joint 3 — elbow
        "wrist_roll",      # Joint 4 — wrist roll (腕翻转)
        "wrist_flex",      # Joint 5 — wrist flex (腕俯仰)
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
