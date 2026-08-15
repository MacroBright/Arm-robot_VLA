"""ZDT 驱动常量与配置.

组帧约定 (参考固件 can.c can_SendCmd, spec §2.1):
  扩展帧 ID = (地址<<8) | 包序号
  数据段    = [功能码, 参数..., 0x6B]
  参数 >7 字节 → 拆多帧, 每帧重复功能码
"""
from dataclasses import dataclass, field

# 关节→CAN 地址 (J1→02 ... J6→07), 帧 ID 高字节
JOINT_ADDRS: list[int] = [0x02, 0x03, 0x04, 0x05, 0x06, 0x07]

# 帧校验字节 (数据段末字节固定)
CHECKSUM: int = 0x6B

# ZDT 功能码
F_ENABLE: int = 0xF3      # 使能/失能
F_STOP: int = 0xFE        # 立即停止
F_POS: int = 0xFB         # 直通限速位置
F_VEL: int = 0xF6         # 速度模式
F_READ_POS: int = 0x36    # 读实时位置
F_READ_CUR: int = 0x27    # 读相电流
F_ARRIVED: int = 0xFD     # 到位帧 (数据段[0])

# 换算倍率
POS_SCALE: float = 10.0   # 位置 ×10
VEL_SCALE: float = 10.0   # 速度 RPM×10

# 关节限位表 (度) — 真机扫掠后修正 (spec §2.2)
# TODO(bring-up): J2 INIT=45° 与 LIMITS 下界 90° 矛盾, Task 8 真机扫掠后修正
DEFAULT_LIMITS: list[tuple[float, float]] = [
    (0.0, 360.0),    # J1 shoulder_pan
    (90.0, 180.0),   # J2 shoulder_lift
    (-90.0, 90.0),   # J3 elbow_flex
    (-90.0, 90.0),   # J4 wrist_flex
    (0.0, 90.0),     # J5 wrist_roll
    (0.0, 360.0),    # J6 gripper
]

# 软复位初始位 (与固件 soft_reset 一致)
# TODO(bring-up): J2 INIT=45° 与 LIMITS 下界 90° 矛盾, Task 8 真机扫掠后修正
INIT_POSE_DEG: list[float] = [90.0, 45.0, 90.0, 90.0, 0.0, 0.0]


@dataclass
class ZdtConfig:
    channel: str = "can0"
    bitrate: int = 500_000
    timeout_s: float = 0.1
    retries: int = 3
    speed_rpm: float = 60.0
    watchdog_s: float = 0.5
    joint_addrs: list[int] = field(default_factory=lambda: list(JOINT_ADDRS))
    limits: list[tuple[float, float]] = field(
        default_factory=lambda: list(DEFAULT_LIMITS))
