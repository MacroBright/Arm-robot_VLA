"""scripts/teleop/teleop_config.py — 视觉遥操系统全参数集中配置文件 (Modular Teleoperation Configuration).

【设计说明】
本文件是真机视觉遥操系统的“单一可信配置源 (Single Source of Truth)”。
所有关于【灵敏度档位】、【各关节独立倍率】、【底层电机极限】、【预设作业姿态】与【视觉滤波参数】
均集中配置于此，且每个参数均附带：
  1. 作用对象 (硬件电机 / 算法模块 / UI层)
  2. 物理单位 (RPM, mm/s, rad/s, 度, 归一化比例等)
  3. 调参影响与工程建议范围
如需调整参数，直接编辑本文件 (或对应的 teleop_config.yaml) 即可立即在遥操中生效！
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


# ==============================================================================
# 1. 灵敏度档位系统配置 (Gear & Sensitivity Configuration)
# ==============================================================================

@dataclass
class SingleGearSetting:
    """单个灵敏度档位参数配置."""
    name: str                   # 档位名称 (用于控制台日志与 HUD 提示)
    badge: str                  # 界面角标 (如 "LOW", "MID", "HIGH")
    color: Tuple[int, int, int] # OpenCV BGR 颜色格式 (如 (0, 230, 100)=浅绿, (0, 220, 255)=明黄, (0, 120, 255)=橙红)

    # ── 平移线速度控制 ──
    lin_scale: float
    """
    【笛卡尔平移速度比例 (Linear Speed Scale)】
    - 作用对象: 机械臂末端在基坐标系下的 XYZ 轴平移线速度 (vx, vy, vz).
    - 物理单位: 归一化比例 (0.01 ~ 1.0, 1.0 表示 100% 映射).
    - 调参建议:
        * 低速档: 0.02 ~ 0.04 (3%~4%), 用于精准找穴、微距贴合;
        * 中速档: 0.05 ~ 0.08 (5%~8%), 日常标准推拿揉捏，平顺不急促;
        * 高速档: 0.08 ~ 0.15 (8%~15%), 跨区域大范围换位.
    """

    # ── 姿态旋转角速度控制 ──
    max_omega: float
    """
    【姿态最大角速度上限 (Max Angular Velocity)】
    - 作用对象: 虚拟手势摇杆 (Roll 滚转 / Pitch 俯仰) 输出的最大旋转角速度.
    - 物理单位: rad/s (弧度/秒, 1.0 rad/s ≈ 57.3°/s).
    - 调参建议:
        * 低速档: 0.20 ~ 0.40 rad/s (约 11°~23°/s), 适合精细微调角度;
        * 中速档: 0.50 ~ 0.80 rad/s (约 28°~46°/s), 适合平稳调整手腕姿态;
        * 高速档: 0.90 ~ 1.50 rad/s (约 51°~86°/s), 适合快速翻转手腕.
    """

    # ── 动态扫掠加速度 ──
    gain_xyz: float
    """
    【动态挥手位移增益 (Dynamic Swipe Acceleration Gain)】
    - 作用对象: 快速挥动手掌时产生的位移放大倍率 (鼠标级动力学加速).
    - 物理单位: 无量纲倍数 (1.0 ~ 3.0).
    - 调参建议:
        * 1.0: 纯线性映射 (手移多少臂移多少);
        * 1.1 ~ 1.3: 轻微加速，快速挥手时单次可跨越更远距离，减少反复离合.
    """


@dataclass
class GearConfig:
    """3 档灵敏度变速箱系统全局配置."""
    default_gear: int = 2       # 默认启动档位 (1=低速, 2=中速, 3=高速)
    gear_1_low: SingleGearSetting = field(default_factory=lambda: SingleGearSetting(
        name="1.低速档",
        badge="LOW",
        color=(0, 230, 100),    # 浅绿色
        lin_scale=0.030,        # 3.0%
        max_omega=0.30,         # 0.30 rad/s (17.2°/s)
        gain_xyz=1.0,           # 1.0x 纯线性
    ))
    gear_2_mid: SingleGearSetting = field(default_factory=lambda: SingleGearSetting(
        name="2.中速档",
        badge="MID",
        color=(0, 220, 255),    # 明黄色
        lin_scale=0.050,        # 5.0%
        max_omega=0.70,         # 0.70 rad/s (40.1°/s)
        gain_xyz=1.0,           # 1.0x 线性平稳
    ))
    gear_3_high: SingleGearSetting = field(default_factory=lambda: SingleGearSetting(
        name="3.高速档",
        badge="HIGH",
        color=(0, 120, 255),    # 亮橙色
        lin_scale=0.070,        # 7.0%
        max_omega=1.00,         # 1.00 rad/s (57.3°/s)
        gain_xyz=1.1,           # 1.1x 轻度动态加速
    ))


def build_gear_configs(cfg: TeleopConfig) -> dict:
    """由 TeleopConfig 数据类动态构建 HUD 与速度计算字典 (1=LOW, 2=MID, 3=HIGH)."""
    return {
        1: {
            "name": cfg.gear.gear_1_low.name,
            "badge": cfg.gear.gear_1_low.badge,
            "color": cfg.gear.gear_1_low.color,
            "lin_scale": cfg.gear.gear_1_low.lin_scale,
            "gain_xyz": cfg.gear.gear_1_low.gain_xyz,
            "max_omega": cfg.gear.gear_1_low.max_omega,
        },
        2: {
            "name": cfg.gear.gear_2_mid.name,
            "badge": cfg.gear.gear_2_mid.badge,
            "color": cfg.gear.gear_2_mid.color,
            "lin_scale": cfg.gear.gear_2_mid.lin_scale,
            "gain_xyz": cfg.gear.gear_2_mid.gain_xyz,
            "max_omega": cfg.gear.gear_2_mid.max_omega,
        },
        3: {
            "name": cfg.gear.gear_3_high.name,
            "badge": cfg.gear.gear_3_high.badge,
            "color": cfg.gear.gear_3_high.color,
            "lin_scale": cfg.gear.gear_3_high.lin_scale,
            "gain_xyz": cfg.gear.gear_3_high.gain_xyz,
            "max_omega": cfg.gear.gear_3_high.max_omega,
        },
    }


# ==============================================================================
# 2. 6 关节独立速度补偿配置 (Per-Joint Speed Factor Configuration)
# ==============================================================================

@dataclass
class JointFactorConfig:
    """
    6 关节独立速度倍率与减速比差异补偿配置.

    【背景与原理】
    - J5 (手腕俯仰) 减速比为 27.0:1，输出轴天然比 51:1 的其他轴快 1.88 倍；
    - J1, J2, J3 (臂身大关节，负责空间 XYZ 平移) 和 J4, J6 (手腕滚转) 减速比为 51.0:1；
    - 本配置允许针对每个关节独立分配速度权重，保持整臂协同运作的最佳体感。
    """
    j1_base_yaw: float = 2.0
    """J1 基座回转 / X 轴横向摆臂倍率 (默认 2.0x, 补偿 51:1 减速比)"""

    j2_shoulder_pitch: float = 2.0
    """J2 大臂俯仰 / 前后上下主推力倍率 (默认 2.0x, 补偿 51:1 减速比)"""

    j3_elbow_pitch: float = 2.0
    """J3 小臂俯仰 / 空间伸缩主推力倍率 (默认 2.0x, 补偿 51:1 减速比)"""

    j4_wrist_roll_1: float = 2.0
    """J4 腕部滚转 1 轴倍率 (默认 2.0x, 补偿 51:1 减速比)"""

    j5_wrist_pitch: float = 1.0
    """J5 手腕俯仰 轴倍率 (默认 1.0x 基准, 27:1 减速比响应合适)"""

    j6_wrist_roll_2: float = 2.0
    """J6 腕部滚转 2 / 末端自转倍率 (默认 2.0x, 补偿 51:1 减速比)"""

    def as_list(self) -> List[float]:
        """返回 6 元素浮点列表 [J1, J2, J3, J4, J5, J6]."""
        return [
            self.j1_base_yaw,
            self.j2_shoulder_pitch,
            self.j3_elbow_pitch,
            self.j4_wrist_roll_1,
            self.j5_wrist_pitch,
            self.j6_wrist_roll_2,
        ]


# ==============================================================================
# 3. 底层电机驱动与安全限速配置 (Motor Limits & Safety Configuration)
# ==============================================================================

@dataclass
class MotorLimitConfig:
    """底层闭环步进驱动器 (Emm42 V5.0) 与笛卡尔控制器安全极限."""

    speed_rpm: float = 2800.0
    """
    【0xFD CAN 报文电机轴最高转速上限 (Max Motor Shaft RPM)】
    - 作用对象: 驱动器底层 0xFD 相对位置脉冲下发报文中的速度字段 (Speed RPM).
    - 物理单位: RPM (转/分钟).
    - 说明: 42 步进电机最高额定转速 3000 RPM，2800 RPM 留有安全余量，保证高速运动不失步.
    """

    position_acc: int = 0
    """
    【电机加速度启动档位 (Position Acceleration Step)】
    - 作用对象: 0xFD 报文中的加速度字段 acc.
    - 物理含义: 0 = 无加减速延迟 (直冲最高速, 50ms 周期运动最佳响应); 1~255 为梯形加减速.
    - 说明: 必须设为 0，以彻底消除 1.8s 启动爬坡滞后，保证极速跟手.
    """

    max_vel_mm_s: float = 600.0
    """
    【笛卡尔末端平移最大线速度硬限幅 (Max Cartesian Linear Velocity)】
    - 作用对象: CartesianController.step() 输入线速度向量范数 ||v||.
    - 物理单位: mm/s (毫米/秒).
    """

    max_ang_rad_s: float = 10.0
    """
    【笛卡尔末端旋转最大角速度硬限幅 (Max Cartesian Angular Velocity)】
    - 作用对象: CartesianController.step() 输入角速度向量范数 ||w||.
    - 物理单位: rad/s (弧度/秒).
    """

    max_joint_vel_deg_s: float = 540.0
    """
    【关节输出轴最大角速度 (Max Joint Output Velocity)】
    - 作用对象: 每个控制周期 (50ms) 内允许的单轴最大角速度.
    - 物理单位: deg/s (度/秒, 540°/s = 1.5 圈/秒).
    """

    max_joint_acc_deg_s2: float = 2000.0
    """
    【关节输出轴最大角加速度 (Max Joint Acceleration)】
    - 作用对象: 连续两帧之间允许的最大关节速度突变率.
    - 物理单位: deg/s² (度/秒平方).
    """

    max_dq_deg: float = 30.0
    """
    【单步控制周期最大关节跳变步长 (Max dq per Step)】
    - 作用对象: Damped Least Squares 数值逆解单步最大允许角度增益.
    - 物理单位: deg/step (50ms 内最大允许转动 30 度).
    """


# ==============================================================================
# 4. 预设姿态与关节限位配置 (Preset Poses & Limit Configuration)
# ==============================================================================

@dataclass
class PresetPoseConfig:
    """预设标定作业姿态与固件限位."""

    ready_pose_deg: List[float] = field(default_factory=lambda: [0.0, 60.0, 50.0, 0.0, 120.0, 0.0])
    """
    【按摩准备姿态 (READY Pose)】
    - 关节角度: [J1=0°, J2=60°, J3=50°, J4=0°, J5=120°, J6=0°].
    - 说明: 按 [R] 键时 6 轴同步平缓运动到位，作为推拿作业的安全基准起点.
    """

    home_pose_deg: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    """
    【上电初始复位姿态 (HOME Pose)】
    - 关节角度: 全 0 度 (机械臂开机垂直直立位).
    - 说明: 按 [H] 或 [O] 键时 6 轴同步平缓回零.
    """

    ready_speed_rpm: float = 100.0
    """
    【姿态同步运动时的安全转速 (Safe Preset Move RPM)】
    - 物理单位: RPM (电机轴 100 RPM，输出轴约 11.7°/s，确保平稳优雅无冲撞).
    """


# ==============================================================================
# 5. 视觉跟踪与手势滤波配置 (Vision Tracking & Filter Configuration)
# ==============================================================================

@dataclass
class VisionFilterConfig:
    """RealSense D455 视觉追踪、1€ 滤波器与控制死区配置."""

    # ── 3D 手腕位置 1€ 滤波参数 ──
    pts_min_cutoff: float = 1.5
    """
    【手腕位置滤波最小截止频率 (Min Cutoff Frequency)】
    - 作用对象: 手腕 3D 坐标 (X, Y, Z mm).
    - 物理单位: Hz. 越小静止时越平滑防抖，越大起步越灵敏 (建议 1.0 ~ 2.0).
    """

    pts_beta: float = 0.08
    """
    【手腕位置滤波速度动态系数 (Speed Coefficient Beta)】
    - 作用对象: 快速移动时的动态频宽拓展.
    - 调参建议: 0.05 ~ 0.12 (快速挥手时近乎零相位滞后).
    """

    # ── 3D 手掌姿态 1€ 滤波参数 (李代数正交平滑) ──
    rot_min_cutoff: float = 1.0
    """【掌面姿态滤波最小截止频率 (Min Cutoff Hz)】 (建议 0.8 ~ 1.5)"""

    rot_beta: float = 0.05
    """【掌面姿态滤波速度动态系数 (Beta)】 (建议 0.03 ~ 0.08)"""

    # ── 死区过滤 (Deadbands) ──
    deadband_angle_deg: float = 5.0
    """
    【虚拟摇杆中立死区角度 (Joystick Neutral Deadband)】
    - 作用对象: 手腕倾斜角度 (|angle| <= 5.0° 时判定为中立，保持当前姿态锁定).
    - 物理单位: deg (度).
    """

    deadband_vel_mm_s: float = 10.0
    """
    【平移微颤消除死区 (Translation Jitter Deadband)】
    - 作用对象: 消除人体生理手颤 (低于 10 mm/s 视为静止，不产生漂移).
    - 物理单位: mm/s.
    """


# ==============================================================================
# 6. 灵巧手控制与滤波配置 (Dexterous Hand Configuration)
# ==============================================================================

@dataclass
class HandConfig:
    """LEAP Hand 16-DOF 灵巧手硬件与视觉映射配置."""

    port: str = "/dev/ttyUSB0"
    """
    【灵巧手 Dynamixel 串口路径】
    - 物理硬件: USB 转 TTL 串口适配器 (默认 /dev/ttyUSB0, 亦可自动搜索).
    """

    kP: int = 600
    """【位置环比例增益 kP】 (默认 600, 侧摆轴 0/4/8 自动乘以 0.75)"""

    kI: int = 0
    """【位置环积分增益 kI】 (默认 0)"""

    kD: int = 200
    """【位置环微分增益 kD】 (默认 200, 侧摆轴 0/4/8 自动乘以 0.75)"""

    curr_lim: int = 350
    """
    【电机最大电流限制 (Current Limit)】
    - 物理单位: mA (毫安, 默认 350mA, 防止揉捏过力堵转发热).
    """

    filter_min_cutoff: float = 1.0
    """
    【手指 16 关节 1€ 滤波基准截止频率 (Min Cutoff)】
    - 物理单位: Hz (默认 1.0 Hz, 抑制指尖抖动).
    """

    filter_beta: float = 0.02
    """
    【手指 16 关节 1€ 滤波速度响应系数 (Beta)】
    - 调参建议: 0.01 ~ 0.05 (手速越快截止频率越高，兼顾抗噪与灵敏).
    """

    source_mode: int = 2
    """
    【3D 关键点来源模式】
    - 0: HaMeR 3D (MANO 回归真 3D);
    - 1: MediaPipe World 3D (规范 3D 模型);
    - 2: MediaPipe Pseudo 3D (原伪 3D 深度，实机跟手性与握拳效果最佳, 默认).
    """

    bend_threshold: float = 0.20
    """【手指弯曲判定阈值 (Bend Threshold)】 (rad, 默认 0.20 rad)"""

    hand_type: str = "right"
    """【物理控制目标手 (Hand Target)】 ('right' / 'left' / 'first')"""


# ==============================================================================
# 7. 全局汇总配置主类 (Master Teleoperation Configuration)
# ==============================================================================

@dataclass
class TeleopConfig:
    """视觉遥操系统全参数主配置容器."""
    gear: GearConfig = field(default_factory=GearConfig)
    joint_factor: JointFactorConfig = field(default_factory=JointFactorConfig)
    motor: MotorLimitConfig = field(default_factory=MotorLimitConfig)
    pose: PresetPoseConfig = field(default_factory=PresetPoseConfig)
    vision: VisionFilterConfig = field(default_factory=VisionFilterConfig)
    hand: HandConfig = field(default_factory=HandConfig)

    def to_dict(self) -> Dict[str, Any]:
        """转换为标准字典 (便于序列化为 JSON / YAML)."""
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "TeleopConfig":
        """从字典还原强类型 TeleopConfig 对象."""
        cfg = cls()
        if "gear" in data and isinstance(data["gear"], dict):
            g = data["gear"]
            if "default_gear" in g:
                cfg.gear.default_gear = int(g["default_gear"])
            for g_key in ("gear_1_low", "gear_2_mid", "gear_3_high"):
                if g_key in g and isinstance(g[g_key], dict):
                    setattr(cfg.gear, g_key, SingleGearSetting(**g[g_key]))

        if "joint_factor" in data and isinstance(data["joint_factor"], dict):
            cfg.joint_factor = JointFactorConfig(**data["joint_factor"])

        if "motor" in data and isinstance(data["motor"], dict):
            cfg.motor = MotorLimitConfig(**data["motor"])

        if "pose" in data and isinstance(data["pose"], dict):
            cfg.pose = PresetPoseConfig(**data["pose"])

        if "vision" in data and isinstance(data["vision"], dict):
            cfg.vision = VisionFilterConfig(**data["vision"])

        if "hand" in data and isinstance(data["hand"], dict):
            cfg.hand = HandConfig(**data["hand"])

        return cfg

    def validate(self) -> List[str]:
        """
        全面验证配置参数的物理合理性与安全边界 (Pre-Flight Safety Validator).

        - 若存在严重安全隐患 (如负速度、转速超限、关节倍率异常)，抛出 ValueError 中断启动；
        - 若存在调优建议/提示项，返回警告信息列表 (用于启动时在控制台黄色高亮提示).
        """
        warnings: List[str] = []

        # 1. 档位系统安全校验
        for g_name, g in [("1.低速档", self.gear.gear_1_low),
                          ("2.中速档", self.gear.gear_2_mid),
                          ("3.高速档", self.gear.gear_3_high)]:
            if not (0.001 <= g.lin_scale <= 3.0):
                raise ValueError(f"[{g_name}] 平移比例 lin_scale={g.lin_scale} 超出安全范围 [0.001, 3.0]")
            if not (0.01 <= g.max_omega <= 10.0):
                raise ValueError(f"[{g_name}] 姿态角速度 max_omega={g.max_omega} rad/s 超出安全范围 [0.01, 10.0]")
            if not (1.0 <= g.gain_xyz <= 5.0):
                raise ValueError(f"[{g_name}] 动态增益 gain_xyz={g.gain_xyz} 超出安全范围 [1.0, 5.0]")
            if g.lin_scale > 1.0:
                warnings.append(f"[{g_name}] 平移比例 lin_scale={g.lin_scale:.2f} (>100%) 处于超高速档，请在空旷区域谨慎操作")

        if not (self.gear.gear_1_low.lin_scale <= self.gear.gear_2_mid.lin_scale <= self.gear.gear_3_high.lin_scale):
            warnings.append("档位平移速度非单调递增 (建议 低速档 <= 中速档 <= 高速档)")

        # 2. 6 关节独立倍率安全校验
        factors = self.joint_factor.as_list()
        for idx, f in enumerate(factors, start=1):
            if not (0.1 <= f <= 5.0):
                raise ValueError(f"[关节 J{idx}] 速度倍率 factor={f} 超出安全范围 [0.1, 5.0]")

        # 3. 电机与控制器安全极限校验
        if not (50.0 <= self.motor.speed_rpm <= 3000.0):
            raise ValueError(f"[电机限速] speed_rpm={self.motor.speed_rpm} RPM 超出硬件极限 [50, 3000]")
        if self.motor.position_acc not in range(256):
            raise ValueError(f"[电机加速度] position_acc={self.motor.position_acc} 必须在 0~255 之间")
        if not (10.0 <= self.motor.max_vel_mm_s <= 1000.0):
            raise ValueError(f"[笛卡尔线速度] max_vel_mm_s={self.motor.max_vel_mm_s} mm/s 超出安全范围 [10, 1000]")
        if not (0.5 <= self.motor.max_ang_rad_s <= 15.0):
            raise ValueError(f"[笛卡尔角速度] max_ang_rad_s={self.motor.max_ang_rad_s} rad/s 超出安全范围 [0.5, 15]")
        if not (1.0 <= self.motor.max_dq_deg <= 45.0):
            raise ValueError(f"[单步角度限制] max_dq_deg={self.motor.max_dq_deg} deg/step 超出安全范围 [1, 45]")

        # 4. 预设姿态维度与范围校验
        if len(self.pose.ready_pose_deg) != 6:
            raise ValueError(f"[预设姿态] READY 姿态必须包含 6 关节角度，当前为 {len(self.pose.ready_pose_deg)} 项")
        if len(self.pose.home_pose_deg) != 6:
            raise ValueError(f"[预设姿态] HOME 姿态必须包含 6 关节角度，当前为 {len(self.pose.home_pose_deg)} 项")

        # 5. 视觉滤波参数校验
        if not (0.1 <= self.vision.pts_min_cutoff <= 10.0):
            raise ValueError(f"[视觉滤波] pts_min_cutoff={self.vision.pts_min_cutoff} Hz 超出有效范围 [0.1, 10.0]")
        if not (0.001 <= self.vision.pts_beta <= 1.0):
            raise ValueError(f"[视觉滤波] pts_beta={self.vision.pts_beta} 超出有效范围 [0.001, 1.0]")
        if not (0.5 <= self.vision.deadband_angle_deg <= 20.0):
            raise ValueError(f"[摇杆死区] deadband_angle_deg={self.vision.deadband_angle_deg}° 超出有效范围 [0.5, 20.0]")

        # 6. 灵巧手参数校验
        if not (50 <= self.hand.kP <= 1500):
            raise ValueError(f"[灵巧手] kP={self.hand.kP} 超出安全范围 [50, 1500]")
        if not (10 <= self.hand.kD <= 600):
            raise ValueError(f"[灵巧手] kD={self.hand.kD} 超出安全范围 [10, 600]")
        if not (50 <= self.hand.curr_lim <= 800):
            raise ValueError(f"[灵巧手电流] curr_lim={self.hand.curr_lim} mA 超出安全范围 [50, 800]")
        if not (0.1 <= self.hand.filter_min_cutoff <= 10.0):
            raise ValueError(f"[灵巧手滤波] filter_min_cutoff={self.hand.filter_min_cutoff} Hz 超出有效范围 [0.1, 10.0]")
        if self.hand.source_mode not in (0, 1, 2):
            raise ValueError(f"[灵巧手源] source_mode={self.hand.source_mode} 必须为 0(HAMER), 1(WORLD), 2(PSEUDO)")
        if self.hand.hand_type not in ("right", "left", "first"):
            raise ValueError(f"[灵巧手目标] hand_type={self.hand.hand_type} 必须为 'right', 'left' 或 'first'")

        return warnings

    @classmethod
    def load(cls, file_path: str | Path, validate: bool = True) -> "TeleopConfig":
        """从 .yaml, .json 或 .py 配置文件加载配置 (如果文件不存在则返回默认配置)."""
        path = Path(file_path)
        cfg: Optional[TeleopConfig] = None
        if path.exists():
            suffix = path.suffix.lower()
            if suffix in (".yaml", ".yml"):
                try:
                    import yaml
                    with open(path, "r", encoding="utf-8") as f:
                        content = yaml.safe_load(f)
                        if isinstance(content, dict):
                            cfg = cls.from_dict(content)
                except Exception:
                    # 纯 Python 标准库 YAML 轻量解析器 (无需 PyYAML 第三方依赖)
                    try:
                        with open(path, "r", encoding="utf-8") as f:
                            content = _simple_yaml_parse(f.read())
                            if isinstance(content, dict) and content:
                                cfg = cls.from_dict(content)
                    except Exception as err:
                        print(f"[配置警告] 加载 YAML 失败 ({err})，使用默认配置")
            elif suffix == ".json":
                try:
                    import json
                    with open(path, "r", encoding="utf-8") as f:
                        content = json.load(f)
                        if isinstance(content, dict):
                            cfg = cls.from_dict(content)
                except Exception as e:
                    print(f"[配置警告] 加载 JSON 失败 ({e})，使用默认配置")

        if cfg is None:
            cfg = cls()

        if validate:
            warnings = cfg.validate()
            for w in warnings:
                print(f"[配置安全提示] \033[93m{w}\033[0m")

        return cfg

    def save_yaml(self, file_path: str | Path) -> None:
        """保存为 YAML 格式配置文件 (纯标准库支持)."""
        path = Path(file_path)
        try:
            import yaml
            with open(path, "w", encoding="utf-8") as f:
                yaml.dump(self.to_dict(), f, allow_unicode=True, default_flow_style=False, sort_keys=False)
        except Exception:
            with open(path, "w", encoding="utf-8") as f:
                f.write(_simple_yaml_dump(self.to_dict()))


def _simple_yaml_parse(text: str) -> dict:
    """基于 Python 标准库的轻量级嵌套 YAML 解析器 (零依赖)."""
    import ast
    root: dict = {}
    stack: list = [(0, root)]
    for raw_line in text.splitlines():
        line = raw_line.split("#")[0].rstrip()
        if not line.strip():
            continue
        indent = len(raw_line) - len(raw_line.lstrip())
        while len(stack) > 1 and indent <= stack[-1][0]:
            stack.pop()

        curr_dict = stack[-1][1]
        line = line.strip()
        if ":" in line:
            key, val = line.split(":", 1)
            key = key.strip()
            val = val.strip()
            if not val:
                new_dict: dict = {}
                curr_dict[key] = new_dict
                stack.append((indent, new_dict))
            else:
                if val.lower() == "true":
                    parsed_val: Any = True
                elif val.lower() == "false":
                    parsed_val = False
                elif val.lower() in ("null", "none"):
                    parsed_val = None
                else:
                    try:
                        parsed_val = ast.literal_eval(val)
                    except Exception:
                        parsed_val = val
                curr_dict[key] = parsed_val
    return root


def _simple_yaml_dump(data: dict, indent: int = 0) -> str:
    """基于 Python 标准库的轻量级 YAML 序列化器 (零依赖)."""
    lines = []
    prefix = "  " * indent
    for k, v in data.items():
        if isinstance(v, dict):
            lines.append(f"{prefix}{k}:")
            lines.append(_simple_yaml_dump(v, indent + 1))
        elif isinstance(v, (list, tuple)):
            lines.append(f"{prefix}{k}: {list(v)}")
        elif isinstance(v, str):
            lines.append(f'{prefix}{k}: "{v}"')
        elif isinstance(v, bool):
            lines.append(f"{prefix}{k}: {'true' if v else 'false'}")
        else:
            lines.append(f"{prefix}{k}: {v}")
    return "\n".join(lines)


# 默认单例配置实例 (可以直接 import 使用)
DEFAULT_TELEOP_CONFIG = TeleopConfig()
