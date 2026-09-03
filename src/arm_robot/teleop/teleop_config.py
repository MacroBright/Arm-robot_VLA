"""Arm-robot_VLA/scripts/teleop/teleop_config.py — 视觉遥操系统全参数集中配置文件 (Modular Teleoperation Configuration).

【设计说明】
本文件是真机视觉遥操系统的“单一可信配置源 (Single Source of Truth)”。
所有关于【灵敏度档位】、【各关节独立倍率】、【机械臂关节限位】、【底层电机极限】、【预设作业姿态】、【视觉滤波】与【灵巧手参数】
均集中配置于此。
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
    color: Tuple[int, int, int] # OpenCV BGR 颜色格式

    lin_scale: float
    max_omega: float
    gain_xyz: float


@dataclass
class GearConfig:
    """3 档灵敏度变速箱系统全局配置."""
    default_gear: int = 2       # 默认启动档位 (1=低速, 2=中速, 3=高速)
    gear_1_low: SingleGearSetting = field(default_factory=lambda: SingleGearSetting(
        name="1.低速档",
        badge="LOW",
        color=(0, 230, 100),    # 浅绿色
        lin_scale=0.030,        # 3.0%
        max_omega=0.10,         # 0.10 rad/s (5.7°/s)
        gain_xyz=1.0,           # 1.0x 纯线性
    ))
    gear_2_mid: SingleGearSetting = field(default_factory=lambda: SingleGearSetting(
        name="2.中速档",
        badge="MID",
        color=(0, 220, 255),    # 明黄色
        lin_scale=0.080,        # 8.0%
        max_omega=0.20,         # 0.20 rad/s (11.5°/s)
        gain_xyz=1.0,           # 1.0x 线性平稳
    ))
    gear_3_high: SingleGearSetting = field(default_factory=lambda: SingleGearSetting(
        name="3.高速档",
        badge="HIGH",
        color=(0, 120, 255),    # 亮橙色
        lin_scale=0.085,        # 8.5%
        max_omega=0.25,         # 0.25 rad/s (14.3°/s)
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
            "max_omega": cfg.gear.gear_1_low.max_omega,
            "gain_xyz": cfg.gear.gear_1_low.gain_xyz,
        },
        2: {
            "name": cfg.gear.gear_2_mid.name,
            "badge": cfg.gear.gear_2_mid.badge,
            "color": cfg.gear.gear_2_mid.color,
            "lin_scale": cfg.gear.gear_2_mid.lin_scale,
            "max_omega": cfg.gear.gear_2_mid.max_omega,
            "gain_xyz": cfg.gear.gear_2_mid.gain_xyz,
        },
        3: {
            "name": cfg.gear.gear_3_high.name,
            "badge": cfg.gear.gear_3_high.badge,
            "color": cfg.gear.gear_3_high.color,
            "lin_scale": cfg.gear.gear_3_high.lin_scale,
            "max_omega": cfg.gear.gear_3_high.max_omega,
            "gain_xyz": cfg.gear.gear_3_high.gain_xyz,
        },
    }


# ==============================================================================
# 2. 6 关节独立速度补偿配置 (Per-Joint Speed Factor Configuration)
# ==============================================================================

@dataclass
class JointFactorConfig:
    """6 关节独立速度响应倍率 (针对不同减速比实现手感一致性)."""
    j1_base_yaw: float = 2.5
    j2_shoulder_pitch: float = 2.5
    j3_elbow_pitch: float = 2.5
    j4_wrist_roll_1: float = 1.5
    j5_wrist_pitch: float = 1.0
    j6_wrist_roll_2: float = 1.0

    def as_list(self) -> List[float]:
        return [
            float(self.j1_base_yaw),
            float(self.j2_shoulder_pitch),
            float(self.j3_elbow_pitch),
            float(self.j4_wrist_roll_1),
            float(self.j5_wrist_pitch),
            float(self.j6_wrist_roll_2),
        ]


# ==============================================================================
# 3. 机械臂 6 关节软件限位与缓冲配置 (Joint Limits & Safety Margin in Degrees)
# ==============================================================================

@dataclass
class JointLimitsConfig:
    """机械臂 6 关节物理与软件安全限位配置 (单位: 度 / deg)."""
    j1_base_yaw: list[float] = field(default_factory=lambda: [-1.0, 360.0])
    j2_shoulder_pitch: list[float] = field(default_factory=lambda: [-1.0, 150.0])
    j3_elbow_pitch: list[float] = field(default_factory=lambda: [-1.0, 120.0])
    j4_wrist_roll_1: list[float] = field(default_factory=lambda: [-90.0, 90.0])
    j5_wrist_pitch: list[float] = field(default_factory=lambda: [-1.0, 180.0])
    j6_wrist_roll_2: list[float] = field(default_factory=lambda: [-1.0, 360.0])
    joint_limit_margin_deg: float = 2.0

    def as_list(self) -> list[tuple[float, float]]:
        return [
            (float(self.j1_base_yaw[0]), float(self.j1_base_yaw[1])),
            (float(self.j2_shoulder_pitch[0]), float(self.j2_shoulder_pitch[1])),
            (float(self.j3_elbow_pitch[0]), float(self.j3_elbow_pitch[1])),
            (float(self.j4_wrist_roll_1[0]), float(self.j4_wrist_roll_1[1])),
            (float(self.j5_wrist_pitch[0]), float(self.j5_wrist_pitch[1])),
            (float(self.j6_wrist_roll_2[0]), float(self.j6_wrist_roll_2[1])),
        ]


# ==============================================================================
# 4. 底层电机驱动与安全限速配置 (Motor Limits & Safety Configuration)
# ==============================================================================

@dataclass
class MotorLimitConfig:
    """底层闭环步进驱动器 (Emm42 V5.0) 与笛卡尔控制器安全极限."""
    speed_rpm: float = 2000.0
    position_acc: int = 0
    max_vel_mm_s: float = 600.0
    max_ang_rad_s: float = 10.0
    max_joint_vel_deg_s: float = 540.0
    max_joint_acc_deg_s2: float = 2000.0
    max_dq_deg: float = 30.0


# ==============================================================================
# 5. 预设作业与复位姿态配置 (Preset Poses Configuration)
# ==============================================================================

@dataclass
class PresetPoseConfig:
    ready_pose_deg: List[float] = field(default_factory=lambda: [0.0, 75.0, 55.0, 0.0, 130.0, 0.0])
    home_pose_deg: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    ready_speed_rpm: float = 100.0


# ==============================================================================
# 6. 视觉跟踪与滤波配置 (Vision Tracking & Filter Configuration)
# ==============================================================================

@dataclass
class VisionFilterConfig:
    pts_min_cutoff: float = 1.5
    pts_beta: float = 0.08
    rot_min_cutoff: float = 1.0
    rot_beta: float = 0.05
    deadband_angle_deg: float = 5.0
    deadband_vel_mm_s: float = 10.0


# ==============================================================================
# 7. 灵巧手控制与安全配置 (Dexterous Hand Configuration)
# ==============================================================================

@dataclass
class HandConfig:
    port: str = "/dev/ttyUSB0"
    kP: int = 300
    kI: int = 0
    kD: int = 100
    curr_lim: int = 150
    source_mode: int = 2
    filter_min_cutoff: float = 1.0
    filter_beta: float = 0.02
    bend_threshold: float = 0.20
    hand_type: str = "right"


# ==============================================================================
# 8. 全局汇总配置主类 (Master Teleoperation Configuration)
# ==============================================================================

@dataclass
class TeleopConfig:
    gear: GearConfig = field(default_factory=GearConfig)
    joint_factor: JointFactorConfig = field(default_factory=JointFactorConfig)
    joint_limits: JointLimitsConfig = field(default_factory=JointLimitsConfig)
    motor: MotorLimitConfig = field(default_factory=MotorLimitConfig)
    pose: PresetPoseConfig = field(default_factory=PresetPoseConfig)
    vision: VisionFilterConfig = field(default_factory=VisionFilterConfig)
    hand: HandConfig = field(default_factory=HandConfig)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def default(cls) -> "TeleopConfig":
        return cls()

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "TeleopConfig":
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

        if "joint_limits" in data and isinstance(data["joint_limits"], dict):
            jl = data["joint_limits"]
            cfg.joint_limits = JointLimitsConfig(
                j1_base_yaw=list(jl.get("j1_base_yaw", [-1.0, 360.0])),
                j2_shoulder_pitch=list(jl.get("j2_shoulder_pitch", [-1.0, 150.0])),
                j3_elbow_pitch=list(jl.get("j3_elbow_pitch", [-1.0, 120.0])),
                j4_wrist_roll_1=list(jl.get("j4_wrist_roll_1", [-90.0, 90.0])),
                j5_wrist_pitch=list(jl.get("j5_wrist_pitch", [-1.0, 180.0])),
                j6_wrist_roll_2=list(jl.get("j6_wrist_roll_2", [-1.0, 360.0])),
                joint_limit_margin_deg=float(jl.get("joint_limit_margin_deg", 2.0)),
            )

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

        # 2. 6 关节独立倍率安全校验
        factors = self.joint_factor.as_list()
        for idx, f in enumerate(factors, start=1):
            if not (0.1 <= f <= 5.0):
                raise ValueError(f"[关节 J{idx}] 速度倍率 factor={f} 超出安全范围 [0.1, 5.0]")

        # 3. 机械臂 6 关节限位安全校验
        limits_list = self.joint_limits.as_list()
        for idx, (lo, hi) in enumerate(limits_list, start=1):
            if lo >= hi:
                raise ValueError(f"[关节限位 J{idx}] 下限 lo={lo}° 必须严格小于上限 hi={hi}°")
        if not (0.0 <= self.joint_limits.joint_limit_margin_deg <= 20.0):
            raise ValueError(f"[关节限位] 减速缓冲边界 margin={self.joint_limits.joint_limit_margin_deg}° 超出有效范围 [0.0, 20.0]")

        # 4. 电机与控制器安全极限校验
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

        # 5. 预设姿态维度与限位校验
        if len(self.pose.ready_pose_deg) != 6:
            raise ValueError(f"[预设姿态] READY 姿态必须包含 6 关节角度，当前为 {len(self.pose.ready_pose_deg)} 项")
        if len(self.pose.home_pose_deg) != 6:
            raise ValueError(f"[预设姿态] HOME 姿态必须包含 6 关节角度，当前为 {len(self.pose.home_pose_deg)} 项")

        for idx, (ang, (lo, hi)) in enumerate(zip(self.pose.ready_pose_deg, limits_list), start=1):
            if not (lo <= ang <= hi):
                raise ValueError(f"[预设姿态] READY 姿态 J{idx}={ang}° 超出设置的关节限位 [{lo}°, {hi}°]")
        for idx, (ang, (lo, hi)) in enumerate(zip(self.pose.home_pose_deg, limits_list), start=1):
            if not (lo <= ang <= hi):
                raise ValueError(f"[预设姿态] HOME 姿态 J{idx}={ang}° 超出设置的关节限位 [{lo}°, {hi}°]")

        # 6. 视觉滤波参数校验
        if not (0.1 <= self.vision.pts_min_cutoff <= 10.0):
            raise ValueError(f"[视觉滤波] pts_min_cutoff={self.vision.pts_min_cutoff} Hz 超出有效范围 [0.1, 10.0]")
        if not (0.001 <= self.vision.pts_beta <= 1.0):
            raise ValueError(f"[视觉滤波] pts_beta={self.vision.pts_beta} 超出有效范围 [0.001, 1.0]")
        if not (0.5 <= self.vision.deadband_angle_deg <= 20.0):
            raise ValueError(f"[摇杆死区] deadband_angle_deg={self.vision.deadband_angle_deg}° 超出有效范围 [0.5, 20.0]")

        # 7. 灵巧手参数校验
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

    def save(self, file_path: str | Path) -> None:
        """Save configuration dictionary to a YAML (or JSON fallback) file."""
        p = Path(file_path)
        p.parent.mkdir(parents=True, exist_ok=True)
        data = self.to_dict()
        try:
            import yaml
            with open(p, "w", encoding="utf-8") as f:
                yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
        except ImportError:
            import json
            with open(p, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)

    def save_yaml(self, file_path: str | Path) -> None:
        self.save(file_path)


    @classmethod

    def load(cls, file_path: str | Path, validate: bool = True) -> "TeleopConfig":
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
                except ImportError:
                    try:
                        import json
                        with open(path, "r", encoding="utf-8") as f:
                            content = json.load(f)
                            if isinstance(content, dict):
                                cfg = cls.from_dict(content)
                    except Exception:
                        pass
                except Exception:
                    pass

            elif suffix == ".json":
                try:
                    import json
                    with open(path, "r", encoding="utf-8") as f:
                        content = json.load(f)
                        if isinstance(content, dict):
                            cfg = cls.from_dict(content)
                except Exception:
                    pass

        if cfg is None:
            cfg = cls()

        if validate:
            warnings = cfg.validate()
            for w in warnings:
                print(f"[配置安全提示] \033[93m{w}\033[0m")

        return cfg


DEFAULT_TELEOP_CONFIG = TeleopConfig.default()
