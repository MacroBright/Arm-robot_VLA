"""ZDT 安全层 — 关节模型 + 运动安全状态机.

CAN 直连绕过 STM32 → 失去固件限位开关保护, 本层是**唯一**安全包络:
  · 软限位 clamp (固件表, 抵限位拒绝运动)
  · 低速/小步进硬上限
  · 重力关节 (J2/J3) 臂置需显式确认
  · 状态机: 非 ARMED 不发运动; e_stop 闩锁 STOPPED 直到手动 re_arm
  · e_stop 只广播 0xFE 停止 (不断电不解除力矩, 手臂不掉)
"""
from dataclasses import dataclass
from enum import Enum, auto
from typing import Any, Optional

from .config import (
    DEFAULT_REDUCTION_RATIOS, FIRMWARE_JOINT_LIMITS,
    GRAVITY_JOINTS, JOINT_DIR_POS_BYTE, JOINT_INIT_ANGLE_DEG,
)

# 关节名 (与 massage_robot config 顺序一致)
# 物理构型 (真机确认): J1 底座偏摆 J2 肩抬 J3 肘 J4 腕翻转(roll) J5 腕俯仰(flex) J6 工具滚转
JOINT_NAMES: list[str] = [
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_roll", "wrist_flex", "gripper",
]

# bring-up 运动硬上限
MAX_STEP_DEG: float = 5.0        # 单次微步进最大幅值
MIN_STEP_DEG: float = 0.1
HARD_SPEED_CAP_RPM: float = 30.0  # 试转默认速度上限 (保守)
PULSES_PER_REV: int = 3200        # 16 细分假设 (MStep≠16 时面板另行告警)


class Phase(Enum):
    IDLE = auto()
    ENUMERATED = auto()
    MOTOR_SELECTED = auto()
    ARMED = auto()
    STEPPING = auto()
    STOPPED = auto()


class SafetyError(Exception):
    """安全规则拒绝 (运动请求/臂置等)."""


@dataclass
class MotorState:
    """面板对单个电机的视图 (由 scan/面板更新)."""
    can_id: int
    joint_slot: Optional[int] = None      # 0-based 关节槽 (scheme 裁决后)
    online: bool = False
    fw_ver: Optional[tuple[int, int]] = None
    flags: int = 0
    current_ma: Optional[float] = None
    tracked_deg: float = 0.0              # 命令积分跟踪值
    mstep: Optional[int] = None
    ratio: Optional[float] = None
    params: Any = None
    selected: bool = False
    armed: bool = False
    note: str = ""
    # Commissioning 只读遥测 (Phase 2 全表)
    pos_deg: Optional[float] = None       # 0x36 寄存器角度 (电机轴, 参考值)
    velocity_rpm: Optional[float] = None  # 0x35 转速
    temp_c: Optional[float] = None        # 0x39 温度
    home_flags: int = 0                   # 0x3B 回零/编码器状态


@dataclass
class JointModel:
    index: int                            # 0-based
    name: str
    limits: tuple[float, float]
    reduction_ratio: float
    dir_pos_byte: int                     # 正方向 0xFD dir
    init_deg: float
    gravity: bool


JOINTS: list[JointModel] = [
    JointModel(i, JOINT_NAMES[i], FIRMWARE_JOINT_LIMITS[i],
               DEFAULT_REDUCTION_RATIOS[i], JOINT_DIR_POS_BYTE[i],
               JOINT_INIT_ANGLE_DEG[i], i in GRAVITY_JOINTS)
    for i in range(6)
]


@dataclass
class MovePlan:
    """一次已放行的运动 (面板据此组装 0xFD 帧)."""
    joint_name: str
    delta_deg: float
    target_deg: float
    dir_byte: int
    pulses: int
    speed_rpm: float


class SafetyMachine:
    def __init__(self, pulses_per_rev: int = PULSES_PER_REV):
        self.pulses_per_rev = pulses_per_rev
        self.phase = Phase.IDLE
        self.motors: dict[int, MotorState] = {}
        self.selected_id: Optional[int] = None
        self.step_size_deg: float = 1.0
        self.speed_cap_rpm: float = HARD_SPEED_CAP_RPM

    # ── 枚举/选择 ──
    def set_scan(self, motors: dict[int, MotorState]) -> None:
        self.motors = motors
        self.selected_id = None
        self.phase = Phase.ENUMERATED

    def select(self, can_id: int) -> MotorState:
        if can_id not in self.motors:
            raise SafetyError(f"电机 0x{can_id:02X} 不在枚举结果中")
        for m in self.motors.values():
            m.selected = (m.can_id == can_id)
        self.selected_id = can_id
        if self.phase not in (Phase.STEPPING, Phase.STOPPED):
            self.phase = Phase.MOTOR_SELECTED
        return self.motors[can_id]

    # ── 臂置 ──
    def arm(self, can_id: Optional[int] = None, gravity_confirmed: bool = False) -> MotorState:
        if self.phase in (Phase.STOPPED, Phase.STEPPING):
            raise SafetyError(f"当前状态 {self.phase.name} 不允许臂置")
        cid = can_id or self.selected_id
        if cid is None or cid not in self.motors:
            raise SafetyError("未选择电机")
        m = self.motors[cid]
        if m.joint_slot is not None:
            jm = self._joint_of(m)
            if jm.gravity and not gravity_confirmed:
                raise SafetyError(f"{jm.name}(J{jm.index+1}) 为重力关节, 需显式确认后才可臂置")
        m.armed = True
        self.selected_id = cid
        self.phase = Phase.ARMED
        return m

    def disarm(self) -> None:
        for m in self.motors.values():
            m.armed = False
        if self.phase in (Phase.ARMED, Phase.STEPPING):
            self.phase = Phase.MOTOR_SELECTED

    # ── 运动门禁 ──
    def assert_can_move(self) -> MotorState:
        if self.phase != Phase.ARMED:
            raise SafetyError("必须先枚举 + 选择 + 臂置 (ARMED) 才能运动")
        if self.selected_id is None:
            raise SafetyError("未选择电机")
        m = self.motors[self.selected_id]
        if not m.armed or not m.selected:
            raise SafetyError("电机未臂置")
        if not m.online:
            raise SafetyError("电机不在线")
        return m

    def request_step(self, delta_deg: float) -> MovePlan:
        """请求微步进: 全部安全门禁 + clamp + 方向 + 脉冲换算. 通过则进 STEPPING."""
        m = self.assert_can_move()
        jm = self._joint_of(m)
        # 幅值上限
        if abs(delta_deg) > self.step_size_deg:
            raise SafetyError(f"单次步进 {abs(delta_deg):.1f}° 超过步进幅值 {self.step_size_deg:.1f}°")
        if abs(delta_deg) < MIN_STEP_DEG:
            raise SafetyError(f"步进过小 ({delta_deg:+.2f}°)")
        # 软限位 clamp
        ok, actual, target = self.clamp_delta(jm, m.tracked_deg, delta_deg)
        if not ok or actual == 0:
            raise SafetyError(f"{jm.name} 已抵限位 [{jm.limits[0]:.0f},{jm.limits[1]:.0f}], 拒绝运动")
        # 不变式: clamp 只缩小不放大. |actual| > |delta| 说明 tracked_deg 越出限位
        # (未初始化/状态损坏) — 拒绝而非下发放大后的运动 (防 J2 首步 1°→90° 事故).
        if abs(actual) > abs(delta_deg) + 1e-6:
            raise SafetyError(
                f"{jm.name} 跟踪角 {m.tracked_deg:.1f}° 越出限位 "
                f"[{jm.limits[0]:.0f},{jm.limits[1]:.0f}] — 步进 {delta_deg:+.1f}° 被 clamp "
                f"放大到 {actual:+.1f}°, 拒绝; 需重新枚举复位跟踪角")
        plan = MovePlan(
            joint_name=jm.name,
            delta_deg=actual,
            target_deg=target,
            dir_byte=self.dir_byte_for(jm, actual),
            pulses=self.pulses_for(jm, actual),
            speed_rpm=self.speed_cap_rpm,
        )
        self.phase = Phase.STEPPING
        return plan

    def step_complete(self) -> None:
        if self.phase == Phase.STEPPING:
            self.phase = Phase.ARMED

    # ── e_stop ──
    def e_stop(self) -> None:
        """闩锁 STOPPED (广播 0xFE 由面板执行). 运动被禁直到 re_arm."""
        self.phase = Phase.STOPPED
        for m in self.motors.values():
            m.armed = False

    def re_arm(self, confirmed: bool) -> None:
        """STOPPED → ENUMERATED, 需显式确认."""
        if self.phase != Phase.STOPPED:
            raise SafetyError(f"非 STOPPED 状态 ({self.phase.name}) 无需 re_arm")
        if not confirmed:
            raise SafetyError("re_arm 需显式确认")
        self.phase = Phase.ENUMERATED

    # ── 换算 (纯函数) ──
    @staticmethod
    def clamp_delta(jm: JointModel, tracked_deg: float, delta_deg: float):
        """把 target=tracked+delta clamp 到限位; 返回 (ok, 实际delta, target)."""
        lo, hi = jm.limits
        target = max(lo, min(hi, tracked_deg + delta_deg))
        actual = target - tracked_deg
        return (abs(actual) > 1e-9, actual, target)

    @staticmethod
    def clamp_delta_real(jm: JointModel, real_deg: float, delta_deg: float):
        """B 任务: 基于 0x36 真实位置的软限位 clamp.

        与 clamp_delta 同形, 但 real_deg 来自 0x36 带符号真实位置 (经 CALIB_OFFSETS
        或 CALIB(k,b) 换算), 不是命令积分 tracked_deg. 优势: 外力搬动/失步后仍准确.
        返回 (ok, 实际delta, target). ok=False 表示已在限位边界, delta 被完全吃掉.
        """
        lo, hi = jm.limits
        target = max(lo, min(hi, real_deg + delta_deg))
        actual = target - real_deg
        return (abs(actual) > 1e-9, actual, target)

    @staticmethod
    def drift_check(jm: JointModel, tracked_deg: float, real_deg: float,
                    threshold_deg: float = 2.0) -> tuple[bool, float]:
        """B 任务: 命令积分 vs 0x36 真实位置 漂移守卫.

        返回 (ok, drift). drift = |tracked - real|, ok = drift < threshold.
        超阈值时调用方应: 告警 / 重锚 (anchor_pose) / 急停 (严重时).
        threshold_deg 默认 2° (可按关节精度调). 重力关节建议更严 (1°).
        """
        drift = abs(tracked_deg - real_deg)
        return (drift < threshold_deg, drift)

    @staticmethod
    def dir_byte_for(jm: JointModel, delta_deg: float) -> int:
        """正方向 → 关节 postive_direction dir 字节; 负方向取反."""
        return jm.dir_pos_byte if delta_deg >= 0 else 1 - jm.dir_pos_byte

    def pulses_for(self, jm: JointModel, delta_deg: float) -> int:
        """脉冲 = |Δ| × ratio × pulses_per_rev / 360 (固件公式)."""
        return int(round(abs(delta_deg) * jm.reduction_ratio
                         * self.pulses_per_rev / 360))

    # ── 辅助 ──
    @staticmethod
    def _joint_of(m: MotorState) -> JointModel:
        if m.joint_slot is None:
            raise SafetyError(f"电机 0x{m.can_id:02X} 未映射到关节槽 (先枚举裁决 scheme)")
        return JOINTS[m.joint_slot]
