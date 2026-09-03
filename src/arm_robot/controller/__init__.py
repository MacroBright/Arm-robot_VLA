"""High-level arm controller, safety gates, and state machine."""

from .config import (
    CALIB_OFFSETS,
    DEFAULT_REDUCTION_RATIOS,
    FIRMWARE_JOINT_LIMITS,
    JOINT_ADDRS,
    JOINT_INIT_ANGLE_DEG,
    ZdtConfig,
)
from .controller import ZdtController
from .params import (
    CAN_BAUD_LABELS,
    CHECKSUM_LABELS,
    F_CHANGE_ID,
    F_READ_PARAMS,
    F_WRITE_PARAMS,
    MSTEP_LABELS,
    RESPONSE_LABELS,
)
from .safety import (
    JOINTS,
    JOINT_NAMES,
    JointModel,
    MotorState,
    Phase,
    RobotPhase,
    RobotStateMachine,
    SafetyError,
    SafetyMachine,
    verify_enumeration,
)

# Friendly aliases
ArmState = RobotPhase

__all__ = [
    "ArmState",
    "CALIB_OFFSETS",
    "CAN_BAUD_LABELS",
    "CHECKSUM_LABELS",
    "DEFAULT_REDUCTION_RATIOS",
    "FIRMWARE_JOINT_LIMITS",
    "F_CHANGE_ID",
    "F_READ_PARAMS",
    "F_WRITE_PARAMS",
    "JOINTS",
    "JOINT_ADDRS",
    "JOINT_INIT_ANGLE_DEG",
    "JOINT_NAMES",
    "JointModel",
    "MSTEP_LABELS",
    "MotorState",
    "Phase",
    "RESPONSE_LABELS",
    "RobotPhase",
    "RobotStateMachine",
    "SafetyError",
    "SafetyMachine",
    "ZdtConfig",
    "ZdtController",
    "verify_enumeration",
]
