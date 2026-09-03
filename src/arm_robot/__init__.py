"""Arm-robot_VLA — 6-DOF Robotic Arm Control & LeRobot Embodied AI Platform."""

__version__ = "0.2.0"

# Expose primary driver, controller, and kinematics interfaces
from .controller import (
    ArmState,
    FIRMWARE_JOINT_LIMITS,
    JOINT_ADDRS,
    JOINT_INIT_ANGLE_DEG,
    JOINT_NAMES,
    RobotStateMachine,
    SafetyError,
    ZdtConfig,
    ZdtController,
)
from .driver import (
    CanFrame,
    CanTransport,
    SocketCanTransport,
    ZdtBus,
    ZdtDriver,
    ZdtDriverError,
    scan_bus,
)
from .kinematics import (
    CartesianCommand,
    CartesianController,
    JointState,
    fk_mdh,
    geometric_jacobian,
    ik_dls,
)

__all__ = [
    "ArmState",
    "CanFrame",
    "CanTransport",
    "CartesianCommand",
    "CartesianController",
    "FIRMWARE_JOINT_LIMITS",
    "JOINT_ADDRS",
    "JOINT_INIT_ANGLE_DEG",
    "JOINT_NAMES",
    "JointState",
    "RobotStateMachine",
    "SafetyError",
    "SocketCanTransport",
    "ZdtBus",
    "ZdtConfig",
    "ZdtController",
    "ZdtDriver",
    "ZdtDriverError",
    "__version__",
    "fk_mdh",
    "geometric_jacobian",
    "ik_dls",
    "scan_bus",
]
