"""Verification test for standard package structure and backward-compatible shims."""

import pytest


def test_arm_robot_exports():
    """Verify primary exports from arm_robot package."""
    import arm_robot

    assert hasattr(arm_robot, "ZdtController")
    assert hasattr(arm_robot, "CartesianController")
    assert hasattr(arm_robot, "SocketCanTransport")
    assert hasattr(arm_robot, "ZdtDriver")
    assert hasattr(arm_robot, "ZdtBus")
    assert hasattr(arm_robot, "fk_mdh")
    assert hasattr(arm_robot, "ik_dls")
    assert hasattr(arm_robot, "RobotStateMachine")
    assert hasattr(arm_robot, "SafetyError")


def test_arm_robot_submodule_structure():
    """Verify arm_robot submodules are importable."""
    from arm_robot.driver.can_transport import SocketCanTransport
    from arm_robot.driver.zdt_driver import ZdtDriver
    from arm_robot.kinematics.kinematics import fk_mdh
    from arm_robot.kinematics.cartesian import CartesianController
    from arm_robot.controller.controller import ZdtController
    from arm_robot.controller.safety import RobotStateMachine

    assert SocketCanTransport is not None
    assert ZdtDriver is not None
    assert fk_mdh is not None
    assert CartesianController is not None
    assert ZdtController is not None
    assert RobotStateMachine is not None


def test_backward_compatibility_shims():
    """Verify lerobot_robot_massage shims correctly forward to arm_robot."""
    import lerobot_robot_massage
    import lerobot_robot_massage.zdt.config as legacy_config
    import lerobot_robot_massage.zdt.controller as legacy_controller
    import lerobot_robot_massage.zdt.can_transport as legacy_transport
    import lerobot_robot_massage.zdt.safety as legacy_safety
    import lerobot_robot_massage.zdt.types as legacy_types
    import arm_robot.controller.config as new_config
    import arm_robot.controller.controller as new_controller
    import arm_robot.driver.can_transport as new_transport
    import arm_robot.controller.safety as new_safety
    import arm_robot.kinematics.types as new_types

    # Verify identity between legacy shim and new implementation
    assert legacy_config.ZdtConfig is new_config.ZdtConfig
    assert legacy_controller.ZdtController is new_controller.ZdtController
    assert legacy_transport.SocketCanTransport is new_transport.SocketCanTransport
    assert legacy_safety.RobotStateMachine is new_safety.RobotStateMachine
    assert legacy_types.CartesianCommand is new_types.CartesianCommand
    assert legacy_types.JointState is new_types.JointState
