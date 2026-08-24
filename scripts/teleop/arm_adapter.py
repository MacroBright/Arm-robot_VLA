"""scripts/teleop/arm_adapter.py — 仿真/真机臂统一接口 (spec TASK-02/24).

SimulationArmAdapter: 复用 ArmClient (MuJoCo socket), 保持仿真遥操能力.
RealArmAdapter: 封装 CartesianController → ZdtController, 是唯一真机笛卡尔入口;
  不实现 CAN 协议 / 不重复 IK / 不直接操作电机帧. 视觉层只产 CartesianCommand.
"""
from __future__ import annotations

import numpy as np

from lerobot_robot_massage.zdt.types import (
    CartesianCommand, EEPose, JointState,
)


class SimulationArmAdapter:
    """MuJoCo 仿真臂 (ArmClient socket 协议)."""

    def __init__(self, arm_client):
        self._arm = arm_client

    def connect(self) -> None:
        self._arm.remote_enable()

    def disconnect(self) -> None:
        self._arm.remote_disable()
        self._arm.close()

    def get_joint_state(self) -> JointState:
        angles, vels, loads = self._arm.get_state()
        return JointState(q=tuple(angles), dq=tuple(vels),
                          current_ma=tuple(loads))

    def get_ee_pose(self) -> EEPose:
        ep = self._arm.get_ee_pose()
        if ep is None:
            raise RuntimeError("仿真未返回末端位姿 (get_ee_pose)")
        pos_m, quat = ep
        position = np.asarray(pos_m, float) * 1000.0          # m → mm
        rotation = _quat_to_rotmat(quat)
        return EEPose(position=position, rotation=rotation)

    def move_cartesian_velocity(self, cmd: CartesianCommand) -> None:
        # 仿真协议约定: end_event 输入按 mm/s + rad/s 解释 (回归测试同约定)
        self._arm.end_event(*cmd.twist)

    def reset(self) -> None:
        self._arm.soft_reset()

    def e_stop(self) -> None:
        self._arm.e_stop()

    def state(self) -> str:
        return "TELEOP"


class RealArmAdapter:
    """真机臂: 封装 CartesianController (spec §6.1). 不直接操作 CAN/IK.

    P0-①: connect() 只到 SAFE_IDLE (枚举+验证), 不自动 arm/使能扭矩;
    arm(gravity_confirmed) 由调用方显式调用 (重力关节 J2/J3 需确认).
    """

    def __init__(self, ctrl, **cart_kwargs):
        from lerobot_robot_massage.zdt.cartesian import CartesianController
        self._ctrl = ctrl
        self._cart = CartesianController(ctrl, **cart_kwargs)

    def connect(self) -> None:
        self._ctrl.connect()                       # SAFE_IDLE, 不使能扭矩

    def arm(self, gravity_confirmed: bool = False) -> None:
        """显式臂置 (使能扭矩) — 调用方在用户确认后调用."""
        self._ctrl.arm(gravity_confirmed)

    def enter_teleop(self) -> None:
        self._ctrl.enter_teleop()

    def exit_teleop(self) -> None:
        self._ctrl.exit_teleop()

    def disconnect(self) -> None:
        try:
            self._ctrl.disarm()
        finally:
            self._ctrl.disconnect()

    def get_joint_state(self) -> JointState:
        st = self._ctrl.get_real_state()
        return JointState(q=tuple(st["q"]), dq=tuple(st["velocity"]),
                          current_ma=tuple(st["current"]),
                          flags=tuple(int(f) for f in st["flags"]),
                          status=st["status"])

    def get_real_joint_angles(self) -> list[float]:
        return self._ctrl.read_real_angles(use_kb=True)

    def get_ee_pose(self) -> EEPose:
        # P1-⑥: 经公共接口 get_current_pose(), 不依赖 Controller 私有 FK
        return self._cart.get_current_pose()

    def move_cartesian_velocity(self, cmd: CartesianCommand) -> None:
        self._cart.step(*cmd.linear_velocity, *cmd.angular_velocity,
                        cmd_ts=cmd.timestamp)

    def step_pose(self, p_des, R_des, **kw) -> None:
        self._cart.step_pose(p_des, R_des, **kw)

    def reset(self) -> None:
        self._cart.ready()

    def e_stop(self) -> None:
        self._ctrl.e_stop()

    def state(self) -> str:
        return self._ctrl.robot.phase.name


def _quat_to_rotmat(q) -> np.ndarray:
    """(w,x,y,z) → SO(3) (测试/仿真用)."""
    w, x, y, z = (float(v) for v in q)
    n = np.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
