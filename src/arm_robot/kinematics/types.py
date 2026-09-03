"""zdt/types.py — 遥操/控制层共享数据类型 (spec TASK-06/24).

内部姿态表示约定: EEPose.rotation 为 SO(3) 旋转矩阵 (非 RPY/Euler);
轴角/四元数只是导出形态 (to_rotation_vector / to_quaternion), 供控制
(log_so3) 与录制 (JSONL quaternion) 使用.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass(frozen=True)
class CartesianCommand:
    """末端 6DOF 速度命令 (mm/s + rad/s, 基座系)."""
    linear_velocity: Tuple[float, float, float]
    angular_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    timestamp: float = 0.0   # 产生时刻 (time.monotonic 秒)

    @property
    def twist(self) -> np.ndarray:
        """合并 6-vector [vx,vy,vz, wx,wy,wz]."""
        return np.array(list(self.linear_velocity) + list(self.angular_velocity),
                        dtype=float)


@dataclass(frozen=True)
class JointState:
    """6 轴输出轴真实角 (anchor 帧, deg) + 滤波速度 + 电流 + 全轴 flags."""
    q: Tuple[float, float, float, float, float, float]
    dq: Tuple[float, float, float, float, float, float] = (0.0,) * 6
    current_ma: Tuple[float, ...] = ()
    flags: Tuple[int, ...] = ()          # 6 轴完整状态位 (P2-⑨): 任一轴堵转/失能可见
    status: str = ""


@dataclass(frozen=True)
class EEPose:
    """末端位姿. 内部姿态表示 = SO(3) 旋转矩阵 (非 RPY/Euler)."""
    position: np.ndarray   # (3,) mm, 基座系
    rotation: np.ndarray   # (3,3) ∈ SO(3)

    def to_quaternion(self) -> Tuple[float, float, float, float]:
        """→ (w, x, y, z) (JSONL 录制用)."""
        return rotmat_to_quat(self.rotation)

    def to_rotation_vector(self) -> np.ndarray:
        """→ 轴角向量 (rad). 内部控制表示."""
        from .kinematics import log_so3
        return log_so3(self.rotation)


def rotmat_to_quat(R) -> Tuple[float, float, float, float]:
    """SO(3) → 四元数 (w, x, y, z). Shepperd 稳健法, 近 π 不失精度."""
    R = np.asarray(R, float)
    tr = float(np.trace(R))
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([w, x, y, z], dtype=float)
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    q = q / n
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
