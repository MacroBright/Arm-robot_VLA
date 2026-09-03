"""Kinematics, Jacobian, and Cartesian control algorithms."""

from .cartesian import CartesianController
from .kinematics import (
    D_H,
    adaptive_damping,
    anchor_to_source,
    damped_ls,
    fk_mdh,
    ik_analytic,
    ik_position,
    jacobian,
    log_so3,
    singularity_metrics,
    source_to_anchor,
)
from .types import (
    CartesianCommand,
    EEPose,
    JointState,
    rotmat_to_quat,
)
from .workspace import BoxWorkspace, CartesianVelocityLimiter

# Aliases
MDH_TABLE = D_H
ik_dls = damped_ls
geometric_jacobian = jacobian

__all__ = [
    "BoxWorkspace",
    "CartesianCommand",
    "CartesianController",
    "CartesianVelocityLimiter",
    "D_H",
    "EEPose",
    "JointState",
    "MDH_TABLE",
    "adaptive_damping",
    "anchor_to_source",
    "damped_ls",
    "fk_mdh",
    "geometric_jacobian",
    "ik_analytic",
    "ik_dls",
    "ik_position",
    "jacobian",
    "log_so3",
    "rotmat_to_quat",
    "singularity_metrics",
    "source_to_anchor",
]
