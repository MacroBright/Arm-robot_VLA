"""MuJoCo simulation and digital twin environment."""

from .mujoco_sim import MuJoCoArm, main as run_sim

# Friendly alias
MuJoCoArmSim = MuJoCoArm

__all__ = [
    "MuJoCoArm",
    "MuJoCoArmSim",
    "run_sim",
]
