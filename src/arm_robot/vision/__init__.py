"""Vision tracking and gesture estimation modules for Arm-robot_VLA."""

from .camera import CameraSource, RealSenseSource, open_realsense
from .hand_tracker import HandResult, HandTracker
from .wrist_tracker import (
    WristTracker,
    backproject,
    build_palm_pts,
    delta_to_velocity,
    median_depth_at,
    palm_basis,
    pitch_angle,
    roll_angle,
    rot_error_angvel,
)

# Friendly alias
HandTrackingResult = HandResult

__all__ = [
    "CameraSource",
    "HandResult",
    "HandTracker",
    "HandTrackingResult",
    "RealSenseSource",
    "WristTracker",
    "backproject",
    "build_palm_pts",
    "delta_to_velocity",
    "median_depth_at",
    "open_realsense",
    "palm_basis",
    "pitch_angle",
    "roll_angle",
    "rot_error_angvel",
]
