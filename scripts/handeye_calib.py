"""手眼标定：相机系→机器人基座系的旋转 R（差分遥操只需旋转）。

方式 A: 直接填相机安装欧拉角 → rot_from_euler。
方式 B: N 点 Procrustes（≥4 非共面）：手到已知物理位置 + 臂端对应位置。
方式 C: 轴对齐向导 (Arm 侧 demo_arm_teleop.py 中通过 K 键流程调用)。

历史路径: 此文件原位于 Leap_Hand/python/gesture_mapping/handeye_calib.py,
2026-08 迁移到本仓. apply_rotation() 函数已被内联到
Leap_Hand/python/gesture_mapping/wrist_tracker.py (那里只有它被共享
调用, 不再需要跨仓 sys.path 注入).
"""
import json
from pathlib import Path
from typing import Union

import numpy as np

_Path = Union[str, Path]


def rot_from_euler(rx_deg: float, ry_deg: float, rz_deg: float) -> np.ndarray:
    """绕 X→Y→Z（相机系）的旋转矩阵 R(3,3)。列向量应用: v_base = R @ v_cam。"""
    rx, ry, rz = np.radians([rx_deg, ry_deg, rz_deg])
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def procrustes_rotation(src_pts, dst_pts) -> np.ndarray:
    """最小化 Σ||R@p_i − q_i||² 的旋转 R。src/dst: (N,3)。返回 R(3,3)。"""
    src = np.asarray(src_pts, float).T   # (3,N)
    dst = np.asarray(dst_pts, float).T   # (3,N)
    H = src @ dst.T
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    return R


def apply_rotation(R: np.ndarray, pts) -> np.ndarray:
    """R(3,3) 作用于 (N,3) 点集（每行一个列向量）。"""
    pts = np.asarray(pts, float)
    return (R @ pts.T).T


def save_calib(path: _Path, R: np.ndarray) -> None:
    Path(path).write_text(json.dumps({"R": np.asarray(R).tolist()}))


def load_calib(path: _Path) -> np.ndarray:
    data = json.loads(Path(path).read_text())
    return np.array(data["R"])


# 基座方向码: 1=+X 2=-X 3=+Y 4=-Y 5=+Z(上) 6=-Z(下)
_BASE_DIR_CODES = {
    1: np.array([1.0, 0.0, 0.0]), 2: np.array([-1.0, 0.0, 0.0]),
    3: np.array([0.0, 1.0, 0.0]), 4: np.array([0.0, -1.0, 0.0]),
    5: np.array([0.0, 0.0, 1.0]), 6: np.array([0.0, 0.0, -1.0]),
}


def solve_handeye(cam_dirs, base_codes):
    """从 (相机系单位方向, 基座方向码) 配对解手眼旋转 R。

    cam_dirs: (N,3) 相机系单位方向（操作者沿某方向挥手的位移方向）
    base_codes: 长度 N 的 int 列表，操作者指定"臂应去"的基座方向码(1-6)
    返回 R(3,3) 使 R @ cam_dir_i ≈ base_dir(base_codes[i])。
    """
    src = np.asarray(cam_dirs, float)
    dst = np.array([_BASE_DIR_CODES[c] for c in base_codes], float)
    return procrustes_rotation(src, dst)
