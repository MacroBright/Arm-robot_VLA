"""手腕 3D + 掌参考系 → 机械臂位置跟随遥操 (核心遥操模块)。

纯函数部分: 深度反投影、掌参考系、俯仰/滚转角、delta→速度。
WristTracker 位置跟随范式 (参考 teleop_gesture_toolbox TeleoperationByDrawing):
  手位移 → 目标末端位置 → P 位置环 → 速度命令; 松开 H 重锚定 (走哪停哪)。

注: 2026-08 handeye_calib.py 整体迁去 Arm-robot_VLA 仓 (本仓不再依赖 Arm),
apply_rotation() 函数被内联到本文件 (_apply_rotation, 下方) 避免跨仓库 sys.path.
"""
import math
from typing import Optional, Tuple

import numpy as np

try:
    from arm_robot.kinematics.filter import OneEuroFilter
except (ImportError, ValueError):
    from ..kinematics.filter import OneEuroFilter




def _apply_rotation(R: np.ndarray, pts) -> np.ndarray:
    """R(3,3) 作用于 (N,3) 点集 (每行一个列向量).

    本地副本 — 源自 handeye_calib.apply_rotation. 2026-08 因 handeye_calib.py
    整体迁去 Arm-robot_VLA 仓 (跨仓库不再可达), 留本文件做共享.
    任何修改请同步 Arm-robot_VLA/scripts/handeye_calib.apply_rotation.
    """
    pts = np.asarray(pts, float)
    return (R @ pts.T).T


# MediaPipe 21 点索引
_WRIST = 0
_MCP_INDEX = 5
_MCP_MIDDLE = 9
_MCP_MIDDLE_TIP = 12
_MCP_PINKY = 17

K = Tuple[float, float, float, float]  # (fx, fy, cx, cy)


# ── 深度反投影 ──────────────────────────────────────────────

def backproject(u: float, v: float, depth_mm: float, K: K) -> np.ndarray:
    """像素 (u,v) + 深度(mm) → 相机系 3D (mm)。"""
    fx, fy, cx, cy = K
    Z = float(depth_mm)
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    return np.array([X, Y, Z])


def median_depth_at(depth: np.ndarray, u: float, v: float,
                    patch: int = 7, max_patch: int = 21) -> float:
    """wrist / keypoint 邻域多尺度中值深度(mm)。越界/全 0 返回 nan。"""
    h, w = depth.shape
    for p in (patch, 15, max_patch):
        r = p // 2
        u0, u1 = max(0, int(u) - r), min(w, int(u) + r + 1)
        v0, v1 = max(0, int(v) - r), min(h, int(v) + r + 1)
        patch_d = depth[v0:v1, u0:u1]
        patch_d = patch_d[patch_d > 0]
        if patch_d.size > 0:
            return float(np.median(patch_d))
    return float("nan")


def build_palm_pts(hand, depth: Optional[np.ndarray],
                   K: Optional[K]) -> Optional[np.ndarray]:
    """从 HandResult + 对齐深度反投影出所需关键点 (21,3) 相机系 mm。

    当 depth 与 K 存在时采用硬件级多尺度中值深度反投影；
    当 depth 为 None (纯彩色单目流) 时，无缝回退至 MediaPipe 3D 度量骨骼 (world_landmarks)，
    确保在任何传感器模式下均能 100% 灵敏追踪手腕位移与掌面姿态。
    """
    needed = (_WRIST, _MCP_INDEX, _MCP_MIDDLE, _MCP_MIDDLE_TIP, _MCP_PINKY)

    if depth is not None and K is not None:
        h, w = depth.shape
        lm = hand.landmarks
        pts = np.zeros((21, 3))

        valid_depths = []
        points_uv = {}
        points_z = {}
        for i in needed:
            u = float(np.clip(lm[i].x * w, 0.0, float(w - 1)))
            v = float(np.clip(lm[i].y * h, 0.0, float(h - 1)))
            points_uv[i] = (u, v)
            z = median_depth_at(depth, u, v, patch=7, max_patch=25)
            points_z[i] = z
            if math.isfinite(z) and z > 100.0:
                valid_depths.append(z)

        if len(valid_depths) >= 1:
            median_z = float(np.median(valid_depths))
            for i in needed:
                u, v = points_uv[i]
                z = points_z[i]
                if not math.isfinite(z) or z <= 100.0:
                    z = median_z
                pts[i] = backproject(u, v, z, K)
            return pts

    # ── 单目 RGB 纯视觉 3D 几何回退 ──
    pts = np.zeros((21, 3))
    wlm = getattr(hand, "world_landmarks", None)
    lm = getattr(hand, "landmarks", None)

    if wlm is not None and len(wlm) == 21:
        # world_landmarks: 单位米, 真实物理 3D 相对几何 (mm)
        for i in range(21):
            pts[i] = np.array([
                wlm[i].x - wlm[_WRIST].x,
                wlm[i].y - wlm[_WRIST].y,
                wlm[i].z - wlm[_WRIST].z,
            ]) * 1000.0

        if lm is not None:
            # 视口空间绝对手腕中心: X[-300~+300mm], Y[-225~+225mm], Z[500mm基准]
            wrist_x_mm = (lm[_WRIST].x - 0.5) * 600.0
            wrist_y_mm = (lm[_WRIST].y - 0.5) * 450.0
            wrist_z_mm = 500.0 + (lm[_WRIST].z * 600.0)
            pts[:, 0] += wrist_x_mm
            pts[:, 1] += wrist_y_mm
            pts[:, 2] += wrist_z_mm
        return pts

    elif lm is not None and len(lm) == 21:
        for i in range(21):
            pts[i] = np.array([
                (lm[i].x - 0.5) * 600.0,
                (lm[i].y - 0.5) * 450.0,
                500.0 + (lm[i].z * 600.0),
            ])
        return pts

    return None


# ── 掌参考系 ────────────────────────────────────────────────

def palm_basis(pts21: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(f, n, lat) 单位向量。f: wrist→中指MCP; n: 掌法线; lat: cross(f,n)。"""
    f = pts21[_MCP_MIDDLE] - pts21[_WRIST]
    f = f / (np.linalg.norm(f) + 1e-9)
    across = pts21[_MCP_PINKY] - pts21[_MCP_INDEX]
    n = np.cross(across, f)
    n = n / (np.linalg.norm(n) + 1e-9)
    lat = np.cross(f, n)
    return f, n, lat


# ── 旋转角 (相对参考) ───────────────────────────────────────

def pitch_angle(f_base: np.ndarray) -> float:
    """f 相对基座系水平面的俯仰角 (rad), f 向上为正。"""
    return math.asin(max(-1.0, min(1.0, float(f_base[2]))))


def _proj_perp(v: np.ndarray, axis: np.ndarray) -> np.ndarray:
    axis = axis / (np.linalg.norm(axis) + 1e-9)
    v = v - (v @ axis) * axis
    return v / (np.linalg.norm(v) + 1e-9)


def roll_angle(n_base: np.ndarray, f_base: np.ndarray,
               n_ref: np.ndarray, f_ref: np.ndarray) -> float:
    """掌滚转角 (rad): n 绕 f 轴相对参考的有向转角 (旋前为正)。"""
    a = _proj_perp(n_ref, f_ref)
    b = _proj_perp(n_base, f_ref)
    return math.atan2(np.dot(np.cross(a, b), f_ref), np.dot(a, b))


# ── delta → 速度 ────────────────────────────────────────────

def delta_to_velocity(delta: float, gain: float, deadzone: float,
                      max_vel: float = 1.0) -> float:
    """delta → [-1,1] 速度: 死区内 0, 线性增益, 饱和 max_vel。"""
    d = float(delta)
    if abs(d) < deadzone:
        return 0.0
    v = (d - math.copysign(deadzone, d)) * gain
    return float(np.clip(v, -max_vel, max_vel))


# ── 四元数 / 姿态误差 → 角速度 ──────────────────────────────

def quat_to_rot(q):
    """wxyz 四元数 → 3x3 旋转矩阵 (numpy)."""
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
        [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)]])


def rot_error_angvel(R_target, R_cur, gain):
    """目标姿态→当前姿态的角速度 (rad/s, 归一化后∈[-1,1]).
    error = R_target @ R_cur.T → 轴角 → w = gain·clip(angle) · axis.
    返回 (3,) 单位速度系数."""
    R_e = R_target @ R_cur.T
    cos_a = (np.trace(R_e) - 1.0) / 2.0
    cos_a = float(np.clip(cos_a, -1.0, 1.0))
    angle = math.acos(cos_a)
    if angle < 1e-6:
        return np.zeros(3)
    ax = np.array([R_e[2,1]-R_e[1,2], R_e[0,2]-R_e[2,0], R_e[1,0]-R_e[0,1]])
    n = np.linalg.norm(ax)
    if n < 1e-6:
        return np.zeros(3)
    ax = ax / n
    w = gain * math.degrees(angle)   # 角度→速度 (gain≈0.02/deg)
    return np.clip(w * ax, -1.0, 1.0)


# ── WristTracker: 末端 6DOF 位姿跟随遥操 (手 6DOF → 位姿目标 → 位置/姿态环 → end_event) ──

class WristTracker:
    """末端 6DOF 位姿跟随遥操: 手 6DOF 增量 → 末端位姿目标 → 位置/姿态环 → end_event 速度.

    update() 每帧需喂入末端位姿反馈 ee_pose = (pos_mm3, quat_wxyz)（来自 get_ee_pose）.
    按住 H 时手相对锚点的位置/姿态增量决定末端目标位姿; 松开时 capture() 重锚定 (走哪停哪).
    无手 / 无参考 / 无反馈 → 全 0 (不再差分降级; 真机 M3 补 FK 后再启用).
    """

    def __init__(self, R: np.ndarray,
                 scale_pos: float = 1.0,          # 手位移mm → 末端目标mm
                 k_pos: float = 0.06,             # 位置环增益 (1/mm): 17mm误差→~0.9满速 (2026-08-12 调大减滞后)
                 k_ang: float = 0.10,             # 姿态环增益 (1/deg): 10°误差→~0.9满速 (2026-08-12 调大减滞后)
                 deadzone_pos_mm: float = 5.0,    # 位置死区 (防末端抖动)
                 deadzone_ang_deg: float = 2.0,   # 姿态死区 (角度误差忽略下限)
                 dt: float = 1.0 / 30.0,
                 min_cutoff: float = 8.0, beta: float = 0.02,
                 j5_range=(0.0, 90.0), j4_range=(-180.0, 180.0)):   # 保留兼容参数 (不再直接用于命令)
        self.R = np.asarray(R, float)
        self.scale_pos = scale_pos
        self.k_pos, self.k_ang = k_pos, k_ang
        self.deadzone_pos_mm, self.deadzone_ang_deg = deadzone_pos_mm, deadzone_ang_deg
        self.dt = dt
        self.j5_range, self.j4_range = j5_range, j4_range
        # 手参考 + 臂锚点
        self._ref_pts = None
        self._ref_f = self._ref_n = None
        self._has_ref = False
        self._pos_filt = OneEuroFilter(3, min_cutoff=min_cutoff, beta=beta)
        self._anchor_wrist = None     # 手锚点位置 (相机系 mm)
        self._anchor_hand_rot = None  # 手锚点姿态 R_hand (3x3, 相机系列向量)
        self._anchor_ee_pos = None    # 末端锚点位置 (基座系 mm)
        self._anchor_ee_rot = None    # 末端锚点姿态 R (3x3)
        # 深度时域中值 (Z 轴鲁棒性, 借鉴 toolbox temporal filter)
        self._depth_buf = []
        self.DEPTH_BUF_N = 5
        # 诊断 (HUD 依赖)
        self.last_delta_base = np.zeros(3)
        self.last_target_ee = np.zeros(3)
        self.last_roll_deg = 0.0
        self.last_pitch_deg = 0.0

    def capture(self, pts21_cam, wrist_mm, ee_pose, j5_deg=0.0, j4_deg=0.0) -> None:
        """捕获手参考 + 臂锚点 (clutch 按下/松开时调用).

        pts21_cam=(21,3)相机系mm (None 只清手参考);
        wrist_mm=腕心位置(基座系mm, 位置锚点); ee_pose=(pos_mm3, quat_wxyz) 或 None (姿态锚点).
        """
        if pts21_cam is not None:
            self._ref_pts = np.asarray(pts21_cam, float)
            f, n, _ = palm_basis(self._ref_pts)
            self._ref_f = _apply_rotation(self.R, np.array([f]))[0]
            self._ref_n = _apply_rotation(self.R, np.array([n]))[0]
            self._pos_filt.reset()
            self._has_ref = True
            self._anchor_wrist = self._ref_pts[_WRIST].copy()
            self._anchor_hand_rot = np.stack([f, n, np.cross(f, n)], axis=1)  # 相机系列向量
        else:
            self._has_ref = False
        if wrist_mm is not None:
            self._anchor_ee_pos = np.asarray(wrist_mm, float)
        if ee_pose is not None:
            _, quat = ee_pose
            self._anchor_ee_rot = quat_to_rot(quat)
        self._depth_buf.clear()
        self.last_delta_base = np.zeros(3)
        self.last_target_ee = (np.array(self._anchor_ee_pos) if self._anchor_ee_pos is not None
                               else np.zeros(3))
        self.last_roll_deg = 0.0
        self.last_pitch_deg = 0.0

    def update(self, pts21_cam, wrist_mm, ee_pose, j5_deg=0.0, j4_deg=0.0):
        """末端 6DOF 位姿跟随: 返回 (vx,vy,vz, wx,wy,wz) ∈[-1,1].

        位置反馈用腕心 wrist_mm(基座系mm, 解耦), 姿态反馈用 ee_pose(末端四元数).
        无手 / 无参考 / 反馈缺失 → 全 0.
        """
        if (pts21_cam is None or not self._has_ref
                or wrist_mm is None or ee_pose is None):
            return (0.0,) * 6
        pts = np.asarray(pts21_cam, float)

        # 深度时域中值: wrist 深度(Z)滚动中值, 去飞点
        if self._depth_buf or pts[0][2] > 0:
            self._depth_buf.append(float(pts[0][2]))
            if len(self._depth_buf) > self.DEPTH_BUF_N:
                self._depth_buf.pop(0)
        if self._depth_buf:
            pts = pts.copy()
            pts[0][2] = float(np.median(self._depth_buf))

        # 位置增量: 手位移 (相机系) → 基座系 → 末端目标位置
        wrist = self._pos_filt(pts[_WRIST])
        dpos_cam = wrist - self._anchor_wrist
        dpos_base = _apply_rotation(self.R, np.array([dpos_cam]))[0]
        self.last_delta_base = dpos_base
        target_pos = self._anchor_ee_pos + dpos_base * self.scale_pos
        self.last_target_ee = target_pos

        # 姿态增量: 手旋转增量 → 末端目标姿态
        f, n, _ = palm_basis(pts)
        R_hand_now = np.stack([f, n, np.cross(f, n)], axis=1)
        dRot = R_hand_now @ self._anchor_hand_rot.T    # 手旋转增量 (相对锚点)
        target_rot = dRot @ self._anchor_ee_rot        # 末端目标姿态 (基座系)

        # 位置环用腕心反馈(解耦), 姿态环用末端四元数反馈
        _, ee_quat = ee_pose
        v_lin = np.array([delta_to_velocity(target_pos[i] - wrist_mm[i], self.k_pos,
                                            self.deadzone_pos_mm) for i in range(3)])
        w_ang = rot_error_angvel(target_rot, quat_to_rot(ee_quat), self.k_ang)

        # 诊断 (HUD): 手滚转/俯仰相对参考 (基座系)
        f_base = _apply_rotation(self.R, np.array([f]))[0]
        n_base = _apply_rotation(self.R, np.array([n]))[0]
        self.last_pitch_deg = math.degrees(pitch_angle(f_base))
        self.last_roll_deg = math.degrees(roll_angle(n_base, f_base, self._ref_n, self._ref_f))

        return (float(v_lin[0]), float(v_lin[1]), float(v_lin[2]),
                float(w_ang[0]), float(w_ang[1]), float(w_ang[2]))

    def no_hand(self):
        self._pos_filt.reset()
        self.last_roll_deg = 0.0
        self.last_pitch_deg = 0.0
        return (0.0,) * 6
