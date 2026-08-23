"""Modified DH 运动学模块 — 移植源项目 zero-robotic-arm 解析逆解.

约定 (2026-08-21 数值验证):
  * DH 表来自固件 robot.c D_H[6][4]; FK/IK 用纯 θ (忽略 offset 列),
    与源项目 Simulink 推导一致.
  * 关节角空间 = 固件 current_angle (输出轴真实角, anchor 同基准).
    复位姿态 (g_joints_init current_angle) = [90,90,-90,0,90,0]°;
    T_0_6_RESET = FK(复位角) — 数值验证 diff=0.
  * 单位: 角度 deg, 长度 mm.
  * IK 输出 θ 即输出轴真实角 (anchor 空间).

移植来源: zero-robotic-arm-master/2. Software/robot/Core/Src/robot_kinematics.c
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np

# 固件 DH 表 (robot.c:25-30). 列 = [a, alpha, d, theta_offset]; offset 忽略.
D_H: list[list[float]] = [
    [0, 0, 0, math.pi / 2],
    [0, math.pi / 2, 0, math.pi / 2],
    [200.0, math.pi, 0, -math.pi / 2],
    [47.63, -math.pi / 2, -184.5, 0],
    [0, math.pi / 2, 0, math.pi / 2],
    [0, math.pi / 2, 0, 0],
]

# 固件复位位姿关节角 (g_joints_init current_angle, robot.c:44-51) — 纯 θ 空间.
RESET_POSE_DEG: list[float] = [90.0, 90.0, -90.0, 0.0, 90.0, 0.0]

# 固件复位位姿矩阵 T_0_6_reset (robot.c:33-38) — FK(RESET_POSE_DEG) 应等于它.
T_0_6_RESET: np.ndarray = np.array([
    [0, -1, 0, 0],
    [0, 0, -1, -47.63],
    [1, 0, 0, 15.5],
    [0, 0, 0, 1],
], dtype=float)

# 源项目关节角限位 (g_joints_init min/max, robot.c:44-51) — IK source 帧限位.
SOURCE_JOINT_LIMITS: list[tuple[float, float]] = [
    (0.0, 360.0),    # J1
    (90.0, 180.0),   # J2
    (-90.0, 90.0),   # J3
    (-90.0, 90.0),   # J4
    (0.0, 90.0),     # J5
    (0.0, 360.0),    # J6
]

# 坐标帧桥接: 本项目 anchor 帧开机姿态=全 0; 源项目 IK 帧复位角=RESET_POSE_DEG.
# 约定: 开机姿态 (anchor 0) == 源项目复位姿态 (source RESET_POSE_DEG),
#      即 q_anchor = q_source − RESET_POSE_DEG. ⚠ 待真机 anchor 标定确认
#      (若开机姿态 ≠ 源复位姿态, 需按实测更新此常量).
SOURCE_TO_ANCHOR_OFFSET: list[float] = list(RESET_POSE_DEG)

# 各关节旋转权重 (固件 joint_weight, robot.c:41): 多解选择时按"离当前角加权最近"择优.
JOINT_WEIGHT: list[float] = [5.0, 3.0, 3.0, 1.0, 1.0, 1.0]

# 角度误差容忍 (弧度, 对齐固件 ROBOT_ERROR_RANGE 语义)
_ERROR_RANGE: float = 1e-3

_DH_ARR = np.asarray(D_H, dtype=float)


def _fk_frames(q_deg: list[float] | np.ndarray,
               dh: np.ndarray | None = None) -> list[np.ndarray]:
    """modified-DH 逐帧正运动学, 返回 [T_0_0 .. T_0_6] 共 7 个 4×4.

    与 fk_mdh 同源 (fk_mdh 取其末帧), 供几何雅可比复用中间帧.
    约定同 fk_mdh: 纯 θ (忽略 DH 第 4 列 offset), 角度为输出轴真实角 (deg).
    """
    if dh is None:
        dh = _DH_ARR
    q = np.radians(np.asarray(q_deg, dtype=float))
    frames = [np.eye(4)]
    for i in range(6):
        a, al, d, _ = dh[i]
        th = q[i]
        ct, st = math.cos(th), math.sin(th)
        ca, sa = math.cos(al), math.sin(al)
        Ti = np.array([
            [ct, -st, 0, a],
            [st * ca, ct * ca, -sa, -d * sa],
            [st * sa, ct * sa, ca, d * ca],
            [0, 0, 0, 1],
        ])
        frames.append(frames[-1] @ Ti)
    return frames


def fk_mdh(q_deg: list[float] | np.ndarray,
           dh: np.ndarray | None = None) -> np.ndarray:
    """Modified DH 正运动学.

    约定: 纯 θ (忽略 DH 第 4 列 offset), 角度输入为输出轴真实角 (deg).
    返回 4×4 齐次变换矩阵 T (行优先).

    Args:
        q_deg: 6 关节角 (度).
        dh: DH 表 (6×4, 默认 D_H). 列 = [a, alpha, d, theta_offset].

    Returns:
        np.ndarray 4×4 末端位姿矩阵.
    """
    return _fk_frames(q_deg, dh)[-1]


def jacobian(q_deg: list[float] | np.ndarray,
             dh: np.ndarray | None = None) -> np.ndarray:
    """几何雅可比 6×6 (行=关节 j, 列=[vx,vy,vz,wx,wy,wz]).

    关节 j 的转轴 = 轴帧 A_j = T_0_j @ Rx(α_j)@Tx(a_j) 的 z 轴 (Rz(q)·Tz(d)
    之前). ⚠ 不是 T_0_j 的 z 轴 — 纯 θ 约定下后者会错 (J2 差 145 mm/rad).
    与 fk_mdh 中心差分逐列 0 误差 (test_jacobian_matches_finite_difference).

    单位: 平移列 mm/rad, 旋转列无量纲. 输入为输出轴真实角 (deg).

    Args:
        q_deg: 6 关节角 (度).
        dh: DH 表 (6×4, 默认 D_H).

    Returns:
        np.ndarray 6×6, J[j,:3]=平移列, J[j,3:]=旋转列.
    """
    if dh is None:
        dh = _DH_ARR
    frames = _fk_frames(q_deg, dh)
    pe = frames[6][:3, 3]
    J = np.zeros((6, 6))
    for j in range(6):
        a, al, _, _ = dh[j]
        ca, sa = math.cos(al), math.sin(al)
        # Rx(α)@Tx(a): 链接的恒定部分 (Rz(q)·Tz(d) 之前) — 关节轴在此轴帧上
        Ax = np.array([
            [1, 0, 0, a],
            [0, ca, -sa, 0],
            [0, sa, ca, 0],
            [0, 0, 0, 1],
        ])
        A = frames[j] @ Ax
        z, p = A[:3, 2], A[:3, 3]
        J[j, :3] = np.cross(z, pe - p)
        J[j, 3:] = z
    return J


def damped_ls(J: np.ndarray, twist: np.ndarray, lam: float,
              weights: Optional[list[float]] = None) -> np.ndarray:
    """加权阻尼最小二乘: 极小化 ‖W(J.T·dq − twist)‖² + λ²‖dq‖².

    用于数值 IK 局部映射: 在当前位置把末端位移/姿态误差映射为关节增量.

    Args:
        J: 6×6 几何雅可比 (行=关节, 列=v|w).
        twist: 6-vector [vx,vy,vz, wx,wy,wz] (mm + rad).
        lam: 阻尼系数 (>0 使 A 恒正定, np.linalg.solve 永稳定). 单位须与
            J 平移列一致 (mm) — 本项目 J 为 mm/rad, λ≈10mm (臂长 ~200mm 的 5%).
        weights: 6-vector 对 twist 各分量的权重 (None=全 1). 姿态权重 >1 使
            解优先满足姿态保持 (ω=0).

    Returns:
        dq: 关节增量 (弧度).
    """
    M = J.T
    W2 = (np.diag([w * w for w in weights]) if weights is not None
          else np.eye(6))
    A = M.T @ W2 @ M + lam * lam * np.eye(6)
    return np.linalg.solve(A, M.T @ W2 @ twist)


# ── 解析逆解 (移植 robot_kinematics.c) ─────────────────────────

def _calc_theta3(T: np.ndarray, a2: float, a3: float, d4: float) -> tuple[float, float]:
    """解 θ3 (2 组). 移植 robot_kinematics.c calc_theta3."""
    px, py, pz = T[0, 3], T[1, 3], T[2, 3]
    _2_a2_d4 = 2 * a2 * d4
    _2_pow_a2_2 = 2 * a2 * a2
    _2_pow_a3_2 = 2 * a3 * a3
    _2_pow_d4_2 = 2 * d4 * d4
    const_eq1 = (-a2 ** 4 + _2_pow_a2_2 * (a3 ** 2 + d4 ** 2) - a3 ** 4
                 - 2 * a3 ** 2 * d4 ** 2 - d4 ** 4)
    const_eq2 = -a2 ** 2 + 2 * a2 * a3 - a3 ** 2 - d4 ** 2
    pow_px_2 = px ** 2
    pow_py_2 = py ** 2
    pow_pz_2 = pz ** 2
    pow_distance_2 = pow_px_2 + pow_py_2 + pow_pz_2
    eq1 = (const_eq1 + _2_pow_a2_2 * pow_distance_2
           + _2_pow_a3_2 * pow_distance_2 + _2_pow_d4_2 * pow_distance_2
           - px ** 4 - py ** 4 - pz ** 4
           - 2 * pow_px_2 * (pow_py_2 + pow_pz_2) - 2 * pow_py_2 * pow_pz_2)
    if eq1 < 0:
        return math.nan, math.nan     # workspace 外: C 里 sqrt(-) → NaN, 解被剔除
    sq = math.sqrt(eq1)
    denom = const_eq2 + pow_distance_2
    if abs(denom) < 1e-12:
        return math.nan, math.nan
    u1 = -(_2_a2_d4 + sq) / denom
    u2 = -(_2_a2_d4 - sq) / denom
    return 2 * math.atan(u1), 2 * math.atan(u2)


def _calc_theta2(theta3: float, T: np.ndarray, a2: float, a3: float,
                 d4: float) -> tuple[float, float]:
    """解 θ2 (2 组, 对应一个 θ3). 移植 __robot_kinematics_calc_theta2."""
    pz = T[2, 3]
    _pow_a2_2 = a2 ** 2
    _pow_a3_2 = a3 ** 2
    _pow_d4_2 = d4 ** 2
    _2_a2_a3 = 2 * a2 * a3
    _2_a2_d4 = 2 * a2 * d4
    cos_theta3 = math.cos(theta3)
    sin_theta3 = math.sin(theta3)
    const_eq1 = _pow_a2_2 + _pow_a3_2 + _pow_d4_2
    inner = const_eq1 + _2_a2_a3 * cos_theta3 - _2_a2_d4 * sin_theta3 - pz ** 2
    if inner < 0:
        return math.nan, math.nan     # workspace 外 (C: sqrt(-)→NaN, 解剔除)
    eq1 = math.sqrt(inner)
    eq2 = a3 * cos_theta3 - d4 * sin_theta3
    eq3 = d4 * cos_theta3 - pz + a3 * sin_theta3
    if abs(eq3) < 1e-12:
        return math.nan, math.nan
    u1 = -(a2 + eq1 + eq2) / eq3
    u2 = -(a2 - eq1 + eq2) / eq3
    return 2 * math.atan(u1), 2 * math.atan(u2)


def _calc_theta1(theta2: float, theta3: float, T: np.ndarray,
                 a2: float, a3: float, d4: float) -> float:
    """解 θ1. 移植 __robot_kinematics_calc_theta1."""
    px, py = T[0, 3], T[1, 3]
    diff = theta2 - theta3
    cd = math.cos(diff)
    sd = math.sin(diff)
    c2, s2 = math.cos(theta2), math.sin(theta2)
    c3, s3 = math.cos(theta3), math.sin(theta3)
    eq1 = a2 * c2 + a3 * cd + d4 * sd
    denom_ratio = px + eq1
    if abs(denom_ratio) < 1e-12:
        return math.nan                # 除零 (C 中 NaN, 解剔除)
    ratio = (-px + eq1) / denom_ratio
    if ratio < 0:
        return math.nan                # workspace 外 (C: sqrt(-)→NaN, 解剔除)
    u_theta1 = math.sqrt(ratio)
    eq2 = (2 * u_theta1 * (c2 * (a2 + a3 * c3 - d4 * s3)
                           + s2 * (d4 * c3 + a3 * s3))) / (u_theta1 ** 2 + 1)
    if abs(py - eq2) > _ERROR_RANGE:
        u_theta1 = -u_theta1
    return 2 * math.atan(u_theta1)


def _calc_theta5(theta1: float, theta2: float, theta3: float,
                 T: np.ndarray) -> float:
    """解 θ5. 移植 __robot_kinematics_calc_theta5."""
    nx, ny, nz = T[0, 0], T[1, 0], T[2, 0]
    ox, oy, oz = T[0, 1], T[1, 1], T[2, 1]
    ax, ay, az = T[0, 2], T[1, 2], T[2, 2]
    c1, s1 = math.cos(theta1), math.sin(theta1)
    c2, s2 = math.cos(theta2), math.sin(theta2)
    c3, s3 = math.cos(theta3), math.sin(theta3)
    r31 = (nx * c1 * c3 * s2 - nz * s2 * s3 - nx * c1 * c2 * s3
           - nz * c2 * c3 - ny * c2 * s1 * s3 + ny * c3 * s1 * s2)
    r32 = (ox * c1 * c3 * s2 - oz * s2 * s3 - ox * c1 * c2 * s3
           - oz * c2 * c3 - oy * c2 * s1 * s3 + oy * c3 * s1 * s2)
    r33 = (ax * c1 * c3 * s2 - az * s2 * s3 - ax * c1 * c2 * s3
           - az * c2 * c3 - ay * c2 * s1 * s3 + ay * c3 * s1 * s2)
    theta5_zyz = math.atan2(math.sqrt(r31 ** 2 + r32 ** 2), r33)
    return -theta5_zyz + math.pi


def _calc_theta4(theta1: float, theta2: float, theta3: float, theta5: float,
                 T: np.ndarray) -> float:
    """解 θ4. 移植 __robot_kinematics_calc_theta4. 奇异 (θ5≈0/π) 返回 0."""
    theta5_zyz = math.pi - theta5
    if (abs(theta5_zyz) < _ERROR_RANGE
            or abs(theta5_zyz - math.pi) < _ERROR_RANGE):
        return 0.0
    ax, ay, az = T[0, 2], T[1, 2], T[2, 2]
    c1, s1 = math.cos(theta1), math.sin(theta1)
    c2, s2 = math.cos(theta2), math.sin(theta2)
    c3, s3 = math.cos(theta3), math.sin(theta3)
    theta4 = 0.0  # 坐标系对齐
    c4, s4 = math.cos(theta4), math.sin(theta4)
    r23 = (ax * c4 * s1 - ay * c1 * c4 + az * c2 * s3 * s4
           - az * c3 * s2 * s4 - ax * c1 * c2 * c3 * s4
           - ay * c2 * c3 * s1 * s4 - ax * c1 * s2 * s3 * s4
           - ay * s1 * s2 * s3 * s4)
    r13 = (ax * s1 * s4 - ay * c1 * s4 - az * c2 * c4 * s3
           + az * c3 * c4 * s2 + ax * c1 * c2 * c3 * c4
           + ay * c2 * c3 * c4 * s1 + ax * c1 * c4 * s2 * s3
           + ay * c4 * s1 * s2 * s3)
    return math.atan2(r23, r13)


def _calc_theta6(theta1: float, theta2: float, theta3: float, theta4: float,
                 theta5: float, T: np.ndarray) -> float:
    """解 θ6. 移植 __robot_kinematics_calc_theta6. 奇异分支用 θ5 退化公式."""
    theta5_zyz = math.pi - theta5
    nx, ny, nz = T[0, 0], T[1, 0], T[2, 0]
    ox, oy, oz = T[0, 1], T[1, 1], T[2, 1]
    c1, s1 = math.cos(theta1), math.sin(theta1)
    c2, s2 = math.cos(theta2), math.sin(theta2)
    c3, s3 = math.cos(theta3), math.sin(theta3)
    if (abs(theta5_zyz) < _ERROR_RANGE
            or abs(theta5_zyz - math.pi) < _ERROR_RANGE):
        # 奇异分支: θ6 由 o/n 列直接解
        r12 = (-oz * c2 * s3 + oz * c3 * s2 + ox * c1 * c2 * c3
               + oy * c2 * c3 * s1 + ox * c1 * s2 * s3 + oy * s1 * s2 * s3)
        r11 = (-nz * c2 * s3 + nz * c3 * s2 + nx * c1 * c2 * c3
               + ny * c2 * c3 * s1 + nx * c1 * s2 * s3 + ny * s1 * s2 * s3)
        theta6_zyz = math.atan2(-r12, r11)
        return theta6_zyz - math.pi
    r32 = (ox * c1 * c3 * s2 - oz * s2 * s3 - ox * c1 * c2 * s3
           - oz * c2 * c3 - oy * c2 * s1 * s3 + oy * c3 * s1 * s2)
    r31 = (nx * c1 * c3 * s2 - nz * s2 * s3 - nx * c1 * c2 * s3
           - nz * c2 * c3 - ny * c2 * s1 * s3 + ny * c3 * s1 * s2)
    theta6_zyz = math.atan2(r32, -r31)
    return theta6_zyz - math.pi


def ik_analytic(T_target: np.ndarray, dh: np.ndarray | None = None,
                current_deg: list[float] | None = None,
                joint_limits: list[tuple[float, float]] | None = None
                ) -> Optional[list[float]]:
    """解析逆运动学 — 移植源项目 robot_kinematics.c.

    流程: 解 4 组候选角 (θ3×θ2) → θ1/θ5/θ4/θ6 → 弧度转 0-360° → 关节限位
    折叠 (±360° 映射) → 按 joint_weight 加权"离当前角最近"选最优解.

    Args:
        T_target: 4×4 目标位姿矩阵 (平移列 = 目标 xyz mm).
        dh: DH 表 (默认 D_H).
        current_deg: 当前关节角 (度), 用于多解择优; None 默认全 0.
        joint_limits: 每关节 (min, max) 度; None 不折叠 (全部有效).

    Returns:
        最优 6 关节角 (度), 或 None (无有效解 / workspace 外).

    Note:
        * 第一版仅位置: T_target 旋转部分应为 T_0_6_RESET (姿态锁定), 但本函数
          接受任意旋转矩阵 (θ5/θ4/θ6 依赖旋转列).
        * 与源项目一致: 输出即输出轴真实角 (anchor 空间).
    """
    if dh is None:
        dh = _DH_ARR
    a2, a3, d4 = dh[2, 0], dh[3, 0], dh[3, 2]
    cur = (np.radians(np.asarray(current_deg, dtype=float)) if current_deg
           else np.zeros(6))

    # θ3 两组 → 各 θ2 两组 → 4 组候选
    th3_1, th3_2 = _calc_theta3(T_target, a2, a3, d4)
    cand = []  # list of [th1..th6]
    for th3 in (th3_1, th3_2):
        if math.isnan(th3):
            continue                    # workspace 外, 跳过该 θ3 支
        th2_1, th2_2 = _calc_theta2(th3, T_target, a2, a3, d4)
        for th2 in (th2_1, th2_2):
            if math.isnan(th2):
                continue
            th1 = _calc_theta1(th2, th3, T_target, a2, a3, d4)
            if math.isnan(th1):
                continue
            th5 = _calc_theta5(th1, th2, th3, T_target)
            th4 = _calc_theta4(th1, th2, th3, th5, T_target)
            th6 = _calc_theta6(th1, th2, th3, th4, th5, T_target)
            cand.append([th1, th2, th3, th4, th5, th6])
    if not cand:
        return None

    # 弧度 → 0-360° (源项目 radians_to_degrees_0_360)
    def to_deg_0_360(rad: float) -> float:
        d = math.degrees(rad) % 360.0
        if d < 0:
            d += 360.0
        if abs(d - 360.0) < 1e-6:
            d = 0.0
        return d

    deg_solutions = [[to_deg_0_360(v) for v in sol] for sol in cand]
    n_cand = len(deg_solutions)

    # 关节限位折叠: <min +360, >max −360, 仍越界 → 标记无效 (源项目 joint_angle_map)
    valid = [True] * n_cand
    if joint_limits is not None:
        for i in range(n_cand):
            for j in range(6):
                lo, hi = joint_limits[j]
                ang = deg_solutions[i][j]
                if abs(ang - lo) < _ERROR_RANGE:
                    ang = lo
                if abs(ang - hi) < _ERROR_RANGE:
                    ang = hi
                if ang < lo:
                    ang += 360.0
                elif ang > hi:
                    ang -= 360.0
                if ang < lo or ang > hi:
                    valid[i] = False
                    break
                deg_solutions[i][j] = ang

    # 加权最近邻择优 (源项目 get_optimal_result)
    best_idx = -1
    min_diff = float("inf")
    for i in range(n_cand):
        if not valid[i]:
            continue
        diff = sum(abs(math.radians(deg_solutions[i][j]) - cur[j])
                   * JOINT_WEIGHT[j] for j in range(6))
        if diff < min_diff:
            min_diff = diff
            best_idx = i
    if best_idx == -1:
        return None
    return deg_solutions[best_idx]


# 便捷: 用复位姿态矩阵做 3DOF 位置 IK (第一版用途)
def ik_position(xyz: list[float], current_deg: list[float] | None = None,
                joint_limits: list[tuple[float, float]] | None = None,
                base_T: np.ndarray | None = None,
                frame: str = "anchor") -> Optional[list[float]]:
    """3DOF 位置 IK: 姿态锁 T_0_6_RESET, 只求目标 xyz 的关节角.

    Args:
        xyz: 目标末端位置 (mm, 基座系).
        current_deg: 当前关节角 (度), 多解择优.
        joint_limits: 关节限位 (度); None = 不折叠 (全部候选有效).
        base_T: 姿态基准矩阵 (默认 T_0_6_RESET).
        frame: "source" (源项目帧, 复位=RESET_POSE_DEG) 或 "anchor"
            (本项目帧, 开机姿态=全 0). anchor 帧对 q_source 做
            q_source − SOURCE_TO_ANCHOR_OFFSET 偏移并归一到 0-360°.

    Returns:
        最优 6 关节角 (度, 按 frame), 或 None.
    """
    T = (base_T.copy() if base_T is not None else T_0_6_RESET.copy())
    T[0, 3], T[1, 3], T[2, 3] = xyz[0], xyz[1], xyz[2]
    sol = ik_analytic(T, current_deg=current_deg, joint_limits=joint_limits)
    if sol is None:
        return None
    if frame == "anchor":
        # q_anchor = q_source − offset, 归一到 [0, 360)
        sol = [(v - SOURCE_TO_ANCHOR_OFFSET[i]) % 360.0 for i, v in enumerate(sol)]
    return sol


def source_to_anchor(q_source: list[float]) -> list[float]:
    """source 帧关节角 → anchor 帧 (开机=0). 逐关节减 offset 归一到 [0,360)."""
    return [(v - SOURCE_TO_ANCHOR_OFFSET[i]) % 360.0
            for i, v in enumerate(q_source)]


def anchor_to_source(q_anchor: list[float]) -> list[float]:
    """anchor 帧关节角 → source 帧 (复位=RESET_POSE_DEG)."""
    return [(v + SOURCE_TO_ANCHOR_OFFSET[i]) % 360.0
            for i, v in enumerate(q_anchor)]


# ── SO(3)/奇异度扩展 (2026-08-23, spec TASK-25) ─────────────

# 雅可比位置列归一化尺度: 臂特征长度 (≈ 连杆 3 长度 200mm).
# 归一化后 SVD 的条件数与位置/旋转列的绝对单位无关 (修订 #1).
JACOBIAN_LENGTH_SCALE_MM: float = 200.0


def log_so3(R: np.ndarray, theta_small: float = 1e-6,
            theta_pi: float = 1e-3) -> np.ndarray:
    """SO(3) → 轴角向量 ∈ R³ (rad). 内部控制旋转表示 (禁止 Euler 累加).

    三分支保证近 0/近 π 稳健 (修订 #3):
      * θ < theta_small   → 一阶近似 ½·unskew(R−Rᵀ), 避开 1/sinθ;
      * sinθ≈0 且 θ≈π      → 从对称部 R≈2uuᵀ−I 提取轴 (对角 + 行列符号),
                             取 θ=π (exp(±πu) 同 R, 符号确定性归一);
      * 常规支             → log = unskew(R−Rᵀ) · θ/sinθ.
    """
    R = np.asarray(R, float)
    cos_t = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    theta = math.acos(cos_t)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    n = float(np.linalg.norm(w))                 # = 2·sinθ
    if theta < theta_small:
        return 0.5 * w
    if n < 1e-8 or (math.pi - theta) < theta_pi:
        d = np.clip((np.diag(R) + 1.0) / 2.0, 0.0, 1.0)
        u = np.sqrt(d)
        # 用非对角元恢复符号 (R_ij = 2·u_i·u_j)
        if u[0] > 1e-6:
            u[1] = math.copysign(u[1], R[0, 1])
            u[2] = math.copysign(u[2], R[0, 2])
        elif u[1] > 1e-6:
            u[2] = math.copysign(u[2], R[1, 2])
        i = int(np.argmax(np.abs(u)))            # 全局符号确定性归一
        if u[i] < 0.0:
            u = -u
        return math.pi * u
    return w * (theta / n)


def singularity_metrics(J: np.ndarray, length_scale: float = JACOBIAN_LENGTH_SCALE_MM) -> dict:
    """雅可比 SVD 奇异度指标 — 位置列先按 length_scale 归一化 (修订 #1).

    J 列 = [vx,vy,vz,wx,wy,wz]; 位置列 (前 3) mm, 旋转列无量纲.
    归一化后条件数与单位无关; DLS 的物理 λ 仍用未归一化 J (本函数只做检测).

    Returns:
        {sigma_min, sigma_max, condition_number, manipulability, length_scale}
        condition_number = sigma_max/sigma_min (sigma_min→0 → inf, 调用方处理).
    """
    Jn = np.asarray(J, float).copy()
    Jn[:, :3] /= length_scale
    s = np.linalg.svd(Jn, compute_uv=False)
    sigma_min = float(s[-1])
    sigma_max = float(s[0])
    return {
        "sigma_min": sigma_min,
        "sigma_max": sigma_max,
        "condition_number": (sigma_max / sigma_min if sigma_min > 1e-12
                             else float("inf")),
        "manipulability": float(np.prod(s)),
        "length_scale": float(length_scale),
    }


def adaptive_damping(metrics: dict, base_lam: float,
                     near_ratio: float = 0.3, sing_ratio: float = 0.1,
                     lam_max: float | None = None) -> tuple[float, float]:
    """三档阻尼 + 速度缩放. 返回 (λ, velocity_scale).

    阈值 = sigma_min/sigma_max (归一化后单位无关; 真机 bring-up 实测校准).
      NORMAL       (ratio > near_ratio): (base_lam, 1.0)
      NEAR_SINGULAR: λ↑ scale↓ 线性内插, 实际参与 (twist *= scale)
      SINGULAR     (ratio ≤ sing_ratio): (lam_max, 0.0) → 调用方停车/拒绝
    """
    ratio = metrics["sigma_min"] / max(metrics["sigma_max"], 1e-12)
    if ratio > near_ratio:
        return base_lam, 1.0
    lam_max = lam_max if lam_max is not None else base_lam * 5.0
    if ratio <= sing_ratio:
        return lam_max, 0.0
    t = (ratio - sing_ratio) / (near_ratio - sing_ratio)   # (0,1), 越近奇异越小
    lam = base_lam + (lam_max - base_lam) * (1.0 - t)
    return lam, t
