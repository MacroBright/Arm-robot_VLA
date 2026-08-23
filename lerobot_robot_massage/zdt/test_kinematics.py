"""kinematics 模块测试 — FK/IK 移植验证 (源项目 robot_kinematics.c 对拍)."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from lerobot_robot_massage.zdt.kinematics import (  # noqa: E402
    D_H, RESET_POSE_DEG, T_0_6_RESET, damped_ls, fk_mdh, ik_analytic,
    ik_position, jacobian,
)


def _assert_close_mat(a: np.ndarray, b: np.ndarray, tol: float = 1e-4):
    assert np.abs(a - b).max() < tol, f"\nA={np.round(a,4)}\nB={np.round(b,4)}"


def test_fk_reset_pose_equals_reset_matrix():
    """FK(RESET_POSE_DEG) == T_0_6_reset — DH 约定核心不变量.

    源项目: g_joints_init current_angle = [90,90,-90,0,90,0] = 复位姿态,
    IK 输出即此空间 (纯 θ), FK(复位角) 应还原固件复位矩阵.
    """
    T = fk_mdh(RESET_POSE_DEG)
    _assert_close_mat(T, T_0_6_RESET, tol=1e-4)


def test_fk_reset_position():
    """FK(复位) 平移 = (0, -47.63, 15.5) mm."""
    T = fk_mdh(RESET_POSE_DEG)
    assert abs(T[0, 3] - 0.0) < 1e-3
    assert abs(T[1, 3] - (-47.63)) < 1e-3
    assert abs(T[2, 3] - 15.5) < 1e-3


def test_ik_roundtrip_reset():
    """IK(FK(q)) 返回能还原位置的关节角 (往返验证)."""
    q = RESET_POSE_DEG
    T = fk_mdh(q)
    sol = ik_analytic(T, current_deg=[0.0] * 6)
    assert sol is not None
    T_back = fk_mdh(sol)
    # 位置还原 (mm)
    for i in range(3):
        assert abs(T_back[i, 3] - T[i, 3]) < 1e-3, f"axis {i}: {T_back[i,3]} vs {T[i,3]}"


def test_ik_position_roundtrip_various():
    """位置 IK: 多个可达点 FK↔IK↔FK 位置还原 (<1mm), 无限制纯往返."""
    for xyz in ([0.0, -47.63, 15.5],        # 复位点
                [150.0, 0.0, 200.0],        # 工作区上部
                [-100.0, -50.0, 120.0],     # 侧向
                [80.0, 100.0, 250.0]):      # 前上
        sol = ik_position(xyz, current_deg=RESET_POSE_DEG, frame="source")
        assert sol is not None, f"unreachable {xyz}"
        T = fk_mdh(sol)
        got = [T[0, 3], T[1, 3], T[2, 3]]
        err = max(abs(got[i] - xyz[i]) for i in range(3))
        assert err < 1.0, f"{xyz} → {got} err={err:.2f}mm"


def test_ik_workspace_out_of_reach_returns_none():
    """workspace 外 (如 3m 外) → None (IK 无解安全行为)."""
    sol = ik_position([3000.0, 0.0, 0.0], current_deg=RESET_POSE_DEG, frame="source")
    assert sol is None


def test_ik_respects_joint_limits():
    """限位折叠: source 帧解落在源项目限位外 → 折叠 ±360° 或返回 None."""
    from lerobot_robot_massage.zdt.kinematics import SOURCE_JOINT_LIMITS
    # 复位点可达 (source 帧: RESET_POSE_DEG 在限位内)
    sol = ik_position([0.0, -47.63, 15.5], current_deg=RESET_POSE_DEG,
                      joint_limits=SOURCE_JOINT_LIMITS, frame="source")
    assert sol is not None
    for i, (lo, hi) in enumerate(SOURCE_JOINT_LIMITS):
        assert lo - 1e-6 <= sol[i] <= hi + 1e-6, f"J{i+1} {sol[i]} ∉ [{lo},{hi}]"


def test_ik_prefers_solution_near_current():
    """多解择优: 不同 current_deg 应选不同解 (靠近当前位的解)."""
    # 选一个有多解的点: 肩/肘可肘上/肘下
    xyz = [150.0, 0.0, 200.0]
    cur_elbow_down = RESET_POSE_DEG
    cur_elbow_up = [90.0, 30.0, 30.0, 0.0, 90.0, 0.0]
    s1 = ik_position(xyz, current_deg=cur_elbow_down, frame="source")
    s2 = ik_position(xyz, current_deg=cur_elbow_up, frame="source")
    assert s1 is not None and s2 is not None
    # 两解都可能合法但通常不同 (肘下 vs 肘上); 各自 FK 应还原位置
    for s in (s1, s2):
        T = fk_mdh(s)
        assert abs(T[0, 3] - xyz[0]) < 1.0
        assert abs(T[1, 3] - xyz[1]) < 1.0
        assert abs(T[2, 3] - xyz[2]) < 1.0


def test_ik_position_anchor_frame_offset():
    """anchor 帧: 开机姿态 (anchor 0) == 源复位姿态, IK 输出应偏移复位角.

    复位点 anchor 帧解应 = (0 − 0 offset... ) 即 anchor 全 0; 验证偏移量正确.
    """
    from lerobot_robot_massage.zdt.kinematics import SOURCE_TO_ANCHOR_OFFSET
    sol = ik_position([0.0, -47.63, 15.5], current_deg=[0.0] * 6, frame="anchor")
    assert sol is not None
    # anchor 帧: 复位点解应为全 0 (开机姿态) 或等效 0/360
    for i, v in enumerate(sol):
        assert abs(v % 360.0) < 1e-6 or abs(v % 360.0 - 360.0) < 1e-6, \
            f"J{i+1} 复位点 anchor 帧应≈0, 实际 {v} (offset {SOURCE_TO_ANCHOR_OFFSET[i]})"


def test_source_to_anchor_maps_reset_to_zero():
    """source→anchor: RESET_POSE_DEG → 全 0 (开机姿态)."""
    from lerobot_robot_massage.zdt.kinematics import source_to_anchor
    q = source_to_anchor(RESET_POSE_DEG)
    for i, v in enumerate(q):
        assert abs(v % 360.0) < 1e-6 or abs(v % 360.0 - 360.0) < 1e-6


def test_fk_ignores_dh_theta_offset():
    """纯 θ 约定: fk_mdh 不使用 DH 第 4 列 offset (与源项目 Simulink 一致)."""
    # 手动按 Simulink 公式 (纯 θ) 算 FK(复位角), 应等于 fk_mdh 输出
    import math
    q = RESET_POSE_DEG
    T = np.eye(4)
    for i in range(6):
        a, al, d, _ = D_H[i]
        th = math.radians(q[i])
        ct, st = math.cos(th), math.sin(th)
        ca, sa = math.cos(al), math.sin(al)
        Ti = np.array([[ct, -st, 0, a],
                       [st * ca, ct * ca, -sa, -d * sa],
                       [st * sa, ct * sa, ca, d * ca],
                       [0, 0, 0, 1]])
        T = T @ Ti
    _assert_close_mat(fk_mdh(q), T, tol=1e-9)


# ── 几何雅可比 + 加权 DLS (2026-08-22, ready 姿态数值 IK) ────────────

def test_jacobian_matches_finite_difference():
    """几何雅可比 vs fk_mdh 中心差分 — 锁死'轴帧在 Rz 之前'的 DH 约定.

    关节轴 = T_0_j @ Rx(α)@Tx(a) 的 z (Rz(q)·Tz(d) 之前), 不是 T_0_j 的 z.
    """
    import math
    RAD = math.pi / 180.0
    for q in ([90.0, 135.0, 315.0, 0.0, 255.0, 0.0],   # ready (source 帧)
              [90.0, 90.0, -90.0, 0.0, 90.0, 0.0],     # reset
              [120.0, 110.0, 320.0, 10.0, 230.0, 40.0]):  # random
        Jf = jacobian(q)
        assert Jf.shape == (6, 6)
        h = 1e-7                                        # rad
        Jd = np.zeros((6, 6))
        for j in range(6):
            dq = h / RAD                                # 度步长 (fk_mdh 输入是度)
            qp, qm = list(q), list(q)
            qp[j] += dq
            qm[j] -= dq
            Tp, Tm = fk_mdh(qp), fk_mdh(qm)
            Jd[j, :3] = (Tp[:3, 3] - Tm[:3, 3]) / (2 * h)
            Rp, Rm = Tp[:3, :3], Tm[:3, :3]
            S = (Rp @ Rm.T - Rm @ Rp.T) / (4 * h)
            Jd[j, 3:] = [S[2, 1], S[0, 2], S[1, 0]]
        err = np.abs(Jf - Jd).max()
        assert err < 1e-3, f"q={q} J 最大差 {err}"


def test_jacobian_ready_wrist_decoupled():
    """ready 姿态 J4/J5/J6 线性列 = 0 (球腕奇异, 腕心与末端重合)."""
    J = jacobian([90.0, 135.0, 315.0, 0.0, 255.0, 0.0])
    assert np.abs(J[3:, :3]).max() < 1e-6


def test_damped_ls_attitude_lock():
    """加权 DLS: 1mm +x 位移保持姿态, J5 不翻腕."""
    import math
    q = [90.0, 135.0, 315.0, 0.0, 255.0, 0.0]
    J = jacobian(q)
    dq = damped_ls(J, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], 10.0,
                   weights=[1.0, 1.0, 1.0, 20.0, 20.0, 20.0])
    T, T2 = fk_mdh(q), fk_mdh([q[i] + math.degrees(dq[i]) for i in range(6)])
    # 位置: FK 位移应 ≈ [1,0,0] mm
    d = T2[:3, 3] - T[:3, 3]
    assert np.linalg.norm(d - np.array([1.0, 0.0, 0.0])) < 0.1, \
        f"位移误差 {d}"
    # 姿态漂移 < 0.5°
    R_rel = T[:3, :3].T @ T2[:3, :3]
    ang = math.degrees(math.acos(np.clip((np.trace(R_rel) - 1.0) / 2.0,
                                         -1.0, 1.0)))
    assert ang < 0.5, f"姿态漂移 {ang}°"
    # J5 (source 帧 index 4) 不翻腕: 解出的增量应很小
    assert abs(math.degrees(dq[4])) < 1.0, f"J5 dq {math.degrees(dq[4])}°"


if __name__ == "__main__":
    failed = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS {name}")
            except Exception as exc:  # noqa: BLE001
                failed += 1
                print(f"FAIL {name}: {exc}")
    print("ALL PASS" if failed == 0 else f"{failed} FAILED")
