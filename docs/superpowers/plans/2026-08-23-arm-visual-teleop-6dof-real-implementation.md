# 真机 6DOF 视觉遥操实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将已批准的真机 6DOF 视觉遥操设计（spec）的全部可代码化差距落地——types/kinematics/workspace/safety 状态机/controller/adapter/watchdog/recording/real_arm_teleop——并通过仿真回归验证；真机 TASK-27~34 保持 pending。

**Architecture:** 严格按依赖链分层实现：`types → kinematics → workspace → safety/state-machine → controller → adapter → watchdog → recording → real_arm_teleop → simulation regression`。控制层 `CartesianController` 是唯一笛卡尔运动入口，负责测量单调 dt（有界）、陈旧命令看门狗（单调期限）、6DOF 安全链与 SO(3) 姿态闭环；`RobotStateMachine` 是整臂生命周期门禁，枚举硬不变式失败即 FAULT 闩锁、禁止 ARM。

**Tech Stack:** Python 3.10+，numpy（无新增第三方依赖），测试用 pytest / 各 test 文件自带 `__main__` runner。真机 CAN 走 `zdt` 模块；视觉复用 Leap_Hand 共享模块（camera / hand_tracker / wrist_tracker）。

**Spec:** `Arm-robot_VLA/docs/superpowers/specs/2026-08-23-arm-visual-teleop-6dof-real-design.md`（本文档只实现其中"可代码化"部分，即上游 Plan TASK-01~26 + TASK-35；TASK-27~34 真机验证路径不执行，保持 `pending: 真机验证`）。

## Global Constraints

- **内部姿态表示 = SO(3) 矩阵 / 轴角向量**；RPY/Euler 仅可作可选安全约束（`step_pose` 的 `rpy_limits`），**绝不进入内部控制回路**（禁止 Euler 累加）。
- **奇异度指标先归一化再 SVD**：`singularity_metrics` 对雅可比位置列按 `JACOBIAN_LENGTH_SCALE_MM=200.0` 归一化，条件数与单位无关；`adaptive_damping` 返回的 λ/scale 必须实际参与运动（`twist *= scale`、`damped_ls` 用 λ），非仅 telemetry。
- **测量单调 dt**：`CartesianController.step()` 用 `time.monotonic()` 实测帧间隔，钳到 `[dt_min_s, dt_max_s]`，不假设固定 20Hz；首帧用 `dt_default_s = 1/loop_hz`。
- **陈旧命令看门狗最终归控制层**：`step(cmd_ts=...)` 用单调期限判定陈旧（`now - cmd_ts > stale_cmd_max_s` → 拒动），视觉层 VisionWatchdog 只做视觉信号分级。
- **枚举硬不变式**：`connect()` 扫描后必须 6 轴全在线 + 关节槽一一映射；任一缺失/重复/未映射/越界 → `robot.fault(reason)` 闩锁 FAULT 并抛 `SafetyError`，**禁止 ARM**。
- **CartesianController 是唯一笛卡尔运动入口**：adapter/遥操不得绕过它直接发 joint/CAN 命令；`RealArmAdapter` 不实现 CAN 协议、不重复 IK、不直接操作电机帧。
- **reset()/ready() 是实际运动操作**：必须显式 ARM（状态机 ARMED）后才可调用，**绝不**作为 `connect()` 的隐式路径或 LeRobot 生命周期自动动作；`MassageRobot.reset()` 需 `config.gravity_confirm=True` 显式确认才允许自动 ready（见 Task 5d 调用方适配）。Episode 纯软件复位（不改关节）应走独立路径，不得复用 `reset()`。
- 任务 T5 按评审拆分为 **T5a~T5d**（driver/scan → controller 生命周期 → get_real_state → 调用方适配），依赖顺序不变，每个子任务独立测试 + 独立 commit。
- **测试命令统一用 `conda run -n leap_hand python -m pytest <文件> -q`**（spec §8 基线命令；`leap_hand` 为 Python 3.14）。⚠ `zdt/types.py` 与标准库 `types` 同名：`python lerobot_robot_massage/zdt/test_types.py` 直跑时脚本目录 zdt/ 在 sys.path 上会遮蔽 stdlib `types`（3.14 下 dataclasses→re→enum→types 链崩）；**必须用 pytest**（rootdir 不把 zdt/ 放进 sys.path）。生产包导入（`from lerobot_robot_massage.zdt.types import ...`）无此问题。
- 单位：角度 deg、长度 mm、角速度 rad/s、时间戳/期限一律 `time.monotonic()` 秒。
- **现有 171 测试终态全绿**。以下是有意行为变化，对应测试在所属任务内同步更新并随提交保持绿：
  1. `connect()` 不再 `set_torque(True)`（改由 `arm()` 使能扭矩）；
  2. `CartesianController.step()` 引入测量 dt 与 ARMED 门禁（多帧距离测试改注入 FakeClock，step 测试需先置 ARMED）；
  3. `connect()` 走 scan/verify 流程（`test_connect_failure_estop_and_close` 重写为新语义）。
- **TASK-27~34（真机 bring-up / 无动力 / 低速 / 视觉真机测试）本计划不执行**，仅在第 12 步上游 Plan 中标记 `pending: 真机验证`。

## 修订落实映射（六条 + dt）

| 修订 | 落实任务 |
|---|---|
| 1. 雅可比归一化后再算奇异度 | Task 2 `singularity_metrics(J, length_scale)` + Task 6 step 中参与 λ/scale |
| 2. SO(3)/轴角为内部姿态表示；RPY 仅可选安全约束 | Task 1 `EEPose.rotation=SO(3)` + Task 6 `step_pose`（仅 `log_so3`，RPY 只在可选 `rpy_limits`） |
| 3. log_so3 近 0/近 π 稳健 + 单测 | Task 2 `log_so3` 三分支（近恒等 / 常规 / 近 π）+ 专项测试 |
| 4. 陈旧命令看门狗归控制层（单调期限） | Task 6 `step(cmd_ts)` 单调期限判定 + Task 8 VisionWatchdog 依赖控制层权威 |
| 5. EpisodeRecorder JSONL 引用真实相机帧文件 | Task 9 `camera_frames` 相对路径 + 帧落盘 |
| 6. 枚举/校验失败 = 硬不变式 | Task 4 `verify_enumeration` + Task 5 `connect()` 执行 |
| 7. 测量单调 dt 有界（非固定 20Hz） | Task 6 step 测量 dt + `dt_min_s/dt_max_s` |

## 依赖链 → 任务

```
types(T1) → kinematics(T2) → workspace(T3) → safety/state-machine(T4)
→ controller: T5a(driver/scan) → T5b(生命周期) → T5c(get_real_state) → T5d(调用方)
→ CartesianController(T6) → adapter(T7) → watchdog(T8) → recording(T9)
→ real_arm_teleop(T10) → simulation regression(T11) → 文档收尾 + 上游 Plan 标记(T12)
```

> T5 按评审拆为 T5a~T5d：每个子任务独立测试 + 独立 commit + 独立评审，顺序不变。

---

## Task 1: `zdt/types.py` — 共享数据类型

**Files:**
- Create: `Arm-robot_VLA/lerobot_robot_massage/zdt/types.py`
- Test: `Arm-robot_VLA/lerobot_robot_massage/zdt/test_types.py`

**Interfaces:**
- Produces:
  - `CartesianCommand(linear_velocity: tuple[float,float,float], angular_velocity: tuple[float,float,float] = (0.,0.,0.), timestamp: float = 0.0)` — frozen dataclass；`.twist -> np.ndarray(6)`
  - `JointState(q: tuple[float,...], dq: tuple[float,...] = (0.,)*6, current_ma: tuple[float,...] = (), flags: tuple[int, ...] = (), status: str = "")`
  - `EEPose(position: np.ndarray, rotation: np.ndarray)` — `position` (3,) mm，`rotation` (3,3) **SO(3)**；`.to_quaternion() -> tuple[w,x,y,z]`；`.to_rotation_vector() -> np.ndarray(3)`
  - `rotmat_to_quat(R) -> tuple[float,float,float,float]`（Shepperd 稳健法，wxyz）

- [ ] **Step 1: 写失败测试**

```python
"""zdt/types.py 测试 — 共享数据契约 (spec TASK-06/24)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import numpy as np

from lerobot_robot_massage.zdt.types import (  # noqa: E402
    CartesianCommand, EEPose, JointState, rotmat_to_quat,
)


def _rotz(deg):
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def test_cartesian_command_defaults():
    cmd = CartesianCommand(linear_velocity=(1.0, 2.0, 3.0))
    assert cmd.angular_velocity == (0.0, 0.0, 0.0)
    assert cmd.timestamp == 0.0
    assert cmd.twist.shape == (6,)
    np.testing.assert_allclose(cmd.twist, [1, 2, 3, 0, 0, 0])


def test_cartesian_command_immutable():
    cmd = CartesianCommand(linear_velocity=(1.0, 0.0, 0.0), timestamp=12.5)
    try:
        cmd.timestamp = 1.0
        raise AssertionError("frozen dataclass 应拒绝修改")
    except Exception:
        pass


def test_joint_state_defaults():
    js = JointState(q=(0.0,) * 6)
    assert len(js.dq) == 6
    assert js.flags == () and js.status == ""


def test_joint_state_flags_six_axis():
    # 6 轴 flags 全量保存 (P2-⑨): 任一轴堵转/失使能在 observation 可见
    js = JointState(q=(0.0,) * 6, flags=(1, 2, 4, 8, 0, 1))
    assert len(js.flags) == 6
    assert js.flags[2] == 4        # J3 堵转标志可见


def test_ee_pose_identity_quaternion():
    p = EEPose(position=np.zeros(3), rotation=np.eye(3))
    w, x, y, z = p.to_quaternion()
    assert abs(w - 1.0) < 1e-9 and abs(x) < 1e-9 and abs(y) < 1e-9 and abs(z) < 1e-9


def test_ee_pose_known_rotation_quaternion():
    # Rz(90°): 四元数 w=cos45, z=sin45
    p = EEPose(position=np.zeros(3), rotation=_rotz(90.0))
    w, x, y, z = p.to_quaternion()
    assert abs(w - np.cos(np.pi / 4)) < 1e-9
    assert abs(z - np.sin(np.pi / 4)) < 1e-9
    assert abs(x) < 1e-9 and abs(y) < 1e-9


def test_rotmat_to_quat_unit_norm_robust():
    # 近 π 旋转 + 随机旋转: 四元数必须单位模长
    for axis in (np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0])):
        K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        R = np.eye(3) + K * np.pi + (K @ K) * 2.0
        q = np.array(rotmat_to_quat(R))
        assert abs(np.linalg.norm(q) - 1.0) < 1e-9, f"axis={axis} q={q}"


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
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd Arm-robot_VLA && conda run -n leap_hand python -m pytest lerobot_robot_massage/zdt/test_types.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'lerobot_robot_massage.zdt.types'`

- [ ] **Step 3: 写实现**

```python
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
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd Arm-robot_VLA && conda run -n leap_hand python -m pytest lerobot_robot_massage/zdt/test_types.py -q`
Expected: PASS (ALL PASS)

- [ ] **Step 5: 提交**

```bash
git add Arm-robot_VLA/lerobot_robot_massage/zdt/types.py Arm-robot_VLA/lerobot_robot_massage/zdt/test_types.py
git -C Arm-robot_VLA commit -m "feat(zdt): types.py 共享数据契约 (CartesianCommand/JointState/EEPose, SO(3) 内部表示)"
```

---

## Task 2: `kinematics.py` — log_so3 / singularity_metrics / adaptive_damping

**Files:**
- Modify: `Arm-robot_VLA/lerobot_robot_massage/zdt/kinematics.py`
- Test: `Arm-robot_VLA/lerobot_robot_massage/zdt/test_kinematics.py`

**Interfaces:**
- Consumes: 现有 `jacobian`、`fk_mdh`（已有）。
- Produces:
  - `JACOBIAN_LENGTH_SCALE_MM: float = 200.0`
  - `log_so3(R: np.ndarray, theta_small: float = 1e-6, theta_pi: float = 1e-3) -> np.ndarray` — 轴角向量 (rad)，近 0/近 π 稳健
  - `singularity_metrics(J: np.ndarray, length_scale: float = JACOBIAN_LENGTH_SCALE_MM) -> dict` — 位置列先 `/= length_scale` 再 SVD；返回 `{sigma_min, sigma_max, condition_number, manipulability, length_scale}`
  - `adaptive_damping(metrics: dict, base_lam: float, near_ratio: float = 0.3, sing_ratio: float = 0.1, lam_max: float | None = None) -> tuple[float, float]`

- [ ] **Step 1: 写失败测试（追加到 test_kinematics.py 末尾）**

```python
# ── SO(3)/奇异度 (2026-08-23, spec TASK-25) ────────────────

def _exp_so3(w):
    """Rodrigues: 轴角 → SO(3). 测试辅助, 不依赖 scipy."""
    w = np.asarray(w, float)
    theta = float(np.linalg.norm(w))
    if theta < 1e-12:
        return np.eye(3)
    u = w / theta
    K = np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]])
    return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)


def test_log_so3_identity_is_zero():
    assert np.linalg.norm(log_so3(np.eye(3))) < 1e-12


def test_log_so3_small_angle_stable():
    # 近零: 一阶支, 无 1/sinθ 放大
    w = np.array([1e-8, 2e-8, -1e-8])
    got = log_so3(_exp_so3(w))
    assert np.linalg.norm(got) < 1e-6
    np.testing.assert_allclose(got, w, atol=1e-9)


def test_log_so3_known_axis_angle():
    w = np.array([0.3, -0.2, 0.5])
    got = log_so3(_exp_so3(w))
    np.testing.assert_allclose(got, w, atol=1e-9)


def test_log_so3_norm_equals_angle():
    for angle in (0.05, 1.2, 2.5, 3.0):
        w = np.array([1.0, 2.0, -1.0])
        w = w / np.linalg.norm(w) * angle
        assert abs(float(np.linalg.norm(log_so3(_exp_so3(w)))) - angle) < 1e-9


def test_log_so3_near_pi_roundtrip():
    # 近 π: 对角提取支, exp∘log ≈ R
    for axis in (np.array([1.0, 0, 0]), np.array([0.6, 0.8, 0.0]), np.array([0.3, -0.4, 0.85])):
        u = axis / np.linalg.norm(axis)
        R = _exp_so3(u * (math.pi - 1e-4))
        back = _exp_so3(log_so3(R))
        assert np.abs(back - R).max() < 1e-3, f"axis={u} err={np.abs(back-R).max()}"


def test_log_so3_exact_pi_roundtrip():
    for axis in (np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0]),
                 np.array([1.0, 1.0, 1.0])):
        u = axis / np.linalg.norm(axis)
        R = _exp_so3(u * math.pi)
        back = _exp_so3(log_so3(R))
        assert np.abs(back - R).max() < 1e-6, f"axis={u} err={np.abs(back-R).max()}"


def test_ee_pose_to_rotation_vector_matches_log_so3():
    # 修订 #2 链路: types.EEPose.to_rotation_vector == kinematics.log_so3
    # (该方法在 Task 1 定义但依赖本任务的 log_so3, 故测试放这里)
    from lerobot_robot_massage.zdt.types import EEPose
    w0 = np.array([0.3, -0.2, 0.5])
    p = EEPose(position=np.zeros(3), rotation=_exp_so3(w0))
    np.testing.assert_allclose(p.to_rotation_vector(), w0, atol=1e-9)


def test_singularity_metrics_unit_independent():
    # 修订 #1: 归一化不变性 — 位置列与 length_scale 同步缩放 (单位换算) 不改变条件数.
    # 固定 length_scale 下, cond 只取决于 J[:, :3]/length_scale 之比:
    # J2 = J×0.01 且 length_scale=2.0 (=200×0.01) → 比值不变 → cond 不变.
    from lerobot_robot_massage.zdt.kinematics import jacobian
    q = [90.0, 135.0, 315.0, 0.0, 255.0, 0.0]
    J = jacobian(q)
    m1 = singularity_metrics(J)
    J2 = J.copy()
    J2[:, :3] *= 0.01                     # 平移单位换算 (位置列缩小 100×)
    m2 = singularity_metrics(J2, length_scale=2.0)   # length_scale 同步缩 100×
    assert abs(m1["condition_number"] - m2["condition_number"]) < 1e-6
    assert m1["sigma_min"] <= m1["sigma_max"] + 1e-12


def test_singularity_metrics_order_and_manip():
    from lerobot_robot_massage.zdt.kinematics import jacobian
    for q in ([90.0, 135.0, 315.0, 0.0, 255.0, 0.0], [90.0, 90.0, -90.0, 0.0, 90.0, 0.0]):
        m = singularity_metrics(jacobian(q))
        assert m["sigma_min"] >= 0
        assert m["sigma_max"] > 0
        assert m["manipulability"] > 0
        assert m["condition_number"] >= 1.0


def test_adaptive_damping_normal_band():
    m = {"sigma_min": 0.5, "sigma_max": 1.0, "condition_number": 2.0,
         "manipulability": 0.3, "length_scale": 200.0}
    lam, scale = adaptive_damping(m, 10.0, near_ratio=0.3, sing_ratio=0.1)
    assert lam == 10.0 and scale == 1.0


def test_adaptive_damping_singular_band():
    m = {"sigma_min": 0.05, "sigma_max": 1.0, "condition_number": 20.0,
         "manipulability": 0.0, "length_scale": 200.0}
    lam, scale = adaptive_damping(m, 10.0, near_ratio=0.3, sing_ratio=0.1)
    assert scale == 0.0
    assert lam == 50.0  # lam_max = base*5


def test_adaptive_damping_near_band_interpolates():
    m = {"sigma_min": 0.2, "sigma_max": 1.0, "condition_number": 5.0,
         "manipulability": 0.1, "length_scale": 200.0}
    lam, scale = adaptive_damping(m, 10.0, near_ratio=0.3, sing_ratio=0.1)
    assert 0.0 < scale < 1.0
    assert 10.0 < lam < 50.0
```

- [ ] **Step 2: 运行测试确认失败**

Run: `python Arm-robot_VLA/lerobot_robot_massage/zdt/test_kinematics.py`
Expected: FAIL — `ImportError: cannot import name 'log_so3'`（或 singularity_metrics / adaptive_damping）

- [ ] **Step 3: 写实现（追加到 kinematics.py）**

```python
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
```

（`damped_ls` 与现有函数不变；`math`/`np` 已在模块顶部 import。）

- [ ] **Step 4: 运行测试确认通过**

Run: `python Arm-robot_VLA/lerobot_robot_massage/zdt/test_kinematics.py`
Expected: PASS（旧测试 + 新增全部通过）

- [ ] **Step 5: 提交**

```bash
git add Arm-robot_VLA/lerobot_robot_massage/zdt/kinematics.py Arm-robot_VLA/lerobot_robot_massage/zdt/test_kinematics.py
git -C Arm-robot_VLA commit -m "feat(zdt): log_so3 近0/近π稳健 + 归一化奇异度指标 + 三档自适应阻尼 (修订#1/#3)"
```

---

## Task 3: `zdt/workspace.py` — BoxWorkspace + CartesianVelocityLimiter

**Files:**
- Create: `Arm-robot_VLA/lerobot_robot_massage/zdt/workspace.py`
- Test: `Arm-robot_VLA/lerobot_robot_massage/zdt/test_workspace.py`

**Interfaces:**
- Produces:
  - `BoxWorkspace(xyz_min, xyz_max)` — `contains(xyz) -> bool`；`clamp(xyz) -> np.ndarray`；`scale_velocity(vel, pos, dt) -> tuple[np.ndarray, list[int]]`（**velocity 在前**，与 `CartesianVelocityLimiter.__call__(vel, pos, dt)` 同序）
  - `CartesianVelocityLimiter(max_vel_mm_s, workspace: BoxWorkspace | None = None)` — `__call__(vel, pos, dt) -> tuple[np.ndarray, list[int]]`

- [ ] **Step 1: 写失败测试**

```python
"""workspace 盒 + 速度限幅器测试 (spec TASK-13)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import numpy as np

from lerobot_robot_massage.zdt.workspace import (  # noqa: E402
    BoxWorkspace, CartesianVelocityLimiter,
)


def _box():
    return BoxWorkspace([-200.0, -200.0, 0.0], [200.0, 200.0, 300.0])


def test_contains_inside_outside_boundary():
    b = _box()
    assert b.contains([0.0, 0.0, 150.0])
    assert not b.contains([500.0, 0.0, 150.0])
    assert b.contains([200.0, -200.0, 300.0])   # 边界含


def test_clamp_components():
    b = _box()
    got = b.clamp([500.0, -500.0, 150.0])
    np.testing.assert_allclose(got, [200.0, -200.0, 150.0])


def test_scale_velocity_inside_unchanged():
    b = _box()
    v, clamped = b.scale_velocity(np.array([10.0, 0.0, 0.0]),
                                  np.array([0.0, 0.0, 150.0]), dt=0.1)
    np.testing.assert_allclose(v, [10.0, 0.0, 0.0])
    assert clamped == []


def test_scale_velocity_heading_out_stops_at_wall():
    b = _box()
    # x=190, 200mm/s 沿 +x, dt=0.1 → 目标 210 > 200 → 缩到刚好停在 200
    v, clamped = b.scale_velocity(np.array([200.0, 0.0, 0.0]),
                                  np.array([190.0, 0.0, 150.0]), dt=0.1)
    np.testing.assert_allclose(v, [100.0, 0.0, 0.0])   # (200-190)/0.1 = 100
    assert clamped == [0]


def test_scale_velocity_inside_high_velocity_unchanged():
    b = _box()
    # z=100, 300mm/s 向上, dt=0.1 → 目标 130 < 300 不越界 → 不变
    v, clamped = b.scale_velocity(np.array([0.0, 0.0, 300.0]),
                                  np.array([0.0, 0.0, 100.0]), dt=0.1)
    np.testing.assert_allclose(v, [0.0, 0.0, 300.0])
    assert clamped == []


def test_scale_velocity_already_out_blocked_axis():
    b = _box()
    # 已越界 (x=250>200) 再往外走 → 该轴速度置 0 (blocked)
    v, clamped = b.scale_velocity(np.array([100.0, 0.0, 0.0]),
                                  np.array([250.0, 0.0, 150.0]), dt=0.1)
    np.testing.assert_allclose(v, [0.0, 0.0, 0.0])
    assert clamped == [0]


def test_scale_velocity_moving_back_allowed():
    b = _box()
    v, clamped = b.scale_velocity(np.array([-100.0, 0.0, 0.0]),
                                  np.array([250.0, 0.0, 150.0]), dt=0.1)
    np.testing.assert_allclose(v, [-100.0, 0.0, 0.0])
    assert clamped == []


def test_limiter_clamps_max_speed():
    lim = CartesianVelocityLimiter(max_vel_mm_s=20.0)
    v, clamped = lim(np.array([100.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]), dt=0.1)
    assert abs(float(np.linalg.norm(v)) - 20.0) < 1e-9
    assert clamped == []


def test_limiter_applies_box():
    box = _box()
    lim = CartesianVelocityLimiter(max_vel_mm_s=200.0, workspace=box)
    # z=290, 300mm/s 向上, dt=0.1 → 目标 320 > 300 → 缩到 (300-290)/0.1=100
    v, clamped = lim(np.array([0.0, 0.0, 300.0]), np.array([0.0, 0.0, 290.0]), dt=0.1)
    np.testing.assert_allclose(v, [0.0, 0.0, 100.0])
    assert clamped == [2]


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
```

- [ ] **Step 2: 运行测试确认失败**

Run: `python Arm-robot_VLA/lerobot_robot_massage/zdt/test_workspace.py`
Expected: FAIL — `ModuleNotFoundError: No module named 'lerobot_robot_massage.zdt.workspace'`

- [ ] **Step 3: 写实现**

```python
"""zdt/workspace.py — 笛卡尔工作空间盒 + 速度限幅器 (spec TASK-13).

盒状工作空间是首版安全包络 (Sphere/Cylinder/Patient ROI 留待后续, YAGNI).
速度限幅器 = 最大末端速度 + 盒约束, 在 CartesianController.step 第 2/4 步调用.
"""
from __future__ import annotations

import numpy as np


class BoxWorkspace:
    """轴对齐盒工作空间 (xyz mm, 基座系)."""

    def __init__(self, xyz_min, xyz_max):
        self.min = np.asarray(xyz_min, dtype=float)
        self.max = np.asarray(xyz_max, dtype=float)
        if np.any(self.min >= self.max):
            raise ValueError(f"盒越界: min={self.min} max={self.max}")

    def contains(self, xyz) -> bool:
        p = np.asarray(xyz, dtype=float)
        return bool(np.all(p >= self.min) and np.all(p <= self.max))

    def clamp(self, xyz) -> np.ndarray:
        return np.clip(np.asarray(xyz, dtype=float), self.min, self.max)

    def scale_velocity(self, vel, pos, dt):
        """p_des = pos + vel·dt 越盒 → 该分量缩到盒边界 (至多到墙).

        ⚠ 参数顺序: (vel, pos, dt) — 与 CartesianVelocityLimiter.__call__ 同序.
        纪律: 只缩放/拒绝, 绝不放大, 也绝不自动纠偏 —
          盒内: 朝外越界分量缩到墙 (至多到墙), 朝内放行;
          盒外 (已越界): 朝内放行, 朝外/静止 → 该轴速度置 0 并标记 clamped
                         (静止时调用方据此拒绝整条命令).
        Returns (v_scaled, clamped_axes).
        """
        v = np.asarray(vel, dtype=float)
        p = np.asarray(pos, dtype=float)
        dt = float(dt)
        v_out = v.copy()
        clamped: list[int] = []
        for i in range(3):
            if p[i] < self.min[i] or p[i] > self.max[i]:
                # 已越界: 朝内放行; 朝外/静止 → 置 0 + clamped
                if v[i] > 0 and p[i] > self.max[i]:
                    v_out[i] = 0.0
                    clamped.append(i)
                elif v[i] < 0 and p[i] < self.min[i]:
                    v_out[i] = 0.0
                    clamped.append(i)
                elif abs(v[i]) < 1e-12:
                    v_out[i] = 0.0
                    clamped.append(i)
            else:
                target_i = p[i] + v[i] * dt
                if v[i] > 0 and target_i > self.max[i]:
                    v_out[i] = min(v[i], max(0.0, (self.max[i] - p[i]) / dt))
                    clamped.append(i)
                elif v[i] < 0 and target_i < self.min[i]:
                    v_out[i] = max(v[i], min(0.0, (self.min[i] - p[i]) / dt))
                    clamped.append(i)
        return v_out, clamped


class CartesianVelocityLimiter:
    """末端速度限幅: 最大速度 + 盒工作空间."""

    def __init__(self, max_vel_mm_s: float, workspace: BoxWorkspace | None = None):
        self.max_vel = float(max_vel_mm_s)
        self.workspace = workspace

    def __call__(self, vel, pos, dt):
        v = np.asarray(vel, dtype=float)
        n = float(np.linalg.norm(v))
        if n > self.max_vel and n > 1e-12:
            v = v * (self.max_vel / n)
        clamped: list[int] = []
        if self.workspace is not None:
            v, clamped = self.workspace.scale_velocity(v, pos, dt)
        return v, clamped
```

- [ ] **Step 4: 运行测试确认通过**

Run: `python Arm-robot_VLA/lerobot_robot_massage/zdt/test_workspace.py`
Expected: PASS

- [ ] **Step 5: 提交**

```bash
git add Arm-robot_VLA/lerobot_robot_massage/zdt/workspace.py Arm-robot_VLA/lerobot_robot_massage/zdt/test_workspace.py
git -C Arm-robot_VLA commit -m "feat(zdt): workspace.py 盒工作空间 + 笛卡尔速度限幅器 (TASK-13)"
```

---

## Task 4: `safety.py` — RobotStateMachine + 枚举硬不变式

**Files:**
- Modify: `Arm-robot_VLA/lerobot_robot_massage/zdt/safety.py`
- Test: `Arm-robot_VLA/lerobot_robot_massage/zdt/test_robot_state.py`

**Interfaces:**
- Consumes: `MotorState`（已有）。
- Produces:
  - `RobotPhase(Enum)` — DISCONNECTED, CONNECTED, ENUMERATED, SAFE_IDLE, ARMED, TELEOP, FAULT, STOPPED
  - `RobotStateMachine(num_joints=6)` — 方法 `on_connected / on_enumerated(motors) / on_safe_idle / arm(gravity_confirmed=False) / enter_teleop / exit_teleop / disarm / e_stop / fault(reason) / re_arm(confirmed) / assert_armed / assert_teleop`；属性 `phase`、`fault_reason`
  - `verify_enumeration(motors: dict[int, MotorState], num_joints: int = 6) -> list[str]` — 硬不变式，空列表=通过

- [ ] **Step 1: 写失败测试**

```python
"""RobotStateMachine 状态机 + 枚举硬不变式测试 (spec §5.1, 修订 #6)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from lerobot_robot_massage.zdt.safety import (  # noqa: E402
    MotorState, RobotPhase, RobotStateMachine, SafetyError, verify_enumeration,
)


def _motors(addrs=(0x02, 0x03, 0x04, 0x05, 0x06, 0x07)):
    return {a: MotorState(can_id=a, online=True, joint_slot=i)
            for i, a in enumerate(addrs)}


def _happy_machine():
    sm = RobotStateMachine()
    sm.on_connected()
    sm.on_enumerated(_motors())
    sm.on_safe_idle()
    return sm


def test_happy_path_full_cycle():
    sm = RobotStateMachine()
    sm.on_connected()
    sm.on_enumerated(_motors())
    sm.on_safe_idle()
    sm.arm(gravity_confirmed=True)
    sm.enter_teleop()
    assert sm.phase == RobotPhase.TELEOP
    sm.exit_teleop()
    assert sm.phase == RobotPhase.ARMED
    sm.e_stop()
    assert sm.phase == RobotPhase.STOPPED
    sm.re_arm(confirmed=True)
    assert sm.phase == RobotPhase.ENUMERATED
    sm.on_safe_idle()
    assert sm.phase == RobotPhase.SAFE_IDLE


def test_illegal_transition_rejected():
    sm = RobotStateMachine()
    try:
        sm.on_safe_idle()      # DISCONNECTED 直接跳 SAFE_IDLE
        raise AssertionError("应抛 SafetyError")
    except SafetyError:
        pass
    assert sm.phase == RobotPhase.DISCONNECTED


def test_arm_requires_gravity_confirmation():
    sm = _happy_machine()
    try:
        sm.arm(gravity_confirmed=False)
        raise AssertionError("应抛 SafetyError")
    except SafetyError:
        pass
    assert sm.phase == RobotPhase.SAFE_IDLE


def test_arm_requires_safe_idle():
    sm = RobotStateMachine()
    sm.on_connected()
    try:
        sm.arm(gravity_confirmed=True)
        raise AssertionError("应抛 SafetyError")
    except SafetyError:
        pass


def test_estop_latches():
    sm = _happy_machine()
    sm.arm(gravity_confirmed=True)
    sm.e_stop()
    assert sm.phase == RobotPhase.STOPPED
    try:
        sm.arm(gravity_confirmed=True)
        raise AssertionError("STOPPED 后应拒绝 arm")
    except SafetyError:
        pass
    try:
        sm.re_arm(confirmed=False)
        raise AssertionError("re_arm 无确认应拒绝")
    except SafetyError:
        pass


def test_fault_latches_and_reason():
    sm = _happy_machine()
    sm.fault("J4 MISSING")
    assert sm.phase == RobotPhase.FAULT
    assert sm.fault_reason == "J4 MISSING"
    try:
        sm.arm(gravity_confirmed=True)
        raise AssertionError("FAULT 后应拒绝 arm")
    except SafetyError:
        pass
    sm.re_arm(confirmed=True)
    assert sm.phase == RobotPhase.ENUMERATED


def test_terminal_latches_idempotent():
    # 闩锁幂等: FAULT 后 e_stop 不覆盖成 STOPPED (保留原因); STOPPED 后 fault 保持 STOPPED
    sm = _happy_machine()
    sm.fault("bus dead")
    sm.e_stop()
    assert sm.phase == RobotPhase.FAULT
    sm2 = _happy_machine()
    sm2.e_stop()
    sm2.fault("later fault")
    assert sm2.phase == RobotPhase.STOPPED


def test_disarm_returns_to_safe_idle():
    sm = _happy_machine()
    sm.arm(gravity_confirmed=True)
    sm.enter_teleop()
    sm.disarm()
    assert sm.phase == RobotPhase.SAFE_IDLE


def test_assert_armed_gates():
    sm = _happy_machine()
    try:
        sm.assert_armed()
        raise AssertionError("SAFE_IDLE 不应通过 assert_armed")
    except SafetyError:
        pass
    sm.arm(gravity_confirmed=True)
    sm.assert_armed()
    sm.enter_teleop()
    sm.assert_armed()      # TELEOP 也算 armed
    try:
        sm.assert_teleop()  # 非 TELEOP
        raise AssertionError("ARMED 不应通过 assert_teleop")
    except SafetyError:
        pass


def test_verify_enumeration_ok():
    assert verify_enumeration(_motors()) == []


def test_verify_enumeration_missing_motor():
    problems = verify_enumeration(_motors(addrs=(0x02, 0x03, 0x04, 0x05, 0x06)))
    assert any("J6 MISSING" in p for p in problems)


def test_verify_enumeration_duplicate_slot():
    motors = _motors()
    motors[0x08] = MotorState(can_id=0x08, online=True, joint_slot=0)   # 与 0x02 同槽
    problems = verify_enumeration(motors)
    assert any("重复" in p for p in problems)


def test_verify_enumeration_unmapped():
    motors = _motors()
    motors[0x09] = MotorState(can_id=0x09, online=True, joint_slot=None)
    problems = verify_enumeration(motors)
    assert any("未映射" in p for p in problems)


def test_verify_enumeration_offline():
    motors = _motors()
    motors[0x02].online = False
    problems = verify_enumeration(motors)
    assert any("不在线" in p for p in problems)


def test_verify_enumeration_wrong_count():
    problems = verify_enumeration(_motors(addrs=(0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08)))
    assert any("7 台" in p for p in problems)


def test_on_enumerated_hard_invariant_faults():
    # 硬不变式: 缺失任一电机 → on_enumerated 抛 SafetyError (调用方转 FAULT)
    sm = RobotStateMachine()
    sm.on_connected()
    motors = _motors(addrs=(0x02, 0x03, 0x04, 0x05, 0x06))   # 缺 J6
    try:
        sm.on_enumerated(motors)
        raise AssertionError("应抛 SafetyError")
    except SafetyError:
        pass
    assert sm.phase == RobotPhase.CONNECTED   # 状态不前进


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
```

- [ ] **Step 2: 运行测试确认失败**

Run: `python Arm-robot_VLA/lerobot_robot_massage/zdt/test_robot_state.py`
Expected: FAIL — `ImportError: cannot import name 'RobotStateMachine'`

- [ ] **Step 3: 写实现（追加到 safety.py，紧接现有 SafetyMachine 之后）**

```python
# ── 整臂生命周期状态机 (2026-08-23, spec §5.1) ─────────────

class RobotPhase(Enum):
    """整臂生命周期门禁 (与 SafetyMachine 的单电机枚举互补, 不合并)."""
    DISCONNECTED = auto()
    CONNECTED = auto()
    ENUMERATED = auto()
    SAFE_IDLE = auto()
    ARMED = auto()
    TELEOP = auto()
    FAULT = auto()
    STOPPED = auto()


class RobotStateMachine:
    """整臂门禁: connect→enumerate→safe_idle→arm→teleop; e_stop/fault 闩锁.

    枚举硬不变式 (修订 #6): on_enumerated 校验 6 轴在线 + 关节槽一一映射,
    任一缺失/重复/未映射 → 抛 SafetyError → 调用方 fault() (禁止 ARM).
    """

    def __init__(self, num_joints: int = 6):
        self.num_joints = num_joints
        self._phase = RobotPhase.DISCONNECTED
        self.fault_reason: str = ""

    @property
    def phase(self) -> RobotPhase:
        return self._phase

    def _require(self, *phases: RobotPhase) -> None:
        if self._phase not in phases:
            raise SafetyError(
                f"非法状态转移: {self._phase.name} → 需 {'/'.join(p.name for p in phases)}")

    def on_connected(self) -> None:
        self._require(RobotPhase.DISCONNECTED)
        self._phase = RobotPhase.CONNECTED

    def on_enumerated(self, motors: dict[int, MotorState]) -> None:
        """硬不变式: 校验通过才前进 ENUMERATED, 否则抛 SafetyError (不前进)."""
        self._require(RobotPhase.CONNECTED)
        problems = verify_enumeration(motors, self.num_joints)
        if problems:
            raise SafetyError("枚举不变式失败: " + "; ".join(problems))
        self._phase = RobotPhase.ENUMERATED

    def on_safe_idle(self) -> None:
        self._require(RobotPhase.ENUMERATED)
        self._phase = RobotPhase.SAFE_IDLE

    def arm(self, gravity_confirmed: bool = False) -> None:
        """SAFE_IDLE → ARMED. 重力关节 (J2/J3) 需显式二次确认."""
        self._require(RobotPhase.SAFE_IDLE)
        if not gravity_confirmed:
            raise SafetyError("重力关节 J2/J3 需显式二次确认才可臂置")
        self._phase = RobotPhase.ARMED

    def enter_teleop(self) -> None:
        self._require(RobotPhase.ARMED)
        self._phase = RobotPhase.TELEOP

    def exit_teleop(self) -> None:
        self._require(RobotPhase.TELEOP)
        self._phase = RobotPhase.ARMED

    def disarm(self) -> None:
        """ARMED/TELEOP → SAFE_IDLE (失能扭矩后回到安全待命)."""
        self._require(RobotPhase.ARMED, RobotPhase.TELEOP)
        self._phase = RobotPhase.SAFE_IDLE

    def e_stop(self) -> None:
        """* → STOPPED (闩锁). 幂等: 已处 FAULT/STOPPED 时不改写 (保留 FAULT 原因)."""
        if self._phase not in (RobotPhase.FAULT, RobotPhase.STOPPED):
            self._phase = RobotPhase.STOPPED

    def fault(self, reason: str) -> None:
        """* → FAULT (闩锁). 幂等; 安全效果等同 STOPPED; 恢复需 re_arm(confirmed)."""
        if self._phase not in (RobotPhase.FAULT, RobotPhase.STOPPED):
            self._phase = RobotPhase.FAULT
            self.fault_reason = reason

    def re_arm(self, confirmed: bool) -> None:
        """STOPPED/FAULT → ENUMERATED, 需显式确认 (随后重新 on_safe_idle)."""
        self._require(RobotPhase.STOPPED, RobotPhase.FAULT)
        if not confirmed:
            raise SafetyError("re_arm 需显式确认")
        self._phase = RobotPhase.ENUMERATED

    def assert_armed(self) -> None:
        if self._phase not in (RobotPhase.ARMED, RobotPhase.TELEOP):
            raise SafetyError(f"需 ARMED/TELEOP, 当前 {self._phase.name}")

    def assert_teleop(self) -> None:
        if self._phase != RobotPhase.TELEOP:
            raise SafetyError(f"需 TELEOP, 当前 {self._phase.name}")


def verify_enumeration(motors: dict[int, MotorState], num_joints: int = 6) -> list[str]:
    """枚举硬不变式 (修订 #6): 全部 num_joints 在线 + 关节槽一一映射.

    违例项 (任一存在 → 禁止 ARM):
      * 电机数量 != num_joints
      * 某电机不在线 / 未映射关节槽 (joint_slot is None)
      * 某槽位缺失 (MISSING) / 多台映射到同一槽位 (重复)
      * 槽位越界 (>= num_joints)

    Returns:
        list[str] 违例描述, 空列表 = 通过.
    """
    problems: list[str] = []
    if len(motors) != num_joints:
        problems.append(f"发现 {len(motors)} 台电机, 期望 {num_joints}")
    slots: dict[int, list[int]] = {}
    for cid, m in motors.items():
        if not m.online:
            problems.append(f"0x{cid:02X} 不在线")
        if m.joint_slot is None:
            problems.append(f"0x{cid:02X} 未映射关节槽")
        else:
            slots.setdefault(m.joint_slot, []).append(cid)
    for s in range(num_joints):
        if s not in slots:
            problems.append(f"J{s + 1} MISSING")
        elif len(slots[s]) > 1:
            problems.append(f"J{s + 1} 重复映射 {slots[s]}")
    for s, cids in sorted(slots.items()):
        if s >= num_joints:
            problems.append(f"关节槽 {s} 越界 ({cids})")
    return problems
```

- [ ] **Step 4: 运行测试确认通过**

Run: `python Arm-robot_VLA/lerobot_robot_massage/zdt/test_robot_state.py`
Expected: PASS（旧 test_safety.py 不受影响）

- [ ] **Step 5: 提交**

```bash
git add Arm-robot_VLA/lerobot_robot_massage/zdt/safety.py Arm-robot_VLA/lerobot_robot_massage/zdt/test_robot_state.py
git -C Arm-robot_VLA commit -m "feat(zdt): RobotStateMachine 整臂门禁 + 枚举硬不变式 verify_enumeration (修订#6)"
```

---

## Task 5: 控制器生命周期（拆 T5a~T5d）— driver/scan → 生命周期 → get_real_state → 调用方

> 按评审拆分：每个子任务独立测试 + 独立 commit + 独立评审，顺序不变（T4 → T5a → T5b → T5c → T5d → T6）。

**Files:**
- Modify: `Arm-robot_VLA/lerobot_robot_massage/zdt/zdt_driver.py`（`read_version`、`_request`/`_recv_for` 超时覆盖）
- Modify: `Arm-robot_VLA/lerobot_robot_massage/zdt/scan.py`（`scan_via_driver`）
- Modify: `Arm-robot_VLA/lerobot_robot_massage/zdt/config.py`（新增字段）
- Modify: `Arm-robot_VLA/lerobot_robot_massage/zdt/controller.py`（connect/arm/disarm/get_real_state）
- Modify: `Arm-robot_VLA/lerobot_robot_massage/massage_robot.py`、`scripts/control/cartesian_keyboard.py`、`scripts/bringup/zdt_bringup.py`（调用方适配）
- Test: `Arm-robot_VLA/lerobot_robot_massage/zdt/test_driver.py`、`test_scan.py`、`test_controller.py`

**Interfaces:**
- Consumes: Task 4 的 `RobotStateMachine` / `verify_enumeration` / `SafetyError`。
- Produces:
  - `ZdtDriver.read_version(addr, timeout_s=None, retries=None) -> tuple[int,int] | None`
  - `scan.scan_via_driver(driver, id_range=None, timeout_s=None, retries=None) -> ScanResult`
  - `ZdtController.connect() / arm(gravity_confirmed=False) / disarm() / enter_teleop() / exit_teleop() / fault(reason) / re_arm(confirmed=False) / get_real_state() -> dict`；`self.robot: RobotStateMachine`
  - ZdtConfig 新字段：`max_vel_mm_s=20.0, max_ang_rad_s=1.0, max_joint_vel_deg_s=60.0, max_joint_acc_deg_s2=200.0, joint_limit_margin_deg=2.0, kp_pos=2.0, kr_ori=2.0, ik_near_ratio=0.3, ik_sing_ratio=0.1, workspace_min=None, workspace_max=None, dt_min_factor=0.5, dt_max_factor=3.0, stale_cmd_max_s=0.25, vel_filter_alpha=0.2`

### T5a: ZdtDriver.read_version + scan.scan_via_driver

- [ ] **Step 1: 写失败测试（driver.read_version + scan.scan_via_driver）**

在 `test_driver.py` 追加：

```python
def test_read_version_returns_fw_hw():
    t = FakeTransport()
    drv = ZdtDriver(t, timeout_s=0.001, retries=0)
    t.inject(0x02, 0x1F, bytes([0x00, 0x02, 0x03]) + b"\x6b")   # fw=2, hw=3
    ver = drv.read_version(0x02)
    assert ver == (2, 3)


def test_read_version_missing_motor_returns_none():
    t = FakeTransport()
    drv = ZdtDriver(t, timeout_s=0.001, retries=0)
    assert drv.read_version(0x01) is None    # 无注入 → 超时
```

在 `test_scan.py` 追加：

```python
def test_scan_via_driver_firmware_scheme():
    from lerobot_robot_massage.zdt.zdt_driver import ZdtDriver
    from lerobot_robot_massage.zdt.fakes import FakeTransport
    from lerobot_robot_massage.zdt.scan import scan_via_driver
    t = FakeTransport()
    drv = ZdtDriver(t, timeout_s=0.001, retries=0)
    for addr in range(0x01, 0x07):
        t.inject(addr, 0x1F, bytes([0x00, 0x01, 0x01]) + b"\x6b")
    res = scan_via_driver(drv, id_range=(1, 8))
    assert res.scheme == "firmware"
    assert len(res.found) == 6
    assert res.found[0x01].joint_slot == 0
    assert res.found[0x06].joint_slot == 5
```

- [ ] **Step 2: 运行确认失败**

Run: `python Arm-robot_VLA/lerobot_robot_massage/zdt/test_driver.py && python Arm-robot_VLA/lerobot_robot_massage/zdt/test_scan.py`
Expected: FAIL — `AttributeError: 'ZdtDriver' object has no attribute 'read_version'` 等

- [ ] **Step 3: 写实现（zdt_driver.py + scan.py）**

`zdt_driver.py` 新增：

```python
    def read_version(self, addr: int, timeout_s: Optional[float] = None,
                     retries: Optional[int] = None) -> Optional[tuple[int, int]]:
        """探测 0x1F 版本. 超时/无响应 → None (枚举时作为离线判定)."""
        try:
            data = self._request(addr, bytes([0x1F]), expect_response=True,
                                 timeout_s=timeout_s, retries=retries)
        except ZdtDriverError:
            return None
        if data is None or len(data) < 3:
            return None
        return data[1], data[2]
```

并在 `_request` / `_recv_for` 增加超时/重试覆盖（向后兼容，默认 None 用 self 值）：

```python
    def _request(self, addr: int, body: bytes, expect_response: bool,
                 timeout_s: Optional[float] = None,
                 retries: Optional[int] = None) -> Optional[bytes]:
        payload = add_checksum(body)
        frames = encode_frame(addr, payload)
        t_out = timeout_s if timeout_s is not None else self.timeout_s
        r_retries = retries if retries is not None else self.retries
        for attempt in range(r_retries + 1):
            for f in frames:
                try:
                    self._t.send(f)
                except CanTransportError as exc:
                    raise TransportError(
                        f"send 失败 addr={addr:#04x} func={payload[0]:#04x}") from exc
            if not expect_response:
                return None
            resp = self._recv_for(addr, payload[0], timeout_s=t_out)
            if resp is not None:
                return resp
            logger.warning("timeout addr=%02X func=%02X attempt=%d/%d",
                           addr, payload[0], attempt, r_retries)
        raise TimeoutError(f"addr={addr:#04x} func={payload[0]:#04x} 超时")

    def _recv_for(self, addr: int, func: int,
                  timeout_s: Optional[float] = None) -> Optional[bytes]:
        t_out = timeout_s if timeout_s is not None else self.timeout_s
        deadline = time.monotonic() + t_out
        while time.monotonic() < deadline:
            try:
                frame = self._t.recv(t_out)
            except CanTransportError as exc:
                raise TransportError(
                    f"recv 失败 addr={addr:#04x} func={func:#04x}") from exc
            if frame is None:
                continue
            r_addr, _seq, data = parse_frame(frame)
            if not verify_checksum(data):
                logger.warning("checksum fail addr=%02X data=%s",
                               r_addr, data.hex())
                continue
            r_func = data[0]
            if r_func == 0xFD and self.on_arrived is not None:
                self.on_arrived(r_addr)
                continue
            if r_addr == addr and r_func == func:
                return data
        return None
```

`scan.py` 追加：

```python
def scan_via_driver(driver, id_range: Optional[tuple[int, int]] = None,
                    timeout_s: Optional[float] = None,
                    retries: Optional[int] = None) -> ScanResult:
    """用 ZdtDriver 探测 (0x1F) 枚举 — controller.connect 专用, 免 ZdtBus.

    与 scan_bus 同裁决/同槽映射逻辑, 但复用 driver 的同步请求原语 (不回绕
    ZdtBus 的后台读线程), 避免与 ZdtDriver 的同步 recv 冲突.
    """
    lo, hi = id_range or DEFAULT_SCAN_RANGE
    result = ScanResult()
    for cid in range(lo, hi + 1):
        if cid == 0:
            continue
        ver = driver.read_version(cid, timeout_s=timeout_s, retries=retries)
        if ver is not None:
            result.found[cid] = MotorState(can_id=cid, online=True, fw_ver=ver,
                                           tracked_deg=0.0)
    if not result.found:
        result.scheme = None
        result.warnings.append(
            "总线无响应: 检查供电 / 波特率 / 驱动板 P_Serial=CAN1_MAP / "
            "Response 非 None")
        return result
    scheme, warnings = resolve_scheme(set(result.found))
    result.scheme = scheme
    result.warnings = warnings
    if scheme == "firmware":
        _assign_slots(result.found, slot_of=lambda cid: cid - 1)
    elif scheme == "pc":
        _assign_slots(result.found, slot_of=lambda cid: cid - 2)
    return result
```

- [ ] **Step 4: 运行确认通过**

Run: `python Arm-robot_VLA/lerobot_robot_massage/zdt/test_driver.py && python Arm-robot_VLA/lerobot_robot_massage/zdt/test_scan.py`
Expected: PASS

- [ ] **Step 5: 提交（本子步）**

```bash
git add Arm-robot_VLA/lerobot_robot_massage/zdt/zdt_driver.py Arm-robot_VLA/lerobot_robot_massage/zdt/scan.py Arm-robot_VLA/lerobot_robot_massage/zdt/test_driver.py Arm-robot_VLA/lerobot_robot_massage/zdt/test_scan.py
git -C Arm-robot_VLA commit -m "feat(zdt): read_version + scan_via_driver (connect 枚举用, 免 ZdtBus)"
```

### T5b: ZdtController 生命周期（connect/arm/disarm/teleop/e_stop 门禁 + config 字段）

- [ ] **Step 6: 写失败测试（controller 生命周期）**

`config.py` 新增字段（先写，这是实现依赖）：

```python
    # ── 遥操/笛卡尔 (2026-08-23, spec §4) ──
    max_vel_mm_s: float = 20.0
    max_ang_rad_s: float = 1.0
    max_joint_vel_deg_s: float = 60.0
    max_joint_acc_deg_s2: float = 200.0
    joint_limit_margin_deg: float = 2.0
    kp_pos: float = 2.0
    kr_ori: float = 2.0
    ik_near_ratio: float = 0.3
    ik_sing_ratio: float = 0.1
    workspace_min: Optional[list] = None      # None = 不启用盒约束 (待真机标定)
    workspace_max: Optional[list] = None
    dt_min_factor: float = 0.5
    dt_max_factor: float = 3.0
    stale_cmd_max_s: float = 0.25
    vel_filter_alpha: float = 0.2
```

在 `test_controller.py` 追加：

```python
# ── 2026-08-23: connect 生命周期 (scan/verify + 状态机) ─────
from lerobot_robot_massage.zdt.safety import (  # noqa: E402
    MotorState, RobotPhase, SafetyError,
)

def _mk_armed(ctrl):
    """把注入式 ZdtController 的状态机直接推进到 ARMED (纯状态, 无 CAN)."""
    ctrl.robot.on_connected()
    motors = {a: MotorState(can_id=a, online=True, joint_slot=i)
              for i, a in enumerate(ctrl.config.joint_addrs)}
    ctrl.robot.on_enumerated(motors)
    ctrl.robot.on_safe_idle()
    ctrl.robot.arm(gravity_confirmed=True)
    return ctrl


def test_connect_scan_verify_reaches_safe_idle():
    t = FakeTransport()
    cfg = ZdtConfig(timeout_s=0.001, retries=0, reduction_ratios=[1.0] * 6,
                    calib=[(1.0, 0.0)] * 6)
    ctrl = ZdtController(config=cfg, transport=t)
    for addr in range(0x01, 0x07):                       # firmware scheme
        t.inject(addr, 0x1F, bytes([0x00, 0x01, 0x01]) + b"\x6b")
    for addr in range(0x01, 0x07):                       # sync 读 0x36
        t.inject(addr, F_READ_POS, b"\x00\x00\x00\x00\x00" + b"\x6b")
    ctrl.connect()
    assert ctrl.robot.phase == RobotPhase.SAFE_IDLE
    assert ctrl._connected is True
    assert ctrl.config.joint_addrs == [0x01, 0x02, 0x03, 0x04, 0x05, 0x06]


def test_connect_enumeration_failure_latches_fault():
    # 只探测到 5 台 → 缺 J6 → 硬不变式 → FAULT + 抛 SafetyError + 不使能扭矩
    t = FakeTransport()
    cfg = ZdtConfig(timeout_s=0.001, retries=0, reduction_ratios=[1.0] * 6,
                    calib=[(1.0, 0.0)] * 6)
    ctrl = ZdtController(config=cfg, transport=t)
    for addr in range(0x01, 0x06):
        t.inject(addr, 0x1F, bytes([0x00, 0x01, 0x01]) + b"\x6b")
    try:
        ctrl.connect()
        raise AssertionError("枚举失败应抛 SafetyError")
    except SafetyError:
        pass
    assert ctrl.robot.phase == RobotPhase.FAULT
    assert ctrl.robot.fault_reason
    assert ctrl._connected is False
    assert not any(f.data and f.data[0] == 0xF3 for f in t.sent)   # 未使能扭矩


def test_connect_no_longer_enables_torque():
    # 修订: connect 不再 set_torque(True); arm() 才使能
    t = FakeTransport()
    cfg = ZdtConfig(timeout_s=0.001, retries=0, reduction_ratios=[1.0] * 6,
                    calib=[(1.0, 0.0)] * 6)
    ctrl = ZdtController(config=cfg, transport=t)
    for addr in range(0x01, 0x07):
        t.inject(addr, 0x1F, bytes([0x00, 0x01, 0x01]) + b"\x6b")
    for addr in range(0x01, 0x07):
        t.inject(addr, F_READ_POS, b"\x00\x00\x00\x00\x00" + b"\x6b")
    ctrl.connect()
    assert not any(f.data and f.data[0] == 0xF3 for f in t.sent)
    ctrl.arm(gravity_confirmed=True)
    assert ctrl.robot.phase == RobotPhase.ARMED
    assert any(f.data and f.data[0] == 0xF3 for f in t.sent)


def test_arm_requires_safe_idle():
    ctrl, t = _mk()
    try:
        ctrl.arm(gravity_confirmed=True)
        raise AssertionError("DISCONNECTED 不应能 arm")
    except SafetyError:
        pass


def test_disarm_disables_torque_and_returns_safe_idle():
    ctrl, t = _mk()
    _mk_armed(ctrl)
    ctrl.disarm()
    assert ctrl.robot.phase == RobotPhase.SAFE_IDLE
    assert any(f.data and f.data[0] == 0xF3 and len(f.data) > 2 and f.data[2] == 0x00
               for f in t.sent)


def test_connect_enumeration_failure_cleans_up():
    # 无任何回帧 → 扫描全空 → 硬不变式失败 → SafetyError + FAULT + 清理
    # (read_version 吞掉 TransportError, 总线失败表现为"枚举失败"; e_stop 广播 + close)
    t = FakeTransport()
    cfg = ZdtConfig(timeout_s=0.001, retries=0, reduction_ratios=[1.0] * 6,
                    calib=[(1.0, 0.0)] * 6)
    ctrl = ZdtController(config=cfg, transport=t)
    try:
        ctrl.connect()
        raise AssertionError("connect 应抛 SafetyError")
    except SafetyError:
        pass
    assert t.sent[-1].arbitration_id == 0x0000        # e_stop 广播
    assert t.closed is True
    assert ctrl._connected is False
    assert ctrl.robot.phase == RobotPhase.FAULT        # 闩锁 FAULT, 禁止 arm
```

（`test_controller.py` 已 import `FakeTransport` 等；新增 `MotorState/RobotPhase/SafetyError` import。注意 `read_version` 会吞掉 `TransportError`（`ZdtDriverError` 子类），因此扫描阶段的总线失败表现为"枚举失败 → SafetyError"；`sync()` 阶段的 `read_pos` 不吞 `TransportError`，总线死亡会直接上抛——connect 的 `except Exception` 统一清理后 re-raise。）

- [ ] **Step 7: 运行确认失败**

Run: `python Arm-robot_VLA/lerobot_robot_massage/zdt/test_controller.py`
Expected: FAIL — 多个断言（connect 仍旧行为；`arm`/`get_real_state`/`robot` 不存在等）

- [ ] **Step 8: 写实现（controller.py）**

`controller.py` 顶部 import 增加：

```python
from .safety import MotorState, RobotPhase, RobotStateMachine, SafetyError
from .scan import scan_via_driver
```

`__init__` 增加：

```python
        self.robot = RobotStateMachine()
        self._last_scan = None
        self._last_real_q: Optional[list[float]] = None
        self._last_real_ts: Optional[float] = None
        self._vel = [0.0] * len(self.config.joint_addrs)
```

`connect()` 重写：

```python
    def connect(self) -> None:
        if self._transport is None:
            from .can_transport import SocketCanTransport
            self._transport = SocketCanTransport(self.config.channel,
                                                 self.config.bitrate)
        try:
            self._transport.open()
            self._driver = ZdtDriver(self._transport,
                                     timeout_s=self.config.timeout_s,
                                     retries=self.config.retries)
            self.robot = RobotStateMachine()          # 新连接 = 新生命周期
            self.robot.on_connected()
            self._scan_and_verify()                   # 枚举 + 硬不变式
            self.sync()                               # 读 0x36 对齐 tracked
            self.robot.on_safe_idle()
        except Exception:
            self._connected = False
            if self._driver is not None:
                try:
                    self.e_stop()
                except ZdtDriverError:
                    logger.warning("connect 失败后 e_stop 发送失败", exc_info=True)
            try:
                self._transport.close()
            except Exception:  # noqa: BLE001
                logger.warning("connect 失败后 transport close 失败", exc_info=True)
            raise
        self._connected = True
        self._last_io_s = time.monotonic()
        logger.info("ZDT CAN connected: %s (6 drives verified)", self.config.channel)

    def _scan_and_verify(self) -> None:
        """扫描 + 枚举硬不变式 (修订 #6). 任一违例 → fault 闩锁 + 抛 SafetyError."""
        scan = scan_via_driver(self._driver, timeout_s=self.config.timeout_s,
                               retries=0)
        self._last_scan = scan
        problems = verify_enumeration(scan.found)
        if problems:
            self.robot.fault("枚举失败: " + "; ".join(problems))
            raise SafetyError("枚举失败, 进入 FAULT: " + "; ".join(problems))
        self.robot.on_enumerated(scan.found)
        # 采纳实际发现的寻址方案 (firmware 1..6 / pc 2..7), 后续 IO 用真实地址
        slot_addrs = [None] * 6
        for cid, m in scan.found.items():
            if m.joint_slot is not None:
                slot_addrs[m.joint_slot] = cid
        if any(a is None for a in slot_addrs):
            raise SafetyError("枚举后关节槽地址不完整")   # 理论不可达 (verify 已拦)
        self.config.joint_addrs = list(slot_addrs)
```

新增生命周期方法（放在 `sync()` 之后）：

```python
    def arm(self, gravity_confirmed: bool = False) -> None:
        """SAFE_IDLE → 使能扭矩 → ARMED. 重力关节 J2/J3 需二次确认."""
        self.robot.arm(gravity_confirmed)             # 门禁 + 重力确认 (无 IO)
        try:
            self.set_torque(True)
        except Exception:
            self.robot.disarm()                       # 使能失败回滚到 SAFE_IDLE
            raise

    def disarm(self) -> None:
        try:
            self.set_torque(False)
        finally:
            self.robot.disarm()

    def enter_teleop(self) -> None:
        self.robot.enter_teleop()

    def exit_teleop(self) -> None:
        self.robot.exit_teleop()

    def fault(self, reason: str) -> None:
        self.robot.fault(reason)

    def re_arm(self, confirmed: bool = False) -> None:
        """STOPPED/FAULT → 重枚举验证 → SAFE_IDLE. 需显式确认."""
        scan = scan_via_driver(self._driver, timeout_s=self.config.timeout_s,
                               retries=0)
        problems = verify_enumeration(scan.found)
        if problems:
            self.robot.fault("重枚举失败: " + "; ".join(problems))
            raise SafetyError("重枚举失败: " + "; ".join(problems))
        self.robot.re_arm(confirmed)
        slot_addrs = [None] * 6
        for cid, m in scan.found.items():
            if m.joint_slot is not None:
                slot_addrs[m.joint_slot] = cid
        self.config.joint_addrs = list(slot_addrs)
        self.sync()
        self.robot.on_safe_idle()
```

`e_stop()` 增加状态机闩锁：

```python
    def e_stop(self) -> None:
        self._driver.stop_all()
        self.robot.e_stop()                            # 闩锁 STOPPED
        self._last_io_s = time.monotonic()
        logger.warning("EMERGENCY STOP broadcast")
```

> `get_real_state()` 的实现移到 T5c（见下），此处不重复。

- [ ] **Step 9: 运行测试确认通过**

Run: `python Arm-robot_VLA/lerobot_robot_massage/zdt/test_controller.py`
Expected: PASS（旧 controller 测试 + 新增全部通过）

- [ ] **Step 10: 提交（controller 生命周期）**

```bash
git add Arm-robot_VLA/lerobot_robot_massage/zdt/config.py Arm-robot_VLA/lerobot_robot_massage/zdt/controller.py Arm-robot_VLA/lerobot_robot_massage/zdt/test_controller.py
git -C Arm-robot_VLA commit -m "feat(zdt): connect 走 scan/verify 硬不变式 + arm/disarm/teleop 门禁 (修订#6, connect 不再自动使能)"
```

### T5c: get_real_state（0x36 真实观测 + 滤波差分速度）

- [ ] **Step 9c: 写失败测试（get_real_state）**

在 `test_controller.py` 追加（复用 T5b 的 `_mk_armed`）：

```python
def test_get_real_state_fields():
    ctrl, t = _mk()
    _mk_armed(ctrl)
    # 注入顺序必须匹配读取顺序: 全 q (0x36) → 全 current (0x27) → 全 flags (0x3A)
    for addr in ctrl.config.joint_addrs:
        t.inject(addr, F_READ_POS, b"\x00\x00\x00\x00\x00" + b"\x6b")
    for addr in ctrl.config.joint_addrs:
        t.inject(addr, 0x27, b"\x00" + bytes([50]) + b"\x6b")
    for addr in ctrl.config.joint_addrs:
        t.inject(addr, 0x3A, b"\x01\x6b")
    st = ctrl.get_real_state()
    assert set(st) == {"q", "velocity", "current", "flags", "status"}
    assert len(st["q"]) == 6 and len(st["velocity"]) == 6
    assert len(st["current"]) == 6 and len(st["flags"]) == 6
    assert st["status"] == "ARMED"


def test_get_real_state_velocity_filters():
    # 两次读取不同真实角 → 滤波有限差分 dq 非零
    ctrl, t = _mk()
    _mk_armed(ctrl)
    for addr in ctrl.config.joint_addrs:
        t.inject(addr, F_READ_POS, b"\x00\x00\x00\x00\x00" + b"\x6b")
    for addr in ctrl.config.joint_addrs:
        t.inject(addr, 0x27, b"\x00" + bytes([50]) + b"\x6b")
    for addr in ctrl.config.joint_addrs:
        t.inject(addr, 0x3A, b"\x01\x6b")
    st0 = ctrl.get_real_state()
    assert all(v == 0.0 for v in st0["velocity"])   # 首帧无差分
    # 第二帧: q 前进 1°
    for addr, deg in zip(ctrl.config.joint_addrs, [1.0] * 6):
        v = int(round(abs(deg) * 65536.0 / 360.0)) & 0xFFFFFFFF
        t.inject(addr, F_READ_POS, bytes([0x00, (v >> 24) & 0xFF,
                                          (v >> 16) & 0xFF, (v >> 8) & 0xFF,
                                          v & 0xFF]) + b"\x6b")
    for addr in ctrl.config.joint_addrs:
        t.inject(addr, 0x27, b"\x00" + bytes([50]) + b"\x6b")
    for addr in ctrl.config.joint_addrs:
        t.inject(addr, 0x3A, b"\x01\x6b")
    st1 = ctrl.get_real_state()
    assert any(abs(v) > 0.0 for v in st1["velocity"])   # 差分后非零
```

- [ ] **Step 10c: 运行确认失败**

Run: `python Arm-robot_VLA/lerobot_robot_massage/zdt/test_controller.py`
Expected: FAIL — `AttributeError: 'ZdtController' object has no attribute 'get_real_state'`

- [ ] **Step 11c: 写实现（controller.py 追加 get_real_state）**

```python
    def get_real_state(self) -> dict:
        """0x36 真实观测 (spec §5.3): {q, velocity, current, flags, status}.

        q = 真实输出角 (anchor, use_kb=True); velocity = 低通有限差分 dq
        (vel_filter_alpha, 驱动器 0x35 实时速度留待后续); current/flags 逐轴.
        """
        q = self.read_real_angles(use_kb=True)
        now = time.monotonic()
        dq = [0.0] * len(q)
        if self._last_real_q is not None and self._last_real_ts is not None:
            dt = max(1e-3, now - self._last_real_ts)
            raw = [(q[i] - self._last_real_q[i]) / dt for i in range(len(q))]
            alpha = self.config.vel_filter_alpha
            self._vel = [alpha * raw[i] + (1.0 - alpha) * self._vel[i]
                         for i in range(len(q))]
            dq = list(self._vel)
        else:
            self._vel = [0.0] * len(q)
        self._last_real_q = list(q)
        self._last_real_ts = now
        currents: list[float] = []
        flags: list[int] = []
        for addr in self.config.joint_addrs:
            currents.append(self._driver.read_current(addr))
            flags.append(self._driver.read_flag(addr))
        return {"q": list(q), "velocity": dq, "current": currents,
                "flags": flags, "status": self.robot.phase.name}
```

- [ ] **Step 12c: 运行测试确认通过**

Run: `python Arm-robot_VLA/lerobot_robot_massage/zdt/test_controller.py`
Expected: PASS（T5b + T5c 全部通过）

- [ ] **Step 13c: 提交（T5c）**

```bash
git add Arm-robot_VLA/lerobot_robot_massage/zdt/controller.py Arm-robot_VLA/lerobot_robot_massage/zdt/test_controller.py
git -C Arm-robot_VLA commit -m "feat(zdt): get_real_state 0x36 真实观测 + 滤波差分速度 (T5c, spec §5.3)"
```

### T5d: 调用方适配（spec §5.4）

`scripts/control/cartesian_keyboard.py` 的 `_build_cartesian`：

```python
def _build_cartesian(iface: str) -> CartesianController:
    cfg = ZdtConfig(channel=iface, bitrate=500_000)
    ctrl = ZdtController(cfg)
    ctrl.connect()                                  # 枚举+验证 → SAFE_IDLE
    _print_status(CartesianController(ctrl))        # 打印状态后要求显式臂置
    print("[键盘遥操] 回车确认臂置 (重力关节 J2/J3 二次确认), 或 Ctrl-C 退出...")
    input()
    ctrl.arm(gravity_confirmed=True)                # 使能扭矩 → ARMED
    return CartesianController(ctrl, max_vel_mm_s=BASE_VEL, loop_hz=LOOP_HZ)
```

（`_print_status` 已接受 `cart`，此处用临时 cart 只读状态，不改其签名。）

`scripts/bringup/zdt_bringup.py` 的 `torque` 分支：

```python
        elif args.cmd == "torque":
            if args.state == 1:
                ctrl.arm(gravity_confirmed=True)
                print("[torque] 已臂置 (set_torque on + ARMED)")
            else:
                ctrl.disarm()
                print("[torque] 已解除 (set_torque off + SAFE_IDLE)")
```

`lerobot_robot_massage/massage_robot.py` 的 `reset()`（`move_to_ready_on_connect` 路径在 connect 后需先 arm 再 ready）：

```python
    def reset(self) -> None:
        if self.config.transport != "can":
            logger.info("reset(): transport=%s 不支持自动 ready, 跳过", self.config.transport)
            return
        if not self.config.gravity_confirm:
            raise RuntimeError(
                "reset() 需先显式确认重力关节 (config.gravity_confirm=True) 才能自动 ready")
        self._protocol.arm(gravity_confirmed=True)   # connect 不再自动使能 → 显式 arm
        targets = self._protocol.ready()
        logger.info("reset() → 按摩准备姿态 %s", targets)
```

`disconnect()` 中 `set_torque(False)` 改走 `disarm()`：

```python
        try:
            self._protocol.disarm()
        except (SerialProtocolError, ZdtDriverError):
            pass
```

给 `MassageRobotConfig`（`lerobot_robot_massage/config_massage_robot.py`）增加字段：

```python
    gravity_confirm: bool = False   # reset() 自动 ready 前需确认重力关节 (J2/J3)
```

`scripts/bringup/test_zdt_bringup_import.py` 与 `test_massage_robot_can.py` 只测 argparse/config 构造，不受影响。

- [ ] **Step 12: 全量回归确认调用方改动无破坏**

Run: `cd Arm-robot_VLA && python -m pytest lerobot_robot_massage/zdt scripts/bringup -q`
Expected: PASS（zdt 全量 + bringup 冒烟）

- [ ] **Step 13: 提交（调用方适配）**

```bash
git add Arm-robot_VLA/scripts/control/cartesian_keyboard.py Arm-robot_VLA/scripts/bringup/zdt_bringup.py Arm-robot_VLA/lerobot_robot_massage/massage_robot.py Arm-robot_VLA/lerobot_robot_massage/config_massage_robot.py
git -C Arm-robot_VLA commit -m "feat(teleop): connect 后显式 arm 确认 (keyboard/bringup/massage_robot 适配 spec §5.4)"
```

---

## Task 6: CartesianController — 测量 dt / 陈旧命令 / 6DOF 安全链 / step_pose

**Files:**
- Modify: `Arm-robot_VLA/lerobot_robot_massage/zdt/cartesian.py`
- Modify: `Arm-robot_VLA/lerobot_robot_massage/zdt/testutil.py`（加 `FakeClock`）
- Modify: `Arm-robot_VLA/lerobot_robot_massage/zdt/test_cartesian.py`（现有测试用 armed helper + FakeClock）
- Test: `Arm-robot_VLA/lerobot_robot_massage/zdt/test_cartesian.py`

**Interfaces:**
- Consumes: Task 2 `log_so3`/`singularity_metrics`/`adaptive_damping`；Task 3 `CartesianVelocityLimiter`/`BoxWorkspace`；Task 4 `RobotPhase`。
- Produces:
  - `CartesianController.step(vx, vy, vz, wx=0.0, wy=0.0, wz=0.0, cmd_ts=None) -> dict`
  - `CartesianController.step_pose(p_des, R_des, cmd_ts=None, rpy_anchor=None, rpy_limits=None) -> dict`
  - 返回 dict 键：`moved, reason, target_xyz, sigma_min, condition, lambda, scale, alarms`（按路径子集）
  - `testutil.FakeClock(t0=0.0)`：`__call__()/tick(dt)`

- [ ] **Step 1: 写失败测试（先加 FakeClock + 改造现有测试）**

`testutil.py` 追加：

```python
class FakeClock:
    """测试用单调钟: 每次 step() 后 tick(dt) 推进, 模拟帧间隔."""

    def __init__(self, t0: float = 0.0):
        self.t = float(t0)

    def __call__(self) -> float:
        return self.t

    def tick(self, dt: float) -> None:
        self.t += float(dt)
```

`test_cartesian.py` 顶部 import 增加，并把现有 `_mk` 保留、`_ready_cart` 改为先置 ARMED + 注入 FakeClock：

```python
from lerobot_robot_massage.zdt.safety import MotorState, RobotPhase  # noqa: E402
from lerobot_robot_massage.zdt.testutil import FakeClock  # noqa: E402

def _arm_robot(ctrl):
    ctrl.robot.on_connected()
    motors = {a: MotorState(can_id=a, online=True, joint_slot=i)
              for i, a in enumerate(ADDRS)}
    ctrl.robot.on_enumerated(motors)
    ctrl.robot.on_safe_idle()
    ctrl.robot.arm(gravity_confirmed=True)
    return ctrl

def _mk_armed(calib=None):
    ctrl, t = _mk(calib)
    _arm_robot(ctrl)
    return ctrl, t

def _ready_cart(loop_hz=20.0, max_vel=20.0, clock=None, stale_cmd_max_s=0.25):
    ctrl, t = _mk_armed()
    cart = CartesianController(ctrl, loop_hz=loop_hz, max_vel_mm_s=max_vel,
                               clock=clock or FakeClock(),
                               stale_cmd_max_s=stale_cmd_max_s)
    return ctrl, t, cart
```

（**现有测试迁移**：所有调用 `cart.step(...)` 的测试必须经 ARMED 门禁——`_mk()` 直接构造的改为 `_mk_armed()`（`test_step_zero_velocity_target_is_current_ee` / `test_step_positive_vx_moves_along_x` / `test_step_velocity_clamped` / `test_step_zero_velocity_never_alarms`）；多帧距离测试（`test_step_ready_pose_tracks_position` / `test_step_unreachable_target_hits_limit_alarm`）额外每帧 `clock.tick(0.05)` 推进测量 dt（`_ready_cart(clock=...)`）；单帧断言（首帧 dt=dt_default=0.05）不需推进。`test_get_ee_xyz_reports_reset_position` 不调用 step，无需改动。）

新增测试（6DOF / dt / 陈旧命令 / 安全链 / step_pose）：

```python
def test_step_6dof_passes_angular_velocity():
    """6DOF: 角速度进入 twist, 遥测含 λ/scale/condition."""
    ctrl, t, cart = _ready_cart(max_vel=20.0)
    _inject_anchor_pose(t, READY_ANCHOR, n=6)
    res = cart.step(0.0, 0.0, 0.0, wx=0.5, wy=0.0, wz=0.0)
    assert res["moved"] is True
    assert "lambda" in res and "scale" in res and "condition" in res


def test_step_angular_velocity_clamped():
    ctrl, t, cart = _ready_cart(max_vel=20.0)
    _inject_anchor_pose(t, READY_ANCHOR, n=6)
    res = cart.step(0.0, 0.0, 0.0, wx=100.0, wy=0.0, wz=0.0)   # 100 rad/s >> 1
    assert res["moved"] is True


def test_step_requires_armed():
    ctrl, t = _mk()                     # 未 arm 的状态机 (DISCONNECTED)
    cart = CartesianController(ctrl)
    _inject_anchor_zero(t, n=1)
    res = cart.step(0.0, 0.0, 0.0)
    assert res["moved"] is False
    assert res["reason"].startswith("not_armed")


def test_step_measured_dt_uses_frame_gap():
    clock = FakeClock()
    ctrl, t, cart = _ready_cart(clock=clock)
    q = list(READY_ANCHOR)
    _inject_anchor_pose(t, q, n=3)
    cart.step(10.0, 0.0, 0.0)              # 首帧 dt=dt_default=0.05
    q = list(ctrl._tracked_angles)
    clock.tick(0.04)
    _inject_anchor_pose(t, q, n=3)
    res = cart.step(10.0, 0.0, 0.0)        # 第二帧 dt=0.04
    x0 = fk_mdh(anchor_to_source(q))[0, 3]
    assert abs(res["target_xyz"][0] - x0 - 10.0 * 0.04) < 0.1


def test_step_dt_bounded_by_dt_max():
    clock = FakeClock()
    ctrl, t, cart = _ready_cart(clock=clock)
    q = list(READY_ANCHOR)
    _inject_anchor_pose(t, q, n=3)
    cart.step(10.0, 0.0, 0.0)
    q = list(ctrl._tracked_angles)
    clock.tick(5.0)                         # 挂起 5s → dt 钳到 dt_max=3*0.05=0.15
    _inject_anchor_pose(t, q, n=3)
    res = cart.step(10.0, 0.0, 0.0)
    x0 = fk_mdh(anchor_to_source(q))[0, 3]
    assert abs(res["target_xyz"][0] - x0 - 10.0 * 0.15) < 0.1


def test_step_dt_bounded_by_dt_min():
    clock = FakeClock()
    ctrl, t, cart = _ready_cart(clock=clock)
    q = list(READY_ANCHOR)
    _inject_anchor_pose(t, q, n=3)
    cart.step(10.0, 0.0, 0.0)
    q = list(ctrl._tracked_angles)
    clock.tick(1e-9)                        # 时钟未推进 → dt 钳到 dt_min=0.5*0.05
    _inject_anchor_pose(t, q, n=3)
    res = cart.step(10.0, 0.0, 0.0)
    x0 = fk_mdh(anchor_to_source(q))[0, 3]
    assert abs(res["target_xyz"][0] - x0 - 10.0 * 0.025) < 0.1


def test_step_stale_command_refused():
    """陈旧命令看门狗 (控制层权威, 单调期限)."""
    clock = FakeClock()
    ctrl, t, cart = _ready_cart(clock=clock, stale_cmd_max_s=0.25)
    _inject_anchor_pose(t, READY_ANCHOR, n=1)
    clock.tick(0.3)                         # cmd_ts=0.0 距今 0.3s > 0.25s
    res = cart.step(10.0, 0.0, 0.0, cmd_ts=0.0)
    assert res["moved"] is False
    assert res["reason"] == "stale_command"
    assert not any(f.data and f.data[0] == 0xFD for f in t.sent)


def test_step_fresh_command_accepted():
    clock = FakeClock()
    ctrl, t, cart = _ready_cart(clock=clock, stale_cmd_max_s=0.25)
    _inject_anchor_pose(t, READY_ANCHOR, n=6)
    res = cart.step(10.0, 0.0, 0.0, cmd_ts=clock.t)   # 新鲜
    assert res["moved"] is True


def test_step_workspace_blocks_outside_box():
    from lerobot_robot_massage.zdt.workspace import BoxWorkspace
    ctrl, t, cart = _ready_cart()
    T = fk_mdh(anchor_to_source(READY_ANCHOR))
    x0 = T[0, 3]
    # 盒 x∈(x0, x0+10] 严格排除当前位 (已越界且静止) → workspace_blocked
    box = BoxWorkspace([x0 + 0.01, -500.0, -500.0], [x0 + 10.0, 500.0, 500.0])
    cart.workspace = box
    cart._limiter.workspace = box
    _inject_anchor_pose(t, READY_ANCHOR, n=1)
    res = cart.step(0.0, 0.0, 0.0)
    assert res["moved"] is False and res["reason"] == "workspace_blocked"


def test_scale_toward_limits_progressive():
    """两层不变式 (评审 §六): 预测限位只渐进减速, 硬拒绝交给 check_limits_real."""
    ctrl, t, cart = _ready_cart()
    q_anchor = np.array([0.0, 148.0, 50.0, 0.0, 120.0, 0.0])   # J2 靠近上限 150
    q_src = np.array(anchor_to_source(q_anchor))
    dq = np.zeros(6)
    dq[1] = math.radians(1.0)                     # 下一帧 J2=149, margin 内 → 渐进缩小
    scaled = cart._scale_toward_limits(q_src, dq)
    assert 0.0 < scaled[1] < dq[1]
    dq[1] = math.radians(3.0)                     # 下一帧 J2=151 > 150 → 缩到 0
    scaled2 = cart._scale_toward_limits(q_src, dq)
    assert scaled2[1] == 0.0


def test_step_singular_band_refuses_motion():
    """奇异度实际参与: 强制 sing_ratio 覆盖常态 → SINGULAR 拒动 (非仅 telemetry)."""
    ctrl, t, cart = _ready_cart()
    cart.sing_ratio = 1.5                # ratio ≤ 1 恒成立 → 必入 SINGULAR
    cart.near_ratio = 2.0
    _inject_anchor_pose(t, READY_ANCHOR, n=1)
    res = cart.step(10.0, 0.0, 0.0)
    assert res["moved"] is False and res["reason"] == "singular"
    assert res["scale"] == 0.0 and res["lambda"] > 10.0


def test_step_near_singular_scales_twist():
    """NEAR 带: scale<1 且 λ>base → 实际参与速度缩放 (按当前位实测 ratio 设带)."""
    from lerobot_robot_massage.zdt.kinematics import (
        anchor_to_source, jacobian, singularity_metrics,
    )
    ctrl, t, cart = _ready_cart()
    m = singularity_metrics(jacobian(anchor_to_source(READY_ANCHOR)))
    ratio = m["sigma_min"] / max(m["sigma_max"], 1e-12)
    cart.sing_ratio = ratio / 2.0        # 实际 ratio 落 NEAR 带 (确定)
    cart.near_ratio = ratio * 1.5
    _inject_anchor_pose(t, READY_ANCHOR, n=6)
    res = cart.step(10.0, 0.0, 0.0)
    assert res["moved"] is True
    assert 0.0 < res["scale"] < 1.0
    assert res["lambda"] > 10.0


def test_step_pose_reaches_target():
    """step_pose: SE(3) 误差 → 位置+姿态环 → step. 姿态误差经 log_so3 (无 Euler 累加)."""
    clock = FakeClock()
    ctrl, t, cart = _ready_cart(clock=clock)
    q = list(READY_ANCHOR)
    for _ in range(3):
        _inject_anchor_pose(t, q, n=3)
        T = fk_mdh(anchor_to_source(q))
        p_des = T[:3, 3] + np.array([5.0, 0.0, 0.0])
        res = cart.step_pose(p_des, T[:3, :3])
        assert res["moved"] is True
        q = list(ctrl._tracked_angles)
        clock.tick(0.05)
    ee = fk_mdh(anchor_to_source(q))[:3, 3]
    assert ee[0] > T[:3, 3][0] + 1.0        # 沿 +x 移动


def test_step_pose_rpy_safety_clamps_orientation():
    """可选 RPY 安全约束 (相对 anchor): 大姿态误差被 clamp 到界内, 内部仍 SO(3)."""
    ctrl, t, cart = _ready_cart()
    _inject_anchor_pose(t, READY_ANCHOR, n=3)
    T = fk_mdh(anchor_to_source(READY_ANCHOR))
    roll = np.array([[1, 0, 0], [0, np.cos(0.4), -np.sin(0.4)],
                     [0, np.sin(0.4), np.cos(0.4)]])
    R_des = T[:3, :3] @ roll
    res = cart.step_pose(T[:3, 3], R_des,
                         rpy_anchor=T[:3, :3],
                         rpy_limits=(np.array([-0.1, -0.1, -0.1]),
                                     np.array([0.1, 0.1, 0.1])))
    assert res["moved"] is True             # clamp 后仍在安全范围, 不拒绝
```

- [ ] **Step 2: 运行确认失败**

Run: `python Arm-robot_VLA/lerobot_robot_massage/zdt/test_cartesian.py`
Expected: FAIL — `cart.step(10,0,0)` 返回 `not_armed`；`FakeClock` 缺失等

- [ ] **Step 3: 写实现（cartesian.py）**

重写模块 docstring 并改造 `CartesianController`。构造器签名改为（其余参数沿用）：

```python
    def __init__(self, ctrl: ZdtController,
                 max_vel_mm_s: Optional[float] = None,
                 loop_hz: float = 20.0,
                 joint_limits: Optional[list[tuple[float, float]]] = None,
                 ik_lambda: float = 10.0,
                 orient_weight: float = 20.0,
                 max_dq_deg: float = 2.0,
                 max_ang_rad_s: Optional[float] = None,
                 max_joint_vel_deg_s: Optional[float] = None,
                 max_joint_acc_deg_s2: Optional[float] = None,
                 joint_limit_margin_deg: Optional[float] = None,
                 kp_pos: Optional[float] = None,
                 kr_ori: Optional[float] = None,
                 near_ratio: Optional[float] = None,
                 sing_ratio: Optional[float] = None,
                 lam_max: Optional[float] = None,
                 workspace: Optional[BoxWorkspace] = None,
                 dt_min_factor: Optional[float] = None,
                 dt_max_factor: Optional[float] = None,
                 stale_cmd_max_s: Optional[float] = None,
                 clock: Callable[[], float] = time.monotonic):
        cfg = ctrl.config
        self.ctrl = ctrl
        self.max_vel_mm_s = (max_vel_mm_s if max_vel_mm_s is not None
                             else cfg.max_vel_mm_s)
        self.max_ang_rad_s = (max_ang_rad_s if max_ang_rad_s is not None
                              else cfg.max_ang_rad_s)
        self.max_joint_vel_deg_s = (max_joint_vel_deg_s
                                    if max_joint_vel_deg_s is not None
                                    else cfg.max_joint_vel_deg_s)
        self.max_joint_acc_deg_s2 = (max_joint_acc_deg_s2
                                     if max_joint_acc_deg_s2 is not None
                                     else cfg.max_joint_acc_deg_s2)
        self.joint_limit_margin_deg = (joint_limit_margin_deg
                                       if joint_limit_margin_deg is not None
                                       else cfg.joint_limit_margin_deg)
        self.kp_pos = kp_pos if kp_pos is not None else cfg.kp_pos
        self.kr_ori = kr_ori if kr_ori is not None else cfg.kr_ori
        self.near_ratio = near_ratio if near_ratio is not None else cfg.ik_near_ratio
        self.sing_ratio = sing_ratio if sing_ratio is not None else cfg.ik_sing_ratio
        self.lam_max = lam_max
        self.joint_limits = (joint_limits
                             if joint_limits is not None
                             else list(FIRMWARE_JOINT_LIMITS))
        self.ik_lambda = ik_lambda
        self.orient_weight = orient_weight
        self.max_dq_deg = max_dq_deg
        # 测量单调 dt (修订 #7): 首帧 dt_default, 之后实测并钳到 [dt_min, dt_max]
        self.loop_hz = loop_hz
        self.dt_default_s = 1.0 / loop_hz
        self.dt_min_s = ((dt_min_factor if dt_min_factor is not None else cfg.dt_min_factor)
                         * self.dt_default_s)
        self.dt_max_s = ((dt_max_factor if dt_max_factor is not None else cfg.dt_max_factor)
                         * self.dt_default_s)
        self.stale_cmd_max_s = (stale_cmd_max_s if stale_cmd_max_s is not None
                                else cfg.stale_cmd_max_s)
        self.clock = clock
        self._last_step_mono: Optional[float] = None
        self._last_dq: Optional[np.ndarray] = None
        self.workspace = workspace
        self._limiter = CartesianVelocityLimiter(self.max_vel_mm_s, workspace)
```

（`from .workspace import BoxWorkspace, CartesianVelocityLimiter`、`from .safety import RobotPhase`、`from .types import EEPose`、`from .kinematics import adaptive_damping, log_so3, singularity_metrics`、`import time` 加入顶部 import；删除 `self.dt_s`。）

替换 `step()` 与 `_read_current_ee`，新增 `_measure_dt`、`_armed_or_error`、`get_current_pose`、`_step_from_state`、`step_pose`、`_scale_toward_limits`、`_clamp_rpy_relative`、`_rotmat_to_rpy`、`_rpy_to_rotmat`（`_read_current_ee` 删除，`get_ee_xyz` 改为经 `_read_current_pose` 取位置）：
```python
    def get_ee_xyz(self) -> list[float]:
        """当前末端位置 (mm, 基座系) — 调试/显示用."""
        p, _, _ = self._read_current_pose()
        return p.tolist()
```

```python
    def _read_current_pose(self) -> tuple[np.ndarray, np.ndarray, list[float]]:
        """读 0x36 真实角 (anchor) → source → FK. 返回 (p, R, q_anchor)."""
        q_anchor = self.ctrl.read_real_angles(use_kb=True)
        q_src = anchor_to_source(q_anchor)
        T = fk_mdh(q_src)
        return T[:3, 3].copy(), T[:3, :3].copy(), q_anchor

    def _measure_dt(self) -> tuple[float, float]:
        """测量单调 dt (修订 #7): 首帧 dt_default, 之后实测并钳到 [dt_min, dt_max]."""
        now = self.clock()
        if self._last_step_mono is None:
            dt = self.dt_default_s
        else:
            dt = max(self.dt_min_s, min(self.dt_max_s, now - self._last_step_mono))
        self._last_step_mono = now
        return dt, now

    def _armed_or_error(self) -> Optional[dict]:
        """ARMED/TELEOP 门禁. 非门禁态返回错误 dict, 否则 None."""
        phase = self.ctrl.robot.phase
        if phase not in (RobotPhase.ARMED, RobotPhase.TELEOP):
            return {"moved": False, "reason": f"not_armed({phase.name})",
                    "target_xyz": None}
        return None

    def get_current_pose(self) -> EEPose:
        """当前末端位姿 (SO(3)) — Adapter/遥操公共接口 (P1-⑥), 不暴露内部 FK."""
        p, R, _ = self._read_current_pose()
        return EEPose(position=p, rotation=R)

    def step(self, vx: float, vy: float, vz: float,
             wx: float = 0.0, wy: float = 0.0, wz: float = 0.0,
             cmd_ts: Optional[float] = None) -> dict:
        """6DOF 笛卡尔速度闭环 (spec §4.1). 向后兼容 3DOF 调用 (ω 默认 0).

        cmd_ts: 命令产生时刻 (time.monotonic). 陈旧 (now-cmd_ts > stale_cmd_max_s)
        拒动 (控制层看门狗最终权威). 返回 {moved, reason, target_xyz, sigma_min,
        condition, lambda, scale, alarms?}.
        """
        err = self._armed_or_error()
        if err is not None:
            return err
        dt, now = self._measure_dt()
        if cmd_ts is not None and (now - cmd_ts) > self.stale_cmd_max_s:
            return {"moved": False, "reason": "stale_command", "target_xyz": None}

        # 速度 + 姿态速度钳制 (纯速度钳, 不走盒限幅)
        v = np.array([vx, vy, vz], dtype=float)
        w = np.array([wx, wy, wz], dtype=float)
        nv = float(np.linalg.norm(v))
        if nv > self.max_vel_mm_s and nv > 1e-12:
            v = v * (self.max_vel_mm_s / nv)
        nw = float(np.linalg.norm(w))
        if nw > self.max_ang_rad_s and nw > 1e-12:
            w = w * (self.max_ang_rad_s / nw)

        # 单次状态读取 → 统一安全链 (P1-⑤: 一次控制周期只读一次真实关节)
        p_act, _R_act, q_anchor = self._read_current_pose()
        return self._step_from_state(p_act, q_anchor, v, w, dt)

    def step_pose(self, p_des, R_des, cmd_ts: Optional[float] = None,
                  rpy_anchor: Optional[np.ndarray] = None,
                  rpy_limits: Optional[tuple[np.ndarray, np.ndarray]] = None) -> dict:
        """目标位姿接口 (spec §4.2). SE(3) 误差 → _step_from_state.

        姿态误差 R_err = R_des @ R_act.T → e_R = log_so3 (内部 SO(3)/轴角,
        禁止 Euler 累加). RPY 仅在可选安全约束 rpy_limits 使用 (相对 rpy_anchor).
        与 step 共享单次状态读取 (P1-⑤).
        """
        err = self._armed_or_error()
        if err is not None:
            return err
        dt, now = self._measure_dt()
        if cmd_ts is not None and (now - cmd_ts) > self.stale_cmd_max_s:
            return {"moved": False, "reason": "stale_command", "target_xyz": None}

        p_act, R_act, q_anchor = self._read_current_pose()   # 一次读取
        R_target = np.asarray(R_des, dtype=float)
        if rpy_limits is not None and rpy_anchor is not None:
            R_target = self._clamp_rpy_relative(R_target,
                                                np.asarray(rpy_anchor, dtype=float),
                                                rpy_limits)
        e_p = np.asarray(p_des, dtype=float) - p_act
        v = np.clip(self.kp_pos * e_p, -self.max_vel_mm_s, self.max_vel_mm_s)
        R_err = R_target @ R_act.T
        e_R = log_so3(R_err)
        w = np.clip(self.kr_ori * e_R, -self.max_ang_rad_s, self.max_ang_rad_s)
        return self._step_from_state(p_act, q_anchor, v, w, dt)

    def _step_from_state(self, p_act: np.ndarray, q_anchor: list[float],
                         v: np.ndarray, w: np.ndarray, dt: float) -> dict:
        """安全链主路径 (spec §1.3 严格顺序), 基于已读取状态, 单周期一次闭环."""
        # workspace limiter (基于 p_act; 零速度盒内命令放行 → 需 clamped 非空)
        v, clamped = self._limiter(v, p_act, dt)
        if float(np.linalg.norm(v)) < 1e-9 and clamped:
            return {"moved": False, "reason": "workspace_blocked",
                    "target_xyz": p_act.tolist()}

        # 奇异度指标 (归一化) + 自适应阻尼 (λ+scale 实际参与)
        q_src = anchor_to_source(q_anchor)
        J = jacobian(q_src)
        metrics = singularity_metrics(J)
        lam, scale = adaptive_damping(metrics, self.ik_lambda,
                                      near_ratio=self.near_ratio,
                                      sing_ratio=self.sing_ratio,
                                      lam_max=self.lam_max)
        if scale <= 0.0:
            return {"moved": False, "reason": "singular",
                    "target_xyz": p_act.tolist(),
                    "sigma_min": metrics["sigma_min"],
                    "condition": metrics["condition_number"],
                    "lambda": lam, "scale": scale}

        # 加权 DLS (twist 已乘 scale → 奇异规避实际生效)
        twist = np.append(v * dt, w * dt) * scale
        weights = [1.0, 1.0, 1.0] + [self.orient_weight] * 3
        dq = damped_ls(J, twist, lam, weights=weights)

        # 预测关节限位缩放 (margin 渐进, 只渐进减速; 硬拒绝交给 check_limits_real)
        dq = self._scale_toward_limits(q_src, dq)

        # velocity/acceleration 限制
        dq = np.clip(dq, -math.radians(self.max_dq_deg),
                     math.radians(self.max_dq_deg))
        max_dq_vel = math.radians(self.max_joint_vel_deg_s) * dt
        dq = np.clip(dq, -max_dq_vel, max_dq_vel)
        if self._last_dq is not None:
            max_dq_acc = math.radians(self.max_joint_acc_deg_s2) * dt
            dq = self._last_dq + np.clip(dq - self._last_dq,
                                         -max_dq_acc, max_dq_acc)
        self._last_dq = dq

        # 目标角 → anchor → 归一化 → 实时限位守卫 (最终硬拒绝) → 下发
        q_src_target = q_src + np.degrees(dq)
        q_anchor_target = source_to_anchor(q_src_target)
        q_anchor_target = [((x + 180.0) % 360.0) - 180.0 for x in q_anchor_target]
        alarms = self.ctrl.check_limits_real(q_anchor_target, use_kb=True)
        if alarms:
            return {"moved": False, "reason": "limit_alarm", "alarms": alarms,
                    "target_xyz": (p_act + v * dt).tolist(),
                    "sigma_min": metrics["sigma_min"],
                    "condition": metrics["condition_number"],
                    "lambda": lam, "scale": scale}
        self.ctrl.set_joints_safe(q_anchor_target, use_kb=True)
        return {"moved": True,
                "target_xyz": (p_act + v * dt).tolist(),
                "sigma_min": metrics["sigma_min"],
                "condition": metrics["condition_number"],
                "lambda": lam, "scale": scale}

    def _scale_toward_limits(self, q_src: np.ndarray, dq: np.ndarray) -> np.ndarray:
        """预测关节限位缩放: 越近限位越缩 (margin 渐进), 越界 → 缩到 0."""
        q_src_next = q_src + np.degrees(dq)
        q_anchor_next = source_to_anchor(q_src_next)
        q_anchor_next = [((x + 180.0) % 360.0) - 180.0 for x in q_anchor_next]
        scale = np.ones(6)
        margin = self.joint_limit_margin_deg
        for i in range(6):
            lo, hi = self.joint_limits[i]
            dist = min(q_anchor_next[i] - lo, hi - q_anchor_next[i])
            if dist < margin:
                scale[i] = max(0.0, dist / max(margin, 1e-9))
        return dq * scale

    def _clamp_rpy_relative(self, R_target, R_anchor, rpy_limits):
        """可选安全约束 (修订 #2): 目标相对 anchor 的 RPY clamp. 仅此用 RPY."""
        R_rel = R_anchor.T @ R_target
        rpy = np.array(self._rotmat_to_rpy(R_rel))
        rpy_c = np.clip(rpy, np.asarray(rpy_limits[0], float),
                        np.asarray(rpy_limits[1], float))
        return R_anchor @ self._rpy_to_rotmat(rpy_c)

    @staticmethod
    def _rotmat_to_rpy(R):
        """SO(3) → (roll, pitch, yaw) (intrinsic ZYX). 仅供可选安全约束."""
        sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        if sy > 1e-9:
            roll = math.atan2(R[2, 1], R[2, 2])
            pitch = math.atan2(-R[2, 0], sy)
            yaw = math.atan2(R[1, 0], R[0, 0])
        else:
            roll = math.atan2(-R[1, 2], R[1, 1])
            pitch = math.atan2(-R[2, 0], sy)
            yaw = 0.0
        return roll, pitch, yaw

    @staticmethod
    def _rpy_to_rotmat(rpy):
        r, p, y = rpy
        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)
        cy, sy = math.cos(y), math.sin(y)
        Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
        Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
        return Rz @ Ry @ Rx
```

- [ ] **Step 4: 运行测试确认通过**

Run: `python Arm-robot_VLA/lerobot_robot_massage/zdt/test_cartesian.py`
Expected: PASS（旧测试经 armed helper / FakeClock 改造后 + 新增全绿）

- [ ] **Step 5: 全量回归 + 提交**

Run: `cd Arm-robot_VLA && python -m pytest lerobot_robot_massage/zdt -q`
Expected: PASS

```bash
git add Arm-robot_VLA/lerobot_robot_massage/zdt/cartesian.py Arm-robot_VLA/lerobot_robot_massage/zdt/testutil.py Arm-robot_VLA/lerobot_robot_massage/zdt/test_cartesian.py
git -C Arm-robot_VLA commit -m "feat(zdt): CartesianController 6DOF 安全链 + 测量单调dt + 陈旧命令看门狗 + step_pose (修订#2/#4/#7)"
```

---

## Task 7: `scripts/teleop/arm_adapter.py` — Simulation/Real 统一接口

**Files:**
- Create: `Arm-robot_VLA/scripts/teleop/arm_adapter.py`
- Test: `Arm-robot_VLA/scripts/teleop/test_adapter.py`

**Interfaces:**
- Consumes: Task 1 `CartesianCommand`/`EEPose`/`JointState`；Task 5/6 的 `ZdtController`/`CartesianController`；现有 `ArmClient`。
- Produces:
  - `SimulationArmAdapter(arm_client)` — connect/disconnect/get_joint_state/get_ee_pose/move_cartesian_velocity/reset/e_stop
  - `RealArmAdapter(ctrl, **cart_kwargs)` — `connect()`（**只到 SAFE_IDLE，不自动 arm**）、`arm(gravity_confirmed=False)`、`enter_teleop()`/`exit_teleop()`、`disconnect()`、`get_joint_state()`、`get_real_joint_angles()`、`get_ee_pose()`（经 `cart.get_current_pose()`）、`move_cartesian_velocity()`、`step_pose()`、`reset()`、`e_stop()`、`state()`

- [ ] **Step 1: 写失败测试**

```python
"""arm_adapter 测试 — Simulation/Real 统一接口 (spec §6.1)."""
import sys
import unittest.mock as mock
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import numpy as np

from lerobot_robot_massage.zdt.types import CartesianCommand  # noqa: E402
from lerobot_robot_massage.zdt.config import F_READ_POS, ZdtConfig  # noqa: E402
from lerobot_robot_massage.zdt.controller import ZdtController  # noqa: E402
from lerobot_robot_massage.zdt.fakes import FakeTransport  # noqa: E402
from lerobot_robot_massage.zdt.safety import MotorState  # noqa: E402
from lerobot_robot_massage.zdt.testutil import FakeClock  # noqa: E402
from arm_adapter import RealArmAdapter, SimulationArmAdapter  # noqa: E402

ADDRS = [0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
READY_ANCHOR = [0.0, 60.0, 50.0, 0.0, 120.0, 0.0]


def _arm_robot(ctrl):
    ctrl.robot.on_connected()
    motors = {a: MotorState(can_id=a, online=True, joint_slot=i)
              for i, a in enumerate(ADDRS)}
    ctrl.robot.on_enumerated(motors)
    ctrl.robot.on_safe_idle()
    ctrl.robot.arm(gravity_confirmed=True)
    return ctrl


def _inject_anchor_pose(t, q_anchor):
    for addr, deg in zip(ADDRS, q_anchor):
        v = int(round(abs(deg) * 65536.0 / 360.0)) & 0xFFFFFFFF
        sign = 0x01 if deg < 0 else 0x00
        t.inject(addr, F_READ_POS,
                 bytes([sign, (v >> 24) & 0xFF, (v >> 16) & 0xFF,
                        (v >> 8) & 0xFF, v & 0xFF]) + b"\x6b")


def _real_adapter():
    t = FakeTransport()
    cfg = ZdtConfig(timeout_s=0.001, retries=0, reduction_ratios=[1.0] * 6,
                    calib=[(1.0, 0.0)] * 6)
    ctrl = ZdtController(config=cfg, transport=t)
    _arm_robot(ctrl)
    clock = FakeClock()
    adapter = RealArmAdapter(ctrl, clock=clock)
    return adapter, t, clock


def test_sim_adapter_maps_velocity_to_end_event():
    arm = mock.Mock()
    a = SimulationArmAdapter(arm)
    a.connect()
    arm.remote_enable.assert_called_once()
    a.move_cartesian_velocity(CartesianCommand((1.0, 2.0, 3.0), (0.1, 0.0, 0.0)))
    arm.end_event.assert_called_once()
    np.testing.assert_allclose(arm.end_event.call_args[0], [1.0, 2.0, 3.0, 0.1, 0.0, 0.0])


def test_sim_adapter_ee_pose_mm_and_rotmat():
    arm = mock.Mock()
    arm.get_ee_pose.return_value = ([0.1, 0.2, 0.3], [1.0, 0.0, 0.0, 0.0])
    a = SimulationArmAdapter(arm)
    ep = a.get_ee_pose()
    np.testing.assert_allclose(ep.position, [100.0, 200.0, 300.0])   # m → mm
    np.testing.assert_allclose(ep.rotation, np.eye(3), atol=1e-9)


def test_sim_adapter_joint_state():
    arm = mock.Mock()
    arm.get_state.return_value = ([0.0, 60.0, 50.0, 0.0, 120.0, 0.0],
                                  [0.0] * 6, [10.0] * 6)
    a = SimulationArmAdapter(arm)
    js = a.get_joint_state()
    assert list(js.q) == [0.0, 60.0, 50.0, 0.0, 120.0, 0.0]
    assert len(js.current_ma) == 6


def test_real_adapter_move_cartesian_sends_fd():
    adapter, t, clock = _real_adapter()
    for _ in range(3):                     # step: FK + check_limits + set_joints_safe
        _inject_anchor_pose(t, READY_ANCHOR)
    clock.tick(0.05)
    adapter.move_cartesian_velocity(
        CartesianCommand((10.0, 0.0, 0.0), (0.0, 0.0, 0.0), timestamp=clock.t))
    assert any(f.data and f.data[0] == 0xFD for f in t.sent)


def test_real_adapter_connect_does_not_arm():
    # P0-①: connect() 只到 SAFE_IDLE, 不使能扭矩; arm() 由调用方显式调用
    t = FakeTransport()
    cfg = ZdtConfig(timeout_s=0.001, retries=0, reduction_ratios=[1.0] * 6,
                    calib=[(1.0, 0.0)] * 6)
    ctrl = ZdtController(config=cfg, transport=t)
    for addr in range(0x01, 0x07):
        t.inject(addr, 0x1F, bytes([0x00, 0x01, 0x01]) + b"\x6b")
    for addr in range(0x01, 0x07):
        t.inject(addr, F_READ_POS, b"\x00\x00\x00\x00\x00" + b"\x6b")
    adapter = RealArmAdapter(ctrl)
    adapter.connect()
    assert ctrl.robot.phase.name == "SAFE_IDLE"
    assert not any(f.data and f.data[0] == 0xF3 for f in t.sent)   # 未使能扭矩
    adapter.arm(gravity_confirmed=True)
    assert ctrl.robot.phase.name == "ARMED"
    assert any(f.data and f.data[0] == 0xF3 for f in t.sent)


def test_real_adapter_get_ee_pose_uses_fk():
    adapter, t, clock = _real_adapter()
    from lerobot_robot_massage.zdt.kinematics import fk_mdh, anchor_to_source
    _inject_anchor_pose(t, READY_ANCHOR)
    ep = adapter.get_ee_pose()
    T = fk_mdh(anchor_to_source(READY_ANCHOR))
    np.testing.assert_allclose(ep.position, T[:3, 3], atol=1e-3)
    np.testing.assert_allclose(ep.rotation, T[:3, :3], atol=1e-6)


def test_real_adapter_does_not_touch_driver_directly():
    # RealArmAdapter 只经 cart/ctrl 公共 API — 源码不应引用 _driver
    src = Path(Path(__file__).resolve().parent / "arm_adapter.py").read_text()
    assert "_driver" not in src


def test_real_adapter_state_and_estop():
    adapter, t, clock = _real_adapter()
    assert adapter.state() == "ARMED"
    adapter.e_stop()
    assert t.sent[-1].arbitration_id == 0x0000


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
```

- [ ] **Step 2: 运行确认失败**

Run: `python Arm-robot_VLA/scripts/teleop/test_adapter.py`
Expected: FAIL — `ModuleNotFoundError: No module named 'arm_adapter'`

- [ ] **Step 3: 写实现**

```python
"""scripts/teleop/arm_adapter.py — 仿真/真机臂统一接口 (spec TASK-02/24).

SimulationArmAdapter: 复用 ArmClient (MuJoCo socket), 保持仿真遥操能力.
RealArmAdapter: 封装 CartesianController → ZdtController, 是唯一真机笛卡尔入口;
  不实现 CAN 协议 / 不重复 IK / 不直接操作电机帧. 视觉层只产 CartesianCommand.
"""
from __future__ import annotations

import numpy as np

from lerobot_robot_massage.zdt.types import (  # noqa: E402
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
```

- [ ] **Step 4: 运行确认通过**

Run: `python Arm-robot_VLA/scripts/teleop/test_adapter.py`
Expected: PASS

- [ ] **Step 5: 提交**

```bash
git add Arm-robot_VLA/scripts/teleop/arm_adapter.py Arm-robot_VLA/scripts/teleop/test_adapter.py
git -C Arm-robot_VLA commit -m "feat(teleop): Simulation/Real ArmAdapter 统一接口 (TASK-02/24)"
```

---

## Task 8: `scripts/teleop/watchdog.py` — VisionWatchdog 分级

**Files:**
- Create: `Arm-robot_VLA/scripts/teleop/watchdog.py`
- Test: `Arm-robot_VLA/scripts/teleop/test_watchdog.py`

**Interfaces:**
- Produces:
  - `WatchdogAction(Enum)` — OK / DECAY / STOP / ESTOP
  - `VisionWatchdog(conf_threshold=0.5, depth_invalid_hold_s=0.2, wrist_jump_mm=150.0, loss_stop_s=0.4, estop_s=1.0, decay_rate=0.5)`；`update(*, hand_present, hand_confidence, depth_valid, wrist_mm, now) -> tuple[WatchdogAction, float]`
  - **无 `stale_cmd_s`/`cmd_ts`**（P0-②）：陈旧命令判定归 `CartesianController.step(cmd_ts)`，控制层唯一权威；本看门狗只负责视觉健康（置信/深度/腕跳变/手丢失）。

- [ ] **Step 1: 写失败测试**

```python
"""VisionWatchdog 分级策略测试 (spec §6.2, TASK-16)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from watchdog import VisionWatchdog, WatchdogAction  # noqa: E402


def _wd(**kw):
    d = dict(conf_threshold=0.5, loss_stop_s=0.4, estop_s=1.0, decay_rate=0.5,
             wrist_jump_mm=150.0, depth_invalid_hold_s=0.2)
    d.update(kw)
    return VisionWatchdog(**d)


def _upd(w, *, hand_present=True, conf=0.9, depth_valid=True, wrist=(0.0, 0.0, 100.0),
         now=0.1):
    return w.update(hand_present=hand_present, hand_confidence=conf,
                    depth_valid=depth_valid, wrist_mm=wrist, now=now)


def test_ok_when_hand_confident():
    w = _wd()
    action, scale = _upd(w)
    assert action == WatchdogAction.OK and scale == 1.0


def test_low_confidence_escalates_to_decay():
    w = _wd()
    _upd(w, conf=0.3, now=0.1)               # 首帧建立 loss_start
    action, scale = _upd(w, conf=0.3, now=0.2)   # loss_s=0.1 → scale=0.95
    assert action == WatchdogAction.DECAY
    assert 0.0 < scale < 1.0


def test_depth_invalid_escalates():
    w = _wd()
    action, _ = _upd(w, depth_valid=False)
    assert action == WatchdogAction.DECAY


def test_depth_invalid_beyond_hold_stops():
    # P2-⑧: depth_invalid_hold_s=0.2 内只 DECAY, 超过后按 loss 升级
    w = _wd()
    a0, _ = _upd(w, depth_valid=False, now=0.1)
    assert a0 == WatchdogAction.DECAY
    a1, s1 = _upd(w, depth_valid=False, now=0.5)   # 0.4s > hold 0.2 且 > loss_stop 0.4
    assert a1 == WatchdogAction.STOP and s1 == 0.0


def test_hand_lost_prolonged_stops():
    w = _wd()
    _, _ = _upd(w, hand_present=False, conf=0.0, wrist=None, now=0.1)
    action, scale = _upd(w, hand_present=False, conf=0.0, wrist=None, now=0.5)
    assert action == WatchdogAction.STOP and scale == 0.0


def test_hand_lost_long_estops():
    w = _wd()
    for now in (0.1, 0.5, 1.2):
        _, _ = _upd(w, hand_present=False, conf=0.0, wrist=None, now=now)
    action, _ = _upd(w, hand_present=False, conf=0.0, wrist=None, now=1.3)
    assert action == WatchdogAction.ESTOP


def test_decay_is_gradual_not_hold():
    # 禁止无限保持上一帧: 丢失后 scale 单调下降
    w = _wd()
    _upd(w, hand_present=False, conf=0.0, wrist=None, now=0.05)  # 建立 loss_start
    s0 = _upd(w, hand_present=False, conf=0.0, wrist=None, now=0.15)[1]  # 0.95
    s1 = _upd(w, hand_present=False, conf=0.0, wrist=None, now=0.25)[1]  # 0.90
    assert s1 < s0 < 1.0


def test_wrist_jump_stops():
    w = _wd()
    _upd(w, now=0.1)
    action2, _ = _upd(w, wrist=(300.0, 0.0, 100.0), now=0.2)
    assert action2 == WatchdogAction.STOP


def test_recovery_after_loss_returns_ok():
    w = _wd()
    _upd(w, hand_present=False, conf=0.0, wrist=None, now=0.1)
    action, scale = _upd(w, now=0.3)
    assert action == WatchdogAction.OK and scale == 1.0


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
```

- [ ] **Step 2: 运行确认失败**

Run: `python Arm-robot_VLA/scripts/teleop/test_watchdog.py`
Expected: FAIL — `ModuleNotFoundError: No module named 'watchdog'`

- [ ] **Step 3: 写实现**

```python
"""scripts/teleop/watchdog.py — 视觉遥操分级看门狗 (spec §6.2, TASK-16).

职责边界 (P0-②): 陈旧命令判定归控制层 (CartesianController.step 的 cmd_ts
单调期限检查), 本看门狗**不接收 cmd_ts** — 只对视觉信号分级:
手丢失 / 低置信 / 深度无效 / 腕跳变, 产出 (action, velocity_scale).
禁止无限保持上一帧命令 (持续丢失 → 停; 严重 → e_stop 请求).
"""
from __future__ import annotations
from enum import Enum, auto

import numpy as np


class WatchdogAction(Enum):
    OK = auto()       # 正常 → 传递命令
    DECAY = auto()    # 短暂丢失 → 速度衰减 (hold with decay)
    STOP = auto()     # 持续丢失/跳变 → 停止 (命令清零)
    ESTOP = auto()    # 严重丢失 → 请求 e_stop


class VisionWatchdog:
    """视觉信号分级: 手丢失/低置信/深度无效/腕跳变."""

    def __init__(self, conf_threshold: float = 0.5,
                 depth_invalid_hold_s: float = 0.2,
                 wrist_jump_mm: float = 150.0,
                 loss_stop_s: float = 0.4,
                 estop_s: float = 1.0,
                 decay_rate: float = 0.5):
        self.conf_threshold = conf_threshold
        self.depth_invalid_hold_s = depth_invalid_hold_s
        self.wrist_jump_mm = wrist_jump_mm
        self.loss_stop_s = loss_stop_s
        self.estop_s = estop_s
        self.decay_rate = decay_rate
        self._loss_start: float | None = None
        self._last_wrist: np.ndarray | None = None

    def update(self, *, hand_present: bool, hand_confidence: float,
               depth_valid: bool, wrist_mm, now: float) -> tuple[WatchdogAction, float]:
        """每帧调用. Returns (action, velocity_scale), scale∈[0,1]."""
        vision_ok = (hand_present and hand_confidence >= self.conf_threshold
                     and depth_valid)
        if vision_ok:
            # 腕跳变 → 立即 STOP (速度不连续)
            if wrist_mm is not None and self._last_wrist is not None:
                if float(np.linalg.norm(np.asarray(wrist_mm) - self._last_wrist)) \
                        > self.wrist_jump_mm:
                    self._loss_start = now
                    return WatchdogAction.STOP, 0.0
            self._last_wrist = np.asarray(wrist_mm) if wrist_mm is not None else None
            self._loss_start = None
            return WatchdogAction.OK, 1.0

        # 视觉丢失: 记录丢失起点, 按时长分级
        if self._loss_start is None:
            self._loss_start = now
        loss_s = now - self._loss_start
        # 深度无效的短暂期 (< hold_s) 只衰减不升级 (P2-⑧)
        if not depth_valid and loss_s < self.depth_invalid_hold_s:
            scale = max(0.0, 1.0 - self.decay_rate * loss_s)
            return WatchdogAction.DECAY, scale
        if loss_s >= self.estop_s:
            return WatchdogAction.ESTOP, 0.0
        if loss_s >= self.loss_stop_s:
            return WatchdogAction.STOP, 0.0
        scale = max(0.0, 1.0 - self.decay_rate * loss_s)
        return WatchdogAction.DECAY, scale
```

- [ ] **Step 4: 运行确认通过**

Run: `python Arm-robot_VLA/scripts/teleop/test_watchdog.py`
Expected: PASS

- [ ] **Step 5: 提交**

```bash
git add Arm-robot_VLA/scripts/teleop/watchdog.py Arm-robot_VLA/scripts/teleop/test_watchdog.py
git -C Arm-robot_VLA commit -m "feat(teleop): VisionWatchdog 分级策略 (DECAY/STOP/ESTOP, 禁无限保持) (TASK-16)"
```

---

## Task 9: `zdt/recording.py` — EpisodeRecorder（JSONL 引用真实相机帧文件）

**Files:**
- Create: `Arm-robot_VLA/lerobot_robot_massage/zdt/recording.py`
- Test: `Arm-robot_VLA/lerobot_robot_massage/zdt/test_recording.py`

**Interfaces:**
- Produces:
  - `EpisodeRecorder(out_dir, save_frames=True, frame_format="png")` — `start_episode() -> str`；`add_record(observation, action, safety, color=None, depth=None, camera_ts=None) -> None`；`finish_episode() -> dict`
  - 观察字段含 `camera_frames: {"color": rel, "depth": rel}`（相对 episode 目录）与 `camera_ts`（修订 #5）

- [ ] **Step 1: 写失败测试**

```python
"""EpisodeRecorder 测试 — JSONL schema + observation/action 分离 + 帧文件引用 (spec TASK-35)."""
import json
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import numpy as np

from lerobot_robot_massage.zdt.recording import EpisodeRecorder  # noqa: E402


def _out(tag):
    p = Path("/tmp") / f"rec_{tag}"
    if p.exists():
        shutil.rmtree(p)
    return p


def _obs(q=(0.0,) * 6):
    return {
        "q": list(q),
        "dq": [0.0] * 6,
        "current": [10.0] * 6,
        "ee_pose": {"position": [0.0, 0.0, 0.0],
                    "quaternion": [1.0, 0.0, 0.0, 0.0]},
        "hand_pose": {"position": [0.0, 0.0, 100.0],
                      "orientation": [0.0, 0.0, 0.0], "confidence": 0.9},
    }


def _act():
    return {"cartesian_command": {"linear_velocity": [1.0, 0.0, 0.0],
                                  "angular_velocity": [0.0, 0.0, 0.0],
                                  "timestamp": 0.0},
            "commanded_joint_target": [0.0] * 6}


def _safety():
    return {"phase": "TELEOP", "sigma_min": 0.1, "condition": 5.0, "workspace_ok": True}


def test_add_record_writes_valid_jsonl_schema():
    out = _out("a")
    rec = EpisodeRecorder(out)
    ep = rec.start_episode()
    rec.add_record(_obs(), _act(), _safety(), camera_ts=1.5)
    rec.finish_episode()
    lines = (out / ep / "data.jsonl").read_text().strip().splitlines()
    assert len(lines) == 1
    row = json.loads(lines[0])
    assert set(row) == {"timestamp", "observation", "action", "safety"}
    assert set(row["observation"]) == {"q", "dq", "current", "ee_pose", "hand_pose"}
    assert set(row["action"]) == {"cartesian_command", "commanded_joint_target"}
    assert row["safety"]["phase"] == "TELEOP"
    # observation / action 分离 (spec §7)
    assert row["observation"]["q"][0] == 0.0
    assert row["action"]["cartesian_command"]["linear_velocity"] == [1.0, 0.0, 0.0]


def test_records_camera_frames_files():
    out = _out("b")
    rec = EpisodeRecorder(out)
    ep = rec.start_episode()
    color = (np.random.rand(4, 4, 3) * 255).astype("uint8")
    depth = (np.random.rand(4, 4) * 1000).astype("uint16")
    rec.add_record(_obs(), _act(), _safety(), color=color, depth=depth, camera_ts=2.0)
    rec.finish_episode()
    row = json.loads((out / ep / "data.jsonl").read_text().strip().splitlines()[0])
    frames = row["observation"]["camera_frames"]
    assert set(frames) == {"color", "depth"}
    assert row["observation"]["camera_ts"] == 2.0
    # 文件真实存在 (修订 #5: 引用帧文件, 非仅时间戳)
    assert (out / ep / frames["color"]).exists()
    assert (out / ep / frames["depth"]).exists()
    assert not frames["color"].startswith(str(ep))     # 相对路径


def test_no_frames_when_save_frames_false():
    out = _out("c")
    rec = EpisodeRecorder(out, save_frames=False)
    ep = rec.start_episode()
    rec.add_record(_obs(), _act(), _safety(), color=np.zeros((2, 2, 3), "uint8"))
    rec.finish_episode()
    row = json.loads((out / ep / "data.jsonl").read_text().strip().splitlines()[0])
    assert "camera_frames" not in row["observation"]
    assert not (out / ep / "frames").exists()


def test_multiple_episodes_separate_dirs():
    out = _out("d")
    rec = EpisodeRecorder(out)
    ep1 = rec.start_episode()
    rec.add_record(_obs(), _act(), _safety())
    rec.finish_episode()
    ep2 = rec.start_episode()
    rec.add_record(_obs(), _act(), _safety())
    rec.finish_episode()
    assert ep1 != ep2
    assert (out / ep1 / "data.jsonl").exists()
    assert (out / ep2 / "data.jsonl").exists()


def test_finish_returns_stats():
    out = _out("e")
    rec = EpisodeRecorder(out)
    ep = rec.start_episode()
    rec.add_record(_obs(), _act(), _safety())
    stats = rec.finish_episode()
    assert stats["records"] == 1
    assert stats["path"] == str(out / ep)


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
```

- [ ] **Step 2: 运行确认失败**

Run: `python Arm-robot_VLA/lerobot_robot_massage/zdt/test_recording.py`
Expected: FAIL — `ModuleNotFoundError: No module named 'lerobot_robot_massage.zdt.recording'`

- [ ] **Step 3: 写实现**

```python
"""zdt/recording.py — EpisodeRecorder: 每 episode 一个 JSONL + 相机帧文件 (spec TASK-35).

修订 #5: JSONL 的 observation.camera_frames 引用**真实落盘的帧文件** (相对路径),
非仅时间戳; color/depth 由调用方传入 ndarray, 本模块负责写盘并回填引用.
observation / action 明确分离, 直接服务后续 ACT / Diffusion Policy / SmolVLA.
"""
from __future__ import annotations

import json
import shutil
import time
from pathlib import Path

import numpy as np


class EpisodeRecorder:
    """增量 JSONL 录制: start_episode → add_record×N → finish_episode."""

    def __init__(self, out_dir, save_frames: bool = True,
                 frame_format: str = "png"):
        self.out_dir = Path(out_dir)
        self.save_frames = save_frames
        self.frame_format = frame_format
        self._episode_dir: Path | None = None
        self._frames_dir: Path | None = None
        self._jsonl = None
        self._frame_idx = 0

    def start_episode(self) -> str:
        # P0-③: 纳秒级精度, 同秒启动多个 episode 也不会覆盖
        ep_id = f"episode_{time.strftime('%Y%m%d_%H%M%S')}_{time.time_ns()}"
        self._episode_dir = self.out_dir / ep_id
        if self._episode_dir.exists():
            shutil.rmtree(self._episode_dir)
        self._episode_dir.mkdir(parents=True, exist_ok=True)
        self._frames_dir = self._episode_dir / "frames"
        self._jsonl = open(self._episode_dir / "data.jsonl", "w", encoding="utf-8")
        self._frame_idx = 0
        return ep_id

    def add_record(self, observation: dict, action: dict, safety: dict,
                   color=None, depth=None, camera_ts=None) -> None:
        """写一条 JSONL. color/depth 为 (H,W[,3]) / (H,W) 数组时落盘并回填引用."""
        if self._jsonl is None:
            raise RuntimeError("先调用 start_episode()")
        obs = dict(observation)
        if self.save_frames and (color is not None or depth is not None):
            rel = self._save_frames(color, depth)
            obs["camera_frames"] = rel
            obs["camera_ts"] = float(camera_ts) if camera_ts is not None \
                else time.monotonic()
        row = {
            "timestamp": time.monotonic(),
            "observation": obs,
            "action": dict(action),
            "safety": dict(safety),
        }
        self._jsonl.write(json.dumps(row) + "\n")

    def finish_episode(self) -> dict:
        """落盘 JSONL 并返回统计. 之后可 start_episode 新 episode."""
        if self._jsonl is None or self._episode_dir is None:
            raise RuntimeError("无进行中的 episode")
        self._jsonl.flush()
        with self._episode_dir.joinpath("data.jsonl").open() as f:
            n_records = sum(1 for _ in f)
        self._jsonl.close()
        n_frames = len(list(self._frames_dir.glob(f"*.{self.frame_format}"))) \
            if self._frames_dir is not None else 0
        stats = {"records": n_records, "frames": n_frames,
                 "path": str(self._episode_dir)}
        self._jsonl = None
        return stats

    def _save_frames(self, color, depth) -> dict[str, str]:
        assert self._frames_dir is not None
        rel: dict[str, str] = {}
        idx = self._frame_idx
        if color is not None:
            fn = f"{idx:06d}_color.{self.frame_format}"
            self._write_image(self._frames_dir / fn, color)
            rel["color"] = f"frames/{fn}"
        if depth is not None:
            fn = f"{idx:06d}_depth.{self.frame_format}"
            self._write_image(self._frames_dir / fn, depth)
            rel["depth"] = f"frames/{fn}"
        self._frame_idx += 1
        return rel

    @staticmethod
    def _write_image(path: Path, img) -> None:
        img = np.asarray(img)
        path.parent.mkdir(parents=True, exist_ok=True)
        try:
            import cv2
            cv2.imwrite(str(path), img)
        except ImportError:
            path.write_bytes(img.tobytes())   # 无 cv2 时写裸数据 (测试环境兜底)
```

- [ ] **Step 4: 运行确认通过**

Run: `python Arm-robot_VLA/lerobot_robot_massage/zdt/test_recording.py`
Expected: PASS

- [ ] **Step 5: 提交**

```bash
git add Arm-robot_VLA/lerobot_robot_massage/zdt/recording.py Arm-robot_VLA/lerobot_robot_massage/zdt/test_recording.py
git -C Arm-robot_VLA commit -m "feat(zdt): EpisodeRecorder JSONL + 相机帧文件引用 (修订#5, TASK-35)"
```

---

## Task 10: `scripts/teleop/real_arm_teleop.py` — 真机视觉遥操入口

**Files:**
- Create: `Arm-robot_VLA/scripts/teleop/real_arm_teleop.py`
- Test: `Arm-robot_VLA/scripts/teleop/test_real_arm_teleop.py`

**Interfaces:**
- Consumes: Task 7 `RealArmAdapter`；Task 8 `VisionWatchdog`/`WatchdogAction`；Task 9 `EpisodeRecorder`；Leap_Hand 共享 `camera/hand_tracker/wrist_tracker`。
- Produces:
  - `RealArmTeleop(adapter, watchdog, recorder, hand_provider, key_provider)` — `run_once(cmd_ts, now) -> dict`（`{action, cmd, phase}`，可无相机单测）
  - `main()` — 装配真机依赖（RealSense + HandTracker + WristTracker + handeye + adapter + watchdog + recorder）

- [ ] **Step 1: 写失败测试**

```python
"""real_arm_teleop 管线测试 — 无相机, 用 fake provider 驱动 run_once (spec §6.2)."""
import sys
import shutil
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from lerobot_robot_massage.zdt.types import CartesianCommand, JointState  # noqa: E402
from lerobot_robot_massage.zdt.recording import EpisodeRecorder  # noqa: E402
from watchdog import VisionWatchdog  # noqa: E402
from real_arm_teleop import RealArmTeleop  # noqa: E402


class _FakeAdapter:
    def __init__(self):
        self.calls = []
        self.e_stop_calls = 0
        self.phase = "ARMED"

    def move_cartesian_velocity(self, cmd: CartesianCommand):
        self.calls.append(("move", cmd))

    def e_stop(self):
        self.e_stop_calls += 1

    def get_joint_state(self):
        return JointState(q=(0.0,) * 6)

    def state(self):
        return self.phase


def _out(tag):
    p = Path("/tmp") / f"teleop_{tag}"
    if p.exists():
        shutil.rmtree(p)
    return p


def _hand(**kw):
    d = {"hand_present": True, "confidence": 0.9, "depth_valid": True,
         "wrist_mm": (0.0, 0.0, 100.0), "velocity": (5.0, 0.0, 0.0)}
    d.update(kw)
    return d


def test_fresh_command_flows_to_adapter():
    adapter = _FakeAdapter()
    wd = VisionWatchdog()
    rec = EpisodeRecorder(_out("fresh"))
    rec.start_episode()
    teleop = RealArmTeleop(adapter, wd, rec, hand_provider=lambda: _hand(),
                           key_provider=lambda: None)
    out = teleop.run_once(cmd_ts=0.1, now=0.15)
    assert out["action"] == "OK"
    assert adapter.calls and adapter.calls[-1][0] == "move"
    assert list(adapter.calls[-1][1].linear_velocity) == [5.0, 0.0, 0.0]
    rec.finish_episode()


def test_stale_command_no_motion():
    adapter = _FakeAdapter()
    wd = VisionWatchdog()
    rec = EpisodeRecorder(_out("stale"))
    rec.start_episode()
    teleop = RealArmTeleop(adapter, wd, rec, hand_provider=lambda: _hand(),
                           key_provider=lambda: None)
    out = teleop.run_once(cmd_ts=0.0, now=0.9)   # 陈旧
    assert out["action"] == "STOP"
    assert adapter.calls == [] or adapter.calls[-1][0] != "move"
    rec.finish_episode()


def test_estop_calls_adapter_estop():
    adapter = _FakeAdapter()
    wd = VisionWatchdog()
    rec = EpisodeRecorder(_out("estop"))
    rec.start_episode()
    hand = _hand(hand_present=False, confidence=0.0, depth_valid=False, wrist_mm=None)
    teleop = RealArmTeleop(adapter, wd, rec, hand_provider=lambda: hand,
                           key_provider=lambda: None)
    teleop.run_once(cmd_ts=0.0, now=0.1)
    teleop.run_once(cmd_ts=0.0, now=0.5)
    out = teleop.run_once(cmd_ts=0.0, now=1.2)   # 连续丢失 > estop_s
    assert out["action"] == "ESTOP"
    assert adapter.e_stop_calls == 1
    rec.finish_episode()


def test_key_estop_immediate():
    adapter = _FakeAdapter()
    wd = VisionWatchdog()
    rec = EpisodeRecorder(_out("key_estop"))
    rec.start_episode()
    teleop = RealArmTeleop(adapter, wd, rec, hand_provider=lambda: _hand(),
                           key_provider=lambda: ord("y"))
    out = teleop.run_once(cmd_ts=0.1, now=0.15)
    assert out["action"] == "ESTOP"
    assert adapter.e_stop_calls == 1
    rec.finish_episode()


def test_recorder_gets_records():
    adapter = _FakeAdapter()
    wd = VisionWatchdog()
    rec = EpisodeRecorder(_out("recs"))
    rec.start_episode()
    teleop = RealArmTeleop(adapter, wd, rec, hand_provider=lambda: _hand(),
                           key_provider=lambda: None)
    teleop.run_once(cmd_ts=0.1, now=0.15)
    teleop.run_once(cmd_ts=0.2, now=0.25)
    stats = rec.finish_episode()
    assert stats["records"] == 2


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
```

- [ ] **Step 2: 运行确认失败**

Run: `python Arm-robot_VLA/scripts/teleop/test_real_arm_teleop.py`
Expected: FAIL — `ModuleNotFoundError: No module named 'real_arm_teleop'`

- [ ] **Step 3: 写实现**

```python
"""scripts/teleop/real_arm_teleop.py — 真机 6DOF 视觉遥操入口 (spec TASK-23).

管线: RealSense + HandTracker + WristTracker → CartesianCommand → VisionWatchdog
(分级) → RealArmAdapter → CartesianController → ZdtController → CAN.
控制层是陈旧命令最终权威 (step 的 cmd_ts 单调期限); 本入口只做视觉分级 + 组装.
按键: H=clutch, R=reset/ready, Y=e_stop, Q/ESC=安全退出.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from lerobot_robot_massage.zdt.types import CartesianCommand  # noqa: E402
from watchdog import WatchdogAction, VisionWatchdog  # noqa: E402
from arm_adapter import RealArmAdapter  # noqa: E402
from lerobot_robot_massage.zdt.recording import EpisodeRecorder  # noqa: E402


class RealArmTeleop:
    """一帧遥操逻辑 (可无相机单测): hand_provider → cmd → watchdog → adapter."""

    def __init__(self, adapter, watchdog, recorder, hand_provider, key_provider):
        self.adapter = adapter
        self.watchdog = watchdog
        self.recorder = recorder
        self.hand_provider = hand_provider      # () -> dict | None
        self.key_provider = key_provider        # () -> key or None
        self._cmd = CartesianCommand((0.0, 0.0, 0.0))

    def run_once(self, cmd_ts: float, now: float) -> dict:
        """跑一帧: 返回 {action, cmd, phase}. 调用方提供时间 (单调)."""
        key = self.key_provider()
        if key in (ord("y"), ord("Y")):
            self.adapter.e_stop()
            return {"action": "ESTOP", "cmd": self._cmd,
                    "phase": self.adapter.state()}
        if key in (ord("q"), 27):
            return {"action": "QUIT", "cmd": self._cmd,
                    "phase": self.adapter.state()}

        hand = self.hand_provider()
        cmd = self._build_command(hand, cmd_ts)
        action, scale = self.watchdog.update(
            hand_present=bool(hand and hand.get("hand_present")),
            hand_confidence=float(hand.get("confidence", 0.0)) if hand else 0.0,
            depth_valid=bool(hand and hand.get("depth_valid")),
            wrist_mm=hand.get("wrist_mm") if hand else None,
            now=now)   # 陈旧判定归控制层 (adapter.move_cartesian_velocity → step(cmd_ts))

        scaled = CartesianCommand(
            tuple(float(v) * scale for v in cmd.linear_velocity),
            tuple(float(w) * scale for w in cmd.angular_velocity),
            timestamp=cmd.timestamp)
        if action == WatchdogAction.ESTOP:
            self.adapter.e_stop()
        elif action != WatchdogAction.STOP:
            self.adapter.move_cartesian_velocity(scaled)

        self._record(hand, scaled, action.name)
        return {"action": action.name, "cmd": scaled,
                "phase": self.adapter.state()}

    def _build_command(self, hand, cmd_ts: float) -> CartesianCommand:
        """由手部信息合成 CartesianCommand (遥操产线可替换实现)."""
        if hand is None:
            return CartesianCommand((0.0, 0.0, 0.0), timestamp=cmd_ts)
        v = hand.get("velocity") or (0.0, 0.0, 0.0)
        w = hand.get("angular_velocity") or (0.0, 0.0, 0.0)
        return CartesianCommand(tuple(float(x) for x in v),
                                tuple(float(x) for x in w),
                                timestamp=cmd_ts)

    def _record(self, hand, cmd: CartesianCommand, action: str) -> None:
        obs = {
            "q": [0.0] * 6, "dq": [0.0] * 6, "current": [],
            "ee_pose": {"position": [0.0, 0.0, 0.0],
                        "quaternion": [1.0, 0.0, 0.0, 0.0]},
            "hand_pose": ({"position": list(hand.get("wrist_mm", (0, 0, 0))),
                           "orientation": [0.0, 0.0, 0.0],
                           "confidence": hand.get("confidence", 0.0)}
                          if hand else {"position": [], "orientation": [],
                                        "confidence": 0.0}),
        }
        act = {"cartesian_command": {"linear_velocity": list(cmd.linear_velocity),
                                     "angular_velocity": list(cmd.angular_velocity),
                                     "timestamp": cmd.timestamp},
               "commanded_joint_target": [0.0] * 6}
        saf = {"phase": self.adapter.state(), "action": action}
        self.recorder.add_record(obs, act, saf)


def main():
    ap = argparse.ArgumentParser(description="真机 6DOF 视觉遥操")
    ap.add_argument("--iface", default="can0", help="SocketCAN 接口")
    ap.add_argument("--calib", default=str(Path(__file__).parent / "handeye_calib.json"))
    ap.add_argument("--out", default="datasets/teleop_real", help="录制输出目录")
    ap.add_argument("-y", "--gravity-confirm", action="store_true",
                    help="确认重力关节 J2/J3 二次确认 (必须)")
    ap.add_argument("--no-drive", action="store_true", help="只显示不发送")
    args = ap.parse_args()
    if not args.gravity_confirm:
        sys.exit("遥操前必须 -y/--gravity-confirm 确认重力关节 (J2/J3)")

    # 真机依赖装配 (与 demo_arm_teleop 共用 Leap_Hand 共享模块)
    sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "Leap_Hand" / "python"))
    import cv2  # noqa: E402
    from gesture_mapping.camera import open_realsense  # noqa: E402
    from gesture_mapping.hand_tracker import HandTracker  # noqa: E402

    from lerobot_robot_massage.zdt.config import ZdtConfig  # noqa: E402
    from lerobot_robot_massage.zdt.controller import ZdtController  # noqa: E402

    cam = open_realsense()
    if cam is None:
        sys.exit("未检测到 RealSense (D455) 相机")
    tracker = HandTracker(max_num_hands=1)
    ctrl = ZdtController(ZdtConfig(channel=args.iface))
    adapter = RealArmAdapter(ctrl, gravity_confirmed=True)
    watchdog = VisionWatchdog()
    recorder = EpisodeRecorder(args.out)

    from gesture_mapping.wrist_tracker import build_palm_pts  # noqa: E402

    def hand_provider():
        ok, bgr, depth, K = cam.read_with_depth()
        if not ok or bgr is None:
            return None
        hands = tracker.detect(bgr)
        if not hands:
            return None
        pts = build_palm_pts(hands[0], depth, K)
        if pts is None:
            return {"hand_present": True, "confidence": 0.0,
                    "depth_valid": False, "wrist_mm": None}
        return {"hand_present": True, "confidence": 0.9, "depth_valid": True,
                "wrist_mm": tuple(float(v) for v in pts[0])}

    teleop = RealArmTeleop(adapter, watchdog, recorder, hand_provider,
                           key_provider=lambda: 0)
    # P0-①: connect → SAFE_IDLE → 显式 arm (已 -y 确认重力) → TELEOP → reset (实际运动)
    adapter.connect()
    adapter.arm(gravity_confirmed=True)
    adapter.enter_teleop()
    adapter.reset()
    recorder.start_episode()
    try:
        while True:
            now = time.monotonic()
            out = teleop.run_once(cmd_ts=now, now=now)
            if out["action"] == "QUIT":
                break
            if out["action"] == "ESTOP":
                print("[急停] e_stop 已触发")
                break
            cv2.waitKey(1)
    finally:
        try:
            adapter.e_stop()
        except Exception:  # noqa: BLE001
            pass
        recorder.finish_episode()
        adapter.disconnect()
        cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
```

（注：`main()` 的 `hand_provider` 是骨架——把 `WristTracker` 的完整姿态跟随迁移（clutch/重锚/手眼 R 映射）归入真机验证阶段 TASK-27~34。核心管线 `RealArmTeleop` 已可无相机单测。）

- [ ] **Step 4: 运行确认通过**

Run: `python Arm-robot_VLA/scripts/teleop/test_real_arm_teleop.py`
Expected: PASS

- [ ] **Step 5: 提交**

```bash
git add Arm-robot_VLA/scripts/teleop/real_arm_teleop.py Arm-robot_VLA/scripts/teleop/test_real_arm_teleop.py
git -C Arm-robot_VLA commit -m "feat(teleop): real_arm_teleop 真机遥操入口 + 可单测管线 (TASK-23)"
```

---

## Task 11: 仿真回归 — 全栈闭环

**Files:**
- Create: `Arm-robot_VLA/scripts/teleop/test_sim_regression.py`
- Test: 同上

**Interfaces:**
- Consumes: Task 7 `SimulationArmAdapter`；Task 8 `VisionWatchdog`；Task 9 `EpisodeRecorder`；`kinematics.fk_mdh` / `jacobian`（fake server 物理积分）。
- Produces: `FakeMuJoCoServer`（测试辅助，模拟 mujoco_sim 文本协议 + FK 积分）。

- [ ] **Step 1: 写测试**

```python
"""仿真回归 — 全栈闭环: SimulationArmAdapter → fake MuJoCo server → watchdog → recorder.

验证依赖链终点: types → kinematics → adapter → watchdog → recording → teleop 逻辑,
在无硬件/无 MuJoCo 二进制下可跑. fake server 用 zdt 运动学做物理积分, 与控制器同源.
"""
import socket
import sys
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import numpy as np

from lerobot_robot_massage.zdt.types import CartesianCommand  # noqa: E402
from lerobot_robot_massage.zdt.kinematics import fk_mdh, jacobian, RESET_POSE_DEG  # noqa: E402
from lerobot_robot_massage.zdt.recording import EpisodeRecorder  # noqa: E402
from watchdog import VisionWatchdog  # noqa: E402
from arm_adapter import SimulationArmAdapter  # noqa: E402
from real_arm_teleop import RealArmTeleop  # noqa: E402


class FakeMuJoCoServer:
    """模拟 mujoco_sim.py 文本协议 + FK 积分 (end_event → q += J⁻¹·twist·dt)."""

    def __init__(self):
        self.q = list(RESET_POSE_DEG)                 # source 帧
        self.lock = threading.Lock()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("127.0.0.1", 0))
        self._sock.listen(1)
        self.port = self._sock.getsockname()[1]
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        conn, _ = self._sock.accept()
        buf = b""
        with conn:
            while True:
                data = conn.recv(4096)
                if not data:
                    break
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    resp = self._handle(line.decode().strip())
                    if resp is not None:
                        conn.sendall(resp + b"\n")

    def _handle(self, line):
        with self.lock:
            if line.startswith("get_ee_pose"):
                from lerobot_robot_massage.zdt.types import rotmat_to_quat
                T = fk_mdh(self.q)
                p = T[:3, 3] / 1000.0                  # mm → m
                q = rotmat_to_quat(T[:3, :3])          # wxyz
                return (f"EEPOSE:{p[0]:.6f},{p[1]:.6f},{p[2]:.6f},"
                        f"{q[0]:.6f},{q[1]:.6f},{q[2]:.6f},{q[3]:.6f}").encode()
            if line.startswith("get_state"):
                zero = ",0" * 12
                return (f"STATE:{','.join(f'{a:.4f}' for a in self.q)}{zero}").encode()
            if line.startswith("end_event"):
                vals = [float(x) for x in line.split()[1:]]
                twist = np.array(vals)                  # mm/s + rad/s
                J = jacobian(self.q)
                dq = np.linalg.solve(J, twist * 0.02)   # dt=0.02
                self.q = [self.q[i] + np.degrees(dq[i]) for i in range(6)]
                return None
            return None                                 # remote_enable/disable/soft_reset


def _rotation_angle(R):
    """SO(3) → 转角 (rad) (测试辅助)."""
    import math
    return math.acos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))


def _run_sim_sequence(server, adapter, wd, rec, cmd_6, frames=30, dt=0.02):
    """驱动 N 帧固定 6DOF 命令, 返回 (start_T, end_T). 手始终在 → watchdog OK."""
    present = {"hand_present": True, "confidence": 0.9, "depth_valid": True,
               "wrist_mm": (0.0, 0.0, 100.0)}
    teleop = RealArmTeleop(adapter, wd, rec,
                           hand_provider=lambda: dict(present),
                           key_provider=lambda: None)
    teleop._build_command = lambda h, ts: CartesianCommand(
        tuple(cmd_6[:3]), tuple(cmd_6[3:]), timestamp=ts)
    T0 = fk_mdh(np.array(server.q))
    for i in range(frames):
        teleop.run_once(cmd_ts=0.0 + i * dt, now=0.0 + i * dt)
    T1 = fk_mdh(np.array(server.q))
    return T0, T1


def test_sim_regression_full_stack():
    """全栈闭环: adapter + watchdog + recorder, 30 帧纯 +X 位移 > 2mm."""
    server = FakeMuJoCoServer()
    arm = _make_arm_client(server.port)
    adapter = SimulationArmAdapter(arm)
    wd = VisionWatchdog()
    rec = EpisodeRecorder("/tmp/sim_reg_1")
    rec.start_episode()
    adapter.connect()
    T0, T1 = _run_sim_sequence(server, adapter, wd, rec, (5.0, 0.0, 0.0, 0, 0, 0))
    # 30 帧 × 5mm/s × 0.02s = 3mm (+x 净位移, FK 积分误差容忍)
    assert T1[0, 3] - T0[0, 3] > 2.0, f"仿真闭环位移过小: {T1[0,3]-T0[0,3]:.2f}mm"
    stats = rec.finish_episode()
    assert stats["records"] == 30
    adapter.disconnect()
    server._sock.close()


def test_sim_regression_six_dof():
    """P1-⑦: 真正 6DOF — 纯 X/Y/Z 验位置、纯 Rx/Ry/Rz 验姿态."""
    cases = [
        ("X",  (5.0, 0.0, 0.0, 0.0, 0.0, 0.0), "pos", 0),
        ("Y",  (0.0, 5.0, 0.0, 0.0, 0.0, 0.0), "pos", 1),
        ("Z",  (0.0, 0.0, 5.0, 0.0, 0.0, 0.0), "pos", 2),
        ("Rx", (0.0, 0.0, 0.0, 0.15, 0.0, 0.0), "rot", 0),
        ("Ry", (0.0, 0.0, 0.0, 0.0, 0.15, 0.0), "rot", 1),
        ("Rz", (0.0, 0.0, 0.0, 0.0, 0.0, 0.15), "rot", 2),
    ]
    for label, cmd6, kind, idx in cases:
        server = FakeMuJoCoServer()
        arm = _make_arm_client(server.port)
        adapter = SimulationArmAdapter(arm)
        wd = VisionWatchdog()
        rec = EpisodeRecorder(f"/tmp/sim_6dof_{label}")
        rec.start_episode()
        adapter.connect()
        T0, T1 = _run_sim_sequence(server, adapter, wd, rec, cmd6)
        dp = T1[:3, 3] - T0[:3, 3]
        dR = _rotation_angle(T0[:3, :3].T @ T1[:3, :3])
        if kind == "pos":
            assert abs(dp[idx]) > 2.0, f"{label} 位置未动: {dp}"
        else:
            assert dR > 0.05, f"{label} 姿态未动: {dR} rad"
        assert rec.finish_episode()["records"] == 30
        adapter.disconnect()
        server._sock.close()


def test_sim_regression_combined_6dof():
    """组合 [vx,vy,vz,wx,wy,wz] 同时移动位置 + 姿态."""
    server = FakeMuJoCoServer()
    arm = _make_arm_client(server.port)
    adapter = SimulationArmAdapter(arm)
    wd = VisionWatchdog()
    rec = EpisodeRecorder("/tmp/sim_6dof_comb")
    rec.start_episode()
    adapter.connect()
    T0, T1 = _run_sim_sequence(server, adapter, wd, rec,
                               (3.0, 2.0, 1.0, 0.08, 0.06, 0.04))
    dp = T1[:3, 3] - T0[:3, 3]
    dR = _rotation_angle(T0[:3, :3].T @ T1[:3, :3])
    assert float(np.linalg.norm(dp)) > 2.0, f"组合位置未动: {dp}"
    assert dR > 0.05, f"组合姿态未动: {dR} rad"
    adapter.disconnect()
    server._sock.close()


def _make_arm_client(port):
    from arm_client import ArmClient
    import serial
    url = f"socket://127.0.0.1:{port}"
    ser = serial.serial_for_url(url, timeout=0.1)
    return ArmClient(url, ser=ser)


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
```

- [ ] **Step 2: 运行确认失败/通过**

Run: `python Arm-robot_VLA/scripts/teleop/test_sim_regression.py`
Expected: PASS（闭环位移 > 1mm，30 条记录）

（若 `pyserial` 的 `socket://` handler 在本环境不可用，把 `_make_arm_client` 换成最小原生 socket 客户端，实现 `get_ee_pose/get_state/end_event/remote_enable/remote_disable/soft_reset/close` 同协议。）

- [ ] **Step 3: 提交**

```bash
git add Arm-robot_VLA/scripts/teleop/test_sim_regression.py
git -C Arm-robot_VLA commit -m "test(teleop): 仿真全栈闭环回归 (adapter+watchdog+recording, 无硬件)"
```

---

## Task 12: 文档收尾 + 上游 Plan 标记

**Files:**
- Modify: `Arm-robot_VLA/CLAUDE.md`（架构边界 → USB-CAN 直连；ADR 增补）
- Modify: 上游 `TuinaDex_6DOF_视觉遥操实施Plan.md`（TASK-01~26/35 勾选 + TASK-27~34 `pending: 真机验证`）

- [ ] **Step 1: CLAUDE.md 架构更新**

把 `## 三、架构边界` 的边界图与 `## 五、架构决策记录 (ADR)` 更新为当前 USB-CAN 架构：

- 边界图改为：`RealArmAdapter → CartesianController → ZdtController → ZdtDriver → CAN (0xFD + snF + multi_sync)`；删除"STM32 是唯一直接控制电机"的旧描述。
- ADR 增补：
  - **ADR-005**：STM32 网关已移除，PC 直连 USB-CAN（SocketCAN）。安全包络由 PC 层执行：`RobotStateMachine` 门禁 + 0x36 真实位置软限位 + 枚举硬不变式。
  - **ADR-006**：内部姿态表示 = SO(3)/轴角；RPY 仅可选安全约束。
  - **ADR-007**：`CartesianController` 是唯一笛卡尔运动入口；陈旧命令看门狗归控制层（单调期限）。

- [ ] **Step 2: 上游 Plan 勾选**

打开上游 `TuinaDex_6DOF_视觉遥操实施Plan.md`（路径见 spec 头部），把 TASK-01~26 与 TASK-35 标 `[x] 已实现 (2026-08-23)` 并注明实现计划路径；TASK-27~34 标注 `pending: 真机验证`。

- [ ] **Step 3: 全量回归（终态）**

Run: `cd Arm-robot_VLA && python -m pytest lerobot_robot_massage/zdt scripts/teleop scripts/bringup -q`
Expected: PASS（zdt + teleop + bringup 全绿，含全部新增测试）

- [ ] **Step 4: 提交**

```bash
git add Arm-robot_VLA/CLAUDE.md
git -C Arm-robot_VLA commit -m "docs(zdt): CLAUDE.md 更新为 USB-CAN 直连架构 + SO(3)/控制层看门狗 ADR (spec §9)"
```

---

## Self-Review

**1. Spec 覆盖：**
- §1.1 双安全层共存：Task 4（RobotStateMachine）+ 现有 SafetyMachine 保留 ✓
- §1.2 CartesianController 唯一入口：Task 6/7（RealArmAdapter 不碰 CAN）✓
- §1.3 6DOF 安全链顺序：Task 6 step 管线（workspace→orientation→singularity→damping→DLS→joint-limit→vel/acc→set_joints_safe）✓；`singularity_metrics`/`adaptive_damping` 实际参与（非仅 telemetry）→ `test_step_near_singular_scales_twist` ✓
- §2 新文件：types/workspace/recording（Task 1/3/9）、arm_adapter/real_arm_teleop（Task 7/10）✓
- §3 kinematics：log_so3/singularity_metrics/adaptive_damping（Task 2）✓
- §4 CartesianController step/step_pose（Task 6）✓
- §5 状态机/connect/get_real_state/调用方（Task 4/5）✓
- §6 遥操层 adapter/watchdog/real_arm_teleop（Task 7/8/10）✓
- §7 录制（Task 9，含修订 #5 帧文件引用）✓
- §8 测试计划：新增 test_workspace/test_robot_state/test_recording/test_adapter/test_watchdog + 各 test_*.py 增补 ✓
- §9 文档（Task 12）✓
- §10 不做项（0x35 实时速度/Sphere 工作区/null-space）→ 均未实现 ✓

**2. 修订覆盖：** 六条 + dt 全部映射到任务（见"修订落实映射"表）✓

**3. 依赖链：** types(T1)→kinematics(T2)→workspace(T3)→safety/state-machine(T4)→controller(T5+T6)→adapter(T7)→watchdog(T8)→recording(T9)→real_arm_teleop(T10)→simulation regression(T11)→文档(T12) ✓；TASK-27~34 保持 pending ✓

**4. 占位符扫描：** 每个代码步骤含完整可运行代码；`main()` 的 `hand_provider` 标注为骨架并说明补齐归属（TASK-27~34），非占位符。
