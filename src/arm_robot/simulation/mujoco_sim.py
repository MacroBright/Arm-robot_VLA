#!/usr/bin/env python3
r"""MuJoCo 机械臂仿真 TCP 服务。

在 TCP 端口上模拟 STM32 固件的串口文本协议（14 条命令），
使用 MuJoCo 物理引擎驱动 SolidWorks 导出的 6-DOF 机械臂模型。

用法:
  python scripts/mujoco_sim.py                          # 默认端口 5555
  python scripts/mujoco_sim.py --port 5556 --no-viewer   # 无头模式

配合上位机:
  # 手柄遥控:
  python scripts/joystick_control.py --port socket://localhost:5555 --camera 0

  # 数据录制 (另一个终端):
  python scripts/record_sim.py --duration 20

架构:
  主线程:  MuJoCo physics + viewer loop (mj_step @ 50Hz)
  后台线程: TCP server (accept → handle_client)
  共享内存: "mujoco_frame_0" 存放离屏渲染帧 (640x480 BGR)
"""

import argparse
import collections
import math
import os
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

from multiprocessing import shared_memory

import numpy as np

try:
    from .remote_semantics import parse_remote_event
except ImportError:
    from remote_semantics import parse_remote_event

try:
    import mujoco
    import mujoco.viewer
    _HAS_MUJOCO = True
except ImportError:
    mujoco = None  # type: ignore[assignment]
    _HAS_MUJOCO = False


# ── 常量: 与固件参数对齐 ──────────────────────────────────────────────
NUM_JOINTS = 6
JOINT_NAMES_SIM = ["J1_base", "J2_shoulder", "J3_elbow", "J4_wrist_roll",
                   "J5_wrist_flex", "J6_gripper"]
# 初始姿态: 与固件权威 JOINT_INIT_ANGLE_DEG 一致 (zdt/config.py; 旧假值 J2=45° 已废)
INIT_POSE_DEG = [90.0, 90.0, -90.0, 0.0, 90.0, 0.0]
INIT_POSE_RAD = [math.radians(a) for a in INIT_POSE_DEG]

STEP_HZ = 50                      # 物理步进频率
REMOTE_TIMEOUT_S = 0.3            # remote_event 超时清零
REMOTE_GAIN_DEG = 30.0            # remote_event → 关节速度 (无IK模式)
REMOTE_GAIN_RAD = math.radians(REMOTE_GAIN_DEG)
# Jacobian IK 参数
REMOTE_LIN_GAIN = 0.15            # Cartesian 线速度 (m/s per unit)
REMOTE_ANG_GAIN = 2.5             # Cartesian 角速度 (rad/s per unit) (S1: 1.5→2.5 腕部跟手)
IK_DAMPING = 0.05                 # 阻尼伪逆 λ (防奇异)
# Python 端 PID: 每关节 kp/kv
PID_KP = [10.0, 50.0, 30.0, 50.0, 20.0, 3.0]
PID_KV = [3.0, 6.0, 4.0, 20.0, 6.0, 1.0]   # J1/J2/J3 提速(位置跟手), J6 压低(仿真不稳) (2026-08-12)

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
SHM_NAME = "mujoco_frame_0"
SHM_NAME_EE = "mujoco_frame_ee"   # 末端相机共享内存
SHM_HEADER_SIZE = 64

DEFAULT_TCP_PORT = 5555
_SIM_DIR = Path(__file__).resolve().parent
SCENE_PATH = (_SIM_DIR / "scene.xml") if (_SIM_DIR / "scene.xml").exists() else (_SIM_DIR / "mujoco_scene" / "scene.xml")


def load_configured_pose(target: str = "home", cli_val: str | None = None) -> list[float]:
    """读取配置的 Home 姿态或 Ready 姿态 (单位: 角度 deg)。"""
    if cli_val:
        if "," in cli_val:
            vals = [float(x.strip()) for x in cli_val.split(",") if x.strip()]
            if len(vals) >= NUM_JOINTS:
                return vals[:NUM_JOINTS]
        p = Path(cli_val)
        if p.exists():
            if p.suffix.lower() == ".json":
                try:
                    import json
                    with open(p, "r", encoding="utf-8") as f:
                        data = json.load(f)
                    if "angles" in data and len(data["angles"]) >= NUM_JOINTS:
                        return [float(x) for x in data["angles"][:NUM_JOINTS]]
                except Exception as e:
                    log(f"解析 {p} 失败: {e}")
            elif p.suffix.lower() in (".yaml", ".yml"):
                try:
                    import yaml
                    with open(p, "r", encoding="utf-8") as f:
                        data = yaml.safe_load(f)
                    key = f"{target}_pose_deg"
                    if "pose" in data and key in data["pose"]:
                        return [float(x) for x in data["pose"][key][:NUM_JOINTS]]
                except Exception as e:
                    log(f"解析 {p} 失败: {e}")

    # 默认自动探测当前工程目录下的配置文件
    project_root = Path(__file__).resolve().parents[3]
    candidate_json = project_root / "configs" / "home_pose.json"
    candidate_yaml = project_root / "configs" / "teleop_config.yaml"

    if target == "home" and candidate_json.exists():
        try:
            import json
            with open(candidate_json, "r", encoding="utf-8") as f:
                data = json.load(f)
            if "angles" in data and len(data["angles"]) >= NUM_JOINTS:
                return [float(x) for x in data["angles"][:NUM_JOINTS]]
        except Exception:
            pass

    if candidate_yaml.exists():
        try:
            import yaml
            with open(candidate_yaml, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
            key = f"{target}_pose_deg"
            if "pose" in data and key in data["pose"] and len(data["pose"][key]) >= NUM_JOINTS:
                return [float(x) for x in data["pose"][key][:NUM_JOINTS]]
        except Exception:
            pass

    if target == "ready":
        return [0.0, 60.0, 50.0, 0.0, 120.0, 0.0]
    return list(INIT_POSE_DEG)


def log(msg: str) -> None:
    """带时间戳的控制台日志。"""
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


# ═══════════════════════════════════════════════════════════════════════
# MuJoCoArm: MuJoCo 物理引擎封装 + 14 条命令解析
# ═══════════════════════════════════════════════════════════════════════

class MuJoCoArm:
    """6-DOF 机械臂的 MuJoCo 仿真模型 + 固件协议命令解析器。"""

    def __init__(self, scene_path: str, use_ik: bool = True,
                 home_pose_deg: list[float] | None = None,
                 ready_pose_deg: list[float] | None = None) -> None:
        # ── 加载 MuJoCo 模型 ──
        if not _HAS_MUJOCO or mujoco is None:
            raise RuntimeError("未安装 mujoco。请在专属环境 arm_robot 中运行。")
        self.model = mujoco.MjModel.from_xml_path(str(scene_path))
        self.data = mujoco.MjData(self.model)
        self.use_ik = use_ik

        # 初始 Home 姿态与 Ready 姿态 (角度 deg 与 弧度 rad)
        self.home_pose_deg = list(home_pose_deg or load_configured_pose("home"))[:NUM_JOINTS]
        self.home_pose_rad = [math.radians(a) for a in self.home_pose_deg]
        self.ready_pose_deg = list(ready_pose_deg or load_configured_pose("ready"))[:NUM_JOINTS]
        self.ready_pose_rad = [math.radians(a) for a in self.ready_pose_deg]

        # 预取末端 site ID (Jacobian IK 用)
        self._ee_site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
        self._wrist_site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "wrist_site")

        # 初始化到 Home 姿态
        self.data.qpos[:NUM_JOINTS] = self.home_pose_rad
        self.data.ctrl[:NUM_JOINTS] = self.home_pose_rad
        self.data.qvel[:NUM_JOINTS] = 0.0
        mujoco.mj_forward(self.model, self.data)

        # ── 状态 ──
        self._lock = threading.Lock()
        self.remote_enabled = False
        self.torque_on = True
        self.control_mode = "idle"       # idle | cartesian | joint
        self.active_joint = 0            # 逐关节模式下当前关节 (0-indexed)
        self._freeze_ctrl = False     # set_torque 0 时冻结
        self._target_pos = list(self.home_pose_rad)  # set_joints 目标 (rad)
        self._was_remote_active = False
        self._was_any_active = False


        # remote_event 状态 (7 参: p0-p5 + p6→J4)
        self._remote_vals = [0.0] * 7
        self._remote_stamp = 0.0

        # end_event 状态 (末端 6DOF: 线速度 + 角速度 → 全 6×6 DLS IK)
        self._end_vals = [0.0] * 6
        self._end_stamp = 0.0

        # 线程安全状态缓存 (物理线程写, TCP 线程读)
        self._cached_qpos = list(INIT_POSE_RAD)
        self._cached_qvel = [0.0] * NUM_JOINTS
        self._cached_loads = [0.0] * NUM_JOINTS
        self._cached_ee_pos = [0.0, 0.0, 0.0]
        self._cached_ee_quat = np.array([1.0, 0.0, 0.0, 0.0])  # wxyz
        self._cached_wrist_pos = [0.0, 0.0, 0.0]
        self._cached_target_pos = [0.0, 0.0, 0.0]

        # ── 相机 ID (由 camera_server.py 子进程渲染) ──
        self._cam_top_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_CAMERA, "cam_top")
        self._cam_ee_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_CAMERA, "ee_camera")

        # ── 目标小球 (mocap 运动学刚体) ──
        self._target_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "target_ball")
        self._target_mocap_id = self.model.body_mocapid[self._target_body_id]
        self._target_model = "sphere_0.025"  # 当前模型标识 (开放接口)
        self._target_captured = False
        self._target_capture_dist = 0.04  # 触碰阈值 (m)
        self._target_cooldown = 0.0       # 冷却时间, 防重复触发
        # 工作空间边界 (机械臂可达范围, 米)
        self._ws_x = (-0.25, 0.25)
        self._ws_y = (-0.20, 0.25)
        # 趴姿背部穴位: Z 限制在桌面高度 (~5-25cm)
        self._ws_z = (0.05, 0.25)
        self._randomize_target()
        # 缓存初始末端+目标位置 (mj_forward 已在 _move_target 中调用)
        self._cached_ee_pos[:] = self.data.site_xpos[self._ee_site_id]
        _init_q = np.zeros(4)
        mujoco.mju_mat2Quat(_init_q, self.data.site_xmat[self._ee_site_id])
        self._cached_ee_quat = _init_q.copy()   # wxyz
        self._cached_wrist_pos[:] = self.data.site_xpos[self._wrist_site_id]
        self._cached_target_pos[:] = self.data.xpos[self._target_body_id]

        # ── 共享内存帧缓冲 ──
        shm_size = SHM_HEADER_SIZE + FRAME_WIDTH * FRAME_HEIGHT * 3
        try:
            old = shared_memory.SharedMemory(name=SHM_NAME)
            old.close()
            old.unlink()
        except FileNotFoundError:
            pass
        try:
            old = shared_memory.SharedMemory(name=SHM_NAME_EE)
            old.close()
            old.unlink()
        except FileNotFoundError:
            pass
        self._shm = shared_memory.SharedMemory(
            create=True, size=shm_size, name=SHM_NAME)
        self._shm_ee = shared_memory.SharedMemory(
            create=True, size=shm_size, name=SHM_NAME_EE)
        self._frame_counter = 0
        log(f"共享内存已创建: {SHM_NAME}, {SHM_NAME_EE}")

        # ── 日志节流 ──
        self._get_state_count = 0
        self._last_remote_log = 0.0
        self._remote_was_active = False

    # ── 物理步进 ──────────────────────────────────────────────────────

    # ── EMA 滤波状态 (remote_event 平滑, 7 通道含 p6→J4) ─────────────
    _ema_vals: list[float] = [0.0] * 7
    _ema_initialized: bool = False

    def step(self, dt: float) -> None:
        """子步进 + Python PID 力矩控制: 电机 actuator 仅做力矩执行。"""

        # (1) 锁内读取 remote / end 状态
        with self._lock:
            remote_active = (
                self.remote_enabled
                and time.monotonic() - self._remote_stamp < REMOTE_TIMEOUT_S
                and any(abs(v) > 1e-3 for v in self._remote_vals)
            )
            end_active = (
                self.remote_enabled
                and time.monotonic() - self._end_stamp < REMOTE_TIMEOUT_S
                and any(abs(v) > 1e-3 for v in self._end_vals)
            )
            freeze = self._freeze_ctrl
            raw = list(self._remote_vals) if remote_active else [0.0] * 7
            end_raw = list(self._end_vals) if end_active else [0.0] * 6
            self._ema_initialized = self._ema_initialized and remote_active
            # 遥操活跃标志 (无论手势 end_event 还是手柄 remote_event)
            any_active = remote_active or end_active
            # 遥操运动中持续更新或松手瞬间锁定当前实际角度 (悬停不下坠不弹回)
            if any_active or (self._was_any_active and not any_active):
                self._target_pos[:] = self.data.qpos[:NUM_JOINTS].copy()
            self._was_any_active = any_active
            self._was_remote_active = remote_active


            # 保存 set_joints 设置的目标角度 (rad)
            ctrl_targets = self._target_pos.copy()

        # (2) 锁外: EMA + 目标速度
        phys_dt = self.model.opt.timestep
        n_substeps = max(1, int(dt / phys_dt))
        if remote_active:
            # EMA 滤波
            alpha = 0.4
            if not self._ema_initialized:
                self._ema_vals = list(raw)
                self._ema_initialized = True
            else:
                for i in range(7):
                    self._ema_vals[i] = (alpha * raw[i] +
                                         (1 - alpha) * self._ema_vals[i])
            # 摇杆残余死区 (视觉遥操在 PC 端已做死区)
            for i, val in enumerate(self._ema_vals):
                if abs(val) < 0.03:
                    self._ema_vals[i] = 0.0
            # 固件语义 (robot_cmd.c): vx/vy/vz 基座系线速度, p6→J4, p3→J5, p2→J6
            v_lin, j4_coef, j5_coef, j6_coef = parse_remote_event(self._ema_vals)
            v_lin = v_lin * REMOTE_LIN_GAIN           # 系数 → m/s
            j4_vel = j4_coef * REMOTE_GAIN_RAD        # J4 关节速度 rad/s
            j5_vel = j5_coef * REMOTE_GAIN_RAD        # J5 关节速度 rad/s
            j6_vel = j6_coef * REMOTE_GAIN_RAD        # J6 关节速度 rad/s
            v_ang = np.zeros(3)                       # 固件固定末端方向
        else:
            self._ema_initialized = False

        # (3) 子步进 + PID (Jacobian 每子步重算以保证精度)
        lam = IK_DAMPING
        jac_pos = np.zeros((3, self.model.nv))
        jac_rot = np.zeros((3, self.model.nv))
        for _ in range(n_substeps):
            qpos = self.data.qpos[:NUM_JOINTS]
            qvel = self.data.qvel[:NUM_JOINTS]
            gravity = self.data.qfrc_bias[:NUM_JOINTS]

            if freeze:
                self.data.ctrl[:NUM_JOINTS] = 0.0
            elif end_active:
                # 解耦 IK: 位置用腕心(J1-J3), 姿态用末端旋转(J4/J5), J6 剥离
                raw_v = np.array(end_raw[:3], dtype=float)
                # 自动单位适配: 若输入模长 > 2.0 (如 mm/s 速度)，转为 m/s；否则按归一化系数乘增益
                if np.linalg.norm(raw_v) > 2.0:
                    v_lin_e = raw_v / 1000.0
                else:
                    v_lin_e = raw_v * REMOTE_LIN_GAIN
                w_ang_e = np.array(end_raw[3:6], dtype=float) * REMOTE_ANG_GAIN    # rad/s

                dq = np.zeros(NUM_JOINTS)
                if np.any(np.abs(v_lin_e) > 1e-6):
                    # 位置: 腕心 Jacobian (腕部旋转不移动腕心 → 位置/姿态解耦)
                    mujoco.mj_jacSite(self.model, self.data,
                                      jac_pos, jac_rot, self._wrist_site_id)
                    Jp = jac_pos[:, :3]          # J1-J3 驱动腕心位置
                    JJT = Jp @ Jp.T + lam * lam * np.eye(3)
                    dq[:3] = Jp.T @ np.linalg.solve(JJT, v_lin_e)
                if np.any(np.abs(w_ang_e) > 1e-6):
                    # 姿态: 末端旋转 Jacobian (J4/J5 驱动末端姿态)
                    mujoco.mj_jacSite(self.model, self.data,
                                      jac_pos, jac_rot, self._ee_site_id)
                    Jr = jac_rot[:, 3:5]         # J4/J5
                    JJT = Jr @ Jr.T + lam * lam * np.eye(3)
                    dq[3:5] = Jr.T @ np.linalg.solve(JJT, w_ang_e)
                dq = np.clip(dq, -3.0, 3.0)      # 限幅保护，避免奇异区速度超调
                for i in range(NUM_JOINTS):
                    self.data.ctrl[i] = (-PID_KV[i] * (qvel[i] - dq[i])
                                         + gravity[i])

            elif remote_active and self.use_ik:
                # 位置 IK: J1-J3 驱动末端位置, J4/J5/J6 姿态通道直接关节速度
                mujoco.mj_jacSite(self.model, self.data,
                                  jac_pos, jac_rot, self._ee_site_id)
                if np.any(np.abs(v_lin) > 1e-6):
                    Jp = jac_pos[:, :3]        # 仅 J1-J3 驱动位置
                    JJT = Jp @ Jp.T + lam * lam * np.eye(3)
                    dq = np.zeros(NUM_JOINTS)
                    dq[:3] = Jp.T @ np.linalg.solve(JJT, v_lin)
                else:
                    dq = np.zeros(NUM_JOINTS)
                dq[3] = j4_vel   # J4 = 手滚转 (p6)
                dq[4] = j5_vel   # J5 = 手俯仰 (p3)
                dq[5] = j6_vel   # J6 = p2 (当前置 0)
                for i in range(NUM_JOINTS):
                    self.data.ctrl[i] = (-PID_KV[i] * (qvel[i] - dq[i])
                                         + gravity[i])
            elif remote_active:
                # 非 IK 路径: 同参数语义 (vx/vy/vz→J1-J3 关节速度近似, J4/J5/J6 直驱)
                target_vel = [v_lin[0] * REMOTE_GAIN_RAD,
                              v_lin[1] * REMOTE_GAIN_RAD,
                              v_lin[2] * REMOTE_GAIN_RAD,
                              j4_vel, j5_vel, j6_vel]
                for i in range(NUM_JOINTS):
                    self.data.ctrl[i] = (-PID_KV[i] * (qvel[i] - target_vel[i])
                                         + gravity[i])
            else:
                for i in range(NUM_JOINTS):
                    pos_err = ctrl_targets[i] - qpos[i]
                    self.data.ctrl[i] = (PID_KP[i] * pos_err
                                         - PID_KV[i] * qvel[i]
                                         + gravity[i])

            mujoco.mj_step(self.model, self.data)

        # (4) 更新线程安全状态缓存 (TCP 线程从此读取, 避免 data race)
        with self._lock:
            if any_active:
                self._target_pos[:] = self.data.qpos[:NUM_JOINTS].copy()
            self._cached_qpos = self.data.qpos[:NUM_JOINTS].copy()

            self._cached_qvel = self.data.qvel[:NUM_JOINTS].copy()
            self._cached_loads = self.data.qfrc_actuator[:NUM_JOINTS].copy()
            self._cached_ee_pos = self.data.site_xpos[self._ee_site_id].copy()
            _q = np.zeros(4)
            mujoco.mju_mat2Quat(_q, self.data.site_xmat[self._ee_site_id])
            self._cached_ee_quat = _q.copy()   # wxyz
            self._cached_wrist_pos = self.data.site_xpos[self._wrist_site_id].copy()
            self._cached_target_pos = self.data.xpos[self._target_body_id].copy()

        # (4b) 目标小球触碰检测
        if self._target_cooldown > 0:
            self._target_cooldown -= dt
        if self._check_target_capture():
            self._target_captured = True
            self._target_cooldown = 1.0  # 1 秒冷却, 防重复触发
            self._randomize_target()
            log(f"● 目标球已触碰! 新位置: {self.data.xpos[self._target_body_id].round(3)}")
        else:
            self._target_captured = False

        # (5) 离屏渲染: viewer 循环中由 _render_camera() 完成

    # ── 命令解析 ──────────────────────────────────────────────────────

    def handle_line(self, line: str) -> list[str]:
        """解析一条文本命令, 返回响应行列表。"""
        parts = line.strip().split()
        if not parts:
            return []
        cmd = parts[0]

        if cmd == "get_state":
            return [self._state_line()]
        if cmd == "set_joints":
            return self._cmd_set_joints(parts[1:])
        if cmd == "set_torque":
            return self._cmd_set_torque(parts[1:])
        if cmd == "e_stop":
            return self._cmd_e_stop()
        if cmd == "zero":
            return self._cmd_zero()
        if cmd == "remote_enable":
            return self._cmd_remote_enable()
        if cmd == "remote_disable":
            return self._cmd_remote_disable()
        if cmd == "remote_event":
            return self._cmd_remote_event(parts[1:])
        if cmd == "end_event":
            return self._cmd_end_event(parts[1:])
        if cmd == "rel_rotate":
            return self._cmd_rel_rotate(parts[1:])
        if cmd == "soft_reset":
            return self._cmd_soft_reset()
        if cmd == "home":
            return self._cmd_home()
        if cmd == "ready":
            return self._cmd_ready()
        if cmd == "hard_reset":
            return self._cmd_hard_reset()

        if cmd == "auto":
            return self._cmd_auto(parts[1:])
        if cmd == "get_hub":
            return self._cmd_get_hub()
        if cmd == "get_mode":
            return self._cmd_get_mode()
        if cmd == "get_ee":
            return self._cmd_get_ee()
        if cmd == "get_ee_pose":
            return self._cmd_get_ee_pose()
        if cmd == "get_wrist":
            return self._cmd_get_wrist()
        if cmd == "target_pos":
            return self._cmd_target_pos()
        if cmd == "target_reset":
            return self._cmd_target_reset()
        if cmd == "target_set":
            return self._cmd_target_set(parts[1:])
        if cmd in ("stream_start", "stream_stop"):
            log(f"{cmd} (仿真中无操作)")
            return []

        log(f"?? 未知命令: {line!r}")
        return []

    # ── 各命令实现 ────────────────────────────────────────────────────

    def _cmd_set_joints(self, args: list[str]) -> list[str]:
        try:
            vals = [float(v) for v in args[:NUM_JOINTS]]
        except ValueError:
            log(f"!! set_joints 参数无法解析: {args}")
            return ["OK"]
        rads = [math.radians(v) for v in vals]
        with self._lock:
            self._target_pos[:len(rads)] = rads
            self._freeze_ctrl = False
        log(f"set_joints → {['%.1f' % v for v in vals]}")
        return ["OK"]

    def _cmd_set_torque(self, args: list[str]) -> list[str]:
        enable = len(args) > 0 and args[0] == "1"
        with self._lock:
            self.torque_on = enable
            if not enable:
                self._freeze_ctrl = True
                self.data.ctrl[:NUM_JOINTS] = self.data.qpos[:NUM_JOINTS].copy()
            else:
                self._freeze_ctrl = False
        log(f"set_torque → {'ON' if enable else 'FREE'}")
        return ["OK"] if enable else ["OK:FREE"]

    def _cmd_e_stop(self) -> list[str]:
        with self._lock:
            self._target_pos[:] = self.data.qpos[:NUM_JOINTS].copy()
            self.data.qvel[:NUM_JOINTS] = 0.0
            self.remote_enabled = False
            self._remote_vals = [0.0] * 7
            self._end_vals = [0.0] * 6
            self._end_stamp = 0.0
        log("!! e_stop → 全部关节停止, 退出远程模式")
        return ["ESTOP"]

    def _cmd_home(self) -> list[str]:
        with self._lock:
            self._target_pos[:] = self.home_pose_rad
            self.data.qpos[:NUM_JOINTS] = self.home_pose_rad
            self.data.ctrl[:NUM_JOINTS] = self.home_pose_rad
            self.data.qvel[:NUM_JOINTS] = 0.0
            mujoco.mj_forward(self.model, self.data)
            self._freeze_ctrl = False
            self._remote_vals = [0.0] * 7
            self._end_vals = [0.0] * 6
        log(f"home → 回初始姿态: {np.round(self.home_pose_deg, 1)}")
        return ["OK"]

    def _cmd_ready(self) -> list[str]:
        with self._lock:
            self._target_pos[:] = self.ready_pose_rad
            self.data.qpos[:NUM_JOINTS] = self.ready_pose_rad
            self.data.ctrl[:NUM_JOINTS] = self.ready_pose_rad
            self.data.qvel[:NUM_JOINTS] = 0.0
            mujoco.mj_forward(self.model, self.data)
            self._freeze_ctrl = False
            self._remote_vals = [0.0] * 7
            self._end_vals = [0.0] * 6
        log(f"ready → 回准备姿态: {np.round(self.ready_pose_deg, 1)}")
        return ["OK"]

    def _cmd_remote_enable(self) -> list[str]:
        with self._lock:
            self.remote_enabled = True
            self._was_remote_active = False
            self._was_any_active = False
            self._target_pos[:] = self.data.qpos[:NUM_JOINTS].copy()
            self.control_mode = "cartesian"  # 默认笛卡尔模式
        log("remote_enable → 远程模式开启")
        return []

    def _cmd_remote_disable(self) -> list[str]:
        with self._lock:
            self.remote_enabled = False
            self._remote_vals = [0.0] * 7
            self._end_vals = [0.0] * 6
            self._end_stamp = 0.0
            self._was_remote_active = False
            self._was_any_active = False
            self._target_pos[:] = self.data.qpos[:NUM_JOINTS].copy()
        log("remote_disable → 远程模式关闭")
        return []


    def _cmd_zero(self) -> list[str]:
        with self._lock:
            self.data.qpos[:NUM_JOINTS] = 0.0
            self._target_pos[:] = [0.0] * NUM_JOINTS
            mujoco.mj_forward(self.model, self.data)
        log("zero → 当前位置设为零位")
        return []

    def _cmd_remote_event(self, args: list[str]) -> list[str]:
        try:
            vals = [float(v) for v in args[:7]]
        except ValueError:
            log(f"!! remote_event 参数无法解析: {args}")
            return []
        while len(vals) < 7:
            vals.append(0.0)
        with self._lock:
            if self.remote_enabled:
                self._remote_vals = vals
                self._remote_stamp = time.monotonic()
                self.control_mode = "cartesian"
        # 节流日志
        now = time.monotonic()
        active = any(abs(v) > 0.01 for v in vals)
        if not self.remote_enabled and active:
            if now - self._last_remote_log > 1.0:
                self._last_remote_log = now
                log("remote_event 被忽略 (未先发送 remote_enable)")
        elif active and now - self._last_remote_log > 0.25:
            self._last_remote_log = now
            self._remote_was_active = True
        elif not active and self._remote_was_active:
            self._remote_was_active = False
            log("remote_event 归零 (摇杆回中)")
        return []

    def _cmd_end_event(self, args: list[str]) -> list[str]:
        """末端 6DOF 速度命令 (独立于 remote_event 语义)。

        6 通道: [vx, vy, vz, wx, wy, wz] 基座系线速度(m/s系数) + 角速度(rad/s系数).
        use_ik 时走全 6×6 DLS Jacobian (含姿态), 不驱动 remote_event 的 J4/J5/J6 直驱.
        """
        try:
            vals = [float(v) for v in args[:6]]
        except ValueError:
            log(f"!! end_event 参数无法解析: {args}")
            return []
        while len(vals) < 6:
            vals.append(0.0)
        with self._lock:
            if self.remote_enabled:
                self._end_vals = vals
                self._end_stamp = time.monotonic()
                self.control_mode = "cartesian"
        return []

    def _cmd_rel_rotate(self, args: list[str]) -> list[str]:
        try:
            joint = int(args[0]) - 1   # 1-based → 0-based
            delta_deg = float(args[1])
        except (IndexError, ValueError):
            log(f"!! rel_rotate 参数无法解析: {args}")
            return []
        if 0 <= joint < NUM_JOINTS:
            delta_rad = math.radians(delta_deg)
            with self._lock:
                self._target_pos[joint] += delta_rad
                self.control_mode = "joint"
                self.active_joint = joint
                jnt_range = self.model.jnt_range[joint]
                if jnt_range is not None:
                    lo, hi = jnt_range
                    if lo < hi:
                        self._target_pos[joint] = max(lo, min(hi, self._target_pos[joint]))
                self._freeze_ctrl = False
            log(f"rel_rotate → J{joint + 1} {delta_deg:+.1f}°")
        else:
            log(f"!! rel_rotate 关节编号越界: {args[0]}")
        return []

    def _cmd_soft_reset(self) -> list[str]:
        return self._cmd_home()


    def _cmd_hard_reset(self) -> list[str]:
        with self._lock:
            self.data.qpos[:NUM_JOINTS] = 0.0
            self._target_pos[:] = [0.0] * NUM_JOINTS
            self.data.qvel[:NUM_JOINTS] = 0.0
            mujoco.mj_forward(self.model, self.data)
        log("hard_reset → 限位归零 (瞬间完成)")
        return []

    def _cmd_auto(self, args: list[str]) -> list[str]:
        try:
            x, y, z = float(args[0]), float(args[1]), float(args[2])
        except (IndexError, ValueError):
            log(f"!! auto 参数无法解析: {args}")
            return []
        log(f"auto → ({x}, {y}, {z}) (IK 未在仿真中实现)")
        return []

    def _cmd_get_hub(self) -> list[str]:
        """单命令返回 Hub 所需全部数据 (避免多命令响应互相覆盖)。"""
        with self._lock:
            ang = [math.degrees(a) for a in self._cached_qpos]
            vel = [math.degrees(v) for v in self._cached_qvel]
            ld = [abs(l) for l in self._cached_loads]
            ep = list(self._cached_ee_pos)
            tp = list(self._cached_target_pos)
            mode = self.control_mode
            joint = self.active_joint
        jn = JOINT_NAMES_SIM[joint] if 0 <= joint < len(JOINT_NAMES_SIM) else "?"
        # 格式: HUB:ang6,vel6,load6,ee3,target3,mode,joint_idx,joint_name
        vals = (ang + vel + ld + ep + tp +
                [mode, str(joint), jn])
        return ["HUB:" + ",".join(
            f"{v:.4f}" if isinstance(v, float) else str(v)
            for v in vals)]

    def _cmd_get_mode(self) -> list[str]:
        """返回当前控制模式。"""
        with self._lock:
            mode = self.control_mode
            joint = self.active_joint
        jn = JOINT_NAMES_SIM[joint] if 0 <= joint < len(JOINT_NAMES_SIM) else "?"
        return [f"MODE:{mode},{joint},{jn}"]

    def _cmd_get_ee(self) -> list[str]:
        """返回末端世界坐标 + 目标球位置 (训练数据采集用)。"""
        with self._lock:
            ep = list(self._cached_ee_pos)
            tp = list(self._cached_target_pos)
        return [f"EE:{ep[0]:.4f},{ep[1]:.4f},{ep[2]:.4f},"
                f"{tp[0]:.4f},{tp[1]:.4f},{tp[2]:.4f}"]

    def _cmd_get_wrist(self) -> list[str]:
        """返回腕心世界坐标 (m). 位置环反馈用, J4/J5/J6 旋转不移动腕心."""
        with self._lock:
            wp = list(self._cached_wrist_pos)
        return [f"WRIST:{wp[0]:.4f},{wp[1]:.4f},{wp[2]:.4f}"]

    def _cmd_get_ee_pose(self) -> list[str]:
        """返回末端世界位姿 (位置 + 四元数 wxyz). 末端 6DOF 遥操反馈用."""
        with self._lock:
            ep = list(self._cached_ee_pos)
            q = list(self._cached_ee_quat)
        return [f"EEPOSE:{ep[0]:.4f},{ep[1]:.4f},{ep[2]:.4f},"
                f"{q[0]:.4f},{q[1]:.4f},{q[2]:.4f},{q[3]:.4f}"]   # xyz + wxyz

    def _cmd_target_pos(self) -> list[str]:
        """返回目标小球当前位置。"""
        state = self.get_target_state()
        p = state["pos"]
        return [f"TARGET:{p[0]:.4f},{p[1]:.4f},{p[2]:.4f},"
                f"dist={state['dist']:.4f},model={state['model']}"]

    def _cmd_target_reset(self) -> list[str]:
        """强制移动小球到新随机位置。"""
        self._randomize_target()
        p = self.data.xpos[self._target_body_id]
        log(f"target_reset → ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})")
        return [f"TARGET_RESET:{p[0]:.4f},{p[1]:.4f},{p[2]:.4f}"]

    def _cmd_target_set(self, args: list[str]) -> list[str]:
        """设置小球到指定位置: target_set x y z"""
        try:
            x, y, z = float(args[0]), float(args[1]), float(args[2])
        except (IndexError, ValueError):
            return ["ERROR:target_set x y z"]
        self._move_target(x, y, z)
        self._target_cooldown = 0.5
        log(f"target_set → ({x:.3f}, {y:.3f}, {z:.3f})")
        return ["OK"]

    # ── 目标小球 ────────────────────────────────────────────────────

    def _randomize_target(self) -> None:
        """在工作空间内随机放置目标小球 (距底座 >8cm)。"""
        import random
        min_dist = 0.15  # 距 Z 轴最小距离 (m)
        for _ in range(50):  # 拒绝采样, 最多尝试 50 次
            x = random.uniform(*self._ws_x)
            y = random.uniform(*self._ws_y)
            if (x * x + y * y) >= min_dist * min_dist:
                break
        z = random.uniform(*self._ws_z)
        self._move_target(x, y, z)

    def _move_target(self, x: float, y: float, z: float) -> None:
        """移动目标小球到指定位置 (mocap, 世界坐标)。"""
        mid = self._target_mocap_id
        self.data.mocap_pos[mid] = [x, y, z]
        self.data.mocap_quat[mid] = [1.0, 0.0, 0.0, 0.0]
        mujoco.mj_forward(self.model, self.data)  # 更新 xpos

    def _check_target_capture(self) -> bool:
        """检测末端是否触碰到目标小球 (> 触碰阈值 + 冷却时间)。"""
        if self._target_cooldown > 0:
            return False
        ee_pos = self.data.site_xpos[self._ee_site_id]
        ball_pos = self.data.xpos[self._target_body_id]
        dist = float(np.linalg.norm(ee_pos - ball_pos))
        return dist < self._target_capture_dist

    def get_target_state(self) -> dict:
        """返回目标小球状态 (开放接口)。"""
        ball_pos = self.data.xpos[self._target_body_id].copy()
        ee_pos = self.data.site_xpos[self._ee_site_id].copy()
        dist = float(np.linalg.norm(ee_pos - ball_pos))
        return {
            "pos": ball_pos.tolist(),
            "ee_pos": ee_pos.tolist(),
            "dist": round(dist, 4),
            "captured": self._target_captured,
            "model": self._target_model,
        }

    # ── 状态输出 ──────────────────────────────────────────────────────

    def _state_line(self) -> str:
        """构造 STATE: 响应, 从线程安全缓存读取 (避免与 mj_step 竞争)。"""
        with self._lock:
            self._get_state_count += 1
            angles_rad = list(self._cached_qpos)
            vels_rad = list(self._cached_qvel)
            loads = list(self._cached_loads)

        angles_deg = [math.degrees(a) for a in angles_rad]
        vels_degs = [math.degrees(v) for v in vels_rad]
        loads_abs = [abs(l) for l in loads]

        vals = angles_deg + vels_degs + loads_abs
        return "STATE:" + ",".join(f"{v:.2f}" for v in vals)

    def status_summary(self) -> str:
        with self._lock:
            n = self._get_state_count
            self._get_state_count = 0
            ang = " ".join(
                f"{math.degrees(a):7.1f}" for a in self._cached_qpos
            )
            mode = "REMOTE" if self.remote_enabled else "IDLE"
            torque = "ON" if self.torque_on else "FREE"
        return f"J1-J6 [{ang}]  {mode}  扭矩:{torque}  get_state: {n}次/2s"

    def cleanup(self) -> None:
        for shm in (self._shm, self._shm_ee):
            try:
                shm.close()
                shm.unlink()
            except Exception:
                pass
        log("共享内存已释放。")


# ═══════════════════════════════════════════════════════════════════════
# TCP 服务
# ═══════════════════════════════════════════════════════════════════════

def handle_client(conn: socket.socket, arm: MuJoCoArm) -> None:
    """处理单个 TCP 客户端: 按行读取命令, 回写响应。"""
    buf = b""
    conn.settimeout(1.0)
    while True:
        try:
            chunk = conn.recv(4096)
        except socket.timeout:
            continue
        except (ConnectionResetError, ConnectionAbortedError, OSError):
            return
        if not chunk:
            return
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            try:
                text = line.decode("ascii", errors="replace")
            except Exception:
                continue
            responses = arm.handle_line(text)
            for resp in responses:
                try:
                    conn.sendall(resp.encode("ascii") + b"\n")
                except OSError:
                    return


def _client_thread(conn: socket.socket, addr, arm: MuJoCoArm,
                    connected: threading.Event) -> None:
    """每客户端独立线程, 支持手柄+录制同时连接。"""
    handle_client(conn, arm)
    try:
        conn.close()
    except OSError:
        pass
    log(f"客户端断开: {addr[0]}:{addr[1]}")


def tcp_server(arm: MuJoCoArm, port: int,
               connected: threading.Event,
               shutdown: threading.Event) -> None:
    """后台线程: TCP accept + 逐客户端处理。"""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        srv.bind(("127.0.0.1", port))
    except OSError:
        log(f"错误: 端口 {port} 被占用, 请先执行: kill $(lsof -ti:{port})")
        return
    srv.listen(1)
    srv.settimeout(1.0)
    log(f"TCP 服务已启动: 127.0.0.1:{port}")
    log(f"上位机连接方式: --port socket://localhost:{port}")

    while not shutdown.is_set():
        try:
            conn, addr = srv.accept()
        except socket.timeout:
            continue
        log(f"客户端已连接: {addr[0]}:{addr[1]}")
        connected.set()
        # 每连接一个线程 → 手柄和录制可同时连接
        threading.Thread(target=_client_thread,
                         args=(conn, addr, arm, connected),
                         daemon=True).start()

    try:
        srv.close()
    except OSError:
        pass


def status_log(arm: MuJoCoArm, connected: threading.Event,
               shutdown: threading.Event) -> None:
    """后台线程: 每 1s 输出一次状态摘要。"""
    while not shutdown.is_set():
        time.sleep(1.0)
        if connected.is_set():
            log(arm.status_summary())


# ═══════════════════════════════════════════════════════════════════════
# 主入口
# ═══════════════════════════════════════════════════════════════════════

def main() -> None:
    if not _HAS_MUJOCO or mujoco is None:
        print("错误: 当前 Python 环境未安装 mujoco。", flush=True)
        print("💡 提示: 请在专属环境 arm_robot 中运行:", flush=True)
        print("    conda activate arm_robot && arm-robot sim --viewer", flush=True)
        sys.exit(1)


    parser = argparse.ArgumentParser(description="MuJoCo 机械臂仿真 TCP 服务")

    parser.add_argument("-p", "--port", type=int, default=DEFAULT_TCP_PORT,
                        help=f"TCP 监听端口 (默认 {DEFAULT_TCP_PORT})")
    parser.add_argument("--viewer", action="store_true", default=True,
                        help="启动 3D 可视化窗口 (默认开启)")
    parser.add_argument("--no-viewer", dest="viewer", action="store_false",
                        help="以无头模式 (Headless) 运行，不显示 3D 窗口")
    parser.add_argument("--scene", type=str, default=str(SCENE_PATH),
                        help=f"MJCF 场景文件路径 (默认 {SCENE_PATH})")
    parser.add_argument("--ik", action="store_true", default=True,
                        help="启用 Jacobian IK 笛卡尔控制 (默认开启)")
    parser.add_argument("--no-ik", dest="ik", action="store_false",
                        help="禁用 Jacobian IK 笛卡尔控制")
    parser.add_argument("--home-pose", type=str, default=None,
                        help="初始/Home 关节姿态 (角度 deg, 逗号分隔如 90,0,90,-180,0,0 或 JSON 文件路径)")
    parser.add_argument("--ready-pose", type=str, default=None,
                        help="推拿准备姿态 (角度 deg, 逗号分隔或 YAML 文件路径)")

    parser.add_argument("--trail", type=int, default=0, metavar="N",
                        help="显示末端运动轨迹, N=保留点数 (如 --trail 500)")
    parser.add_argument("--no-camera", action="store_true",
                        help="禁用相机渲染子进程 (camera_server.py)")
    parser.add_argument("--camera-gl", type=str, default="glfw",
                        choices=["glfw", "osmesa", "egl"],
                        help="相机渲染 GL 后端 (headless 服务器用 osmesa)")
    args = parser.parse_args()

    # ── 检查场景文件 ──
    scene_path = Path(args.scene)
    if not scene_path.exists():
        log(f"错误: 场景文件不存在: {scene_path}")
        return

    # ── 加载模型 ──
    log(f"加载场景: {scene_path}")
    home_deg = load_configured_pose("home", args.home_pose)
    ready_deg = load_configured_pose("ready", args.ready_pose)
    log(f"机械臂 Home 初始姿态 (J1-J6 deg): {np.round(home_deg, 1)}")
    log(f"机械臂 Ready 准备姿态 (J1-J6 deg): {np.round(ready_deg, 1)}")
    arm = MuJoCoArm(str(scene_path), use_ik=args.ik,
                    home_pose_deg=home_deg, ready_pose_deg=ready_deg)

    if args.ik:
        log("Jacobian IK 笛卡尔控制已启用")
        log("remote_event 语义: vx/vy/vz 基座系线速度 (与固件 robot_cmd.c 一致)")

    # ── 启动后台线程 ──
    connected = threading.Event()
    shutdown = threading.Event()

    tcp = threading.Thread(target=tcp_server,
                           args=(arm, args.port, connected, shutdown),
                           daemon=True)
    tcp.start()

    st = threading.Thread(target=status_log,
                          args=(arm, connected, shutdown),
                          daemon=True)
    st.start()

    camera_proc = None
    if not getattr(args, 'no_camera', False):
        # 启动相机渲染子进程 (独立进程隔离 GL context, 避免 viewer segfault)
        # 用当前 Python 解释器 (兼容 venv / conda / system)
        python_exe = sys.executable
        camera_proc = subprocess.Popen(
            [python_exe, str(Path(__file__).resolve().parent / "camera_server.py"),
             "--port", str(args.port), "--scene", str(scene_path)],
            env={**os.environ, "MUJOCO_GL": args.camera_gl})
        log(f"相机渲染子进程已启动 (PID {camera_proc.pid})")

    # ── 主循环: 物理步进 + viewer ──
    dt = 1.0 / STEP_HZ
    log(f"仿真已启动 (step={STEP_HZ}Hz, dt={dt:.3f}s)")

    if not args.viewer:
        log("无头模式运行, 按 Ctrl+C 退出")
        try:
            while True:
                arm.step(dt)
                time.sleep(dt)
        except KeyboardInterrupt:
            log("Ctrl+C 退出")
    else:
        log("启动 MuJoCo viewer...")
        try:
            with mujoco.viewer.launch_passive(arm.model, arm.data) as viewer:
                log("Viewer 已启动, 关闭窗口或按 Ctrl+C 退出")
                # 末端轨迹缓存
                trail = collections.deque(maxlen=args.trail) if args.trail else None

                while viewer.is_running():
                    t_start = time.monotonic()
                    arm.step(dt)

                    # 帧由 camera_server.py 子进程渲染, 此处仅做 viewer + 轨迹

                    scn = viewer.user_scn
                    scn.ngeom = 0

                    # 末端轨迹
                    if trail is not None:
                        ee_pos = arm.data.site_xpos[arm._ee_site_id].copy()
                        trail.append(ee_pos)
                        alpha_step = 0.8 / max(len(trail), 1)
                        for k, pos in enumerate(trail):
                            if scn.ngeom >= scn.maxgeom: break
                            alpha = 0.2 + k * alpha_step
                            mujoco.mjv_initGeom(
                                scn.geoms[scn.ngeom],
                                mujoco.mjtGeom.mjGEOM_SPHERE,
                                np.array([0.0015, 0.0015, 0.0015]),
                                pos, np.eye(3).flatten(),
                                np.array([0.0, 1.0, 0.3, alpha]))
                            scn.ngeom += 1

                    viewer.sync()
                    elapsed = time.monotonic() - t_start
                    if elapsed < dt:
                        time.sleep(dt - elapsed)
        except KeyboardInterrupt:
            log("Ctrl+C 退出")

    # ── 清理: 先停线程, 再释放共享内存 (避免 segfault) ──
    shutdown.set()
    if camera_proc is not None:
        camera_proc.terminate()
        try:
            camera_proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            camera_proc.kill()
        log(f"相机渲染子进程已终止 (PID {camera_proc.pid})")
    arm.cleanup()
    log("仿真已停止。")


# 延迟导入由 main() 按需执行


if __name__ == "__main__":
    main()
