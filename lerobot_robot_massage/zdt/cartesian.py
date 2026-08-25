"""笛卡尔末端控制器 — 速度积分 → FK 反馈 → 数值 IK → 安全运动闭环 (P1).

架构 (2026-08-22 更新: 解析 IK → 雅可比数值 IK, 2026-08-23 6DOF 安全链闭环):
  * 输入: 每帧末端速度 (vx, vy, vz, wx, wy, wz) 或 step_pose(p_des, R_des).
  * 反馈: 每帧读 0x36 真实角 (经 CALIB(k,b) 换 anchor 帧) → FK 得当前末端.
  * IK: 几何雅可比数值解 (kinematics.jacobian + damped_ls), 局部映射从当前构型出发.
  * 安全链: ARMED 门禁 → 测量 dt → 陈旧命令判定 → 工作空间盒限制 →
    归一化奇异度指标 + 自适应阻尼 (λ + scale) → 预测限位缩放 →
    速度/加速度 clamp → check_limits_real 实时守卫 → 下发.
"""
from __future__ import annotations

import logging
import math
import time
from typing import Callable, Optional

import numpy as np

from .config import FIRMWARE_JOINT_LIMITS
from .controller import ZdtController
from .kinematics import (
    adaptive_damping, anchor_to_source, damped_ls, fk_mdh, jacobian, log_so3,
    singularity_metrics, source_to_anchor,
)
from .safety import RobotPhase
from .types import EEPose
from .workspace import BoxWorkspace, CartesianVelocityLimiter
from .zdt_driver import ZdtDriverError

logger = logging.getLogger(__name__)


class CartesianController:
    """末端笛卡尔速度/位姿闭环控制器.

    用法 (P3 键盘脚本 / P4 遥操):
        ctrl = ZdtController(cfg); ctrl.connect()  # 或注入 FakeTransport
        cart = CartesianController(ctrl)
        cart.ready()                     # 先回准备姿态
        while running:
            cart.step(vx, vy, vz, wx, wy, wz)  # 每帧 (~20Hz)
            cart.tick()                  # 看门狗

    坐标: xyz 基座系 mm, z 向上 (与固件 DH 一致). 速度上限可配.
    """

    def __init__(self, ctrl: ZdtController,
                 max_vel_mm_s: Optional[float] = None,
                 loop_hz: float = 20.0,
                 joint_limits: Optional[list[tuple[float, float]]] = None,
                 ik_lambda: float = 10.0,
                 orient_weight: float = 20.0,
                 max_dq_deg: float = 30.0,
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

    # ── 位置/位姿闭环 ─────────────────────────────────────

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
        dq_scaled = self._scale_toward_limits(q_src, dq)

        # 实时限位守卫: 先检查未缩放 dq 是否越出硬限位 (若越界则告警停轴)
        q_src_raw = q_src + np.degrees(dq)
        q_anchor_raw = source_to_anchor(q_src_raw)
        q_anchor_raw = [((x + 180.0) % 360.0) - 180.0 for x in q_anchor_raw]
        alarms_raw = self.ctrl.check_limits_real(q_anchor_raw, use_kb=True,
                                                 real_angles=q_anchor)
        if alarms_raw:
            return {"moved": False, "reason": "limit_alarm", "alarms": alarms_raw,
                    "target_xyz": (p_act + v * dt).tolist(),
                    "sigma_min": metrics["sigma_min"],
                    "condition": metrics["condition_number"],
                    "lambda": lam, "scale": scale}

        dq = dq_scaled

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
        alarms = self.ctrl.check_limits_real(q_anchor_target, use_kb=True,
                                             real_angles=q_anchor)
        if alarms:
            return {"moved": False, "reason": "limit_alarm", "alarms": alarms,
                    "target_xyz": (p_act + v * dt).tolist(),
                    "sigma_min": metrics["sigma_min"],
                    "condition": metrics["condition_number"],
                    "lambda": lam, "scale": scale}
        self.ctrl.set_joints_safe(q_anchor_target, use_kb=True,
                                  real_angles=q_anchor)
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
            if hi - lo >= 360.0:
                continue  # 360° 旋转关节 (J1/J6) 无硬限位边界, 不参与边界缩放
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

    # ── 安全 / 工具 ───────────────────────────────────────

    def ready(self) -> list[float]:
        """回按摩准备姿态 (复用 controller.ready)."""
        return self.ctrl.ready(use_kb=True)

    def home(self) -> list[float]:
        """回上电初始姿态全 0 (复用 controller.home, 安全速度同步)."""
        return self.ctrl.home(use_kb=True)

    def e_stop(self) -> None:
        """广播急停."""
        self.ctrl.e_stop()

    def tick(self) -> None:
        """看门狗巡检 (调用方每帧调)."""
        try:
            self.ctrl.tick()
        except ZdtDriverError:
            logger.exception("cartesian tick watchdog")

    def get_ee_xyz(self) -> list[float]:
        """当前末端位置 (mm, 基座系) — 调试/显示用."""
        p, _, _ = self._read_current_pose()
        return p.tolist()
