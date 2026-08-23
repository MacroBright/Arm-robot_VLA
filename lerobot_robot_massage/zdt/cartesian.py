"""笛卡尔末端控制器 — 速度积分 → FK 反馈 → 数值 IK → 安全运动闭环 (P1).

架构 (2026-08-22 更新: 解析 IK → 雅可比数值 IK):
  * 输入: 每帧末端速度 vx/vy/vz (mm/s), 20Hz 循环.
  * 反馈: 每帧读 0x36 真实角 (经 CALIB(k,b) 换 anchor 帧) → FK 得当前末端.
    区别于源项目命令积分 — 失步/外力搬动后自愈.
  * IK: 几何雅可比数值解 (kinematics.jacobian + damped_ls), 局部映射从当前
    构型出发, 任意姿态可用 (ready 不翻腕), 姿态保持 ω=0 (锁当前朝向).
    注: 旧解析 ik_analytic (姿态锁 T_0_6_RESET) 在 ready 姿态结构性无解,
    已废弃于生产闭环 (仅测试/参考用).
  * 下发: source 帧解 → anchor 帧 → set_joints_safe (0x36 真实位置限位).
  * 安全: check_limits_real 每帧守卫 (目标越界 → limit_alarm 停车); tick 看门狗.
"""
from __future__ import annotations

import logging
import math
from typing import Optional

import numpy as np

from .config import FIRMWARE_JOINT_LIMITS
from .controller import ZdtController
from .kinematics import (
    anchor_to_source, damped_ls, fk_mdh, jacobian, source_to_anchor,
)
from .zdt_driver import ZdtDriverError

logger = logging.getLogger(__name__)


class CartesianController:
    """末端笛卡尔速度闭环控制器.

    用法 (P3 键盘脚本 / P4 遥操):
        ctrl = ZdtController(cfg); ctrl.connect()  # 或注入 FakeTransport
        cart = CartesianController(ctrl)
        cart.ready()                     # 先回准备姿态
        while running:
            cart.step(vx, vy, vz)        # 每帧 (~20Hz)
            cart.tick()                  # 看门狗

    坐标: xyz 基座系 mm, z 向上 (与固件 DH 一致). 速度上限可配.
    """

    def __init__(self, ctrl: ZdtController,
                 max_vel_mm_s: float = 20.0,
                 loop_hz: float = 20.0,
                 joint_limits: Optional[list[tuple[float, float]]] = None,
                 ik_lambda: float = 10.0,
                 orient_weight: float = 20.0,
                 max_dq_deg: float = 2.0):
        self.ctrl = ctrl
        self.max_vel_mm_s = max_vel_mm_s
        self.dt_s = 1.0 / loop_hz
        # 本项目 anchor 帧限位 (最终安全); set_joints_safe 下发前 clamp.
        self.joint_limits = (joint_limits
                             if joint_limits is not None
                             else list(FIRMWARE_JOINT_LIMITS))
        # 数值 IK (加权 DLS) 参数: 阻尼 / 姿态保持权重 / 每帧 Δq 上限
        self.ik_lambda = ik_lambda
        self.orient_weight = orient_weight
        self.max_dq_deg = max_dq_deg

    # ── 位置闭环 ──────────────────────────────────────────

    def _read_current_ee(self) -> tuple[np.ndarray, list[float]]:
        """读 0x36 真实角 (anchor 帧) → source 帧 → FK.

        Returns:
            (末端 xyz mm, anchor 帧真实关节角). 单次读取供 FK 与 IK 复用.
        """
        q_anchor = self.ctrl.read_real_angles(use_kb=True)
        q_src = anchor_to_source(q_anchor)
        T = fk_mdh(q_src)
        return T[:3, 3].copy(), q_anchor

    def step(self, vx: float, vy: float, vz: float) -> dict:
        """单帧笛卡尔运动: 几何雅可比加权 DLS 局部映射 (数值 IK).

        与旧解析 IK (姿态锁 T_0_6_RESET) 的区别: 局部映射从当前构型出发,
        在 ready 等任意姿态可用、不翻腕; 姿态保持 ω=0 (锁当前朝向).
        workspace 外不再表现为 "ik_no_solution", 而由限位守卫兜底
        (reason="limit_alarm").

        Args:
            vx, vy, vz: 末端速度 (mm/s, 基座系).

        Returns:
            dict: {moved, reason, target_xyz?, alarms?}
              moved=False 且 reason="limit_alarm" → 目标关节角越出限位, 停车.
        """
        # 速度钳制 (本项目限幅)
        v = np.array([vx, vy, vz], dtype=float)
        norm = float(np.linalg.norm(v))
        if norm > self.max_vel_mm_s:
            v = v * (self.max_vel_mm_s / norm)

        # FK 反馈当前末端 + anchor 帧当前角 (单次 0x36 读取)
        ee, q_anchor = self._read_current_ee()
        target_xyz = ee + v * self.dt_s
        dx = v * self.dt_s

        # 数值 IK: 雅可比 DLS, 姿态保持 (ω=0), 每帧 Δq 上限防奇异尖峰
        q_src = anchor_to_source(q_anchor)
        J = jacobian(q_src)
        twist = np.append(dx, np.zeros(3))
        w = [1.0, 1.0, 1.0] + [self.orient_weight] * 3
        dq = damped_ls(J, twist, self.ik_lambda, weights=w)
        dq = np.clip(dq, -math.radians(self.max_dq_deg),
                     math.radians(self.max_dq_deg))
        q_src_target = q_src + np.degrees(dq)
        q_anchor_target = source_to_anchor(q_src_target)
        # 归一化到 [-180,180): source_to_anchor 的 %360 会把 J4(有界[-90,90]) 的
        # 小负角折成 ~359.x → 限位守卫误报越界. 归一化后限位比较与下发一致.
        q_anchor_target = [((v + 180.0) % 360.0) - 180.0 for v in q_anchor_target]

        # C-task 实时限位守卫 (目标越界 → 单轴 stop + 告警)
        alarms = self.ctrl.check_limits_real(q_anchor_target, use_kb=True)
        if alarms:
            logger.warning("cartesian: 限位守卫 %s", alarms)
            return {"moved": False, "reason": "limit_alarm",
                    "target_xyz": target_xyz.tolist(), "alarms": alarms}

        self.ctrl.set_joints_safe(q_anchor_target, use_kb=True)
        return {"moved": True, "target_xyz": target_xyz.tolist()}

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
        ee, _ = self._read_current_ee()
        return ee.tolist()
