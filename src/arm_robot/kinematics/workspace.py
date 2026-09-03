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
