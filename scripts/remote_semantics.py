"""固件 remote_event 语义的纯函数封装（供 mujoco_sim 与单测使用）。

与 STM32 固件 firmware/src/robot_cmd.c 的 robot_remote_event_handle()
逐字节一致:

    remote_event p0 p1 p2 p3 p4 p5
      vx = -p0            # 基座系 x 线速度系数
      vy =  p1            # 基座系 y 线速度系数
      vz = (p4 - p5)/2    # 基座系 z 线速度系数
      rx = -p3            # J5 关节速度系数（末端上下）
      ry =  p2            # J6 关节速度系数（末端旋转）

本模块不含增益/单位换算——调用方（mujoco_sim）自行乘线/角速度增益。
"""
from typing import Sequence, Tuple

import numpy as np


def parse_remote_event(vals: Sequence[float]) -> Tuple[np.ndarray, float, float]:
    """把 6 个 remote_event 参数解析为 (v_lin(3,), j5_coef, j6_coef)。

    v_lin 为基座系线速度系数 [-p0, p1, (p4-p5)/2]；j5/j6 为关节速度系数。
    """
    p0, p1, p2, p3, p4, p5 = (float(v) for v in vals[:6])
    v_lin = np.array([-p0, p1, (p4 - p5) / 2.0])
    j5_coef = -p3
    j6_coef = p2
    return v_lin, j5_coef, j6_coef
