"""固件 remote_event 语义的纯函数封装（供 mujoco_sim 与单测使用）。

与 STM32 固件 firmware/src/robot_cmd.c 的 robot_remote_event_handle()
一致的前 6 参:

    remote_event p0 p1 p2 p3 p4 p5 [p6]
      vx = -p0            # 基座系 x 线速度系数
      vy =  p1            # 基座系 y 线速度系数
      vz = (p4 - p5)/2    # 基座系 z 线速度系数
      rx = -p3            # J5 关节速度系数（末端上下）
      ry =  p2            # J6 关节速度系数（末端旋转）
      j4 =  p6            # J4 主旋转（手滚转）——仿真扩展通道
                          # 固件 sscanf 只读 6 参, 自动忽略 p6 (向后兼容)

本模块不含增益/单位换算——调用方（mujoco_sim）自行乘线/角速度增益。
"""
from typing import Sequence, Tuple

import numpy as np


def parse_remote_event(vals: Sequence[float]) -> Tuple[np.ndarray, float, float, float]:
    """把 7 个 remote_event 参数解析为 (v_lin, j4_coef, j5_coef, j6_coef).

    与固件 robot_cmd.c 一致的前 6 参 (vx=-p0, vy=p1, vz=(p4-p5)/2, rx=-p3→J5, ry=p2→J6);
    **p6 → J4 (仿真扩展通道, 固件 sscanf 只读 6 参会忽略 p6, 向后兼容)**。
    """
    p0, p1, p2, p3, p4, p5, p6 = (float(v) for v in vals[:7])
    v_lin = np.array([-p0, p1, (p4 - p5) / 2.0])
    j4_coef = p6
    j5_coef = -p3
    j6_coef = p2
    return v_lin, j4_coef, j5_coef, j6_coef
