"""Cartesian keyboard teleoperation CLI for Arm-robot_VLA (Real & Simulation)."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Optional

import pygame

from arm_robot.controller.config import JOINT_INIT_ANGLE_DEG, READY_POSE_DEG, ZdtConfig
from arm_robot.controller.controller import ZdtController
from arm_robot.kinematics.cartesian import CartesianController
from arm_robot.kinematics.kinematics import fk_mdh
from arm_robot.kinematics.types import CartesianCommand
from arm_robot.teleop.arm_adapter import SimulationArmAdapter
from arm_robot.teleop.arm_client import ArmClient

# Key -> (vx, vy, vz)
KEY_VEL = {
    pygame.K_w: (1.0, 0.0, 0.0),
    pygame.K_s: (-1.0, 0.0, 0.0),
    pygame.K_a: (0.0, 1.0, 0.0),
    pygame.K_d: (0.0, -1.0, 0.0),
    pygame.K_q: (0.0, 0.0, 1.0),
    pygame.K_e: (0.0, 0.0, -1.0),
}

LOOP_HZ = 30.0
BASE_VEL = 30.0  # mm/s
GEAR_MULTS = [1.0, 2.0, 3.0]
GEAR_NAMES = ["慢速 30mm/s", "中速 60mm/s", "高速 90mm/s"]


def _load_font(size: int) -> pygame.font.Font:
    for name in ("notosanscjksc", "wqy-microhei", "wqy-zenhei", "simhei", "arial"):
        try:
            return pygame.font.SysFont(name, size)
        except Exception:
            pass
    return pygame.font.Font(None, size)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="机械臂 6-DOF 笛卡尔键盘遥操 (支持真机与 MuJoCo 仿真)")
    ap.add_argument("--iface", default="can0", help="SocketCAN 接口 (真机模式, 默认 can0)")
    ap.add_argument("--sim", nargs="?", const="socket://localhost:5555", default=None,
                    help="连接 MuJoCo 仿真 TCP 服务 (默认 socket://localhost:5555)")
    args = ap.parse_args(argv)

    cart: Optional[CartesianController] = None
    sim_client: Optional[ArmClient] = None
    is_sim = args.sim is not None

    if is_sim:
        sim_url = args.sim if args.sim.startswith("socket://") else f"socket://localhost:{args.sim}"
        print(f"[键盘遥操] 连接 MuJoCo 物理仿真节点: {sim_url} ...")
        sim_client = ArmClient.open(sim_url)
        sim_client.remote_enable()
        print("[键盘遥操] 仿真连接成功！按 W/A/S/D/Q/E 控制 3D 机械臂末端。")
    else:
        cfg = ZdtConfig(channel=args.iface, bitrate=500_000)
        ctrl = ZdtController(cfg)
        ctrl.connect()
        print("[键盘遥操] 真机 SocketCAN 已连接，回车确认使能扭矩 (重力关节 J2/J3 二次确认)...")
        input()
        ctrl.arm(gravity_confirmed=True)
        cart = CartesianController(ctrl, max_vel_mm_s=BASE_VEL, loop_hz=LOOP_HZ)

    pygame.init()
    W, H = 840, 420
    screen = pygame.display.set_mode((W, H))
    mode_str = f"MuJoCo Simulation ({args.sim})" if is_sim else f"Real CAN ({args.iface})"
    pygame.display.set_caption(f"Cartesian Keyboard Teleop — {mode_str}")

    f_title = _load_font(36)
    f_val = _load_font(26)
    f_hint = _load_font(20)

    clock = pygame.time.Clock()
    running = True
    gear = 1  # 默认中速
    pressed: set[int] = set()

    try:
        while running:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    running = False
                elif ev.type == pygame.KEYDOWN:
                    if ev.key in (pygame.K_ESCAPE, pygame.K_q) and pygame.key.get_mods() & pygame.KMOD_CTRL:
                        running = False
                    elif ev.key in KEY_VEL:
                        pressed.add(ev.key)
                    elif ev.key in (pygame.K_LSHIFT, pygame.K_RSHIFT):
                        gear = (gear + 1) % len(GEAR_MULTS)
                    elif ev.key == pygame.K_r:
                        if is_sim and sim_client:
                            sim_client.soft_reset()
                        elif cart:
                            cart.ctrl.ready()
                    elif ev.key == pygame.K_SPACE:
                        if is_sim and sim_client:
                            sim_client.e_stop()
                        elif cart:
                            cart.ctrl.e_stop()
                elif ev.type == pygame.KEYUP:
                    pressed.discard(ev.key)

            # 计算按键合成线速度
            vx, vy, vz = 0.0, 0.0, 0.0
            mult = GEAR_MULTS[gear]
            for k in pressed:
                if k in KEY_VEL:
                    dx, dy, dz = KEY_VEL[k]
                    vx += dx * BASE_VEL * mult
                    vy += dy * BASE_VEL * mult
                    vz += dz * BASE_VEL * mult

            # 发送控制指令
            if is_sim and sim_client:
                # 仿真模式: 通过 end_event 发送末端 6DOF 线速度 (mm/s)
                sim_client.end_event(vx, vy, vz, 0.0, 0.0, 0.0)
            elif cart:
                cart.step(CartesianCommand((vx, vy, vz), (0.0, 0.0, 0.0)))

            # UI 渲染
            screen.fill((24, 28, 36))
            title_s = f_title.render("🎮 机械臂 6-DOF 笛卡尔键盘遥控", True, (255, 255, 255))
            screen.blit(title_s, (30, 20))

            mode_color = (0, 220, 255) if is_sim else (0, 255, 120)
            mode_s = f_val.render(f"当前模式: {mode_str} | 档位: {GEAR_NAMES[gear]} (Shift切换)", True, mode_color)
            screen.blit(mode_s, (30, 75))

            vel_s = f_val.render(f"当前输出线速度: Vx={vx:+.1f}, Vy={vy:+.1f}, Vz={vz:+.1f} mm/s", True, (255, 220, 0))
            screen.blit(vel_s, (30, 120))

            hints = [
                "按键映射 (保持本窗口聚焦):",
                "  • W / S : 前进 / 后退 (+X / -X)",
                "  • A / D : 向左 / 向右 (+Y / -Y)",
                "  • Q / E : 抬升 / 下降 (+Z / -Z)",
                "  • Shift : 切换速度档位 (慢 / 中 / 快)",
                "  • R     : 复位回推拿就绪姿态 (Ready Pose)",
                "  • Space : 急停 (E-Stop)",
                "  • Esc   : 退出",
            ]
            y = 180
            for h in hints:
                screen.blit(f_hint.render(h, True, (180, 190, 205)), (30, y))
                y += 28

            pygame.display.flip()
            clock.tick(LOOP_HZ)

    finally:
        pygame.quit()
        if is_sim and sim_client:
            sim_client.end_event(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            sim_client.close()
        elif cart:
            cart.ctrl.stop()
            cart.ctrl.disconnect()
        print("[键盘遥操] 已安全断开退出。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
