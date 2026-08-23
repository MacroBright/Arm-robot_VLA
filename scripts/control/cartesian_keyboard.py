#!/usr/bin/env python3
r"""键盘遥操 IK 末端笛卡尔控制 (P2) — PC 直连 CAN.

用法:
  sudo ip link set can0 type can bitrate 500000 && sudo ip link set can0 up
  conda activate smolvla && python scripts/control/cartesian_keyboard.py [--iface can0]

启动流程:
  连接 → 打开遥操窗口 → 打印当前 6 轴状态 (不阻塞) → 进入遥操循环, 不自动回 ready.
  按 R 回按摩准备姿态, 按 H 回上电姿态 (均安全速度同步); SPACE 急停.

按键映射 (pygame 窗口, 保持窗口聚焦):
  W / S / A / D / Q / E → 末端 ±x / ±y / ±z (基座系)
  Shift      → 循环切换档位: 慢 20 / 中 40 / 快 60 mm/s (窗口实时显示)
  R          → 回按摩准备姿态 (ready, 安全速度同步)
  H          → 回上电初始姿态 (全 0, 安全速度同步)
  SPACE      → 急停 (e_stop)
  ESC / Ctrl+C → 退出 (先 e_stop 再断开)

坐标: xyz 基座系 mm, z 向上; 档位: 慢 20 / 中 40 / 快 60 mm/s (单轴).
"""
import argparse
import os
import sys
import time
from pathlib import Path

import pygame

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from lerobot_robot_massage.zdt.cartesian import CartesianController  # noqa: E402
from lerobot_robot_massage.zdt.config import (  # noqa: E402
    JOINT_INIT_ANGLE_DEG, READY_POSE_DEG, ZdtConfig,
)
from lerobot_robot_massage.zdt.controller import ZdtController  # noqa: E402
from lerobot_robot_massage.zdt.kinematics import anchor_to_source, fk_mdh  # noqa: E402

# 按键 → (vx, vy, vz) 倍率 (基座系). 单轴速度 = max_vel × 倍率.
KEY_VEL: dict[int, tuple[float, float, float]] = {
    pygame.K_w: (1.0, 0.0, 0.0),
    pygame.K_s: (-1.0, 0.0, 0.0),
    pygame.K_a: (0.0, 1.0, 0.0),
    pygame.K_d: (0.0, -1.0, 0.0),
    pygame.K_q: (0.0, 0.0, 1.0),
    pygame.K_e: (0.0, 0.0, -1.0),
}

LOOP_HZ = 20.0
BASE_VEL = 20.0      # mm/s (慢档单轴速度)
# 档位: 慢/中/快, Shift 循环切换 (mult × 单轴速度 = 20/40/60 mm/s)
GEAR_MULTS: list[float] = [1.0, 2.0, 3.0]
GEAR_NAMES: list[str] = ["慢档", "中档", "快档"]

# R/H 目标姿态 (anchor 帧): ready=按摩准备位, home=上电全 0
POSE_TARGETS: dict[str, list[float]] = {
    "ready": list(READY_POSE_DEG),
    "home": list(JOINT_INIT_ANGLE_DEG),
}
POSE_NAMES: dict[str, str] = {"ready": "回ready", "home": "回上电"}

# 系统 CJK 字体候选 — pygame 默认字体无中文字形, 窗口中文会乱码/方块.
_CJK_FONT_PATHS = [
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
]


def _load_font(size: int) -> "pygame.font.Font":
    """加载支持中文的系统字体; 全部缺失时回退 pygame 默认字体 (中文仍乱码)."""
    for path in _CJK_FONT_PATHS:
        if os.path.isfile(path):
            try:
                return pygame.font.Font(path, size)
            except OSError:
                continue
    return pygame.font.Font(None, size)


def _build_cartesian(iface: str) -> CartesianController:
    cfg = ZdtConfig(channel=iface, bitrate=500_000)
    ctrl = ZdtController(cfg)
    ctrl.connect()                     # 使能 + 读一次真实角对齐 tracked
    return CartesianController(ctrl, max_vel_mm_s=BASE_VEL, loop_hz=LOOP_HZ)


def _print_status(cart: CartesianController) -> None:
    """启动后打印当前 6 轴真实状态 (不阻塞), 供操作员确认坐标帧/寄存器对齐.

    与 interactive `all status` 同类信息: 0x36 真实角/电流/标志.
    """
    try:
        q_real = cart.ctrl.read_real_angles(use_kb=True)
        loads = [cart.ctrl._driver.read_current(a)
                 for a in cart.ctrl.config.joint_addrs]
        flags = [cart.ctrl._driver.read_flag(a)
                 for a in cart.ctrl.config.joint_addrs]
    except Exception as exc:  # noqa: BLE001
        print(f"[键盘遥操] 状态读取失败: {type(exc).__name__}: {exc}")
        return
    print("\n[键盘遥操] 当前 6 轴状态 (0x36 真实角):")
    print("  J#   anchor°   ready目标  电流mA  标志")
    for i, addr in enumerate(cart.ctrl.config.joint_addrs):
        fl = flags[i]
        bits = []
        if fl & 0x01:
            bits.append("使能")
        if fl & 0x02:
            bits.append("到位")
        if fl & 0x04:
            bits.append("堵转")
        if fl & 0x08:
            bits.append("堵转保护")
        print(f"  J{i+1} {q_real[i]:+8.2f} {READY_POSE_DEG[i]:+7.1f} "
              f"{loads[i]:6.0f}  0x{fl:02X}[{','.join(bits) or '无'}]")
    print(f"  目标: R=ready {READY_POSE_DEG} / H=回上电姿态全 0 / SPACE=急停")


def main() -> None:
    ap = argparse.ArgumentParser(description="键盘遥操 IK 末端笛卡尔控制")
    ap.add_argument("--iface", default="can0", help="SocketCAN 接口 (默认 can0)")
    args = ap.parse_args()

    cart = _build_cartesian(args.iface)
    try:
        pygame.init()
        W, H = 960, 420
        screen = pygame.display.set_mode((W, H))
        pygame.display.set_caption("Cartesian Keyboard Teleop")
        f_title = _load_font(44)
        f_val = _load_font(38)
        f_small = _load_font(27)
        f_hint = _load_font(24)

        pressed: set[int] = set()
        running = True
        gear = 0                        # Shift 循环切换档位 0=慢 1=中 2=快
        pose_move = None                # None / "ready" / "home" — 姿态运动状态
        note, note_t = "", 0.0          # 瞬态提示 (急停/未运动等), 显示 ~2s
        clock = pygame.time.Clock()

        KEY_NAMES = {
            pygame.K_w: "W(+x)", pygame.K_s: "S(-x)",
            pygame.K_a: "A(+y)", pygame.K_d: "D(-y)",
            pygame.K_q: "Q(+z)", pygame.K_e: "E(-z)",
        }

        def draw_status(v: tuple[float, float, float], ee: list[float],
                        gear: int, keys: set[int], note_text: str,
                        pose: str | None = None):
            screen.fill((12, 12, 14))
            estop = note_text.startswith("已急停")
            title = "Cartesian Keyboard Teleop"
            if estop:
                title += "   [急停]"
            elif pose:
                title += f"   [{POSE_NAMES[pose]} 运动中]"
            else:
                title += "   [运行中]"
            title_color = (255, 90, 90) if estop else (120, 220, 120)
            screen.blit(f_title.render(title, True, title_color), (24, 18))
            # 档位配色: 慢=白 中=浅蓝 快=橙
            gear_colors = [(235, 235, 235), (120, 220, 255), (255, 200, 80)]
            speed_color = gear_colors[gear]
            screen.blit(f_val.render(
                f"速度:  ({v[0]:+6.1f}, {v[1]:+6.1f}, {v[2]:+6.1f}) mm/s"
                f"   [{GEAR_NAMES[gear]} {BASE_VEL * GEAR_MULTS[gear]:.0f} mm/s]",
                True, speed_color), (24, 92))
            screen.blit(f_val.render(
                f"末端:  ({ee[0]:+7.1f}, {ee[1]:+7.1f}, {ee[2]:+7.1f}) mm   (FK 反馈)",
                True, (235, 235, 235)), (24, 152))
            if note_text:
                screen.blit(f_small.render("● " + note_text, True, (255, 200, 80)),
                            (24, 218))
            else:
                key_str = " ".join(KEY_NAMES[k] for k in sorted(keys) if k in KEY_NAMES)
                screen.blit(f_small.render(f"按键:  {key_str if key_str else '(无)'}",
                                           True, (210, 210, 210)), (24, 218))
            screen.blit(f_hint.render(
                "W/S=±x  A/D=±y  Q/E=±z  Shift=切换档位  R=回ready  H=回上电  SPACE=急停  ESC=退出",
                True, (170, 170, 170)), (24, 272))
            screen.blit(f_hint.render(
                "慢 20 / 中 40 / 快 60 mm/s · 保持窗口聚焦 (失焦可能导致按键卡住)",
                True, (130, 130, 130)), (24, 308))
            pygame.display.flip()

        _print_status(cart)             # 窗口已开, 打印状态 (不阻塞)

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_SPACE:
                        print("[键盘遥操] 急停!")
                        cart.e_stop()
                        pose_move = None
                        note, note_t = "已急停 (e_stop)", time.monotonic()
                    elif pose_move is not None:
                        # 姿态运动期间: 忽略运动键/档位 (仅 ESC/SPACE 有效)
                        continue
                    elif event.key == pygame.K_r:
                        print(f"[键盘遥操] 回 ready... 目标 {READY_POSE_DEG} (安全速度同步)")
                        cart.ready()
                        pressed.clear()
                        pose_move = "ready"
                        note, note_t = "回 ready...", time.monotonic()
                    elif event.key == pygame.K_h:
                        print("[键盘遥操] 回上电姿态... (安全速度同步)")
                        cart.home()
                        pressed.clear()
                        pose_move = "home"
                        note, note_t = "回上电姿态...", time.monotonic()
                    elif event.key in (pygame.K_LSHIFT, pygame.K_RSHIFT):
                        gear = (gear + 1) % len(GEAR_MULTS)
                        tag = f"{GEAR_NAMES[gear]} ({BASE_VEL * GEAR_MULTS[gear]:.0f} mm/s)"
                        print(f"[键盘遥操] 速度档位: {tag}")
                        note, note_t = f"速度档位: {tag}", time.monotonic()
                    else:
                        pressed.add(event.key)
                elif event.type == pygame.KEYUP:
                    if pose_move is not None:
                        continue
                    pressed.discard(event.key)

            if pose_move is not None:
                # 姿态运动: 轮询真实角显示进度, 不发 step (电机已在同步运动)
                try:
                    q = cart.ctrl.read_real_angles(use_kb=True)
                    target = POSE_TARGETS[pose_move]
                    max_d = max(abs(((q[i] - target[i] + 180.0) % 360.0) - 180.0)
                                for i in range(6))
                    ee = fk_mdh(anchor_to_source(q))[:3, 3].tolist()
                    note, note_t = (f"{POSE_NAMES[pose_move]} 运动..."
                                    f"最大偏差 {max_d:.1f}°"), time.monotonic()
                    draw_status((0.0, 0.0, 0.0), ee, gear, pressed, note,
                                pose=pose_move)
                    if max_d < 1.0:
                        print(f"[键盘遥操] {POSE_NAMES[pose_move]} 到位 ✅")
                        pose_move = None
                        note, note_t = "", 0.0
                    cart.tick()                # 看门狗
                except Exception as exc:  # noqa: BLE001
                    print(f"[键盘遥操] {pose_move} 轮询异常: {type(exc).__name__}: {exc}")
                    try:
                        cart.e_stop()
                    except Exception:  # noqa: BLE001
                        pass
                    running = False
                    break
            else:
                # 合成速度 (方向向量 × 档位)
                v = [0.0, 0.0, 0.0]
                for key in pressed:
                    if key in KEY_VEL:
                        d = KEY_VEL[key]
                        v[0] += d[0]
                        v[1] += d[1]
                        v[2] += d[2]
                vxyz = tuple(BASE_VEL * GEAR_MULTS[gear] * c for c in v)  # 单轴 20/40/60

                # 单帧闭环: step + 显示 + 看门狗 一体 try, 任何 CAN 异常 → 急停退出
                try:
                    res = cart.step(*vxyz)
                    if not res["moved"]:
                        print(f"[键盘遥操] {res.get('reason','?')} — 未运动")
                        note, note_t = f"未运动: {res.get('reason','?')}", time.monotonic()
                    draw_status(vxyz, cart.get_ee_xyz(), gear, pressed,
                                note if time.monotonic() - note_t < 2.0 else "")
                    cart.tick()                # 看门狗
                except Exception as exc:  # noqa: BLE001
                    print(f"[键盘遥操] 循环异常: {type(exc).__name__}: {exc}")
                    try:
                        cart.e_stop()
                    except Exception:  # noqa: BLE001
                        pass
                    running = False
                    break
            clock.tick(LOOP_HZ)
    finally:
        print("[键盘遥操] 退出: 急停 + 断开")
        try:
            cart.e_stop()
        finally:
            cart.ctrl.disconnect()
        pygame.quit()


if __name__ == "__main__":
    main()
