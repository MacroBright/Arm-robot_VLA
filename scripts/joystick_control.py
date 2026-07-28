#!/usr/bin/env python3
r"""USB 手柄遥控机械臂脚本。

通过 PC USB 手柄 → 串口 → STM32 实现实时遥控。
需要 pygame 和 pyserial。首次使用先: pip install pygame

用法:
  .venv/Scripts/python.exe scripts/joystick_control.py --port COM5
  # 仿真模式: --port socket://localhost:5555
  # 速度调节: --speed 0.3 --joint-step 0.5 --joint-speed 3

控制映射 (Xbox / 通用 USB 手柄):
  ┌─────────────────────────────────────────────┐
  │  左摇杆        →  末端 XY 平移              │
  │  右摇杆        →  末端旋转 (RX/RY)          │
  │  L2 / R2      →  末端 Z 升降               │
  │  A 键          →  remote_enable (进入遥控)  │
  │  B 键          →  remote_disable (退出遥控) │
  │  Y 键          →  e_stop (急停)             │
  │  X 键          →  set_torque 切换           │
  │  十字键 ↑↓     →  逐关节控制 (J1-J6)       │
  │  L1 / R1      →  切换当前关节               │
  │  START        →  zero (当前位置归零)        │
  │  BACK         →  退出脚本                   │
  └─────────────────────────────────────────────┘
"""

import argparse
import sys
import time
import threading
from pathlib import Path

import pygame
import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from lerobot_robot_massage.serial_protocol import EmergencyStopError, SerialProtocol


# ── 手柄轴/按钮索引 (Nintendo Switch Pro / 通用 USB 手柄) ──────────
# 运行 scripts/test_joystick_map.py 可确认你的手柄实际映射
AXIS_LX = 0      # 左摇杆 X
AXIS_LY = 1      # 左摇杆 Y
AXIS_RX = 2      # 右摇杆 X
AXIS_RY = 3      # 右摇杆 Y
BTN_A = 0
BTN_B = 1
BTN_Y = 2        # Nintendo 布局: Y 在 X 位置
BTN_X = 3        # Nintendo 布局: X 在 Y 位置
BTN_LB = 5       # L1
BTN_RB = 6       # R1
BTN_LT = 7       # L2 扳机 (数字按钮)
BTN_RT = 8       # R2 扳机 (数字按钮)
BTN_BACK = 9
BTN_START = 10

DEADZONE = 0.08    # 摇杆死区 (进入阈值)
DEADZONE_EXIT = 0.03  # 迟滞退出阈值
EMA_ALPHA = 0.3      # 轴值指数移动平均系数 (0=无滤波, 1=无平滑)

# ── 遥控参数 ──────────────────────────────────────────────────────────
CMD_INTERVAL = 0.02          # 遥控命令发送间隔 (20ms = 50Hz, 与物理步进对齐)
DEFAULT_SPEED_SCALE = 0.8    # 笛卡尔遥控速度比例 (0.05~1.0, 1.0=固件满速 20mm/s)
DEFAULT_JOINT_STEP = 2.0     # 逐关节模式每次步进角度 (度)
DEFAULT_JOINT_SPEED = 12.0   # 逐关节模式长按连发角速度 (度/s)


class JoystickController:
    """USB 手柄 → 串口 → 机械臂 遥控器"""

    def __init__(self, port: str, camera_index: int = 1,
                 speed_scale: float = DEFAULT_SPEED_SCALE,
                 joint_step: float = DEFAULT_JOINT_STEP,
                 joint_speed: float = DEFAULT_JOINT_SPEED):
        # ── 速度参数 ──
        self.speed_scale = max(0.05, min(1.0, speed_scale))
        self.joint_step = max(0.1, joint_step)  # L1: 禁止零/负步进
        # 长按连发间隔 = 步进角度 / 角速度, 使连发速度 ≈ joint_speed °/s
        self._hat_interval = joint_step / max(joint_speed, 0.1)

        # ── 串口 ──
        self.proto = SerialProtocol(port)
        self.proto.connect()
        self.torque_on = False

        # ── 相机 ──
        # Linux 兼容: CAP_DSHOW 仅 Windows 可用
        cap_backend = getattr(cv2, 'CAP_DSHOW', cv2.CAP_ANY)
        self.cap = cv2.VideoCapture(camera_index, cap_backend)
        self.show_camera = self.cap.isOpened()

        # ── 手柄 ──
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("未检测到 USB 手柄! 请插上后重试。")
        self.js = pygame.joystick.Joystick(0)
        self.js.init()
        print(f"[手柄] {self.js.get_name()}")
        print(f"  轴: {self.js.get_numaxes()}  按钮: {self.js.get_numbuttons()}  十字键: {self.js.get_numhats()}")
        # 诊断: 打印前 8 个轴和按钮数, 帮助确认 L2/R2/RB 等映射
        axes_vals = ", ".join(f"A{i}={self.js.get_axis(i):+.2f}" for i in range(min(8, self.js.get_numaxes())))
        print(f"  轴值: {axes_vals}")
        for i in range(min(16, self.js.get_numbuttons())):
            if self.js.get_button(i):
                print(f"  按钮{i}: 按下")

        # ── 状态 ──
        self._shutdown = False           # 紧急停止标志 (M3: 急停后退出循环)
        self.remote_enabled = False
        self.joint_mode = False          # True = 逐关节模式
        self.current_joint = 0           # 当前选中关节 (0-5)
        self.running = True

        # 按钮/十字键去抖
        self._btn_pressed: dict[int, bool] = {}
        self._hat_pressed: dict[tuple, float] = {}

        # EMA 滤波状态 (轴值平滑)
        self._ema_axes: list[float] = []
        self._ema_init = False

        # 后台线程: 持续发送 remote_event
        self._cmd_thread = threading.Thread(target=self._cmd_loop, daemon=True)
        self._cmd_thread.start()

    # ── 后台命令发送循环 ──────────────────────────────────────────────

    def _cmd_loop(self):
        """每 50ms 发送一次 remote_event (仅当 remote 模式启用且非关节模式)"""
        while self.running:
            if self.remote_enabled and not self.joint_mode:
                axes = self._read_axes()
                self._send_remote_event(axes)
            time.sleep(CMD_INTERVAL)

    def _read_axes(self) -> list[float]:
        """读取所有轴值, 应用 EMA 滤波 + 迟滞死区 + 方向校正"""
        # 读所有摇杆轴 (L2/R2 为按钮, 不走这里)
        raw = [self.js.get_axis(i) for i in range(self.js.get_numaxes())]

        # EMA 滤波
        if not self._ema_init:
            self._ema_axes = list(raw)
            self._ema_init = True
        else:
            for i in range(len(raw)):
                self._ema_axes[i] = (EMA_ALPHA * raw[i] +
                                     (1 - EMA_ALPHA) * self._ema_axes[i])
        filtered = list(self._ema_axes)

        # 迟滞死区 (仅对前 4 个摇杆轴; 扳机轴不在 _read_axes 中处理)
        for i in range(4):
            # 检查每个轴当前是否已经活跃 (上一帧有非零输出)
            was_active = abs(self._ema_axes[i]) > DEADZONE_EXIT if self._ema_init else False
            threshold = DEADZONE_EXIT if was_active else DEADZONE
            if abs(filtered[i]) < threshold:
                filtered[i] = 0.0
            elif abs(filtered[i]) < 0.05 and not was_active:
                filtered[i] = 0.0  # 低于最小输出, 钳制为 0

        # 摇杆 Y 轴方向校正
        filtered[AXIS_LY] = -filtered[AXIS_LY]
        filtered[AXIS_RY] = -filtered[AXIS_RY]
        return filtered

    def _send_remote_event(self, axes: list[float]):
        """发送 remote_event 命令 (柱坐标系, 原点=底座中心, Z=垂直地面)。

        摇杆映射:
          左摇杆左右 → 极角 θ (绕 Z 轴旋转末端)
          左摇杆上下 → 极半径 r (靠近/远离 Z 轴)
          右摇杆 X  → 偏航旋转
          右摇杆 Y  → 俯仰旋转
          L2/R2     → Z 轴升降
        """
        # 左摇杆 → 柱坐标: 极角 θ / 极半径 r
        v_theta = axes[AXIS_LX]     # 左右 → 极角
        v_radius = axes[AXIS_LY]    # 上下 → 极半径

        # 右摇杆 → 旋转
        ryaw = axes[AXIS_RX]
        rpitch = -axes[AXIS_RY]

        # L2/R2 → Z 升降
        lt = 1.0 if self.js.get_button(BTN_LT) else 0.0
        rt = 1.0 if self.js.get_button(BTN_RT) else 0.0
        vz_down = lt
        vz_up = rt

        s = self.speed_scale
        cmd = (f"remote_event {v_theta * s:.3f} {v_radius * s:.3f} "
               f"{ryaw * s:.3f} {rpitch * s:.3f} "
               f"{vz_down * s:.3f} {vz_up * s:.3f}")
        try:
            self.proto.send_command(cmd)
        except Exception:
            pass

    # ── 按钮/十字键事件处理 ───────────────────────────────────────────

    def _handle_btn_press(self, btn: int):
        """按钮按下回调 (去抖)"""
        now = time.time()
        if self._btn_pressed.get(btn, 0.0) + 0.3 > now:
            return  # 去抖 300ms
        self._btn_pressed[btn] = now

        # 诊断: 打印按钮编号 (首次按下时), 帮助确认 RB/L1 等映射
        print(f"  [BTN {btn}]", end=" ")

        if btn == BTN_A:
            if not self.remote_enabled:
                print("[控制] remote_enable → 进入遥控模式")
                self.proto.remote_enable()
                time.sleep(0.5)
                self.remote_enabled = True
            else:
                print("[控制] 已在遥控模式")

        elif btn == BTN_B:
            print("[控制] remote_disable → 退出遥控模式")
            self.proto.remote_disable()
            self.remote_enabled = False

        elif btn == BTN_X:
            self.torque_on = not self.torque_on
            self.proto.set_torque(self.torque_on)
            print(f"[控制] 电机扭矩: {'ON' if self.torque_on else 'FREE'}")

        elif btn == BTN_Y:
            print("[急停] e_stop !!!")
            self.proto.e_stop()
            self.remote_enabled = False
            self.torque_on = False

        elif btn == BTN_LB:
            # 关节模式: 上一个关节
            if not self.joint_mode:
                self.joint_mode = True
                print(f"[关节模式] 启用 | 当前: J{self.current_joint + 1}")
            else:
                self.current_joint = (self.current_joint - 1) % 6
                print(f"[关节模式] 当前关节: J{self.current_joint + 1}")

        elif btn == BTN_RB:
            # 关节模式: 下一个关节
            if not self.joint_mode:
                self.joint_mode = True
                print(f"[关节模式] 启用 | 当前: J{self.current_joint + 1}")
            else:
                self.current_joint = (self.current_joint + 1) % 6
                print(f"[关节模式] 当前关节: J{self.current_joint + 1}")

        elif btn == BTN_START:
            print("[控制] zero → 当前位置归零")
            self.proto.zero()

        elif btn == BTN_BACK:
            print("[退出] 按 BACK 退出脚本")
            self.running = False

    def _handle_hat(self, x: int, y: int):
        """十字键处理 (逐关节旋转 + 退出关节模式)"""
        if x == 0 and y == 0:
            return

        now = time.time()
        key = (x, y)
        if self._hat_pressed.get(key, 0.0) + self._hat_interval > now:
            return  # 连发间隔 = joint_step / joint_speed, 长按平稳连发
        self._hat_pressed[key] = now

        if self.joint_mode:
            if y == 1:   # 十字键 ↑: 正转
                self._joint_step(self.current_joint, self.joint_step)
            elif y == -1:  # 十字键 ↓: 反转
                self._joint_step(self.current_joint, -self.joint_step)
            elif x == 1:  # 十字键 →: 退出关节模式, 回到笛卡尔
                self.joint_mode = False
                print("[关节模式] 已退出, 回到笛卡尔遥控")
            elif x == -1:  # 十字键 ←: 退出关节模式
                self.joint_mode = False
                print("[关节模式] 已退出, 回到笛卡尔遥控")

    def _joint_step(self, joint: int, delta: float):
        """发送 rel_rotate 命令 (单关节步进)"""
        cmd = f"rel_rotate {joint + 1} {delta}"  # +1: 内部0-based → 串口1-based
        try:
            self.proto.send_command(cmd)
        except EmergencyStopError:
            self._handle_e_stop_error()
            return
        except Exception:
            pass
        # 读回角度确认
        try:
            angles, _, _ = self.proto.get_state()
            if len(angles) > joint:
                print(f"  J{joint + 1}: {angles[joint]:7.2f}° (Δ={delta:+.1f}°)")
        except EmergencyStopError:
            self._handle_e_stop_error()
        except Exception:
            pass

    def _handle_e_stop_error(self):
        """通信故障自动急停时的处理 (M3): 醒目提示并停止脚本"""
        print("\n" + "=" * 50)
        print("!! 紧急停止 — 串口通信连续失败, 已自动急停 !!")
        print("请检查硬件连接/固件状态后重新运行脚本")
        print("=" * 50)
        self.remote_enabled = False
        self.torque_on = False
        self.running = False
        self._shutdown = True

    # ── 相机 HUD ──────────────────────────────────────────────────────

    def _draw_hud(self, frame, angles: list[float], fps: float):
        """在相机画面上叠印状态信息"""
        h, w = frame.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX

        # 左侧: 关节角度
        for i, a in enumerate(angles[:6]):
            marker = " ◀" if (self.joint_mode and i == self.current_joint) else "  "
            cv2.putText(frame, f"J{i+1}:{a:6.1f}{marker}",
                        (10, 30 + i * 22), font, 0.5, (0, 255, 0), 1)

        # 右侧: 模式指示 (L3: 关节模式独立于 remote_enabled)
        if self.joint_mode:
            mode_text = f"JOINT J{self.current_joint + 1}"
            mode_color = (255, 200, 0)
        elif self.remote_enabled:
            mode_text = "REMOTE"
            mode_color = (0, 255, 0)
        else:
            mode_text = "IDLE"
            mode_color = (0, 0, 255)
        cv2.putText(frame, mode_text, (w - 140, 30), font, 0.7, mode_color, 2)

        # 扭矩状态
        torque_text = "TORQUE:ON" if self.torque_on else "TORQUE:FREE"
        torque_color = (0, 255, 0) if self.torque_on else (100, 100, 100)
        cv2.putText(frame, torque_text, (w - 160, 58), font, 0.5, torque_color, 1)

        # FPS
        cv2.putText(frame, f"FPS:{fps:4.1f}", (w - 120, h - 15), font, 0.45, (200, 200, 200), 1)

    # ── 主循环 ────────────────────────────────────────────────────────

    def run(self):
        """主事件循环"""
        print("\n[遥控] 已就绪! 操作指南 (柱坐标系, 原点=底座中心, Z=垂直地面):")
        print("   A=进入遥控  B=退出遥控  Y=急停  X=扭矩切换")
        print("   左摇杆左右=极角θ  左摇杆上下=极半径r  右摇杆左右=偏航  右摇杆上下=俯仰  L2/R2=Z升降")
        print("   L1/R1=关节模式/切换 十字键↑↓=关节步进(长按连发) 十字键←→=退出关节模式")
        print(f"   速度: 笛卡尔 {self.speed_scale:.2f}x | "
              f"关节步进 {self.joint_step:.1f}°/次, 长按 {self.joint_step / self._hat_interval:.1f}°/s")
        print("   BACK=退出脚本\n")

        t0 = time.time()
        frames = 0
        ang_msg_count = 0

        while self.running:
            # ── 处理手柄事件 ──
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    self._handle_btn_press(event.button)
                elif event.type == pygame.JOYHATMOTION:
                    self._handle_hat(*event.value)
                elif event.type == pygame.QUIT:
                    self.running = False

            # ── 关节模式: 轮询十字键实现长按连发 ──
            # (JOYHATMOTION 事件仅在状态变化时触发一次, 长按需主动轮询)
            if self.joint_mode and self.js.get_numhats() > 0:
                self._handle_hat(*self.js.get_hat(0))

            # ── 读取状态 (降频: 每 10 帧一次, 避免 IO 锁阻塞 remote_event) ──
            angles = []
            if ang_msg_count % 10 == 0:
                try:
                    angles, _, _ = self.proto.get_state()
                except EmergencyStopError:
                    self._handle_e_stop_error()
                    break
                except Exception:
                    pass

            # ── 相机 HUD ──
            if self.show_camera:
                ret, frame = self.cap.read()
                if ret:
                    frames += 1
                    fps = frames / (time.time() - t0) if time.time() > t0 else 0
                    self._draw_hud(frame, angles, fps)
                    cv2.imshow("Joystick Control", frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        self.running = False
                        break

            # ── 控制台日志 ──
            if ang_msg_count == 0 and angles:
                ang = " ".join(f"{a:6.1f}" for a in angles[:6])
                print(f"  J1-J6: [{ang}]")

            ang_msg_count = (ang_msg_count + 1) % 20

        self.shutdown()

    def shutdown(self):
        """清理资源"""
        self.running = False
        if not self._shutdown:  # 紧急停止跳过多余操作避免覆盖日志
            try:
                self.proto.remote_disable()
            except Exception:
                pass
        time.sleep(0.1)
        self.proto.disconnect()
        if self.show_camera:
            self.cap.release()
        cv2.destroyAllWindows()
        pygame.quit()
        print("[退出] 已安全断开。")


# ── 入口 ──────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="USB 手柄遥控机械臂")
    parser.add_argument("-p", "--port", default="COM5", help="STM32 串口 (默认 COM5)")
    parser.add_argument("-c", "--camera", type=int, default=1, help="USB 摄像头索引 (默认 1)")
    parser.add_argument("-s", "--speed", type=float, default=DEFAULT_SPEED_SCALE,
                        help="笛卡尔遥控速度比例 0.05~1.0 (默认 0.5, 1.0=固件满速)")
    parser.add_argument("--joint-step", type=float, default=DEFAULT_JOINT_STEP,
                        help="关节模式每次步进角度, 度 (默认 1.0)")
    parser.add_argument("--joint-speed", type=float, default=DEFAULT_JOINT_SPEED,
                        help="关节模式长按连发角速度, 度/s (默认 5.0)")
    args = parser.parse_args()

    ctrl = JoystickController(args.port, args.camera,
                              speed_scale=args.speed,
                              joint_step=args.joint_step,
                              joint_speed=args.joint_speed)
    try:
        ctrl.run()
    except KeyboardInterrupt:
        ctrl.shutdown()


if __name__ == "__main__":
    main()
