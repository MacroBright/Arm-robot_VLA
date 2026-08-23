#!/usr/bin/env python3
"""Zero Arm VLA 控制台 Hub — 双相机画面 + 关节数据 + 录制控制。

通过 SharedMemory 读取相机帧 (camera_server.py 渲染),
通过 TCP 读取关节状态 (mujoco_sim.py), 在一个 OpenCV 窗口中集成全部功能。

用法:
  python scripts/control_hub.py --port 5555

键盘:
  R / Space  — 开始/停止录制
  Q / Esc    — 退出

鼠标:
  点击 [开始录制] / [停止录制] 按钮
"""

import argparse
import json
import os
import socket
import sys
import time
from datetime import datetime
from multiprocessing import shared_memory
from pathlib import Path

# 抑制 OpenCV Qt5 后端的字体警告
os.environ.setdefault("QT_LOGGING_RULES", "*.debug=false;*.warning=false")
os.environ.setdefault("QT_QPA_FONTS_DIR", "/usr/share/fonts")

import cv2
import numpy as np

# ── 共享内存布局 (与 camera_server.py / mujoco_sim.py 对齐) ──
SHM_NAME = "mujoco_frame_0"
SHM_NAME_EE = "mujoco_frame_ee"
SHM_HEADER_SIZE = 64
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

JOINT_NAMES = [
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_roll", "wrist_flex", "gripper",
]

# ── Hub 窗口布局 ──
HUB_W, HUB_H = 1340, 660
PANEL_Y = 500
BTN_H = 42
BTN_X1, BTN_X3 = 30, 460
BTN_Y = PANEL_Y + 80
BTN_W = 200


# ═══════════════════════════════════════════════════════════════════════
# TCP 客户端 (读取关节状态)
# ═══════════════════════════════════════════════════════════════════════

class SimSocket:
    """原生 TCP socket 直连仿真后端。"""

    def __init__(self, host: str = "127.0.0.1", port: int = 5555):
        self._addr = (host, port)
        self._sock: socket.socket | None = None
        self._buf = b""

    def connect(self, retries: int = 30) -> bool:
        for i in range(retries):
            self.disconnect()
            self._buf = b""
            try:
                self._sock = socket.create_connection(self._addr, timeout=2.0)
                self._sock.settimeout(0.3)
                self._send("get_state")
                if self._recv_until("STATE:", timeout=1.0):
                    return True
            except (ConnectionRefusedError, OSError, socket.timeout):
                pass
            if i == 0:
                print(f"[hub] 等待仿真 ({self._addr[0]}:{self._addr[1]})",
                      end="", flush=True)
            print(".", end="", flush=True)
            time.sleep(1.0)
        print(f"\n[hub] 无法连接仿真, 请确认 mujoco_sim.py 已启动")
        return False

    def disconnect(self) -> None:
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def get_state(self) -> tuple[list[float], list[float], list[float]]:
        for _ in range(3):
            try:
                self._send("get_state")
                resp = self._recv_until("STATE:", timeout=0.3)
                if resp is None:
                    return [], [], []
                vals = [float(v) for v in resp[6:].strip().split(",")]
                n = len(vals) // 3
                return vals[:n], vals[n:2 * n], vals[2 * n:3 * n]
            except (BrokenPipeError, ConnectionResetError, OSError):
                self.disconnect()
                time.sleep(0.5)
                self.connect()
        return [], [], []

    def _send(self, cmd: str) -> None:
        self._sock.sendall(cmd.encode("ascii") + b"\n")

    def get_hub(self) -> dict | None:
        """单命令获取 Hub 所需全部数据 (避免多命令响应冲突)。"""
        try:
            self._send("get_hub")
            resp = self._recv_until("HUB:", timeout=0.5)
            if resp is None:
                return None
            # HUB:ang6,vel6,load6,ee3,target3,mode,joint_idx,joint_name
            parts = resp[4:].split(",")
            n = 6
            angles = [float(v) for v in parts[:n]]
            vels = [float(v) for v in parts[n:2*n]]
            loads = [float(v) for v in parts[2*n:3*n]]
            ee_pos = [float(v) for v in parts[3*n:3*n+3]]
            target_pos = [float(v) for v in parts[3*n+3:3*n+6]]
            mode = parts[3*n+6]
            joint_idx = int(parts[3*n+7])
            joint_name = parts[3*n+8]
            # 计算 target distance
            ee = np.array(ee_pos)
            tp = np.array(target_pos)
            dist = float(np.linalg.norm(ee - tp))
            return {
                "angles": angles, "vels": vels, "loads": loads,
                "ee_pos": ee_pos, "target_pos": target_pos,
                "mode": (mode, joint_idx, joint_name),
                "target": (target_pos, dist, "sphere"),
            }
        except Exception:
            return None

    def _recv_until(self, prefix: str, timeout: float) -> str | None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            while b"\n" in self._buf:
                line, self._buf = self._buf.split(b"\n", 1)
                text = line.decode("ascii", errors="replace").strip()
                if text.startswith(prefix):
                    return text
            try:
                chunk = self._sock.recv(4096)
                if not chunk:
                    return None
                self._buf += chunk
            except socket.timeout:
                continue
            except OSError:
                return None
        return None


# ═══════════════════════════════════════════════════════════════════════
# 录制状态机
# ═══════════════════════════════════════════════════════════════════════

class Recorder:
    """管理录制状态 + 帧/数据写入。"""

    def __init__(self, output_dir: str = "datasets/sim_v1"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self._active = False
        self._dir: Path | None = None
        self._count = 0
        self._frames: list[dict] = []
        self._t0 = 0.0

    @property
    def active(self) -> bool:
        return self._active

    @property
    def episode_name(self) -> str:
        return self._dir.name if self._dir else "--"

    @property
    def count(self) -> int:
        return self._count

    @property
    def elapsed(self) -> float:
        return (time.monotonic() - self._t0) if self._active else 0.0

    def start(self) -> None:
        if self._active:
            return
        episodes = sorted(self.output_dir.glob("episode_*"))
        eid = len(episodes) + 1
        self._dir = self.output_dir / f"episode_{eid:04d}"
        self._dir.mkdir(parents=True, exist_ok=True)
        self._count = 0
        self._frames = []
        self._t0 = time.monotonic()
        self._active = True
        print(f"\n[hub] ● 录制开始 Episode {eid:04d} → {self._dir}")

    def stop(self) -> dict | None:
        if not self._active:
            return None
        self._active = False
        dur = time.monotonic() - self._t0
        meta = {
            "episode_id": int(self._dir.name.split("_")[-1]),
            "duration_s": round(dur, 2),
            "frames": self._count,
            "fps_actual": round(self._count / dur, 1) if dur > 0 else 0,
            "joint_names": JOINT_NAMES,
            "frame_width": FRAME_WIDTH,
            "frame_height": FRAME_HEIGHT,
            "cameras": ["cam_top", "ee_camera"],
            "recorded_at": datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
        }
        with open(self._dir / "data.json", "w", encoding="utf-8") as f:
            json.dump({"meta": meta, "frames": self._frames}, f,
                      indent=2, ensure_ascii=False)
        print(f"[hub] ■ 录制完成: {self._count} 帧, {dur:.1f}s → {self._dir}")
        return meta

    def save(self, frame_top: np.ndarray | None,
             frame_ee: np.ndarray | None,
             angles: list[float], vels: list[float],
             loads: list[float], ts: float,
             ee_pos: list[float] | None = None,
             target_pos: list[float] | None = None) -> None:
        if not self._active:
            return
        i = self._count
        if frame_top is not None:
            cv2.imwrite(str(self._dir / f"frame_{i:06d}.png"), frame_top)
        if frame_ee is not None:
            cv2.imwrite(str(self._dir / f"ee_frame_{i:06d}.png"), frame_ee)
        frame_data = {
            "timestamp": round(ts, 4),
            "angles": [round(a, 2) for a in angles[:6]],
            "velocities": [round(v, 2) for v in vels[:6]],
            "loads": [round(l, 2) for l in loads[:6]],
        }
        if ee_pos:
            frame_data["ee_pos"] = [round(p, 4) for p in ee_pos]
        if target_pos:
            frame_data["target_pos"] = [round(p, 4) for p in target_pos]
        self._frames.append(frame_data)
        self._count += 1


# ═══════════════════════════════════════════════════════════════════════
# Hub 画面绘制
# ═══════════════════════════════════════════════════════════════════════

def draw_button(canvas: np.ndarray, x: int, y: int, w: int, h: int,
                label: str, bg: tuple) -> None:
    """在 canvas 上绘制一个矩形按钮。"""
    cv2.rectangle(canvas, (x, y), (x + w, y + h), bg, -1, cv2.LINE_AA)
    cv2.rectangle(canvas, (x, y), (x + w, y + h),
                  (bg[0] // 3, bg[1] // 3, bg[2] // 3), 2, cv2.LINE_AA)
    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
    cv2.putText(canvas, label, (x + (w - tw) // 2, y + (h + th) // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)


def build_hub(god: np.ndarray | None, ee: np.ndarray | None,
              angles: list[float], vels: list[float],
              rec: Recorder,
              target: tuple | None = None,
              mode: tuple | None = None) -> np.ndarray:
    """构建完整 Hub 画面。"""
    c = np.zeros((HUB_H, HUB_W, 3), dtype=np.uint8)
    c[:] = (32, 32, 32)

    # ── 双相机画面 ──
    gx, gy = 10, 10
    ex, ey = gx + FRAME_WIDTH + 10, 10

    if god is not None:
        c[gy:gy + god.shape[0], gx:gx + god.shape[1]] = god
    cv2.putText(c, "God-View (cam_top)", (gx + 5, gy + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)

    if ee is not None:
        c[ey:ey + ee.shape[0], ex:ex + ee.shape[1]] = ee
    cv2.putText(c, "EE Camera", (ex + 5, ey + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)

    # ── 底部面板 ──
    cv2.rectangle(c, (0, PANEL_Y), (HUB_W, HUB_H), (24, 24, 24), -1)
    cv2.line(c, (0, PANEL_Y), (HUB_W, PANEL_Y), (80, 80, 80), 1)

    # 关节角度 (大号粗体)
    if len(angles) >= 6:
        jt = "  ".join(f"J{i + 1}:{a:7.1f}°" for i, a in enumerate(angles[:6]))
    else:
        jt = "等待关节数据..."
    cv2.putText(c, jt, (18, PANEL_Y + 28),
                cv2.FONT_HERSHEY_DUPLEX, 0.7, (230, 230, 230), 2, cv2.LINE_AA)

    # 关节速度
    if len(vels) >= 6:
        vt = "  ".join(f"v{i + 1}:{v:6.1f}" for i, v in enumerate(vels[:6]))
        cv2.putText(c, vt, (18, PANEL_Y + 56),
                    cv2.FONT_HERSHEY_DUPLEX, 0.55, (170, 170, 170), 1, cv2.LINE_AA)

    # ── 目标小球状态 ──
    if target is not None:
        t_pos, t_dist, t_model = target
        ttext = (f"目标: ({t_pos[0]:.2f}, {t_pos[1]:.2f}, {t_pos[2]:.2f})  "
                 f"距离末端: {t_dist * 100:.1f} cm")
        if t_dist < 0.04:
            tc = (100, 255, 100)  # 绿色=触碰中
        elif t_dist < 0.10:
            tc = (255, 200, 50)   # 黄色=接近
        else:
            tc = (200, 200, 200)  # 灰色=远离
        cv2.putText(c, ttext, (BTN_X3 + 150, PANEL_Y + 55),
                    cv2.FONT_HERSHEY_DUPLEX, 0.55, tc, 1, cv2.LINE_AA)

    # ── 控制模式 (右下角) ──
    mode_text = "模式: --"
    mode_color = (150, 150, 150)
    if mode is not None:
        m, jidx, jname = mode
        if m == "cartesian":
            mode_text = "模式: 笛卡尔 (Cartesian IK)"
            mode_color = (100, 200, 255)
        elif m == "joint":
            mode_text = f"模式: 逐关节 → {jname}"
            mode_color = (255, 200, 100)
        elif m == "idle":
            mode_text = "模式: 空闲 (按A进入遥控)"
            mode_color = (150, 150, 150)
    (tw, _), _ = cv2.getTextSize(mode_text, cv2.FONT_HERSHEY_DUPLEX, 0.5, 1)
    cv2.putText(c, mode_text, (HUB_W - tw - 20, BTN_Y + 30),
                cv2.FONT_HERSHEY_DUPLEX, 0.5, mode_color, 1, cv2.LINE_AA)

    # ── 按钮 (在关节数据下方) ──
    if rec.active:
        draw_button(c, BTN_X1, BTN_Y, BTN_W, BTN_H,
                    "[R] 停止录制", (60, 60, 180))
    else:
        draw_button(c, BTN_X1, BTN_Y, BTN_W, BTN_H,
                    "[R] 开始录制", (30, 140, 30))
    draw_button(c, BTN_X3, BTN_Y, 130, BTN_H, "[Q] 退出", (120, 40, 40))

    # ── 状态行 (按钮同行右侧) ──
    if rec.active:
        st = (f"● 录制中 | Episode: {rec.episode_name} | "
              f"帧: {rec.count} | 时间: {rec.elapsed:.0f}s")
        sc = (100, 255, 100)
    else:
        st = "○ 就绪 — 按 R 键开始录制"
        sc = (180, 180, 180)
    cv2.putText(c, st, (475, BTN_Y + 30),
                cv2.FONT_HERSHEY_DUPLEX, 0.55, sc, 1, cv2.LINE_AA)

    return c


def read_frame(shm: shared_memory.SharedMemory | None) -> np.ndarray | None:
    if shm is None:
        return None
    try:
        raw = memoryview(shm.buf)[SHM_HEADER_SIZE:
                                  SHM_HEADER_SIZE + FRAME_WIDTH * FRAME_HEIGHT * 3]
        return np.ndarray((FRAME_HEIGHT, FRAME_WIDTH, 3),
                          dtype=np.uint8, buffer=raw).copy()
    except Exception:
        return None


# ═══════════════════════════════════════════════════════════════════════
# 主入口
# ═══════════════════════════════════════════════════════════════════════

def main() -> None:
    parser = argparse.ArgumentParser(description="Zero Arm VLA 控制台 Hub")
    parser.add_argument("--port", type=int, default=5555)
    parser.add_argument("--output", default="datasets/sim_v1")
    parser.add_argument("--fps", type=int, default=25)
    args = parser.parse_args()

    print("=" * 60)
    print("Zero Arm VLA — 控制台 Hub")
    print("=" * 60)
    print("  [R] 开始/停止录制  [Q] 退出")
    print()

    # ── 连接仿真 ──
    sim = SimSocket(port=args.port)
    if not sim.connect():
        sys.exit(1)

    # ── 打开共享内存 ──
    shm, shm_ee = None, None
    for attempt in range(10):
        try:
            shm = shared_memory.SharedMemory(name=SHM_NAME)
            shm_ee = shared_memory.SharedMemory(name=SHM_NAME_EE)
            break
        except FileNotFoundError:
            if attempt == 0:
                print(f"[hub] 等待相机帧...", end="", flush=True)
            print(".", end="", flush=True)
            time.sleep(0.5)
    if shm is None:
        print("\n[hub] 错误: 共享内存不存在, 请确认 camera_server.py 已启动")
        sim.disconnect()
        sys.exit(1)
    print(f"\n[hub] 已连接, Hub 启动 (按 Q 退出)\n")

    rec = Recorder(output_dir=args.output)

    # ── OpenCV 窗口 ──
    WN = "Zero Arm VLA — Control Hub"
    cv2.namedWindow(WN, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    cv2.resizeWindow(WN, HUB_W, HUB_H)

    angles, vels, loads, target, ee_data, mode_data = [], [], [], None, None, None
    last_state = 0.0
    interval = 1.0 / args.fps
    quit_flag = False

    try:
        while not quit_flag:
            t0 = time.monotonic()

            # 状态查询 (10Hz, 单命令避免响应冲突)
            if t0 - last_state > 0.1:
                hub = sim.get_hub()
                if hub is not None:
                    angles = hub["angles"]
                    vels = hub["vels"]
                    loads = hub["loads"]
                    ee_data = (hub["ee_pos"], hub["target_pos"])
                    target = hub["target"]
                    mode_data = hub["mode"]
                last_state = t0

            # 相机帧
            god = read_frame(shm)
            ee = read_frame(shm_ee)

            # 录制
            if rec.active and len(angles) >= 6:
                ee_p, tgt_p = (None, None)
                if ee_data is not None:
                    ee_p, tgt_p = ee_data
                rec.save(god, ee, angles, vels, loads, t0, ee_p, tgt_p)

            # 绘制
            canvas = build_hub(god, ee, angles, vels, rec, target, mode_data)
            cv2.imshow(WN, canvas)

            # 键盘
            key = cv2.waitKey(max(1, int((interval - (time.monotonic() - t0)) * 1000))) & 0xFF
            if key in (ord('r'), ord(' ')):
                if rec.active:
                    rec.stop()
                else:
                    rec.start()
            elif key in (ord('q'), 27):
                quit_flag = True

            elapsed = time.monotonic() - t0
            if elapsed < interval:
                time.sleep(interval - elapsed)

    except KeyboardInterrupt:
        print("\n[hub] 用户中断")
    finally:
        if rec.active:
            rec.stop()
        sim.disconnect()
        if shm:
            shm.close()
        if shm_ee:
            shm_ee.close()
        cv2.destroyWindow(WN)
        print("[hub] Hub 已退出")


if __name__ == "__main__":
    main()
