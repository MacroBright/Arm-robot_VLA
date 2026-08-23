#!/usr/bin/env python3
r"""仿真数据录制脚本 — 记录关节角 + 俯视帧 + 末端相机帧。

通过原生 TCP socket 读取关节状态, SharedMemory 读取渲染帧。

依赖: mujoco_sim.py 在另一个终端运行。

用法:
  python scripts/record_sim.py                           # 录制 20 秒
  python scripts/record_sim.py --duration 30 --fps 30    # 录 30 秒 @ 30fps

输出结构 (LeRobot 标准 PNG 帧序列):
  datasets/sim_v1/episode_0001/
  ├── data.json            ← 关节角度 + 速度 + 负载 + 时间戳
  ├── frame_000000.png     ← 俯视帧 (cam_top, 640x480)
  ├── frame_000001.png
  ├── ee_frame_000000.png  ← 末端相机帧 (ee_camera)
  ├── ee_frame_000001.png
  └── ...
"""

import argparse
import json
import socket
import struct
import sys
import time
from pathlib import Path

from multiprocessing import shared_memory

import cv2
import numpy as np

# ── 共享内存布局 (与 mujoco_sim.py 对齐) ────────────────────────────
SHM_NAME = "mujoco_frame_0"
SHM_NAME_EE = "mujoco_frame_ee"
SHM_HEADER_SIZE = 64
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

JOINT_NAMES = [
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_roll", "wrist_flex", "gripper",
]


class SimSocket:
    """原生 TCP socket 直连仿真后端 (绕过 pyserial socket handler)。"""

    def __init__(self, host: str = "127.0.0.1", port: int = 5555):
        self._addr = (host, port)
        self._sock: socket.socket | None = None
        self._buf = b""

    def connect(self, retries: int = 10) -> bool:
        """连接仿真后端, 重试 retries 次。"""
        for i in range(retries):
            # 先关掉上一次失败遗留的 socket
            self.disconnect()
            self._buf = b""
            try:
                self._sock = socket.create_connection(self._addr, timeout=2.0)
                self._sock.settimeout(1.0)
                self._send("get_state")
                resp = self._recv_until("STATE:", timeout=2.0)
                if resp:
                    print(f"\n已连接仿真后端 {self._addr[0]}:{self._addr[1]}")
                    return True
                # 没收到 STATE 响应
                if i == 0:
                    print(f"等待仿真启动 ({self._addr[0]}:{self._addr[1]})",
                          end="", flush=True)
            except (ConnectionRefusedError, OSError, socket.timeout):
                pass
            print(".", end="", flush=True)
            time.sleep(1.0)
        print(f"\n错误: 无法连接 {self._addr[0]}:{self._addr[1]}")
        print("请确认: bash scripts/startup.sh 已启动并显示 'TCP 服务已启动'")
        return False

    def disconnect(self) -> None:
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def get_state(self) -> tuple[list[float], list[float], list[float]]:
        """读取关节状态, 断线自动重连。"""
        for _ in range(3):  # 最多重试 3 次
            try:
                self._send("get_state")
                resp = self._recv_until("STATE:", timeout=0.5)
                if resp is None:
                    return [], [], []
                data = resp[6:].strip()
                vals = [float(v) for v in data.split(",")]
                n = len(vals) // 3
                return vals[:n], vals[n:2 * n], vals[2 * n:3 * n]
            except (BrokenPipeError, ConnectionResetError, OSError):
                # 断线重连
                self.disconnect()
                time.sleep(0.5)
                if self.connect():
                    continue
        return [], [], []

    def _send(self, cmd: str) -> None:
        self._sock.sendall(cmd.encode("ascii") + b"\n")

    def _recv_until(self, prefix: str, timeout: float) -> str | None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            # 检查缓冲区
            while b"\n" in self._buf:
                line, self._buf = self._buf.split(b"\n", 1)
                text = line.decode("ascii", errors="replace").strip()
                if text.startswith(prefix):
                    return text
            # 读新数据
            try:
                chunk = self._sock.recv(4096)
                if not chunk:
                    return None
                self._buf += chunk
            except socket.timeout:
                continue
            except OSError:
                return None
        # 超时诊断
        if self._buf:
            preview = self._buf[:200].decode("ascii", errors="replace")
            print(f"\n  [诊断] recv 超时, buf={preview!r}", flush=True)
        return None


def _read_frame(shm) -> np.ndarray | None:
    try:
        buf = shm.buf[SHM_HEADER_SIZE:SHM_HEADER_SIZE +
                      FRAME_WIDTH * FRAME_HEIGHT * 3]
        return np.ndarray((FRAME_HEIGHT, FRAME_WIDTH, 3),
                          dtype=np.uint8, buffer=buf).copy()
    except Exception:
        return None


def record_episode(duration_s: int, fps: int, output_dir: str) -> int:
    # ── 1. 连接仿真 ──
    sim = SimSocket()
    if not sim.connect():
        return -1

    # ── 2. 打开共享内存 ──
    shm, shm_ee = None, None
    for attempt in range(5):
        try:
            shm = shared_memory.SharedMemory(name=SHM_NAME)
            break
        except FileNotFoundError:
            if attempt == 0:
                print(f"等待共享内存 {SHM_NAME}...", end="", flush=True)
            else:
                print(".", end="", flush=True)
            time.sleep(0.5)
    if shm is None:
        print(f"\n错误: 共享内存 {SHM_NAME} 不存在")
        print("请确认 mujoco_sim.py 正在运行")
        sim.disconnect()
        return -1
    try:
        shm_ee = shared_memory.SharedMemory(name=SHM_NAME_EE)
    except FileNotFoundError:
        shm_ee = None

    cameras = ["cam_top"] + (["ee_camera"] if shm_ee else [])

    # ── 3. 创建输出目录 ──
    out = Path(output_dir)
    episodes = sorted(out.glob("episode_*"))
    episode_id = len(episodes) + 1
    episode_dir = out / f"episode_{episode_id:04d}"
    episode_dir.mkdir(parents=True, exist_ok=True)

    print(f"\n{'='*60}")
    print(f"录制 Episode {episode_id:04d}")
    print(f"  时长: {duration_s}s @ {fps}fps")
    print(f"  相机: {', '.join(cameras)}")
    print(f"  输出: {episode_dir}")
    print(f"{'='*60}\n")

    interval = 1.0 / fps
    frames: list[dict] = []
    frame_count = 0

    print("录制中", end="", flush=True)
    t0 = time.time()

    try:
        while time.time() - t0 < duration_s:
            loop_start = time.time()

            angles, vels, loads = sim.get_state()
            if not angles:
                print("\n  ⚠ 连接中断, 等待重连...", end="", flush=True)
                time.sleep(1.0)
                continue

            frame_top = _read_frame(shm)
            frame_ee = _read_frame(shm_ee) if shm_ee else None

            if frame_top is not None:
                cv2.imwrite(str(episode_dir / f"frame_{frame_count:06d}.png"),
                            frame_top)
            if frame_ee is not None:
                cv2.imwrite(str(episode_dir / f"ee_frame_{frame_count:06d}.png"),
                            frame_ee)

            frames.append({
                "timestamp": round(loop_start - t0, 4),
                "angles": [round(a, 2) for a in angles[:6]],
                "velocities": [round(v, 2) for v in vels[:6]],
                "loads": [round(l, 2) for l in loads[:6]],
            })

            frame_count += 1
            if frame_count % 30 == 0:
                print(".", end="", flush=True)

            elapsed = time.time() - loop_start
            if elapsed < interval:
                time.sleep(interval - elapsed)

    except KeyboardInterrupt:
        print("\n⏹ 用户中断录制")

    finally:
        sim.disconnect()
        if shm:
            try:
                shm.close()
            except Exception:
                pass
        if shm_ee:
            try:
                shm_ee.close()
            except Exception:
                pass

    actual_dur = time.time() - t0
    actual_fps = frame_count / actual_dur if actual_dur > 0 else 0

    meta = {
        "episode_id": episode_id,
        "duration_s": round(actual_dur, 2),
        "frames": frame_count,
        "fps_target": fps,
        "joint_names": JOINT_NAMES,
        "frame_width": FRAME_WIDTH,
        "frame_height": FRAME_HEIGHT,
        "cameras": cameras,
        "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }
    with open(episode_dir / "data.json", "w", encoding="utf-8") as f:
        json.dump({"meta": meta, "frames": frames}, f, indent=2,
                  ensure_ascii=False)

    print(f"\n\n✅ Episode {episode_id:04d} 录制完成")
    print(f"  帧数: {frame_count}  实际: {actual_dur:.1f}s @ {actual_fps:.1f}fps")
    print(f"  相机: {', '.join(cameras)}")
    print(f"  位置: {episode_dir.resolve()}")
    return frame_count


def main() -> None:
    parser = argparse.ArgumentParser(
        description="仿真数据录制 — 原生 TCP + 双路相机"
    )
    parser.add_argument("--duration", type=int, default=20)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--output", default="datasets/sim_v1")
    args = parser.parse_args()
    record_episode(args.duration, args.fps, args.output)


if __name__ == "__main__":
    main()
