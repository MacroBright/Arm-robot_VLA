#!/usr/bin/env python3
"""离屏相机渲染子进程 — 独立进程隔离 GL context, 避免与 viewer 冲突。

通过 TCP 读取关节状态, 渲染 cam_top + ee_camera 到 SharedMemory,
供 record_sim.py 录制和 camera_display 显示。

用法 (由 mujoco_sim.py 自动启动):
  .venv/bin/python3 scripts/camera_server.py --port 5555
"""

import argparse
import math
import socket
import struct
import sys
import time
from multiprocessing import shared_memory

import numpy as np

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
SHM_NAME = "mujoco_frame_0"
SHM_NAME_EE = "mujoco_frame_ee"
SHM_HEADER_SIZE = 64
NUM_JOINTS = 6


def log(msg: str) -> None:
    print(f"[camera:{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def connect_sim(host: str, port: int, retries: int = 30) -> socket.socket | None:
    """连接仿真 TCP 服务。"""
    for i in range(retries):
        try:
            sock = socket.create_connection((host, port), timeout=2.0)
            sock.settimeout(0.5)
            sock.sendall(b"get_state\n")
            buf = b""
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                try:
                    buf += sock.recv(4096)
                except socket.timeout:
                    continue
                if b"\n" in buf:
                    line = buf.split(b"\n")[0].decode()
                    if line.startswith("STATE:"):
                        log(f"已连接仿真 {host}:{port}")
                        return sock
            sock.close()
        except (ConnectionRefusedError, OSError):
            pass
        if i == 0:
            log(f"等待仿真启动 ({host}:{port})...")
        time.sleep(1.0)
    return None


def read_state(sock: socket.socket) -> list[float] | None:
    """读取关节角度, 返回弧度制列表 (6项)。"""
    try:
        sock.sendall(b"get_state\n")
        buf = b""
        while b"\n" not in buf:
            chunk = sock.recv(4096)
            if not chunk:
                return None
            buf += chunk
        line = buf.split(b"\n")[0].decode("ascii", errors="replace").strip()
        if line.startswith("STATE:"):
            vals = [float(v) for v in line[6:].split(",")]
            return [math.radians(v) for v in vals[:NUM_JOINTS]]
    except (socket.timeout, OSError):
        pass
    return None


def _read_target(sock: socket.socket) -> list[float] | None:
    """读取目标球位置 (mocap 同步用)。"""
    try:
        sock.sendall(b"target_pos\n")
        buf = b""
        while b"\n" not in buf:
            chunk = sock.recv(4096)
            if not chunk:
                return None
            buf += chunk
        line = buf.split(b"\n")[0].decode("ascii", errors="replace").strip()
        if line.startswith("TARGET:"):
            # TARGET:x,y,z,dist=N,model=name
            parts = line[7:].split(",")
            return [float(parts[0]), float(parts[1]), float(parts[2])]
    except (socket.timeout, OSError):
        pass
    return None


def render_fixed_camera(gl_ctx, mjr_ctx, scene, opt, viewport, rgb_buf,
                        cam_id, model, data) -> np.ndarray | None:
    """渲染固定相机 → BGR (H,W,3) uint8。"""
    import mujoco
    gl_ctx.make_current()
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
    cam.fixedcamid = cam_id
    mujoco.mjv_updateScene(model, data, opt, None, cam,
                           mujoco.mjtCatBit.mjCAT_ALL, scene)
    mujoco.mjr_render(viewport, scene, mjr_ctx)
    mujoco.mjr_readPixels(rgb_buf, None, viewport, mjr_ctx)
    return np.flipud(rgb_buf)[..., ::-1]


def main() -> None:
    import mujoco

    parser = argparse.ArgumentParser(description="离屏相机渲染子进程")
    parser.add_argument("--port", type=int, default=5555)
    parser.add_argument("--scene", type=str,
                        default="scripts/mujoco_scene/scene.xml")
    args = parser.parse_args()

    # ── 加载模型 ──
    log(f"加载场景: {args.scene}")
    model = mujoco.MjModel.from_xml_path(args.scene)
    data = mujoco.MjData(model)
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "soft_reset")
    if key_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, key_id)

    # 获取相机 ID
    cam_top_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_CAMERA, "cam_top")
    cam_ee_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_CAMERA, "ee_camera")
    if cam_top_id < 0 or cam_ee_id < 0:
        log(f"错误: 场景缺少相机 cam_top={cam_top_id} ee_camera={cam_ee_id}")
        sys.exit(1)

    # 获取目标球 mocap ID (同步用)
    target_body_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "target_ball")
    target_mocap_id = model.body_mocapid[target_body_id] \
        if target_body_id >= 0 else -1

    # ── 离屏渲染器 (本进程唯一 GL context, 无冲突) ──
    log("创建离屏渲染器...")
    renderer = mujoco.Renderer(model, FRAME_HEIGHT, FRAME_WIDTH)
    gl_ctx = renderer._gl_context
    mjr_ctx = renderer._mjr_context
    scene = renderer._scene
    opt = renderer._scene_option
    viewport = mujoco.MjrRect(0, 0, FRAME_WIDTH, FRAME_HEIGHT)
    rgb_buf = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)

    # ── 打开共享内存 (由 mujoco_sim.py 创建) ──
    for attempt in range(10):
        try:
            shm = shared_memory.SharedMemory(name=SHM_NAME)
            shm_ee = shared_memory.SharedMemory(name=SHM_NAME_EE)
            break
        except FileNotFoundError:
            if attempt == 0:
                log(f"等待共享内存 {SHM_NAME}...")
            time.sleep(0.5)
    else:
        log(f"错误: 共享内存不存在, 请先启动 mujoco_sim.py")
        sys.exit(1)
    log(f"共享内存已打开: {SHM_NAME}, {SHM_NAME_EE}")

    # ── 连接仿真 ──
    sock = connect_sim("127.0.0.1", args.port)
    if sock is None:
        log("错误: 无法连接仿真")
        sys.exit(1)

    frame_count = 0
    log("开始渲染循环 (50Hz)")

    try:
        while True:
            t_start = time.monotonic()

            angles = read_state(sock)
            if angles is None or len(angles) < NUM_JOINTS:
                time.sleep(0.005)
                continue

            data.qpos[:NUM_JOINTS] = angles

            # 同步目标球 mocap 位置 (从 TCP 读取, 否则画面不同步)
            target = _read_target(sock)
            if target is not None:
                data.mocap_pos[target_mocap_id] = target
                data.mocap_quat[target_mocap_id] = [1.0, 0.0, 0.0, 0.0]

            mujoco.mj_forward(model, data)

            # cam_top
            top = render_fixed_camera(gl_ctx, mjr_ctx, scene, opt, viewport,
                                      rgb_buf, cam_top_id, model, data)
            if top is not None:
                shm.buf[SHM_HEADER_SIZE:] = top.tobytes()

            # ee_camera
            ee = render_fixed_camera(gl_ctx, mjr_ctx, scene, opt, viewport,
                                     rgb_buf, cam_ee_id, model, data)
            if ee is not None:
                shm_ee.buf[SHM_HEADER_SIZE:] = ee.tobytes()

            header = struct.pack("d", time.time()) + \
                     struct.pack("q", frame_count) + \
                     struct.pack("i", FRAME_WIDTH) + \
                     struct.pack("i", FRAME_HEIGHT) + \
                     b"\x00" * (SHM_HEADER_SIZE - 24)
            shm.buf[:SHM_HEADER_SIZE] = header
            shm_ee.buf[:SHM_HEADER_SIZE] = header
            frame_count += 1

            elapsed = time.monotonic() - t_start
            if elapsed < 0.02:
                time.sleep(0.02 - elapsed)

    except KeyboardInterrupt:
        pass
    finally:
        renderer.close()
        for s in (shm, shm_ee):
            try:
                s.close()
                s.unlink()
            except Exception:
                pass
        sock.close()
        log("相机渲染进程已退出")


if __name__ == "__main__":
    main()
