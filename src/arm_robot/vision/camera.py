"""Unified camera source for the LEAP Hand gesture pipeline.

Prefers the official Intel RealSense SDK (pyrealsense2) so the D455's
color stream is selected explicitly — scanning raw V4L2 ports is
unreliable because the D455 re-enumerates its color/depth/IR nodes on
every run. Falls back to an OpenCV port scan if the SDK is unavailable.

Both sources expose the same minimal interface:
    ok, bgr_frame = cam.read()
    cam.release()

Workstream: W1 手势映射 — .claude/workstreams/01-gesture-mapping.md
"""

import time
from typing import Optional, Tuple

import cv2
import numpy as np


class CameraSource:
    """Abstract camera source: read() → (ok, BGR frame), release()."""

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        raise NotImplementedError

    def release(self):
        raise NotImplementedError


class RealSenseSource(CameraSource):
    """D455 异步高帧率相机源 (带非阻塞后台采集守护线程)."""

    def __init__(self, width: int = 640, height: int = 480, fps: int = 30,
                 enable_depth: bool = False):
        import pyrealsense2 as rs
        import threading
        self._rs = rs
        self._pipeline = rs.pipeline()
        self._enable_depth = enable_depth
        
        # 针对多相机并发总线环境，优先采用高稳定性 640x480 @30fps
        started = False
        candidates = [
            (width, height, fps),
            (640, 480, 30),
            (1280, 720, 15),
        ]
        for w_c, h_c, fps_c in candidates:
            try:
                config = rs.config()
                config.enable_stream(rs.stream.color, w_c, h_c, rs.format.bgr8, fps_c)
                if enable_depth:
                    config.enable_stream(rs.stream.depth, w_c, h_c, rs.format.z16, fps_c)
                self._profile = self._pipeline.start(config)
                self._width, self._height, self._fps = w_c, h_c, fps_c
                started = True
                break
            except Exception:
                continue

        if not started:
            raise RuntimeError("Failed to start RealSense pipeline with any candidate stream profile")

        self._align = rs.align(rs.stream.color) if enable_depth else None
        self._intrinsics = None
        try:
            intr = (self._profile.get_stream(rs.stream.color)
                    .as_video_stream_profile().get_intrinsics())
            self._intrinsics = (intr.fx, intr.fy, intr.ppx, intr.ppy)
        except Exception:
            self._intrinsics = None

        # 启动非阻塞高帧率取帧守护线程
        self._running = True
        self._lock = threading.Lock()
        self._latest_bgr: Optional[np.ndarray] = None
        self._latest_depth: Optional[np.ndarray] = None
        self._thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._thread.start()

    def _worker_loop(self):
        while self._running:
            try:
                frames = self._pipeline.wait_for_frames(timeout_ms=100)
                if frames is not None:
                    color = frames.get_color_frame()
                    if color:
                        bgr = np.asanyarray(color.get_data()).copy()
                        depth_mm = None
                        if self._enable_depth and self._align:
                            aligned = self._align.process(frames)
                            d = aligned.get_depth_frame()
                            if d:
                                depth_mm = np.asanyarray(d.get_data()).copy()
                        with self._lock:
                            self._latest_bgr = bgr
                            self._latest_depth = depth_mm
            except Exception:
                time.sleep(0.005)

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        with self._lock:
            if self._latest_bgr is not None:
                return True, self._latest_bgr.copy()
            return False, None

    def read_with_depth(self) -> Tuple[bool, Optional[np.ndarray],
                                       Optional[np.ndarray], Optional[tuple]]:
        """返回 (ok, bgr, depth_mm, intrinsics)。depth 已与 color 对齐 (u16 mm)。"""
        with self._lock:
            if self._latest_bgr is not None:
                return True, self._latest_bgr.copy(), (self._latest_depth.copy() if self._latest_depth is not None else None), self._intrinsics
            return False, None, None, None

    def intrinsics(self):
        return self._intrinsics

    def release(self):
        self._running = False
        if hasattr(self, "_thread") and self._thread.is_alive():
            self._thread.join(0.2)
        try:
            self._pipeline.stop()
        except Exception:
            pass


def open_realsense(width: int = 640, height: int = 480, fps: int = 30,
                   enable_depth: bool = False) -> Optional[CameraSource]:
    """Open the RealSense color stream (pure Color 640x480 @30fps for ultra-fast, zero-lag teleop)."""
    try:
        import pyrealsense2 as rs
    except ImportError:
        return None

    try:
        ctx = rs.context()
        if not any(ctx.query_devices()):
            return None
        cam = RealSenseSource(width=width, height=height, fps=fps, enable_depth=enable_depth)
        # 热身循环 (等待 RealSense AE 与传感器硬件管道稳定输出)
        for _ in range(20):
            ok, frame = cam.read()
            if ok and frame is not None:
                return cam
            time.sleep(0.05)
        return cam
    except Exception:
        return None
