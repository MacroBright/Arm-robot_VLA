"""zdt/recording.py — EpisodeRecorder: 每 episode 一个 JSONL + 相机帧文件 (spec TASK-35).

修订 #5: JSONL 的 observation.camera_frames 引用**真实落盘的帧文件** (相对路径),
非仅时间戳; color/depth 由调用方传入 ndarray, 本模块负责写盘并回填引用.
observation / action 明确分离, 直接服务后续 ACT / Diffusion Policy / SmolVLA.
"""
from __future__ import annotations

import json
import shutil
import time
from pathlib import Path

import numpy as np


class EpisodeRecorder:
    """增量 JSONL 录制: start_episode → add_record×N → finish_episode."""

    def __init__(self, out_dir, save_frames: bool = True,
                 frame_format: str = "png"):
        self.out_dir = Path(out_dir)
        self.save_frames = save_frames
        self.frame_format = frame_format
        self._episode_dir: Path | None = None
        self._frames_dir: Path | None = None
        self._jsonl = None
        self._frame_idx = 0

    def start_episode(self) -> str:
        # P0-③: 纳秒级精度, 同秒启动多个 episode 也不会覆盖
        ep_id = f"episode_{time.strftime('%Y%m%d_%H%M%S')}_{time.time_ns()}"
        self._episode_dir = self.out_dir / ep_id
        if self._episode_dir.exists():
            shutil.rmtree(self._episode_dir)
        self._episode_dir.mkdir(parents=True, exist_ok=True)
        self._frames_dir = self._episode_dir / "frames"
        self._jsonl = open(self._episode_dir / "data.jsonl", "w", encoding="utf-8")
        self._frame_idx = 0
        return ep_id

    def add_record(self, observation: dict, action: dict, safety: dict,
                   color=None, depth=None, camera_ts=None) -> None:
        """写一条 JSONL. color/depth 为 (H,W[,3]) / (H,W) 数组时落盘并回填引用."""
        if self._jsonl is None:
            raise RuntimeError("先调用 start_episode()")
        obs = dict(observation)
        if self.save_frames and (color is not None or depth is not None):
            rel = self._save_frames(color, depth)
            obs["camera_frames"] = rel
            obs["camera_ts"] = float(camera_ts) if camera_ts is not None \
                else time.monotonic()
        row = {
            "timestamp": time.monotonic(),
            "observation": obs,
            "action": dict(action),
            "safety": dict(safety),
        }
        self._jsonl.write(json.dumps(row) + "\n")

    def finish_episode(self) -> dict:
        """落盘 JSONL 并返回统计. 之后可 start_episode 新 episode."""
        if self._jsonl is None or self._episode_dir is None:
            raise RuntimeError("无进行中的 episode")
        self._jsonl.flush()
        with self._episode_dir.joinpath("data.jsonl").open() as f:
            n_records = sum(1 for _ in f)
        self._jsonl.close()
        n_frames = len(list(self._frames_dir.glob(f"*.{self.frame_format}"))) \
            if self._frames_dir is not None and self._frames_dir.exists() else 0
        stats = {"records": n_records, "frames": n_frames,
                 "path": str(self._episode_dir)}
        self._jsonl = None
        return stats

    def _save_frames(self, color, depth) -> dict[str, str]:
        assert self._frames_dir is not None
        rel: dict[str, str] = {}
        idx = self._frame_idx
        if color is not None:
            fn = f"{idx:06d}_color.{self.frame_format}"
            self._write_image(self._frames_dir / fn, color)
            rel["color"] = f"frames/{fn}"
        if depth is not None:
            fn = f"{idx:06d}_depth.{self.frame_format}"
            self._write_image(self._frames_dir / fn, depth)
            rel["depth"] = f"frames/{fn}"
        self._frame_idx += 1
        return rel

    @staticmethod
    def _write_image(path: Path, img) -> None:
        img = np.asarray(img)
        path.parent.mkdir(parents=True, exist_ok=True)
        try:
            import cv2
            cv2.imwrite(str(path), img)
        except ImportError:
            path.write_bytes(img.tobytes())   # 无 cv2 时写裸数据 (测试环境兜底)
