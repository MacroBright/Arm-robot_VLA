#!/usr/bin/env python3
"""仿真录制数据 (PNG+JSON) → LeRobot Parquet 格式。

用法:
  python scripts/convert_to_lerobot.py datasets/sim_v1 --output datasets/lerobot_v1

输出 (LeRobot v2 兼容):
  datasets/lerobot_v1/
  ├── data/chunk-000/episode_000000.parquet
  ├── meta/info.json
  └── videos/episode_000000/{cam_top,ee_camera}/

Parquet schema (每行 = 1 帧):
  observation.state      float32[6]   关节角度 (rad)
  observation.ee_pos     float32[3]   末端世界坐标
  observation.target_pos float32[3]   目标球世界坐标
  action                 float32[6]   下一帧关节角度 (行为克隆标签)
  episode_index          int64
  frame_index            int64
  timestamp              float32
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

JOINT_NAMES = [
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_roll", "wrist_flex", "gripper",
]


def convert_episode(ep_dir: Path, out_dir: Path, ep_idx: int) -> int:
    import pyarrow as pa
    import pyarrow.parquet as pq

    dp = ep_dir / "data.json"
    if not dp.exists():
        print(f"  跳过 {ep_dir.name}: 缺少 data.json")
        return 0

    with open(dp) as f:
        data = json.load(f)
    frames = data.get("frames", [])
    n = len(frames)
    if n < 2:
        print(f"  跳过 {ep_dir.name}: 帧数不足 ({n})")
        return 0

    # 解析
    states, ee_p, tgt_p, tss = [], [], [], []
    for fr in frames:
        ang = np.array(fr.get("angles", [0]*6)[:6], dtype=np.float32)
        states.append(np.deg2rad(ang))
        ee_p.append(np.array(fr.get("ee_pos", [0,0,0])[:3], dtype=np.float32))
        tgt_p.append(np.array(fr.get("target_pos", [0,0,0])[:3], dtype=np.float32))
        tss.append(float(fr.get("timestamp", 0)))

    # 动作 = 下一帧关节角度
    actions = states[1:] + [states[-1]]
    n_keep = n - 1
    states, ee_p, tgt_p = states[:n_keep], ee_p[:n_keep], tgt_p[:n_keep]
    actions, tss = actions[:n_keep], tss[:n_keep]

    # 符号链接图像
    for cam, prefix in [("cam_top", "frame"), ("ee_camera", "ee_frame")]:
        cam_dir = out_dir / "videos" / f"episode_{ep_idx:06d}" / cam
        cam_dir.mkdir(parents=True, exist_ok=True)
        for i in range(n_keep):
            src = ep_dir / f"{prefix}_{i:06d}.png"
            dst = cam_dir / f"frame_{i:06d}.png"
            if src.exists() and not dst.exists():
                dst.symlink_to(src.resolve())

    # Parquet
    tbl = pa.table({
        "observation.state": pa.array(
            [s.tolist() for s in states], pa.list_(pa.float32())),
        "observation.ee_pos": pa.array(
            [p.tolist() for p in ee_p], pa.list_(pa.float32())),
        "observation.target_pos": pa.array(
            [p.tolist() for p in tgt_p], pa.list_(pa.float32())),
        "action": pa.array(
            [a.tolist() for a in actions], pa.list_(pa.float32())),
        "episode_index": pa.array([ep_idx]*n_keep, pa.int64()),
        "frame_index": pa.array(list(range(n_keep)), pa.int64()),
        "timestamp": pa.array(tss, pa.float32()),
    })

    chunk = out_dir / "data" / f"chunk-{ep_idx // 1000:03d}"
    chunk.mkdir(parents=True, exist_ok=True)
    pq.write_table(tbl, chunk / f"episode_{ep_idx:06d}.parquet",
                   compression="zstd")
    return n_keep


def main() -> None:
    p = argparse.ArgumentParser(description="数据 → LeRobot Parquet")
    p.add_argument("input", default="datasets/sim_v1")
    p.add_argument("--output", default="datasets/lerobot_v1")
    args = p.parse_args()

    src, out = Path(args.input), Path(args.output)
    if not src.exists():
        print(f"错误: {src} 不存在"); sys.exit(1)

    episodes = sorted(src.glob("episode_*"))
    if not episodes:
        print(f"错误: 未找到 episode_*"); sys.exit(1)

    print(f"输入: {src} ({len(episodes)} episodes)")
    print(f"输出: {out}\n")

    (out / "meta").mkdir(parents=True, exist_ok=True)
    total = 0
    for i, ep in enumerate(episodes):
        n = convert_episode(ep, out, i)
        total += n
        print(f"  Ep {i:04d}: {n} 帧")

    info = {
        "type": "lerobot_v2",
        "robot": "zero_arm_6dof",
        "joint_names": JOINT_NAMES,
        "camera_names": ["cam_top", "ee_camera"],
        "fps": 25,
        "total_episodes": len(episodes),
        "total_frames": total,
        "features": {
            "observation.state": {"shape": [6], "type": "float32"},
            "observation.ee_pos": {"shape": [3], "type": "float32"},
            "observation.target_pos": {"shape": [3], "type": "float32"},
            "action": {"shape": [6], "type": "float32"},
        },
    }
    with open(out / "meta" / "info.json", "w") as f:
        json.dump(info, f, indent=2, ensure_ascii=False)

    print(f"\n✅ {total} 帧, {len(episodes)} episodes → {out.resolve()}")


if __name__ == "__main__":
    main()
