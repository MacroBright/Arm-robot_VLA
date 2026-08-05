#!/usr/bin/env python3
"""评估训练好的 SmolVLA 模型在 MuJoCo 仿真中的表现。

读取观测 → 模型预测动作 → 发送关节指令 → 统计触碰成功率。

用法:
  # 先启动仿真 (另一个终端):
  bash scripts/startup.sh
  
  # 然后运行评估:
  python scripts/evaluate_policy.py --checkpoint outputs/smolvla_massage/checkpoints/last
"""

import argparse, math, os, socket, sys, time
from contextlib import nullcontext
from multiprocessing import shared_memory
from pathlib import Path

import numpy as np
import torch

FRAME_W, FRAME_H = 640, 480
SHM_NAME = "mujoco_frame_0"
SHM_NAME_EE = "mujoco_frame_ee"
SHM_HEADER = 64
NUM_JOINTS = 6

JOINT_NAMES = ["J1","J2","J3","J4","J5","J6"]


class SimClient:
    """TCP 客户端 — 读取状态、发送关节指令。"""
    def __init__(self, host="127.0.0.1", port=5555):
        self._addr = (host, port)
        self._sock = None
        self._buf = b""

    def connect(self):
        for i in range(30):
            try:
                self._sock = socket.create_connection(self._addr, timeout=2.0)
                self._sock.settimeout(0.5)
                self._send("get_state"); self._recv("STATE:", 1.0)
                self._send("remote_enable")
                print(f"  已连接仿真 {self._addr[0]}:{self._addr[1]}")
                return True
            except Exception:
                pass
            if i == 0: print("  等待仿真...", end="", flush=True)
            print(".", end="", flush=True); time.sleep(1.0)
        print(" 失败"); return False

    def get_state(self):
        try:
            self._send("get_state")
            resp = self._recv("STATE:", 0.3)
            if resp is None: return None
            vals = [float(v) for v in resp[6:].split(",")]
            n = len(vals) // 3
            return (np.array(vals[:n]), np.array(vals[n:2*n]), np.array(vals[2*n:3*n]))
        except Exception:
            return None

    def get_target(self):
        try:
            self._send("target_pos")
            resp = self._recv("TARGET:", 0.3)
            if resp is None: return None
            parts = resp[7:].split(",")
            return np.array([float(parts[0]), float(parts[1]), float(parts[2])])
        except Exception:
            return None

    def get_ee(self):
        try:
            self._send("get_ee")
            resp = self._recv("EE:", 0.3)
            if resp is None: return None
            vals = [float(v) for v in resp[3:].split(",")]
            return np.array(vals[:3]), np.array(vals[3:6])
        except Exception:
            return None

    def send_joints(self, angles_deg):
        cmd = "set_joints " + " ".join(f"{a:.2f}" for a in angles_deg)
        self._send(cmd)

    def _send(self, cmd): self._sock.sendall((cmd + "\n").encode())
    def _recv(self, prefix, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            while b"\n" in self._buf:
                line, self._buf = self._buf.split(b"\n", 1)
                t = line.decode(errors="replace").strip()
                if t.startswith(prefix): return t
            try:
                c = self._sock.recv(4096)
                if not c: return None
                self._buf += c
            except socket.timeout: continue
        return None


def read_frame(shm_path: str) -> np.ndarray:
    """从 mmap 文件读取相机帧 → (3, H, W) float32 [0,1]"""
    import mmap, os
    fd = os.open(shm_path, os.O_RDONLY)
    buf = mmap.mmap(fd, SHM_HEADER + FRAME_W * FRAME_H * 3,
                     access=mmap.ACCESS_READ)
    os.close(fd)
    raw = buf[SHM_HEADER:SHM_HEADER + FRAME_W * FRAME_H * 3]
    bgr = np.ndarray((FRAME_H, FRAME_W, 3), dtype=np.uint8, buffer=raw).copy()
    buf.close()
    rgb = bgr[..., ::-1].copy()
    return torch.from_numpy(rgb).permute(2, 0, 1).float() / 255.0


def load_model(checkpoint_dir: str):
    """加载训练好的 SmolVLA 模型。"""
    from lerobot.policies.smolvla.configuration_smolvla import SmolVLAConfig
    from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
    from safetensors.torch import load_file
    import json

    ckpt_path = Path(checkpoint_dir) / "pretrained_model"
    # 加载配置 (过滤非字段的 key, 如 'type')
    with open(ckpt_path / "config.json") as f:
        cfg_dict = json.load(f)
    cfg_dict.pop("type", None)

    # 修复 input_features/output_features: JSON 反序列化后 type 是字符串, 需转枚举
    from lerobot.configs.types import PolicyFeature, FeatureType
    for ft_key in ("input_features", "output_features"):
        if ft_key in cfg_dict and cfg_dict[ft_key]:
            fixed = {}
            for k, v in cfg_dict[ft_key].items():
                if isinstance(v, dict):
                    if isinstance(v.get("type"), str):
                        v["type"] = FeatureType[v["type"]]
                    fixed[k] = PolicyFeature(**v)
                else:
                    fixed[k] = v
            cfg_dict[ft_key] = fixed

    config = SmolVLAConfig(**cfg_dict)

    # 还原默认图像尺寸 (推理时需 ≥8GB 显存)
    # config.resize_imgs_with_padding = (512, 512)  # 默认值

    # 创建模型并加载权重
    policy = SmolVLAPolicy(config)
    state = load_file(str(ckpt_path / "model.safetensors"))
    missing, unexpected = policy.load_state_dict(state, strict=False)
    if missing:
        print(f"  Missing keys: {len(missing)}")

    policy.eval()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model_dtype = torch.float32
    if device.type == "cuda":
        model_dtype = torch.float16
        policy = policy.to(device, dtype=model_dtype)
    else:
        policy = policy.to(device)
    print(f"  模型已加载: {sum(p.numel() for p in policy.parameters()):,} params "
          f"({model_dtype})")

    # 加载 tokenizer (用于生成 language tokens)
    from transformers import AutoTokenizer
    tokenizer = AutoTokenizer.from_pretrained(
        config.vlm_model_name,
        padding="max_length", truncation=True,
        max_length=config.tokenizer_max_length)
    print(f"  Tokenizer 已加载 (max_length={config.tokenizer_max_length})")
    return policy, device, tokenizer


def run_episode(policy, device, sim, shm_path, shm_ee_path, tokenizer,
                model_dtype, max_steps=500, fps=10, debug=False):
    """运行单次评估 episode — 返回是否触碰目标球。"""
    # 读取初始状态
    state = sim.get_state()
    if state is None: return False, 0, None
    angles, _, _ = state
    interval = 1.0 / fps
    touched = False
    debug_log = [] if debug else None

    for step in range(max_steps):
        t0 = time.monotonic()

        # 读取观测
        state = sim.get_state()
        if state is None: break
        angles_deg, vels_deg, _ = state
        angles_rad = np.deg2rad(angles_deg[:6])

        god = read_frame(shm_path)
        ee = read_frame(shm_ee_path)

        # 读取目标球位置
        target = sim.get_target()
        if target is None: break
        # 检测触碰
        ee_pos, ee_rot = sim.get_ee() or (None, None)
        if ee_pos is not None:
            dist = float(np.linalg.norm(ee_pos - target))
        else:
            dist = -1.0
        if ee_pos is not None and dist < 0.04:
            touched = True
            if debug:
                debug_log.append({
                    "step": step, "action_deg": action_deg.tolist(),
                    "current_deg": angles_deg[:6].tolist(), "ee_pos": ee_pos.tolist(),
                    "target": target.tolist(), "dist": dist,
                })
            break

        # 构建 batch (含 language tokens)
        task_str = "Reach the target red ball with the robot arm end-effector."
        tok = tokenizer(task_str, return_tensors="pt",
                        padding="max_length", truncation=True)
        batch = {
            "observation.state":
                torch.from_numpy(angles_rad).float().unsqueeze(0).to(device, model_dtype),
            "observation.images.cam_top": god.unsqueeze(0).to(device, model_dtype),
            "observation.images.ee_camera": ee.unsqueeze(0).to(device, model_dtype),
            "observation.language.tokens":
                tok["input_ids"].to(device),
            "observation.language.attention_mask":
                tok["attention_mask"].to(device).bool(),
            "task": [task_str],
        }

        # 模型预测 (用标准推理 API)
        with torch.no_grad():
            with torch.amp.autocast("cuda") if device.type == "cuda" else nullcontext():
                action_chunk = policy.predict_action_chunk(batch)
                action = action_chunk[0, 0].cpu().numpy()  # [batch, chunk, dim] → [dim]

        # 发送关节指令 (弧度 → 度)
        action_deg = np.rad2deg(action)
        action_deg = np.clip(action_deg, -180, 180)

        # 诊断日志: 记录前10步、每50步、最后10步
        if debug and (step < 10 or step % 50 == 0 or step >= max_steps - 10):
            debug_log.append({
                "step": step, "action_deg": action_deg.tolist(),
                "current_deg": angles_deg[:6].tolist(),
                "ee_pos": ee_pos.tolist() if ee_pos is not None else None,
                "target": target.tolist(), "dist": dist,
            })

        sim.send_joints(action_deg)

        # 限速
        elapsed = time.monotonic() - t0
        if elapsed < interval:
            time.sleep(interval - elapsed)

    return touched, step + 1 if not touched else step + 1, debug_log


def main():
    parser = argparse.ArgumentParser(description="SmolVLA 仿真评估")
    parser.add_argument("--checkpoint", type=str,
                        default="outputs/smolvla_massage/checkpoints/last")
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--max-steps", type=int, default=300)
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--debug", action="store_true",
                        help="打印诊断日志 (action值、关节角、末端距离)")
    args = parser.parse_args()

    print("=" * 60)
    print("SmolVLA 仿真评估")
    print(f"  Checkpoint: {args.checkpoint}")
    print(f"  Episodes: {args.episodes}")
    print("=" * 60)

    # 加载模型
    print("\n[1] 加载模型...")
    policy, device, tokenizer = load_model(args.checkpoint)
    model_dtype = next(policy.parameters()).dtype

    # 连接仿真
    print("[2] 连接仿真...")
    sim = SimClient()
    if not sim.connect():
        print("错误: 无法连接仿真。请先运行 bash scripts/startup.sh")
        sys.exit(1)

    # 打开共享内存 (用 mmap 直读, 避免 Python resource_tracker unlink 问题)
    print("[3] 打开相机帧...")
    shm_path = f"/dev/shm/{SHM_NAME}"
    shm_ee_path = f"/dev/shm/{SHM_NAME_EE}"
    for attempt in range(20):
        if os.path.exists(shm_path) and os.path.exists(shm_ee_path):
            break
        if attempt == 0:
            print(f"  等待共享内存...", end="", flush=True)
        print(".", end="", flush=True)
        time.sleep(0.3)
    else:
        print(f"\n错误: 共享内存文件不存在 ({shm_path})")
        print("请确认 mujoco_sim.py 和 camera_server.py 都在运行")
        sys.exit(1)
    print(" OK")

    # 评估
    print(f"\n[4] 开始评估 ({args.episodes} episodes)...\n")
    successes = 0
    total_steps = 0
    for ep in range(args.episodes):
        # reset target
        sim._send("target_reset")
        time.sleep(0.5)

        touched, steps, debug_log = run_episode(
            policy, device, sim, shm_path, shm_ee_path, tokenizer,
            model_dtype, args.max_steps, args.fps, debug=args.debug)
        successes += int(touched)
        total_steps += steps

        status = "✓ HIT" if touched else "✗ MISS"
        print(f"  Ep {ep+1:>3}/{args.episodes}: {status} ({steps} steps)")

        # 诊断输出
        if debug_log:
            print(f"    --- 诊断日志 (ep {ep+1}) ---")
            for entry in debug_log:
                ad = entry["action_deg"]
                cd = entry["current_deg"]
                print(f"    step={entry['step']:>3d}  "
                      f"action=[{ad[0]:7.1f} {ad[1]:7.1f} {ad[2]:7.1f} "
                      f"{ad[3]:7.1f} {ad[4]:7.1f} {ad[5]:7.1f}]  "
                      f"cur=[{cd[0]:7.1f} {cd[1]:7.1f} {cd[2]:7.1f} "
                      f"{cd[3]:7.1f} {cd[4]:7.1f} {cd[5]:7.1f}]  "
                      f"dist={entry['dist']:.3f}")
                if entry["ee_pos"]:
                    print(f"           "
                          f"ee_pos=({entry['ee_pos'][0]:.3f},{entry['ee_pos'][1]:.3f},{entry['ee_pos'][2]:.3f})  "
                          f"target=({entry['target'][0]:.3f},{entry['target'][1]:.3f},{entry['target'][2]:.3f})")
            print(f"    --- 诊断结束 ---")

    success_rate = successes / args.episodes * 100
    avg_steps = total_steps / args.episodes
    print(f"\n{'='*60}")
    print(f"  成功率: {successes}/{args.episodes} ({success_rate:.0f}%)")
    print(f"  平均步数: {avg_steps:.0f}")
    print(f"{'='*60}")

    print("评估结束")


if __name__ == "__main__":
    main()
