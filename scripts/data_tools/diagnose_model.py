#!/usr/bin/env python3
"""诊断脚本：检查 SmolVLA 模型实际输出的动作值。"""
import json, sys
from pathlib import Path
import numpy as np
import torch
from transformers import AutoTokenizer

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from lerobot.policies.smolvla.configuration_smolvla import SmolVLAConfig
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
from lerobot.configs.types import PolicyFeature, FeatureType
from safetensors.torch import load_file

CKPT = Path("outputs/smolvla_massage/checkpoints/last/pretrained_model")

def load_model():
    with open(CKPT / "config.json") as f:
        cfg_dict = json.load(f)
    cfg_dict.pop("type", None)
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
    print(f"  chunk_size={config.chunk_size}, max_action_dim={config.max_action_dim}")
    print(f"  normalization_mapping: {config.normalization_mapping}")
    print(f"  resize_imgs_with_padding: {config.resize_imgs_with_padding}")

    policy = SmolVLAPolicy(config)
    state = load_file(str(CKPT / "model.safetensors"))
    missing, unexpected = policy.load_state_dict(state, strict=False)
    print(f"  missing={len(missing)}, unexpected={len(unexpected)}")
    policy.eval()
    device = torch.device("cpu")
    policy = policy.to(device)
    tokenizer = AutoTokenizer.from_pretrained(
        config.vlm_model_name, padding="max_length", truncation=True,
        max_length=config.tokenizer_max_length, local_files_only=True)
    return policy, device, tokenizer

def main():
    print("[1] Loading model...")
    policy, device, tokenizer = load_model()

    print("\n[2] Test 1: Fixed angles, random images")
    for angles_deg in ([0, 0, 0, 0, 0, 0], [10, -10, 20, 5, 0, 0], [45, -30, 60, 15, -5, 5]):
        angles_rad = np.deg2rad(angles_deg).astype(np.float32)
        dummy_img = torch.rand(3, 480, 640)
        task_str = "Reach the target red ball with the robot arm end-effector."
        tok = tokenizer(task_str, return_tensors="pt", padding="max_length", truncation=True)
        batch = {
            "observation.state": torch.from_numpy(angles_rad).float().unsqueeze(0),
            "observation.images.cam_top": dummy_img.unsqueeze(0),
            "observation.images.ee_camera": dummy_img.unsqueeze(0),
            "observation.language.tokens": tok["input_ids"],
            "observation.language.attention_mask": tok["attention_mask"].bool(),
            "task": [task_str],
        }
        with torch.no_grad():
            ac = policy.predict_action_chunk(batch)
        a0_deg = np.rad2deg(ac[0, 0].numpy())
        a_stats = f"min={ac.min().item():.4f}, max={ac.max().item():.4f}, mean={ac.mean().item():.4f}"
        print(f"  Input(deg)={angles_deg}")
        print(f"  -> Action[0,0](deg)={np.round(a0_deg, 2)}  [{a_stats}]")

    print("\n[3] Test 2: Different random images, same angles")
    angles_rad = np.deg2rad([10, -20, 30, 10, 0, 0]).astype(np.float32)
    for seed in range(3):
        torch.manual_seed(seed)
        img1 = torch.rand(3, 480, 640)
        img2 = torch.rand(3, 480, 640)
        task_str = "Reach the target red ball with the robot arm end-effector."
        tok = tokenizer(task_str, return_tensors="pt", padding="max_length", truncation=True)
        batch = {
            "observation.state": torch.from_numpy(angles_rad).float().unsqueeze(0),
            "observation.images.cam_top": img1.unsqueeze(0),
            "observation.images.ee_camera": img2.unsqueeze(0),
            "observation.language.tokens": tok["input_ids"],
            "observation.language.attention_mask": tok["attention_mask"].bool(),
            "task": [task_str],
        }
        with torch.no_grad():
            ac = policy.predict_action_chunk(batch)
        a0_deg = np.rad2deg(ac[0, 0].numpy())
        print(f"  Seed {seed}: Action[0,0](deg)={np.round(a0_deg, 2)}")

    print("\n[4] Check: Does model respond to different state inputs?")
    for angles_deg in ([0]*6, [90]*6, [-90]*6):
        angles_rad = np.deg2rad(angles_deg).astype(np.float32)
        dummy_img = torch.rand(3, 480, 640)
        task_str = "Reach the target red ball with the robot arm end-effector."
        tok = tokenizer(task_str, return_tensors="pt", padding="max_length", truncation=True)
        batch = {
            "observation.state": torch.from_numpy(angles_rad).float().unsqueeze(0),
            "observation.images.cam_top": dummy_img.unsqueeze(0),
            "observation.images.ee_camera": dummy_img.unsqueeze(0),
            "observation.language.tokens": tok["input_ids"],
            "observation.language.attention_mask": tok["attention_mask"].bool(),
            "task": [task_str],
        }
        with torch.no_grad():
            ac = policy.predict_action_chunk(batch)
        a0_deg = np.rad2deg(ac[0, 0].numpy())
        print(f"  Input(deg)={angles_deg} -> Action[0,0](deg)={np.round(a0_deg, 2)}")

    print("\nDone.")

if __name__ == "__main__":
    main()
