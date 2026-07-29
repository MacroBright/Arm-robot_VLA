#!/bin/bash
# Zero Arm VLA — 远程 3090 一键部署 + Headless 渲染修复
set -e
HOST="zhuyan@218.194.55.32"
PORT="12345"
LOCAL="/home/bright/office/Arm-robot_VLA"
REMOTE_DIR="/home/zhuyan/Arm-robot_VLA"

rsync_cmd() { rsync -avz --progress -e "ssh -p $PORT" "$@"; }
ssh_cmd()  { ssh -p "$PORT" "$HOST" "$@"; }

echo "=== [1/5] 同步代码 (含 headless 修复) ==="
rsync_cmd --exclude='.venv' --exclude='.deps' --exclude='datasets' \
          --exclude='outputs' --exclude='__pycache__' --exclude='.git' \
          --exclude='backups' "$LOCAL/" "$HOST:$REMOTE_DIR/"

echo "=== [2/5] 同步模型 (3GB) ==="
ssh_cmd "mkdir -p $REMOTE_DIR/outputs"
rsync_cmd "$LOCAL/outputs/smolvla_massage/" "$HOST:$REMOTE_DIR/outputs/smolvla_massage/"

echo "=== [3/5] 安装 OSMESA 离屏渲染 ==="
ssh_cmd "bash $REMOTE_DIR/scripts/setup_headless.sh"

echo "=== [4/5] 安装 Python 依赖 ==="
ssh_cmd "cd $REMOTE_DIR && python3 -m pip install --quiet mujoco opencv-python safetensors transformers av Pillow pyarrow torch lerobot 2>&1 | tail -5"

echo "=== [5/5] 验证模型 ==="
ssh_cmd "cd $REMOTE_DIR && python3 -c '
import json, torch
from lerobot.policies.smolvla.configuration_smolvla import SmolVLAConfig
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
from safetensors.torch import load_file
ckpt = \"outputs/smolvla_massage/checkpoints/030000/pretrained_model\"
with open(f\"{ckpt}/config.json\") as f: cfg = json.load(f)
cfg.pop(\"type\",None)
config = SmolVLAConfig(**cfg)
policy = SmolVLAPolicy(config).cuda().eval()
state = load_file(f\"{ckpt}/model.safetensors\")
policy.load_state_dict(state, strict=False)
print(f\"OK: {sum(p.numel() for p in policy.parameters()):,} params\")
'"

echo ""
echo "=== 部署完成 ==="
echo "在远程服务器上运行评估:"
echo ""
echo "  ssh -p $PORT $HOST"
echo "  cd $REMOTE_DIR"
echo "  conda activate arm_vla"
echo ""
echo "  # 终端1: 仿真 (OSMESA headless)"
echo "  MUJOCO_GL=osmesa python scripts/mujoco_sim.py --ik --camera-gl osmesa &"
echo ""
echo "  # 终端2: 评估"
echo "  sleep 3 && HF_HUB_OFFLINE=1 python scripts/evaluate_policy.py --episodes 30"
