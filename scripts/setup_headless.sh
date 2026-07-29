#!/bin/bash
# Zero Arm VLA — 远程 Headless 服务器渲染修复
# 在 3090 服务器上运行 (需要 sudo 权限安装 OSMESA)
set -e

echo "=== [1/4] 安装 OSMESA 离屏渲染库 ==="
if ! dpkg -l 2>/dev/null | grep -q "libosmesa6 "; then
    sudo apt-get update -qq
    sudo apt-get install -y -qq libosmesa6-dev libosmesa6 libgl1-mesa-dev libegl1-mesa-dev
    echo "OSMESA 已安装"
else
    echo "OSMESA 已存在"
fi

echo ""
echo "=== [2/4] 验证 OSMESA 库 ==="
ldconfig -p 2>/dev/null | grep -i osmesa || echo "⚠ 未找到 libOSMesa，可能需要 sudo ldconfig"
echo "OK"

echo ""
echo "=== [3/4] 测试 MuJoCo OSMESA 渲染 ==="
cd "$(dirname "$0")/.."
python3 -c "
import os
os.environ['MUJOCO_GL'] = 'osmesa'
import mujoco
from mujoco import Renderer
import numpy as np

# 用我们的场景文件测试
model = mujoco.MjModel.from_xml_path('scripts/mujoco_scene/scene.xml')
data = mujoco.MjData(model)
renderer = Renderer(model, 480, 640)
mujoco.mj_step(model, data)
renderer.update_scene(data, camera='cam_top')
pixels = renderer.render()
print(f'OSMESA 渲染成功: {pixels.shape}, mean={pixels.mean():.3f}')
renderer.close()
" && echo "✓ MuJoCo OSMESA 正常工作" || echo "✗ MuJoCo OSMESA 失败"

echo ""
echo "=== [4/4] 后续步骤 ==="
echo "现在可以运行:"
echo "  # 终端1: 启动仿真 (OSMESA 离屏渲染)"
echo "  MUJOCO_GL=osmesa python scripts/mujoco_sim.py --ik --camera-gl osmesa &"
echo ""
echo "  # 终端2: 评估"
echo "  sleep 3 && HF_HUB_OFFLINE=1 python scripts/evaluate_policy.py --episodes 30"
