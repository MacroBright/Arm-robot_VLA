# SmolVLA 推理评估 — 大显存机器部署指引

## 硬件要求

| 需求 | 最低 | 推荐 |
|------|------|------|
| GPU 显存 | **8 GB** | 12+ GB |
| RAM | 8 GB | 16 GB |
| 磁盘 | 3.5 GB | 10 GB |

## 迁移文件清单

```
需复制到目标机器:
  outputs/smolvla_massage/          ← 训练产出 (3 GB)
  scripts/data_tools/evaluate_policy.py        ← 评估脚本
  scripts/simulation/mujoco_sim.py             ← 仿真主程序
  scripts/mujoco_scene/scene.xml    ← 场景模型
  scripts/simulation/camera_server.py          ← 相机渲染
  scripts/simulation/shm_util.py               ← 共享内存工具 (如果存在)
  datasets/lerobot_v1/videos/       ← 视频文件 (如果也要跑完整仿真)
```

**最快方式: 复制整个项目**

```bash
# 本机打包
cd /home/bright/office/Arm-robot_VLA
tar czf arm_vla.tar.gz outputs/ scripts/ datasets/lerobot_v1/videos/

# 传到目标机器
scp arm_vla.tar.gz user@target-machine:/path/to/

# 目标机器解压
tar xzf arm_vla.tar.gz
```

## 目标机器操作

### 1. 安装依赖

```bash
pip install torch mujoco numpy opencv-python safetensors transformers \
            lerobot[smolvla] pyarrow av Pillow
```

### 2. 验证模型能加载（不连仿真）

```bash
python -c "
from lerobot.policies.smolvla.configuration_smolvla import SmolVLAConfig
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
from safetensors.torch import load_file

ckpt = 'outputs/smolvla_massage/checkpoints/030000/pretrained_model'
config = SmolVLAConfig(**__import__('json').load(open(f'{ckpt}/config.json')))
config.type = None  # filter non-field
policy = SmolVLAPolicy(config).cuda().eval()
state = load_file(f'{ckpt}/model.safetensors')
policy.load_state_dict(state, strict=False)
print(f'OK: {sum(p.numel() for p in policy.parameters()):,} params')
"
```

### 3. 启动仿真 + 评估

```bash
# 终端 1: 启动仿真
cd /path/to/Arm-robot_VLA
MUJOCO_GL=glfw python scripts/simulation/mujoco_sim.py --ik --trail 500 --viewer

# 终端 2: 等仿真就绪后运行评估
python scripts/data_tools/evaluate_policy.py \
    --checkpoint outputs/smolvla_massage/checkpoints/last \
    --episodes 30
```

### 4. 预期输出

```
Ep   1/30: ✓ HIT (127 steps)    ← 模型控制机械臂碰到球
Ep   2/30: ✗ MISS (300 steps)   ← 超时未碰到
...
成功率: 18/30 (60%)
```

## 训练结果摘要

| 指标 | 值 |
|------|-----|
| 模型 | SmolVLA-450M |
| 数据 | 9 episodes, 114k 帧 |
| 训练 | 30,000 steps, 1h 24min |
| Loss | 0.041 |
| Checkpoint | `outputs/smolvla_massage/checkpoints/030000/` |
