# Arm-robot_VLA — 部署、调试与具身训练全流程指南 (DEPLOYMENT)

> **适用硬件**：6-DOF 机械臂 + Emm42 V5.0 CAN 步进闭环电机 ×6 (PC 直连 SocketCAN can0 @ 500kbps, ADR-005)
> **软件环境**：Ubuntu 22.04/24.04/26.04 | Conda `smolvla` (Python 3.10) | LeRobot 0.4.4 / PyTorch 2.x

---

## 目录

- [1. 系统架构与控制链路](#1-系统架构与控制链路)
- [2. 硬件连接与接线清单](#2-硬件连接与接线清单)
- [3. 环境搭建与多机无缝迁移](#3-环境搭建与多机无缝迁移)
- [4. SocketCAN 接口配置与 Bringup 验证](#4-socketcan-接口配置与-bringup-验证)
- [5. 多模态实时控制 (手柄 / 键盘 / 视觉)](#5-多模态实时控制-手柄--键盘--视觉)
- [6. LeRobot 示教数据采集](#6-lerobot-示教数据采集)
- [7. SmolVLA 具身大模型训练](#7-smolvla-具身大模型训练)
- [8. 模型推理评估与大显存节点部署](#8-模型推理评估与大显存节点部署)
- [9. 常见故障排查指南 (FAQ)](#9-常见故障排查指南-faq)

---

## 1. 系统架构与控制链路

本机械臂支持两种控制链路（**推荐链路 A：PC SocketCAN 直连**）：

### 链路 A：PC SocketCAN 直连 (ADR-005 主力链路)
```text
┌────────────────────────────────────────────────────────┐
│ 上位机 PC (Python / LeRobot / CartesianController)     │
└──────────────────────────┬─────────────────────────────┘
                           │ SocketCAN (can0 @ 500 kbps)
            ┌──────────────┴──────────────┐
            ▼                             ▼
   Emm42 电机 1 (J1, ID:2)      ...     Emm42 电机 6 (J6, ID:7)
```
- **控制特性**：PC 直发 `0xFD` 相对位置脉冲帧，50ms 周期闭环下发；
- **安全保障**：`ZdtController` 内建 50ms 通信超时心跳保护，丢包/超时立即安全刹车。

### 链路 B：PC-STM32 UART 串口网关 (向下兼容链路)
```text
PC (MassageRobot) ──[UART 115200bps]──> STM32F407 ──[CAN 1Mbps]──> Emm_V5 电机 ×6
```

---

## 2. 硬件连接与接线清单

| 组件 | 连接接口 | 信号定义 | 说明 |
|---|---|---|---|
| **USB-CAN 适配器** | 主机 USB 3.0 | CAN_H, CAN_L, GND | 并联至 6 个 Emm42 驱动器的 CAN 接口 |
| **24V 动力电源** | Emm42 电机驱动端 | V+, V- (24V DC 10A) | 6 轴电机共用总线供电，注意极性 |
| **RealSense D455** | 主机 USB 3.0 蓝色接口 | USB-C 3.2 | 深度与 RGB 图像采集 (640x480 @ 30fps) |
| **USB 遥控手柄** | 主机 USB 接口 | USB HID / 蓝牙 | Xbox / PS / Switch Pro 手柄 |

---

## 3. 环境搭建与多机无缝迁移

### 3.1 一键安装与配置
```bash
cd Arm-robot_VLA
# 激活 smolvla 环境
conda activate smolvla

# 安装依赖并以 editable 模式注册 lerobot 机器人插件
pip install -e lerobot_robot_massage
pip install pygame mujoco opencv-python safetensors transformers
```

### 3.2 跨机器快速迁移清单 (Migration)
如需将项目从当前开发机迁移至新计算节点（如多卡训练机或实机控制工控机）：
1. **安装系统依赖 (OpenGL / MuJoCo 渲染)**：
   ```bash
   sudo apt update && sudo apt install -y \
       libgl1 libglx0 libegl1 libx11-6 libxrandr2 libxinerama1 libxi6 libxcursor1 \
       can-utils xvfb
   ```
2. **打包并传输核心资产**：
   ```bash
   # 打包代码与模型权重
   tar -czf arm_vla_deploy.tar.gz lerobot_robot_massage/ configs/ scripts/ outputs/
   # 传至目标机并解压
   scp arm_vla_deploy.tar.gz user@target-node:/home/user/
   ```

---

## 4. SocketCAN 接口配置与 Bringup 验证

### 4.1 激活 SocketCAN
```bash
sudo ip link set can0 up type can bitrate 500000
# 检查接口状态 (需显示 state UP)
ip -details link show can0
```

### 4.2 运行交互式 Bringup 面板
```bash
python scripts/bringup/zdt_interactive.py --iface can0
```
- 输入 `scan`：扫描总线上 6 个电机 (ID 0x02~0x07)，确认全部处于 `ONLINE`；
- 输入 `arm`：一键给 6 轴步进上电并进入闭环锁定；
- 输入 `ready`：6 轴以 100 RPM 平缓运动到预设工作准备姿态；
- 输入 `disarm` / `estop`：释放力矩或触发急停。

---

## 5. 多模态实时控制 (手柄 / 键盘 / 视觉)

### 5.1 USB 手柄遥控 (Xbox / PS 游戏手柄)
```bash
python scripts/control/joystick_control.py --iface can0 -y
```
- **左摇杆**：末端 XY 水平平移；
- **LT / RT 扳机键**：末端 Z 轴上升 / 下降；
- **右摇杆**：末端 Roll / Pitch 姿态微调；
- **A 键**：进入/退出遥控跟随；**Y 键**：一键急停。

### 5.2 键盘 6-DOF 笛卡尔空间微调
```bash
python scripts/control/cartesian_keyboard.py --iface can0
```
- 详见 [KEYBOARD_TELEOP_GUIDE.md](KEYBOARD_TELEOP_GUIDE.md)。

### 5.3 视觉协同遥操 (机械臂 + 灵巧手 22-DOF)
```bash
# 推荐直接使用根目录统一协同模块
python ../run_teleop.py --iface can0 -y
```

---

## 6. LeRobot 示教数据采集

利用 HuggingFace LeRobot 标准工作流录制真实示教轨迹：

```bash
# 启动 LeRobot 数据集录制 (使用 PC-CAN 直连驱动)
python -m lerobot.scripts.record \
    --robot-path lerobot_robot_massage/config_massage_robot.py \
    --robot-overrides.channel=can0 \
    --fps 30 \
    --repo-id local/massage_roll_dataset \
    --num-episodes 50
```

---

## 7. SmolVLA 具身大模型训练

使用 SmolVLA（基于视觉-语言-动作的流匹配扩散策略）训练推拿模型：

```bash
# 启动单卡或多卡训练
python -m lerobot.scripts.train \
    --config-path configs/train_smolvla.yaml \
    --dataset_repo_id local/massage_roll_dataset \
    --output_dir outputs/smolvla_massage \
    --batch_size 16 \
    --num_workers 8
```

---

## 8. 模型推理评估与大显存节点部署

### 8.1 目标节点硬件要求
- **GPU 显存**：最低 8 GB (推荐 RTX 4080 / 4090 / A100 12GB+)；
- **内存**：16 GB RAM。

### 8.2 加载模型权重执行仿真闭环评估
```bash
python scripts/data_tools/evaluate_policy.py \
    --checkpoint outputs/smolvla_massage/checkpoints/030000/pretrained_model \
    --episodes 20 \
    --render
```

---

## 9. 常见故障排查指南 (FAQ)

### Q1: `can0` 提示 `Network is down` 或连接超时？
- 检查 USB-CAN 模块接线，运行 `sudo ip link set can0 up type can bitrate 500000`；
- 检查终端 120Ω 匹配电阻是否接通。

### Q2: 单个电机无响应或报错？
- 运行 `python scripts/bringup/zdt_interactive.py` 执行 `scan`；
- 查看掉线电机的拨码开关与 CAN ID 冲突情况。

### Q3: 运行控制时机械臂产生剧烈抖动？
- 检查 `configs/massage_robot.yaml` 或 `teleop_config.yaml` 中的 `joint_factor` 倍率是否过高；
- 确认电机加减速档位 `position_acc` 是否设为 `0`（推荐直冲模式配合算法级 1€ 滤波）。
