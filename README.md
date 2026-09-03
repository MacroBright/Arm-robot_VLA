<div align="center">

# Arm-robot_VLA
### 6-DOF 机械臂 SocketCAN 控制与 LeRobot 具身智能套件

[![Python Version](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![SocketCAN](https://img.shields.io/badge/SocketCAN-can0%20%40%20500kbps-E34F26?logo=linux&logoColor=white)](https://www.kernel.org/doc/Documentation/networking/can.txt)
[![Hardware](https://img.shields.io/badge/Motors-6x%20Emm42%20V5.0-4CAF50)](docs/HARDWARE_CAN_SPEC.md)
[![Simulation](https://img.shields.io/badge/Simulation-MuJoCo%203.x-D00000?logo=openai&logoColor=white)](docs/MUJOCO_SIM.md)
[![Vision](https://img.shields.io/badge/Camera-Intel%20RealSense%20D455-0071C5?logo=intel&logoColor=white)](docs/tuinadex_to_lerobot.md)
[![Embodied AI](https://img.shields.io/badge/Embodied%20AI-LeRobot%20%7C%20SmolVLA-FFD21E?logo=huggingface&logoColor=black)](https://github.com/huggingface/lerobot)
[![Unit Tests](https://img.shields.io/badge/Tests-285%20Passed-brightgreen?logo=pytest&logoColor=white)](tests/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

<p align="center">
  <b>Arm-robot_VLA</b> 是 TuinaDex 系统的 6-DOF 机械臂控制子模块，负责 Linux SocketCAN 电机直驱、笛卡尔空间运动学控制、MuJoCo 动力学仿真、手势/键盘遥操，以及 HuggingFace LeRobot 示教数据录制与具身策略训练接入。
</p>

[主要特性](#主要特性) •
[系统架构](#系统架构) •
[硬件规格与拓扑](#硬件规格与拓扑) •
[环境安装](#环境安装) •
[命令行工具](#命令行工具-cli) •
[运行示例](#常用运行示例) •
[Python API](#python-api-示例) •
[相关文档](#相关文档)

</div>

---

## 主要特性

- **SocketCAN 直驱**：基于 Linux SocketCAN (`can0 @ 500kbps`) 直驱 6 台 Emm42 V5.0 步进闭环驱动器，通过 0xFD 扩展帧实现多轴脉冲同步下发，高频轮询 0x1F 标志位与 0x36 绝对编码器位置。
- **运动学与笛卡尔控制**：基于修正 DH (MDH) 几何参数建模，使用阻尼最小二乘法 (DLS) 进行速度级数值逆解，包含奇异点自适应阻尼抑制、单步角度限幅与关节软限位守卫。
- **视觉手势与键盘遥操**：集成 RealSense D455 与手势追踪，提供空格键离合锁定、三档平移/旋转灵敏度调节，支持点按、滚法、俯仰与全自由度 4 种推拿控制模态；同时支持基于终端键盘的 6-DOF 笛卡尔微调。
- **MuJoCo 物理仿真**：内置机械臂 MJCF 物理模型与高精度网格，提供与真实硬件一致的 TCP 通信服务接口。支持自定义 Home / Ready 预设姿态，遥操松开手势或按键后原地刚性悬停，避免回弹与重力下坠。
- **硬件安全机制**：上电默认为未使能力矩的 `SAFE_IDLE` 状态，对 J2/J3 重力关节引入显式确认门禁，支持 50ms 通信看门狗检测与全局广播急停 (`ESTOP`)。
- **LeRobot 具身智能接入**：实现 HuggingFace LeRobot `Robot` 标准基类接口，可直接调用 `lerobot-record` 进行多模态轨迹录制，并支持 SmolVLA 与 ACT 模型训练。

---

## 系统架构

```text
┌───────────────────────────────────────────────────────────────────────────────┐
│                     Layer 4: 具身智能与上层决策 (Embodied AI)                   │
│    SmolVLA-450M / LeRobot Policy / 示教数据录制 (JSONL+MP4) / 跨模块协同      │
├───────────────────────────────────────────────────────────────────────────────┤
│                     Layer 3: 视觉感知与遥操作 (Perception & Teleop)            │
│    RealSense D455 / 手势骨骼追踪 / 离合控制 / 视觉通信看门狗 (VisionWatchdog)    │
├───────────────────────────────────────────────────────────────────────────────┤
│                     Layer 2: 空间运动学与笛卡尔控制 (Kinematics)                │
│    CartesianController (MDH 正逆解 + 阻尼最小二乘 DLS + 单步角度增量限幅)        │
├───────────────────────────────────────────────────────────────────────────────┤
│                     Layer 1: 状态机与安全门禁 (Controller & Safety)            │
│    ZdtController (UNINITIALIZED → SAFE_IDLE → ARMED → TELEOP → STOPPED)       │
├───────────────────────────────────────────────────────────────────────────────┤
│                     Layer 0: 底层通信与电机驱动 (Hardware Driver)              │
│    Linux SocketCAN (can0 @ 500kbps) ↔ Emm42 V5.0 步进闭环驱动器 (J1~J6)        │
│    0xFD 双帧相对位置脉冲下发 / 0x36 多圈绝对位置回读 / 堵转保护检测            │
└───────────────────────────────────────────────────────────────────────────────┘
```

---

## 硬件规格与拓扑

机械臂由 6 个带多圈绝对值磁编码器的 Emm42 V5.0 步进闭环驱动器构成，各轴参数配置如下：

| 关节编号 | 机构功能 | CAN 地址 | 默认减速比 | 软件限位范围 | 属性说明 |
| :---: | :--- | :---: | :---: | :---: | :---: |
| **J1** | 基座偏航 (Base Yaw) | `0x02` | **50.0 : 1** | `[-90.0°, +270.0°]` | 水平旋转轴 |
| **J2** | 大臂俯仰 (Shoulder Pitch) | `0x03` | **50.89 : 1** | `[-10.0°, +150.0°]` | ⚠️ 重力关节（需保持力矩使能） |
| **J3** | 小臂俯仰 (Elbow Pitch) | `0x04` | **50.89 : 1** | `[-30.0°, +120.0°]` | ⚠️ 重力关节（需保持力矩使能） |
| **J4** | 腕部主滚转 (Wrist Roll 1) | `0x05` | **51.0 : 1** | `[-90.0°, +90.0°]` | 腕部差动驱动 |
| **J5** | 腕部俯仰 (Wrist Pitch) | `0x06` | **27.0 : 1** | `[-10.0°, +180.0°]` | 手腕俯仰轴 |
| **J6** | 末端工具滚转 (Wrist Roll 2) | `0x07` | **51.0 : 1** | `[-90.0°, +90.0°]` | 工具法兰输出轴 |

---

## 环境安装

### 1. 创建独立环境并安装

项目推荐使用 Python 3.10 环境（支持 Conda 一键创建或 pip 手动安装）：

```bash
# 方式 A：Conda 一键创建与安装 (推荐)
conda env create -f environment.yml
conda activate arm_robot

# 方式 B：手动创建与 pip 安装
conda create -n arm_robot python=3.10 -y
conda activate arm_robot
cd Arm-robot_VLA
pip install -r requirements.txt
pip install -e .
```


### 2. 配置 SocketCAN（连接真机时需要）

```bash
sudo ip link set can0 type can bitrate 500000 && sudo ip link set can0 up
```

---

## 命令行工具 (CLI)

项目提供统一入口命令 `arm-robot` 及各功能别名：

```bash
arm-robot --help
```

| 子命令 / 别名 | 功能说明 | 常用命令示例 |
|---|---|---|
| **`arm-robot teleop`**<br/>`arm-teleop` | **视觉手势遥操**<br/>开机处于 PAUSED 安全待机，按 `[R]` 平稳运动至 READY 准备位，按 `[SPACE]` 开关手势跟随 | `arm-robot teleop --iface can0 -y`<br/>*(空跑测试: `arm-robot teleop --no-drive`)* |
| **`arm-robot control`**<br/>`arm-control` / `arm-robot interactive` | **底层电机交互调试控制台**<br/>寄存器读写、原始 CAN 帧收发、单轴微步进、参数标定 | `arm-robot control --iface can0` |
| **`arm-robot keyboard`**<br/>`arm-keyboard` | **键盘 6-DOF 笛卡尔遥控**<br/>W/A/S/D/Q/E 笛卡尔控制，按 `[H]` 回初始位，`[R]` 回准备位，`[P]` 打印角度 | `arm-robot keyboard --iface can0`<br/>*(仿真模式: `arm-robot keyboard --sim`)* |
| **`arm-robot sim`**<br/>`arm-sim` | **MuJoCo 物理仿真节点**<br/>启动带 3D 视窗的物理引擎服务，支持自定义 Home/Ready 姿态配置 | `arm-robot sim --viewer` |
| **`arm-robot bringup`**<br/>`arm-bringup` | **硬件上电自检与测试**<br/>总线在线状态检测 (status)、单轴步进微动 (step) | `arm-robot bringup status --iface can0`<br/>`arm-robot bringup step 6 2.0 --iface can0` |
| **`arm-robot calib`**<br/>`arm-calib` | **手眼正交标定向导**<br/>相机到机械臂基座坐标系外参解算，输出至 `configs/handeye_calib.json` | `arm-robot calib` |
| **`arm-robot joystick`**<br/>`arm-joystick` | **手柄遥控**<br/>Xbox / PS 游戏手柄笛卡尔空间遥控 | `arm-robot joystick -s 0.3` |
| **`arm-robot test`** | **运行测试套件**<br/>一键运行全部 285 个单元与集成测试 | `arm-robot test` |

---

## 常用运行示例

### 1. 纯视觉空跑测试（无需连接硬件）
```bash
arm-robot teleop --no-drive
```

### 2. 数字孪生仿真遥控
```bash
# 终端 1：启动 MuJoCo 仿真服务
arm-robot sim --viewer

# 终端 2：使用键盘控制仿真机械臂
arm-robot keyboard --sim

# 终端 3：或使用相机手势控制仿真机械臂
arm-robot teleop --sim
```

### 3. 真机上电自检与微步进
```bash
# 检查 6 轴在网状态
arm-robot bringup status --iface can0

# 启动底层交互调试器，进行单轴点动验证
arm-robot control --iface can0
# 控制台指令示例: j5 -> fdrel 2.0 -> quit
```

### 4. 真机手势推拿遥操闭环
```bash
arm-robot teleop --iface can0 -y
```
- **步骤 1**：启动后处于静止待机状态 (`PAUSED`)；
- **步骤 2**：在弹出的视频窗口中按 **`[R]`** 键，机械臂平稳运动至推拿准备姿态 (`READY`)；
- **步骤 3**：按 **`[SPACE]`** 键接合离合进入实时跟随状态，再次按 **`[SPACE]`** 键随时暂停悬停。

---

## Python API 示例

```python
import time
from arm_robot import ZdtConfig, ZdtController

# 1. 连接 SocketCAN (can0 @ 500kbps)
cfg = ZdtConfig(channel="can0", bitrate=500_000)
arm = ZdtController(cfg)
arm.connect()

# 2. 扫描总线并确认重力关节使能
arm.scan()
arm.arm(gravity_confirmed=True)

# 3. 平滑运动至准备姿态
arm.enter_teleop()
arm.ready()
time.sleep(1.0)

# 4. 读取当前 6 轴物理角度与相电流
state = arm.get_real_state()
print("当前关节角 (deg):", state["q"])
print("当前相电流 (mA):", state["current"])

# 5. 单轴相对运动 (如 J6 旋转 5°)
arm.rel_rotate(joint_id=6, delta_deg=5.0)

# 6. 安全停止并释放连接
arm.stop()
arm.disconnect()
```

---

## 测试验证

项目包含 285 个单元与集成测试，覆盖 CAN 帧编解码、运动学正逆解、阻尼奇异算法、状态机门禁等核心逻辑：

```bash
pytest tests/
```

```text
============================= 285 passed in ~15s =============================
```

---

## 相关文档

- [软硬件架构设计规范 (ARCHITECTURE.md)](docs/ARCHITECTURE.md)
- [底层 CAN 通信协议与寄存器字典 (HARDWARE_CAN_SPEC.md)](docs/HARDWARE_CAN_SPEC.md)
- [键盘笛卡尔遥控指南 (KEYBOARD_TELEOP_GUIDE.md)](docs/KEYBOARD_TELEOP_GUIDE.md)
- [MuJoCo 物理仿真使用说明 (MUJOCO_SIM.md)](docs/MUJOCO_SIM.md)
- [LeRobot 数据集与策略训练接入 (tuinadex_to_lerobot.md)](docs/tuinadex_to_lerobot.md)
- [开发与调试工作流 (WORKFLOW.md)](docs/WORKFLOW.md)

---

## 开源协议

本项目遵循 [Apache License 2.0](LICENSE) 协议。
