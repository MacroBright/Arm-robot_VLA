<div align="center">

# 🦾 Arm-robot_VLA
### 6-DOF 具身智能机械臂控制系统 · SocketCAN 直驱 · 物理数字孪生 · LeRobot 生态

[![Python Version](https://img.shields.io/badge/Python-3.10%20(Recommended)-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![SocketCAN](https://img.shields.io/badge/SocketCAN-can0%20%40%20500kbps-E34F26?logo=linux&logoColor=white)](https://www.kernel.org/doc/Documentation/networking/can.txt)
[![Hardware](https://img.shields.io/badge/Motors-6x%20Emm42%20V5.0%20Closed--Loop-4CAF50)](docs/HARDWARE_CAN_SPEC.md)
[![Simulation](https://img.shields.io/badge/Physics%20Simulation-MuJoCo%203.x-D00000?logo=openai&logoColor=white)](docs/MUJOCO_SIM.md)
[![Vision](https://img.shields.io/badge/Vision%20Sensor-Intel%20RealSense%20D455-0071C5?logo=intel&logoColor=white)](docs/tuinadex_to_lerobot.md)
[![Embodied AI](https://img.shields.io/badge/Embodied%20AI-HuggingFace%20LeRobot%20%7C%20SmolVLA-FFD21E?logo=huggingface&logoColor=black)](https://github.com/huggingface/lerobot)
[![Unit Tests](https://img.shields.io/badge/Unit%20Tests-285%20Passed%20(100%25)-brightgreen?logo=pytest&logoColor=white)](tests/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

<p align="center">
  <b>Arm-robot_VLA</b> 是 TuinaDex 系统的 <b>6-DOF 医疗推拿机械臂与具身智能核心子项目</b>。<br/>
  提供从 <b>Linux 原生 SocketCAN 总线直驱</b>、<b>修正 DH 运动学与 DLS 笛卡尔空间控制</b>、<b>MuJoCo 物理动力学数字孪生</b> 到 <b>HuggingFace LeRobot 数据采集与 SmolVLA 具身策略推理</b> 的全栈软硬件工业级解决方案。
</p>

[✨ 核心亮点](#-核心亮点与特色) •
[📐 架构设计](#-分层架构设计) •
[🔩 关节拓扑](#-机械规格与关节拓扑) •
[⚡ 快速上手](#-快速上手与安装) •
[💻 CLI 速查](#-统一-cli-命令行工具集) •
[🐍 Python SDK](#-python-sdk-快速开发) •
[📚 文档导航](#-技术文档导航)

</div>

---

## 🌟 核心亮点与特色

- 🏎️ **微秒级 SocketCAN 原生直驱**：绕过中间串口转发协议栈，直连 Linux SocketCAN 内核 (`can0 @ 500kbps`)，支持 6 轴 Emm42 V5.0 闭环步进电机的 `0xFD` 相对脉冲多机同步编解码与 `0x1F`/`0x36` 高频状态回读。
- 🎯 **高精空间运动学与奇异点鲁棒控制**：内置修正 DH (MDH) 几何运动学正逆解，采用**阻尼最小二乘法 (DLS)** 与零空间自适应阻尼因子 $\lambda$，杜绝机械臂极限伸展或手腕奇异区的角速度发散。
- 🖐️ **视觉手势直观遥操 (Perception Teleop)**：搭载 RealSense D455 深度相机与解剖学刚体掌骨解耦算法，支持**按键式离合保护 (`[SPACE]`)**、**三档动态灵敏度变速箱 (`[S]/[TAB]`)** 与 **4 种专业推拿姿态约束模态**（点按揉捏、滚法推法、俯仰调节、全 6-DOF）。
- 🌐 **高保真 MuJoCo 物理数字孪生**：基于原生 MJCF 物理模型实现 100% 动力学行为仿真。独创**停止跟随瞬间原地刚性位姿锁定（0 弹回、0 下坠）**，支持 `--home-pose` 与 `--ready-pose` 动态姿态调谐。
- 🛡️ **工业级硬件安全防御机制**：开机 `SAFE_IDLE` 零力矩防护，重力关节（J2/J3）二次安全确认机制，50ms 通通信跳看门狗，软硬件工作空间硬限幅，随时一键广播急停 (`ESTOP`)。
- 🤖 **深度集成 HuggingFace LeRobot**：原生兼容 LeRobot `Robot` 标准接口，无缝串联 `lerobot-record` 多模态示教轨迹录制管线与 SmolVLA 450M / ACT 具身模型端到端训练部署。

---

## 📐 分层架构设计

系统严格遵循工业级机器人高内聚、松耦合的模块化设计分层，确保高频控制的确定性与数据流的高效流转：

```text
┌───────────────────────────────────────────────────────────────────────────────┐
│                     Layer 4: 具身智能与上层决策 (Embodied AI)                   │
│    SmolVLA-450M / LeRobot Policy / 示教录制 (JSONL+MP4) / 跨模块协同遥操      │
├───────────────────────────────────────────────────────────────────────────────┤
│                     Layer 3: 视觉感知与手势交互 (Perception & Teleop)          │
│    Intel RealSense D455 / 解剖掌骨解耦 / 切换离合 / 4级通信看门狗 (VisionWatchdog) │
├───────────────────────────────────────────────────────────────────────────────┤
│                     Layer 2: 空间运动学与笛卡尔控制器 (Kinematics)             │
│    CartesianController (MDH 正逆解 + 阻尼最小二乘 DLS + 单步 ≤2.0° 加速度硬限幅) │
├───────────────────────────────────────────────────────────────────────────────┤
│                     Layer 1: 机器人状态机与安全守卫 (Controller & Safety)      │
│    ZdtController (UNINITIALIZED → SAFE_IDLE → ARMED → TELEOP → STOPPED)       │
├───────────────────────────────────────────────────────────────────────────────┤
│                     Layer 0: 底层通信与执行机构 (Hardware Driver)              │
│    Linux SocketCAN (can0 @ 500kbps) ↔ Emm42 V5.0 闭环步进驱动器 (J1~J6)       │
│    0xFD 双帧相对位置脉冲同步下发 / 0x36 多圈绝对值回读 / 硬件堵转自动保护       │
└───────────────────────────────────────────────────────────────────────────────┘
```

---

## 🔩 机械规格与关节拓扑

机械臂由 6 个配备高精度多圈绝对值磁编码器的 Emm42 V5.0 闭环步进驱动器组成，出厂默认配置如下：

| 关节编号 | 物理功能描述 | CAN 地址 | 默认减速比 | 软件安全限位范围 | 物理重力属性 |
| :---: | :--- | :---: | :---: | :---: | :---: |
| **J1** | 基座偏航旋转 (Base Yaw) | `0x02` | **50.0 : 1** | `[-90.0°, +270.0°]` | 水平旋转轴 |
| **J2** | 大臂俯仰起落 (Shoulder Pitch) | `0x03` | **50.89 : 1** | `[-10.0°, +150.0°]` | ⚠️ **重力关节 (必须保持使能)** |
| **J3** | 小臂俯仰屈伸 (Elbow Pitch) | `0x04` | **50.89 : 1** | `[-30.0°, +120.0°]` | ⚠️ **重力关节 (必须保持使能)** |
| **J4** | 腕部主轴滚转 (Wrist Roll 1) | `0x05` | **51.0 : 1** | `[-90.0°, +90.0°]` | 腕部差动驱动 |
| **J5** | 腕部末端俯仰 (Wrist Pitch) | `0x06` | **27.0 : 1** | `[-10.0°, +180.0°]` | 手腕俯仰调谐 |
| **J6** | 终端法兰自旋 (Wrist Roll 2) | `0x07` | **51.0 : 1** | `[-90.0°, +90.0°]` | 末端执行器工具法兰 |

---

## ⚡ 快速上手与安装

### 1. 创建并激活专属独立环境

推荐使用独立的 Python 3.10 环境（与灵巧手 `leap_hand` 及 `smolvla` 彻底解耦）：

```bash
# 1. 创建机械臂独立虚拟环境
conda create -n arm_robot python=3.10 -y
conda activate arm_robot

# 2. 安装项目本体（以可编辑模式安装）
cd Arm-robot_VLA
pip install -e .
```

### 2. 硬件链路准备 (针对真机)

物理连接 PEAK PCAN-USB 适配器后，一键拉起 SocketCAN 接口：

```bash
sudo ip link set can0 type can bitrate 500000 && sudo ip link set can0 up
```

---

## 💻 统一 CLI 命令行工具集

项目已向系统注册统一的顶级指令 **`arm-robot`**（以及各常用快捷别名），覆盖从标定、仿真、调试到遥操全流程：

```bash
arm-robot --help
```

| 命令 (子命令 / 别名) | 核心功能与场景 | 典型指令示例 |
|---|---|---|
| **`arm-robot teleop`**<br/>`arm-teleop` | **6-DOF 视觉手势遥操**<br/>开机安全 PAUSED 待机，支持按键平稳切 READY/HOME，[SPACE] 离合切换，多模态推拿控制 | `arm-robot teleop --iface can0 -y`<br/>*(空跑测试: `arm-robot teleop --no-drive`)* |
| **`arm-robot control`**<br/>`arm-control` / `arm-robot interactive` | **底层电机协议交互调试控制台**<br/>读写寄存器、原始 CAN 收发、单轴微步进、脉冲刻度标定与驱动测试 (全面替代旧版 panel) | `arm-robot control --iface can0` |
| **`arm-robot keyboard`**<br/>`arm-keyboard` | **键盘 W/A/S/D/Q/E 6-DOF 笛卡尔遥控**<br/>支持仿真与真机，[H] 回初始位，[R] 回准备位，[P] 实时打印 6 轴角度与末端坐标 | `arm-robot keyboard --iface can0`<br/>*(仿真: `arm-robot keyboard --sim`)* |
| **`arm-robot sim`**<br/>`arm-sim` | **MuJoCo 物理动力学仿真 TCP 节点服务**<br/>支持 3D 渲染，原地刚性悬停零下坠，可配置 Home/Ready 姿态 (`configs/home_pose.json`) | `arm-robot sim --viewer` |
| **`arm-robot bringup`**<br/>`arm-bringup` | **硬件上电快速极性自检与微步进**<br/>一键检测全轴状态 (status)、单关节安全微动 (step) | `arm-robot bringup status --iface can0`<br/>`arm-robot bringup step 6 2.0 --iface can0` |
| **`arm-robot calib`**<br/>`arm-calib` | **30 秒手眼正交 SVD 标定向导**<br/>相机系到机械臂基座系外参解算，自动输出至 `configs/handeye_calib.json` | `arm-robot calib` |
| **`arm-robot joystick`**<br/>`arm-joystick` | **游戏手柄 6 轴空间遥控**<br/>Xbox / PS 手柄摇杆遥控，带死区滤波与非线性加速 | `arm-robot joystick -s 0.3` |
| **`arm-robot test`** | **全套自动化单元测试运行器**<br/>一键运行 285 项单元与集成测试套件 | `arm-robot test` |

---

## 🎮 经典工作流演练

### 场景 A：无硬件纯视觉手势空跑
无需连接 RealSense 相机或机械臂，即可体验手部关键点追踪与推拿姿态映射：
```bash
arm-robot teleop --no-drive
```

### 场景 B：数字孪生闭环仿真 (MuJoCo)
启动带交互视窗的物理仿真，并在另一终端使用键盘或视觉遥操控制虚拟机械臂：
```bash
# 终端 1：启动 3D 物理仿真节点
arm-robot sim --viewer

# 终端 2：使用键盘遥控虚拟机械臂
arm-robot keyboard --sim

# 终端 3：或使用 RealSense 相机手势遥控虚拟机械臂
arm-robot teleop --sim
```

### 场景 C：真机上电自检与交互调试
```bash
# 1. 检查总线 6 轴状态
arm-robot bringup status --iface can0

# 2. 启动底层交互调试器，进行单关节微动
arm-robot control --iface can0
# 进入控制台后输入: j5 -> fdrel 2.0 -> quit
```

### 场景 D：真机闭环手势推拿遥操
```bash
arm-robot teleop --iface can0 -y
```
- **步骤 1**：程序启动后默认处于静止待机状态 (`PAUSED`)；
- **步骤 2**：在相机视窗中按 **`[R]`** 键，机械臂平稳运动至推拿准备姿态 (`READY`)；
- **步骤 3**：按 **`[SPACE]`** 键接合离合进入实时手势跟随，挥动手掌即可精准推拿！

---

## 🐍 Python SDK 快速开发

```python
import time
from arm_robot import ZdtConfig, ZdtController
from arm_robot.kinematics import CartesianCommand, CartesianController

# 1. 初始化 CAN 配置与控制器 (连接 can0 @ 500kbps)
cfg = ZdtConfig(channel="can0", bitrate=500_000)
arm = ZdtController(cfg)
arm.connect()

# 2. 安全扫描与重力关节确认
arm.scan()
arm.arm(gravity_confirmed=True)

# 3. 驱动机械臂平滑运动至推拿准备姿态 (READY)
arm.enter_teleop()
arm.ready()
time.sleep(1.0)

# 4. 读取高精度实时物理观测 (0x36 编码器多圈绝对值角度 + 实时电流)
state = arm.get_real_state()
print(f"当前 6 轴角度 (deg): {state['q']}")
print(f"当前 6 轴相电流 (mA): {state['current']}")

# 5. 单轴微步进测试 (例如 J6 末端微转 5 度)
arm.rel_rotate(joint_id=6, delta_deg=5.0)

# 6. 安全停止并释放
arm.stop()
arm.disconnect()
```

---

## 🧪 自动化测试与质量保证

本项目构建了包含 **285 个完备单元测试与集成测试** 的质量防线，涵盖帧编解码、MDH 正逆运动学、DLS 奇异值阻尼算法、状态机生命周期门禁及跨模块兼容性：

```bash
# 运行全部测试 (约 15 秒快速回归)
pytest tests/

# 预期输出:
# ============================= 285 passed in ~15s =============================
```

---

## 📚 技术文档导航

深入了解底层算法与协议，请查阅 [docs/](docs/) 专栏文档：

- 📐 **[软硬件分层架构设计规范 (ARCHITECTURE.md)](docs/ARCHITECTURE.md)**：深入了解状态机硬不变式与四层架构划分。
- 🔌 **[底层 SocketCAN 通信协议与寄存器字典 (HARDWARE_CAN_SPEC.md)](docs/HARDWARE_CAN_SPEC.md)**：详细解析 0xFD/0x1F/0x36 等 CAN 扩展帧格式。
- 🕹️ **[6-DOF 笛卡尔空间键盘遥控指南 (KEYBOARD_TELEOP_GUIDE.md)](docs/KEYBOARD_TELEOP_GUIDE.md)**：键盘 W/A/S/D/Q/E 按键映射、MDH 参数及 DLS 逆解原理。
- 🌐 **[MuJoCo 物理仿真与数字孪生指南 (MUJOCO_SIM.md)](docs/MUJOCO_SIM.md)**：离屏渲染、共享内存流、可配置位姿与防弹回刚性悬停机制。
- 🚀 **[从 TuinaDex 到 LeRobot 22-DOF 全量接入指南 (tuinadex_to_lerobot.md)](docs/tuinadex_to_lerobot.md)**：多模态推拿示教数据采集与 SmolVLA 训练接入。
- 🛠️ **[工程开发与版本管理工作流 (WORKFLOW.md)](docs/WORKFLOW.md)**：固件烧录、通信标定、数据管线全流程规范。

---

## 📄 开源许可证

本项目基于 [Apache License 2.0](LICENSE) 协议开源。欢迎提交 Issue 与 Pull Request！
