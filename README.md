# Arm-robot_VLA — 6-DOF 机械臂控制与 LeRobot 具身智能系统

<p align="center">
  <img src="https://img.shields.io/badge/Python-3.10%20%7C%203.11%20%7C%203.14-blue" alt="Python Version" />
  <img src="https://img.shields.io/badge/SocketCAN-can0%20%40%20500kbps-orange" alt="CAN Bus" />
  <img src="https://img.shields.io/badge/Motors-6x%20Emm42%20V5.0%20Closed--Loop-brightgreen" alt="Motors" />
  <img src="https://img.shields.io/badge/Unit%20Tests-285%20Passed-success" alt="Tests" />
  <img src="https://img.shields.io/badge/Embodied%20AI-LeRobot%20%7C%20SmolVLA-purple" alt="LeRobot" />
  <img src="https://img.shields.io/badge/Simulation-MuJoCo%20Digital%20Twin-red" alt="MuJoCo" />
</p>

`Arm-robot_VLA` 是 TuinaDex 系统的 **6-DOF 机械臂与具身智能核心子项目**，提供基于 Linux 原生 **SocketCAN 直驱的 ZDT 闭环步进驱动体系**、**MDH 空间正逆运动学与 DLS 笛卡尔控制器**、**MuJoCo 物理数字孪生** 以及与 **HuggingFace LeRobot 生态** 的原生接口集成。

---

## 📑 目录

- [一、核心架构与分层设计](#一核心架构与分层设计)
- [二、硬件规格与关节拓扑](#二硬件规格与关节拓扑)
- [三、CLI 命令行工具速查](#三cli-命令行工具速查)
- [四、🐍 Python SDK 快速上手](#四-python-sdk-快速上手)
- [五、🤖 LeRobot 具身智能集成](#五-lerobot-具身智能集成)
- [六、🎮 MuJoCo 物理仿真与数字孪生](#六-mujoco-物理仿真与数字孪生)
- [七、🧪 自动化测试套件](#七-自动化测试套件)
- [八、📚 详细技术文档中心](#八-详细技术文档中心)

---

## 一、核心架构与分层设计

系统采用严格高内聚、低耦合分层设计，确保控制高频实时性与硬件安全性：

```
┌────────────────────────────────────────────────────────────────────────┐
│                   Layer 4: 具身智能与上层决策 (VLA / AI)                   │
│   SmolVLA-450M / LeRobot Policy / 多模态录制 / 跨模块协同 (Co_Teleop)   │
├────────────────────────────────────────────────────────────────────────┤
│                   Layer 3: 视觉遥操与手眼感知 (Perception)                │
│   RealSense D455 + 刚体掌骨姿态解耦 + 四级看门狗 (OK/DECAY/STOP/ESTOP)   │
├────────────────────────────────────────────────────────────────────────┤
│                   Layer 2: 空间运动学与笛卡尔控制器 (Kinematics)          │
│   CartesianController (MDH 正解 + DLS 奇异规避逆解 + 单步 2.0° 硬限幅)    │
├────────────────────────────────────────────────────────────────────────┤
│                   Layer 1: 状态机与安全门禁 (Controller & Safety)         │
│   ZdtController (UNINITIALIZED → SAFE_IDLE → ARMED → TELEOP → STOPPED)  │
├────────────────────────────────────────────────────────────────────────┤
│                   Layer 0: 底层总线与步进驱动 (Hardware Driver)           │
│   Linux SocketCAN (can0 @ 500kbps) ↔ Emm42 V5.0 闭环步进驱动器 ×6      │
│   0xFD 相对脉冲多机同步 / 0x1F 状态轮询 / 0x36 物理绝对位置回读          │
└────────────────────────────────────────────────────────────────────────┘
```

---

## 二、硬件规格与关节拓扑

机械臂由 6 个配备高精度多圈绝对值编码器的 Emm42 V5.0 闭环步进驱动器构成：

| 关节编号 | 关节物理功能 | CAN 地址 | 默认减速比 | 软件安全限位 | 物理重力属性 |
| :---: | :--- | :---: | :---: | :---: | :---: |
| **J1** | 基座旋转 (Base Yaw) | `0x02` | **50.0 : 1** | `[-90.0°, +270.0°]` | 水平旋转 |
| **J2** | 大臂俯仰 (Shoulder Pitch) | `0x03` | **50.89 : 1** | `[-10.0°, +150.0°]` | ⚠️ **重力关节 (需二次确认)** |
| **J3** | 小臂俯仰 (Elbow Pitch) | `0x04` | **50.89 : 1** | `[-30.0°, +120.0°]` | ⚠️ **重力关节 (需二次确认)** |
| **J4** | 腕部主滚转 (Wrist Roll 1) | `0x05` | **51.0 : 1** | `[-90.0°, +90.0°]` | 腕部差动 |
| **J5** | 手腕俯仰 (Wrist Pitch) | `0x06` | **27.0 : 1** | `[-10.0°, +180.0°]` | 腕部俯仰 |
| **J6** | 末端滚转 (Wrist Roll 2) | `0x07` | **51.0 : 1** | `[-90.0°, +90.0°]` | 末端法兰工具 |

---

## 三、环境准备 (Conda Environment)

机械臂子项目拥有专属的独立运行环境 **`arm_robot`**（内置 MuJoCo 物理引擎、MediaPipe 视觉追踪、SocketCAN 驱动与全套依赖）：

```bash
# 激活机械臂专属环境
conda activate arm_robot

# 查看统一控制台
arm-robot --help
```

---

## 四、CLI 命令行工具速查


本项目注册了 7 大标准控制台命令，在激活环境后即可全局直接调用：

| CLI 命令 | 对应功能说明 | 典型运行指令示例 |
|---|---|---|
| **`arm-teleop`** | 6-DOF 视觉手势遥操主程序 | `arm-teleop --speed-scale 0.20 -y` |
| **`arm-control`** | 交互式微调与电机调试控制台 | `arm-control --iface can0` |
| **`arm-panel`** | ncurses 全屏 6 轴电机状态遥测仪表盘 | `arm-panel --iface can0` |
| **`arm-bringup`** | 硬件快速拉起与极性自检自测 | `arm-bringup status` |
| **`arm-calib`** | 30 秒手眼 SVD 正交矩阵标定向导 | `arm-calib --out configs/handeye_calib.json` |
| **`arm-sim`** | 启动 MuJoCo 物理仿真 TCP 节点服务 | `arm-sim --viewer` |
| **`arm-joystick`** | Xbox / PS 游戏手柄 6 轴空间遥控 | `arm-joystick -s 0.3` |

### 快捷体验：无硬件纯视觉空跑
```bash
arm-teleop --no-drive
```

---

## 四、🐍 Python SDK 快速上手

```python
from arm_robot import ZdtConfig, ZdtController
from arm_robot.kinematics import CartesianCommand, CartesianController
import time

# 1. 初始化 CAN 配置与控制器 (连接 can0 @ 500kbps)
cfg = ZdtConfig(channel="can0", bitrate=500_000)
arm = ZdtController(cfg)
arm.connect()

# 2. 安全枚举与硬件自检
arm.scan()
# 显式二次确认重力关节 J2/J3 扭矩
arm.arm(gravity_confirmed=True)

# 3. 驱动机械臂平滑运动到推拿准备姿态
arm.enter_teleop()
arm.ready()  # 移动到 [0°, 75°, 55°, 0°, 130°, 0°]
time.sleep(1.0)

# 4. 读取实时状态
state = arm.get_state()
print(f"当前 6 轴角度 (deg): {state.positions_deg}")
print(f"当前 6 轴相电流 (mA): {state.currents_ma}")

# 5. 安全断开
arm.stop()
arm.disconnect()
```

---

## 五、🤖 LeRobot 具身智能集成

本包原生实现 HuggingFace LeRobot 的 `Robot` 基类，支持无缝使用官方工具链：

```bash
# 1. 使用 LeRobot 官方录制管线采集示教数据
lerobot-record \
  --robot.type=massage_robot \
  --robot.cameras.overhead.device=0 \
  --repo-id=TuinaDex/massage_arm_teleop \
  --num-episodes=50

# 2. SmolVLA 具身策略大模型训练
python -m lerobot.scripts.train \
  --config=configs/train_smolvla.yaml
```

---

## 六、🎮 MuJoCo 物理仿真与数字孪生

在没有物理机械臂时，可通过自带的 MuJoCo 模型进行 100% 动力学行为孪生仿真：

```bash
# 启动 3D 渲染仿真 TCP 服务节点
arm-sim --viewer --trail 300
```
- **MJCF 场景文件**：`src/arm_robot/simulation/scene.xml`
- **高精度几何网格**：`src/arm_robot/simulation/meshes/*.stl`
- **共享内存相机流**：内置 `camera_server.py` 支持 60fps 零拷贝虚拟视角推流。

---

## 七、🧪 自动化测试套件

本项目拥有 **285 个完备的单元测试**，覆盖 CAN 驱动编解码、MDH 正逆解、雅可比矩阵、奇异点阻尼、安全状态机与跨包兼容垫片：

```bash
# 运行全量单元测试 (秒级通过)
pytest tests/
# 预期结果: 285 passed in ~14s
```

---

## 八、📚 详细技术文档中心

更多底层细节请参阅 [docs/](docs/) 文档目录：
- 📐 [软硬件分层架构设计 (ARCHITECTURE.md)](docs/ARCHITECTURE.md)
- 🔌 [底层 CAN 总线通讯协议与寄存器字典 (HARDWARE_CAN_SPEC.md)](docs/HARDWARE_CAN_SPEC.md)
- 🕹️ [6-DOF 笛卡尔键盘遥控使用指南 (KEYBOARD_TELEOP_GUIDE.md)](docs/KEYBOARD_TELEOP_GUIDE.md)
- 🌐 [MuJoCo 物理仿真与数字孪生指南 (MUJOCO_SIM.md)](docs/MUJOCO_SIM.md)
- 🚀 [从 TuinaDex 到 LeRobot 22-DOF 全量接入指南 (tuinadex_to_lerobot.md)](docs/tuinadex_to_lerobot.md)
- 🛠️ [工程开发与发布工作流规范 (WORKFLOW.md)](docs/WORKFLOW.md)
