# Arm-robot_VLA

> 基于 VLA（Vision-Language-Action）的 6-DOF 机械臂按摩系统
> STM32F407VET6 + Emm_V5 CAN 步进电机 + LeRobot 生态集成

---

## 目录结构

```
Arm-robot_VLA/
├── README.md                              ← 本文件
├── CLAUDE.md                              ← 项目总纲（AI 开发指南）
├── .gitignore
│
├── lerobot_robot_massage/                 ← LeRobot 插件包（核心代码）
│   ├── pyproject.toml                     ← 包安装配置
│   ├── __init__.py                        ← 包导出
│   ├── config_massage_robot.py            ← MassageRobotConfig 配置类
│   ├── massage_robot.py                   ← MassageRobot(Robot) 适配器
│   ├── serial_protocol.py                 ← PC↔STM32 串口协议封装
│   └── zdt/                               ← ZDT 步进电机驱动层
│       ├── controller.py                  ← ZdtController（6 轴协调）
│       ├── config.py                      ← ZdtConfig + 电机限位表
│       ├── zdt_driver.py                  ← ZDT 帧协议收发
│       ├── can_transport.py               ← SocketCAN 传输层
│       └── frames.py                      ← 帧解析工具
│
├── configs/                               ← LeRobot 配置文件
│   ├── massage_robot.yaml                 ← 机器人硬件配置
│   └── train_smolvla.yaml                 ← SmolVLA 训练配置
│
├── firmware/                              ← STM32 固件
│   ├── README.md                          ← 固件修改指南
│   └── src/                               ← 关键修改的源文件
│       ├── robot.c                        ← 机器人控制
│       ├── robot_cmd.c                    ← 串口命令（含 LeRobot 4 条新命令）
│       └── FreeRTOSConfig.h               ← 堆内存 30KB
│
├── docs/
│   ├── ARCHITECTURE.md                    ← 系统架构设计
│   ├── DEPLOYMENT.md                      ← **部署教程：从 0 到训练（推荐从此开始）**
│   ├── WORKFLOW.md                        ← 开发工作流 + 文件索引
│   └── SERIAL_COMMANDS.md                 ← 14 条串口命令参考
│
├── scripts/
│   ├── teleop/                            ← 视觉遥操
│   │   ├── demo_arm_teleop.py             ← **人手 6DOF → 末端位姿 → 速度命令**
│   │   ├── arm_client.py                  ← 串口薄客户端
│   │   ├── handeye_calib.py               ← 手眼标定
│   │   └── test_arm_client.py / test_handeye_calib.py
│   ├── simulation/                        ← MuJoCo 仿真
│   │   ├── mujoco_sim.py                  ← **MuJoCo 仿真臂 + 7DOF IK**
│   │   ├── camera_server.py / shm_util.py ← 共享内存相机帧
│   │   ├── record_sim.py                  ← 仿真数据录制
│   │   ├── remote_semantics.py            ← 远程语义解析
│   │   └── mujoco_scene/                  ← 场景资源
│   ├── control/                           ← 实时控制
│   │   ├── joystick_control.py            ← **USB 手柄遥控机械臂**
│   │   └── control_hub.py                 ← 多窗口控制中枢
│   ├── bringup/                           ← 硬件 bringup + 系统脚本
│   │   ├── zdt_bringup.py                 ← ZDT 直连 CAN 命令行
│   │   ├── can_setup.sh / startup.sh
│   │   └── test_zdt_bringup_import.py / test_massage_robot_can.py
│   └── data_tools/                        ← 数据处理与模型诊断
│       ├── convert_to_lerobot.py          ← 数据格式转换
│       ├── diagnose_model.py              ← 模型诊断
│       └── evaluate_policy.py             ← 策略评估
│
├── datasets/                              ← 采集数据集（gitignored）
├── outputs/                               ← 训练模型（gitignored）
└── experiments/                           ← 实验记录
```

---

## 快速开始

### 1. 本地 LeRobot 部署

```powershell
cd Arm-robot_VLA
python -m venv .venv
.venv\Scripts\Activate.ps1

pip install lerobot
pip install -e lerobot_robot_massage
pip install pygame
```

### 2. 验证部署

```powershell
python -c "import lerobot; print(f'lerobot {lerobot.__version__}')"

python -c "import lerobot_robot_massage; \
from lerobot.robots import RobotConfig; \
print(RobotConfig.get_choice_class('massage_robot').__name__)"
# 预期: MassageRobotConfig
```

### 3. Ubuntu conda 环境

```bash
# 一键安装（推荐）：安装 leRobot + 本插件 + 开发依赖，自动处理 editable install
bash scripts/setup_dev.sh

conda activate smolvla
python -c "import lerobot_robot_massage; from lerobot.robots import RobotConfig; \
print(RobotConfig.get_choice_class('massage_robot').__name__)"
# 预期: MassageRobotConfig
```

> 说明：`register_third_party_plugins()` 仅在 lerobot CLI（`lerobot-record` / `lerobot-train` 等）内自动调用，
> 日常跑 CLI 无需手动 import 插件包。详细部署步骤见 `docs/DEPLOYMENT.md` 第 3–4 章。

### 4. 硬件接线

| 连接 | 说明 |
|------|------|
| ST-LINK → STM32 | SWDIO→PA13, SWCLK→PA14（烧录固件用）|
| USB-TTL → STM32 | TX→PA10(RX), RX→PA9(TX), GND→GND（串口通信）|
| STM32 CAN1 → 电机 | CAN_H / CAN_L 并联所有 Emm_V5 |
| 24V → 电机驱动 | 所有 Emm_V5 驱动器需供电 |

### 5. 手柄遥控

```powershell
python scripts/control/joystick_control.py --port COM5 --camera 1
```

按键参考：
- **A** → 进入遥控模式（`remote_enable`）
- **左摇杆** → XY 平移  **右摇杆** → 旋转
- **L2/R2** → Z 升降    **Y** → 急停
- **L1/R1** → 关节模式  **十字键↑↓** → 步进

### 6. 数据采集

见 `docs/DEPLOYMENT.md` 第 9 章，支持两种方式：
- **手动示教**：`set_torque 0` 手拖 → LeRobot record 录制
- **手柄遥操作**：手柄控制 + 同步记录

---

## 硬件

| 组件 | 规格 |
|------|------|
| 主控 | PC-UST-CAN-直连 |
| 电机 | Emm_V5 CAN 步进闭环 ×6（CAN ID 2-7）|
| 通信 | USART1 (PA9/PA10, 115200bps) |
| 相机 | USB 640×480@30fps |
| 手柄 | Xbox / 通用 USB 手柄（pygame）|
| 时钟 | HSE 25MHz → SYSCLK 168MHz |

## 软件

| 软件 | 版本 | 用途 |
|------|------|------|
| Python | 3.10+ | 运行时 |
| LeRobot | 0.4.4 (PyPI) | 数据采集/训练/推理框架 |
| PyTorch | 2.10+ | 深度学习引擎 |
| OpenCV | 4.x | 相机采集 |
| pygame | 2.x | 手柄输入 |

## 串口命令速览

| 命令 | 功能 | 说明 |
|------|------|------|
| `get_state` | 读取关节角度/速度/负载 | 数据采集核心 |
| `set_joints j1...j6` | 设置全部关节目标角度 | **空格分隔** |
| `set_torque 0/1` | 自由/锁定模式 | 0=示教，1=锁定 |
| `e_stop` | 紧急停止 | 优先级最高 |
| `remote_enable/disable` | 远程控制开关 | 手柄笛卡尔控制必需 |
| `remote_event` | 笛卡尔速度控制 | 手柄核心命令 |
| `rel_rotate` | 单关节相对旋转 | 关节模式 |
| `zero` / `hard_reset` / `soft_reset` | 归零与复位 | 校准 |

详见 `docs/SERIAL_COMMANDS.md`（共 14 条）。

## 文档导航

| 要找什么 | 看哪个文件 |
|----------|-----------|
| **从 0 到训练完整教程** | `docs/DEPLOYMENT.md` |
| 怎么部署环境 | DEPLOYMENT.md 第 3 章 |
| 怎么手柄遥控 | DEPLOYMENT.md 第 8 章 |
| 怎么录训练数据 | DEPLOYMENT.md 第 9 章 |
| 怎么训练/推理 | DEPLOYMENT.md 第 10–11 章 |
| 系统架构 | `docs/ARCHITECTURE.md` |
| 串口命令 | `docs/SERIAL_COMMANDS.md` |
| 工作流程 | `docs/WORKFLOW.md` |
| 固件怎么改 | `firmware/README.md` |

## 参考

- LeRobot: https://github.com/huggingface/lerobot
- BYOH 文档: https://huggingface.co/docs/lerobot/integrate_hardware
- SmolVLA: https://huggingface.co/papers/2506.01844
- zero-robotic-arm: https://gitee.com/dearxie/zero-robotic-arm
