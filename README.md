# Arm-robot_VLA — 6-DOF 机械臂控制与 LeRobot 具身智能系统

> `Arm-robot_VLA` 是 TuinaDex 项目的机械臂子系统，涵盖基于 **SocketCAN 直连的 ZDT 闭环步进电机驱动体系**、**6-DOF 空间运动学/动力学/笛卡尔控制器**、**MuJoCo 物理仿真** 以及与 **HuggingFace LeRobot 具身智能生态** 的深度集成（支持 SmolVLA / ACT 策略采集、训练与推理）。

---

## 1. 代码框架与目录结构详解

```text
Arm-robot_VLA/
├── lerobot_robot_massage/                 # LeRobot 机器人插件包 (核心驱动与控制算法)
│   ├── zdt/                               # ★ ZDT 闭环步进驱动器与 6-DOF 运动学核心
│   │   ├── controller.py                  # ZdtController: 6 轴状态机、单步位置积分与同步使能
│   │   ├── cartesian.py                   # CartesianController: 阻尼最小二乘 (DLS) 笛卡尔逆解
│   │   ├── kinematics.py                  # MDH 正逆运动学、空间雅可比矩阵 (Jacobian) 计算
│   │   ├── safety.py                      # 硬件安全门禁 (电压/温度/堵转/通信超时看门狗)
│   │   ├── can_transport.py               # Linux SocketCAN 原生接口通信封装
│   │   ├── zdt_bus.py / zdt_driver.py     # ZDT Emm42 V5.0 驱动器二进制帧编解码 (0xFD/0x1F)
│   │   ├── config.py / params.py          # 减速比 (51:1/27:1)、回原点极性与限位参数表
│   │   ├── scan.py                        # CAN 总线自动扫描与广播枚举
│   │   ├── recording.py                   # 轨迹记录与回放工具
│   │   ├── workspace.py                   # 空间工作空间与奇异点检测
│   │   └── test_*.py                      # 230 项高可靠单元测试套件 (覆盖率 100%)
│   ├── config_massage_robot.py            # MassageRobotConfig: LeRobot 机器人配置类
│   ├── massage_robot.py                   # MassageRobot(Robot): LeRobot 标准机器人接口封装
│   └── serial_protocol.py                 # PC ↔ STM32 串口协议向下兼容封装
│
├── scripts/                               # 运维、测试与控制脚本库
│   ├── bringup/                           # 硬件 Bringup、CAN 诊断与交互面板
│   │   ├── zdt_bringup.py                 # 基础命令行快速使能与单轴调试
│   │   ├── zdt_interactive.py             # 交互式全功能终端控制台 (按键微调/归零/状态查询)
│   │   ├── zdt_panel.py                   # ncurses 终端全屏实时监控面板
│   │   ├── zdt_anchor.py                  # 机械臂预设姿态保存与恢复
│   │   ├── can_setup.sh                   # SocketCAN can0 @ 500kbps 快速启动脚本
│   │   └── startup.sh / deploy_remote.sh  # 系统开机自启与远端部署工具
│   ├── control/                           # 多模态实时控制
│   │   ├── joystick_control.py            # USB 手柄 (Xbox/PS) 遥控机械臂 6-DOF 笛卡尔空间
│   │   ├── cartesian_keyboard.py          # 纯键盘 6 轴空间位姿微调控制
│   │   └── control_hub.py                 # 多窗口集中控制中枢
│   ├── simulation/                        # MuJoCo 物理仿真与数字孪生
│   │   ├── mujoco_sim.py                  # 仿真机械臂节点 (含 7DOF/6DOF IK 与环境交互)
│   │   ├── scene.xml / meshes/            # MuJoCo 机械臂三维模型与 URDF 几何网格
│   │   ├── camera_server.py / shm_util.py # 共享内存多路虚拟相机流广播
│   │   └── record_sim.py                  # 仿真环境数据集采集录制
│   ├── data_tools/                        # 数据处理与模型评估
│   │   ├── convert_to_lerobot.py          # 自定义数据格式转换为 LeRobot 标准数据集
│   │   ├── diagnose_model.py              # 模型权重与推理延迟诊断
│   │   └── evaluate_policy.py             # 离线/在线策略评估与成功率统计
│   └── teleop/                            # 单臂视觉遥操工具集 (跨协同遥操请优先用根目录 Co_Teleop)
│       ├── real_arm_teleop.py             # 单机械臂 6-DOF 视觉遥操
│       ├── handeye_calib.py               # 交互式 3D 手眼标定向导
│       ├── arm_adapter.py                 # 机械臂控制适配层
│       ├── watchdog.py                    # 视觉丢失看门狗
│       └── teleop_config.yaml             # 视觉遥操配置文件
│
├── configs/                               # 训练与算法配置文件
│   ├── massage_robot.yaml                 # 机械臂硬件与接口映射配置
│   └── train_smolvla.yaml                 # SmolVLA 具身大模型训练超参数
│
├── firmware/                              # STM32F407 固件源码 (可选硬件网关)
│   ├── README.md                          # 固件编译与烧录指南
│   └── src/ (robot.c, robot_cmd.c, ...)   # FreeRTOS 多任务调度与双向通信实现
│
├── docs/                                  # 详尽设计与使用文档
│   ├── ARCHITECTURE.md                    # 软硬件分层架构设计文档
│   ├── DEPLOYMENT.md                      # 部署、调试与具身训练全流程指南
│   ├── KEYBOARD_TELEOP_GUIDE.md           # 6-DOF 笛卡尔键盘遥控使用指南与原理
│   ├── MUJOCO_SIM.md                      # MuJoCo 物理仿真与数字孪生使用指南
│   ├── HARDWARE_CAN_SPEC.md               # 底层驱动器、多圈编码器与 CAN 协议手册
│   ├── Emm_V5.0_步进闭环驱动说明书.md      # 驱动器出厂技术说明书
│   ├── SERIAL_COMMANDS.md                 # 串口与 CAN 指令通讯协议字典
│   └── WORKFLOW.md                        # 工程开发与测试工作流规范
│
├── datasets/                              # 录制的示教数据集 (gitignored)
├── outputs/                               # 策略训练权重输出 (gitignored)
└── experiments/                           # 实验记录与日志
```

---

## 2. 子文件夹及子子文件夹功能详解

### 2.1 `lerobot_robot_massage/` (核心算法与驱动包)
- **`zdt/` (核心步进电机驱动与空间控制)**：
  - 本项目的核心底层引擎。实现了 Linux 原生 **SocketCAN 传输 (`can_transport.py`)**，直接向 6 个 Emm42 闭环驱动器发送 `0xFD` 相对脉冲报文与 `0x1F` 状态查询；
  - **`cartesian.py` & `kinematics.py`**：基于修正 DH 参数（MDH）建立正向运动学，并使用阻尼最小二乘法（Damped Least Squares, DLS）进行笛卡尔线速度/角速度到 6 轴角速度的逆解映射，内建奇异点避免与单步 30° 步长硬限幅；
  - **`safety.py`**：实现严格状态机（`UNINITIALIZED → SAFE_IDLE → ARMED → TELEOP → FAULT / STOPPED`），包含 50ms 通信超时心跳保护；
  - **`test_*.py`**：230 个严密的单元测试，确保运动学积分、帧编解码与安全状态机无死角。
- **`config_massage_robot.py` & `massage_robot.py`**：
  - 继承 HuggingFace LeRobot 的 `Robot` 基类，注册为 `massage_robot` 机器人实体，使机械臂能够直接无缝使用 `lerobot-record`, `lerobot-train`, `lerobot-visualize` 命令行工具。

### 2.2 `scripts/` (独立运维与控制工具)
- **`bringup/` (硬件调试与 Bringup)**：
  - 提供系统上线前的一站式检测：`can_setup.sh` 快速配置 CAN 总线波特率；`zdt_interactive.py` 提供单轴点动、回零、电流监控等交互菜单；`zdt_panel.py` 提供终端全屏 HUD 遥测监控。
- **`control/` (手柄与多模态控制)**：
  - `joystick_control.py` 支持通过 Xbox/PS 无线手柄摇杆平滑控制机械臂末端在 XYZ 空间平移与姿态旋转，集成平滑加减速算法。
- **`simulation/` (MuJoCo 仿真与数字孪生)**：
  - 基于 MuJoCo 物理引擎构建了完整的 6-DOF 仿真机械臂，支持共享内存相机画面推流 (`camera_server.py`)，供算法在无物理硬件时进行策略训练与闭环验证。
- **`data_tools/` (数据与模型工具)**：
  - 负责数据集清洗、格式转换 (`convert_to_lerobot.py`) 以及 SmolVLA 具身模型推理诊断 (`diagnose_model.py`)。

---

## 3. 快速上手指引 (Quick Start)

### 3.1 激活 SocketCAN 接口
```bash
sudo ip link set can0 up type can bitrate 500000
```

### 3.2 运行交互式 Bringup 调试面板
```bash
python scripts/bringup/zdt_interactive.py --iface can0
```

### 3.3 运行手柄遥控 (Xbox / USB 手柄)
```bash
python scripts/control/joystick_control.py --iface can0 -y
```

### 3.4 启动 MuJoCo 物理仿真
```bash
python scripts/simulation/mujoco_sim.py
```

### 3.5 运行全套单元测试
```bash
pytest lerobot_robot_massage/zdt/
# 预期: 230 passed in ~3.9s
```

---

## 4. 协同遥操说明
如需进行 **机械臂 + 灵巧手 22-DOF 协同视觉遥操**，请使用顶层平级模块：
👉 [Co_Teleop 模块文档](../Co_Teleop/README.md) 或在根目录直接运行 `python run_teleop.py --iface can0 -y`。
