# Zero Arm VLA — 系统架构文档

> 版本: 0.2.0 | 日期: 2026-07-14 | 状态: Phase 1 (硬件适配)

---

## 一、系统架构总览

```
┌──────────────────────────────────────────────────────────────────────────┐
│                         上位机 PC (LeRobot)                               │
│                                                                          │
│  ┌─────────────┐   ┌──────────────────┐   ┌───────────────────────────┐ │
│  │ USB 游戏手柄 │   │  MassageRobot    │   │ SmolVLA-450M (PyTorch)    │ │
│  │ (pygame)     │──→│  (Robot 子类)     │──→│ VLM + 流匹配动作专家       │ │
│  │ joystick_    │   │                  │   │                           │ │
│  │ control.py   │   │ serial_protocol.py│  │                           │ │
│  └──────────────┘   └──────┬───────────┘   └───────────────────────────┘ │
│  ┌──────────┐              │ 文本协议, 115200bps, \n 分隔               │
│  │ USB 相机  │─────────────┤                                              │
│  │ OpenCV    │              │                                              │
│  │ 640×480   │              │                                              │
│  │ @30fps    │              │                                              │
│  └──────────┘              │                                              │
└────────────────────────────┼──────────────────────────────────────────────┘
                             │  USB-UART (PA9/PA10)
┌────────────────────────────┼──────────────────────────────────────────────┐
│                    STM32F407VET6 (FreeRTOS)                                │
│                             │                                             │
│  ┌──────────────────────────┴─────────────────────────────────────────┐  │
│  │  UART1 中断接收 -> robot_cmd.c 命令解析                              │  │
│  │                                                                     │  │
│  │  已有命令: remote_enable/disable, remote_event,                    │  │
│  │            rel_rotate, auto, hard_reset, soft_reset, zero           │  │
│  │  新增命令: get_state, set_joints, set_torque, e_stop               │  │
│  └────────────────────────────────────────────────────────────────────┘  │
│                             │                                             │
│  ┌──────────────────────────┴─────────────────────────────────────────┐  │
│  │  robot.c 控制任务 (osPriorityRealtime3)                             │  │
│  │  - 逆运动学 (robot_kinematics.c)                                    │  │
│  │  - PID 控制 (robot_pid_run)                                         │  │
│  │  - 路径插值 (robot_path_interpolation_linear)                       │  │
│  │  - 限位开关检测与保护                                                │  │
│  └──────────────────────────┬────────────────────────────────────────┘  │
│                             │ CAN1 (1Mbps)                                │
│  ┌──────────────────────────┴─────────────────────────────────────────┐  │
│  │  Emm_V5 驱动 (Emm_V5.c)                                             │  │
│  │  - 位置控制: Emm_V5_Pos_Control()                                   │  │
│  │  - 速度控制: Emm_V5_Vel_Control()                                   │  │
│  │  - 同步运动: Emm_V5_Synchronous_motion()                            │  │
│  │  - 状态读取: Emm_V5_Read_Sys_Params(addr, S_CPOS/S_VEL/...)         │  │
│  └──────────────────────────┬────────────────────────────────────────┘  │
└─────────────────────────────┼────────────────────────────────────────────┘
                              │  CAN Bus
              ┌───────────────┼───────────────┬──────────────┐
         ┌────┴────┐    ┌────┴────┐    ┌────┴────┐    ┌────┴────┐
         │ Motor 1 │    │ Motor 2 │    │ Motor 3 │    │ Motor N │
         │ Joint 1 │    │ Joint 2 │    │ Joint 3 │    │ Joint N │
         │ ID:2    │    │ ID:3    │    │ ID:4    │    │ ID:7    │
         └─────────┘    └─────────┘    └─────────┘    └─────────┘
```

> 注：CAN ID = 关节编号 + 1（关节1=ID 2, ... 关节6=ID 7）。
> 手柄通过独立路径（joystick_control.py -> 串口 remote_event）直接控制机械臂。

---

## 二、四层架构详解

### Layer 0: 力控安全层 (STM32 硬件)

| 组件 | 功能 | 独立性 |
|------|------|:------:|
| **限位开关** | 每个关节配备物理限位，触发时立即停止 | 硬件级，不受软件控制 |
| **急停** | STM32 检测 `e_stop` 命令，广播停止所有电机 | 固件级，不依赖 PC |
| **力阈值** | `Present_Load` 持续监测，超阈值自动停机 | 固件级 |
| **看门狗** | PC 通信 500ms 无心跳 -> 自动停止 | 固件级 |

### Layer 1: 动作执行层 (PC + STM32)

- **STM32 侧**: PID 闭环位置控制、路径插值、逆运动学
- **PC 侧**: `MassageRobot.send_action()` -> 串口 -> STM32 执行
- **通信**: 文本协议, `set_joints j1 j2 ... j6\n`
- **手柄遥控**: `joystick_control.py` 通过 `remote_event` / `rel_rotate` 直接控制

#### 两条控制路径

```
策略推理路径: SmolVLA -> send_action -> set_joints -> STM32 -> 电机
手柄遥控路径: 手柄 -> remote_event -> STM32 -> 电机 (绕过了 MassageRobot)
```

### Layer 2: 穴位感知层 (PC)

- 输入: RGB-D 相机图像
- 输出: 背部穴位 3D 坐标
- 方案: MediaPipe / 自定义关键点检测网络
- 集成: 独立的 Python 模块，向 VLA 模型提供穴位坐标

### Layer 3: 语义决策层 (PC)

- 模型: SmolVLA-450M
- 输入: 相机帧 + 关节状态 + 自然语言指令
- 输出: 未来 N 步目标关节角度序列
- 推理: 异步模式 (action chunk 预测 + 边执行边推理)

---

## 三、PC<->STM32 串口协议规范

### 物理层

| 参数 | 值 |
|------|-----|
| 接口 | USART1 (PA9=TX, PA10=RX) |
| 波特率 | 115200 bps |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验位 | None |
| 流控 | None |
| 编码 | ASCII, `\n` (LF) 行终止符 |

### 命令表

#### 已有命令（来自 zero-robotic-arm）

| 命令 | 格式 | 功能 |
|------|------|------|
| `remote_enable` | `remote_enable\n` | 使能远程控制 + 软复位 |
| `remote_disable` | `remote_disable\n` | 禁用远程控制 + 软复位 |
| `remote_event` | `remote_event vx vy vz rx ry rz\n` | 笛卡尔空间速度控制 (手柄用) |
| `rel_rotate` | `rel_rotate joint_id angle\n` | 关节相对旋转 (手柄关节模式用) |
| `auto` | `auto x y z\n` | 逆运动学自动定位 |
| `hard_reset` | `hard_reset\n` | 限位开关归零复位 |
| `soft_reset` | `soft_reset\n` | 软件复位 |
| `zero` | `zero\n` | 当前位置设为零位 |

#### 新增命令（LeRobot 适配）

| 命令 | 格式 | 功能 | 响应 |
|------|------|------|------|
| `get_state` | `get_state\n` | 请求当前关节状态 | `STATE:j1,...,j6,s1,...,s6,l1,...,l6\n` |
| `set_joints` | `set_joints j1 j2 j3 j4 j5 j6\n` | 设置全部目标角度(°) | `OK\n` |
| `set_torque` | `set_torque 0/1\n` | 扭矩使能/禁用 | `OK\n` 或 `OK:FREE\n` |
| `e_stop` | `e_stop\n` | 紧急停止所有电机 | `ESTOP\n` |

#### 状态响应格式

```
STATE:j1,j2,j3,j4,j5,j6,s1,s2,s3,s4,s5,s6,l1,l2,l3,l4,l5,l6\n
      +-- 关节角度(°) --++-- 速度(rpm) --++-- 负载(%) --+
```

---

## 四、数据流管线

### 4.1 数据采集管线（手动示教）

```
[人类操作员]
    | 手动拖拽机械臂 (torque off)
    v
[Emm_V5 电机]
    | 编码器读取实际位置
    v
[STM32 get_state]
    | STATE:j1,...,j6,...
    v (串口 115200)
[MassageRobot.get_observation()]
    | + shoulder_pan.pos = 90.0, shoulder_lift.pos = 45.0, ...
    | + cam_top = frame (640x480x3)
    v
[LeRobot record pipeline]
    | 自动加 obs_state / obs_images 前缀 + 时间戳对齐 + parquet 存盘
    v
datasets/massage_v1/
  +-- data/chunk-000001/
  |   +-- episode_0001.parquet
  |   +-- episode_0001/
  |   |   +-- cam_top_000000.png
  |   |   +-- ...
  +-- meta/episodes.jsonl
```

### 4.2 手柄遥操作数据采集

```
[操作员]
    | 摇杆 + 按键
    v
[joystick_control.py]                   [record_trajectory.py / 自定义录制]
    |                                        |
    | remote_event / rel_rotate              | get_state + cam.read_latest()
    v                                        v
[STM32 执行]                              [datasets/raw/episode_XXXX/]
                                              +-- data.json (角度+时间戳)
                                              +-- frame_000000.png
                                              +-- frame_000001.png
                                              +-- ...
```

> 数据采集在同一台 PC 上分两个进程进行，互不干扰。

### 4.3 训练管线

```
datasets/massage_v1/
    |
    v
LeRobot train.py --policy.type=smolvla
    |
    +-- DataLoader: 批量加载 (相机帧, 关节角, 时间戳)
    +-- VLM (SigLIP + SmolLM2): 编码图像 + 指令 -> 特征
    +-- Action Expert (Flow Matching Transformer): 特征 -> 动作序列
    +-- Loss: 流匹配损失
    |
    v
outputs/train/smolvla_massage/
```

### 4.4 推理管线（异步模式）

```
[相机帧 + 指令 "按揉大椎穴"]
    |
    v (异步)
[SmolVLA 推理服务器]           [机械臂执行]
    |                              |
    +-- 预测 action chunk 0 ------+ 执行 chunk 0
    |                              | (边执行边预测)
    +-- 预测 action chunk 1 ------+ 执行 chunk 1
    |                              |
    +-- ...                       ...
```

---

## 五、LeRobot 集成点

### MassageRobot 子类

```
lerobot.robots.Robot (ABC)
    |
    +-- MassageRobot
        +-- 通信: pyserial -> STM32 USART1
        +-- 相机: OpenCVCamera (__init__ 时构建)
        +-- 特征键: {joint}.pos (每关节 float，非整向量)
        +-- connect()        -> 打开串口 + 相机
        +-- disconnect()     -> 关闭串口 + 相机
        +-- get_observation()  -> 返回 {joint}.pos + {camera_name} dict
        +-- send_action()     -> 接收 {joint}.pos dict，发送 set_joints
        +-- calibrate()      -> zero 命令（显式调用）
        +-- configure()      -> set_torque 1
```

### LeRobot 接口规范（0.6+）

| 要求 | 本实现 | 说明 |
|------|--------|------|
| `action_features` 为 `.pos` 键 dict | `{shoulder_pan.pos: float, ...}` | 对齐官方 so_follower |
| `send_action` 收发 dict | `action: dict[str,Any] -> dict[str,Any]` | 遵循 `RobotAction` 类型 |
| 连接前特征可调用 | 相机在 `__init__` 构建 | 消除运行时依赖 |
| 包名前缀 `lerobot_robot_` | `lerobot_robot_massage` | 自动发现规范 |

### 不使用 FeetechMotorsBus 的原因

- `FeetechMotorsBus` 为 RS485 Feetech 协议设计，直接操作舵机寄存器
- 本架构中 STM32 是唯一的电机控制器 (CAN -> Emm_V5)
- PC 无法直接访问电机，因此绕过 MotorsBus 抽象层更合理

---

## 六、Emm_V5 vs Feetech STS3215 架构对比

| 维度 | 本系统 (Emm_V5) | LeRobot 标准 (STS3215) |
|------|:---------------:|:----------------------:|
| 通信总线 | CAN | RS485 (半双工 UART) |
| 电机类型 | 步进闭环电机 | 伺服舵机 |
| 控制模式 | 位置/速度/同步运动 | 位置/速度/PWM |
| PC 连接方式 | 通过 STM32 (串口) | 直接 USB-RS485 |
| LeRobot 驱动 | 自定义 Robot 子类 | FeetechMotorsBus |
| 精度 | 编码器闭环, 0.1 度 | 电位器, 0.3 度 |
| 力矩 | 大 (步进 + 减速器) | 中 (舵机直驱或小减速) |

---

## 七、安全机制设计

### 多层防护

```
Layer 0: 硬件防护
  +-- 物理急停按钮 (切断电机电源)
  +-- 限位开关 (各关节)
  +-- 24V 电源保险丝

Layer 1: STM32 固件防护
  +-- 软件急停 (e_stop 命令)
  +-- 力/扭矩阈值监控 (自动停机)
  +-- 通信看门狗 (500ms 无响应 -> 停机)
  +-- 关节角度限位检查

Layer 2: PC 端防护
  +-- send_action() 前角度裁剪 (+-180 度)
  +-- 连续通信失败检测 (>5 次 -> 触发 e_stop)
  +-- 模型输出安全校验

Layer 3: 操作流程防护
  +-- 先在硅胶假体上验证
  +-- 禁止人体直接测试未验证模型
  +-- 操作员始终在急停按钮旁
```

---

## 八、目录结构

```
zero_arm_VLA/
+-- CLAUDE.md                           # 项目总纲
+-- README.md                           # 项目说明
+-- .gitignore
+-- docs/
|   +-- ARCHITECTURE.md                 # 本文件
|   +-- DEPLOYMENT.md                   # 部署教程 (从0到训练)
|   +-- WORKFLOW.md                     # 工作流
|   +-- SERIAL_COMMANDS.md              # 串口命令参考
+-- firmware/
|   +-- README.md                       # STM32 固件修改指南
|   +-- src/                            # 关键修改源文件
+-- lerobot_robot_massage/
|   +-- __init__.py
|   +-- pyproject.toml                  # 可安装插件包
|   +-- serial_protocol.py              # 串口协议封装
|   +-- config_massage_robot.py         # 配置数据类
|   +-- massage_robot.py               # LeRobot Robot 子类
+-- configs/
|   +-- massage_robot.yaml              # 机器人配置
|   +-- train_smolvla.yaml              # 训练配置
+-- scripts/
|   +-- joystick_control.py             # USB 手柄遥控
|   +-- record_trajectory.py            # 轨迹录制 (待创建)
|   +-- verify_interface.py             # 接口规范验证
|   +-- test_serial.py                  # 串口测试
+-- datasets/                           # [gitignored] 采集数据
+-- outputs/                            # [gitignored] 训练模型
+-- experiments/                        # 实验记录
```

---

## 九、参考

- [zero-robotic-arm 源码](https://gitee.com/dearxie/zero-robotic-arm) (`d:\robo arm\software\zero-robotic-arm`)
- [LeRobot BYOH 文档](https://huggingface.co/docs/lerobot/integrate_hardware)
- [SmolVLA 技术报告](https://huggingface.co/papers/2506.01844)
- [Emm_V5 驱动说明](https://zhangdatou.taobao.com)
- [AmazingHand 灵巧手](https://github.com/pollen-robotics/AmazingHand)
