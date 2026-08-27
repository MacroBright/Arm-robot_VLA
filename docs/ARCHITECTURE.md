# Arm-robot_VLA — 系统架构设计文档 (ARCHITECTURE)

> **版本**：2.0.0 (SocketCAN 直连与 6-DOF 闭环笛卡尔控制器架构, ADR-005)
> **状态**：Production Ready | 单元测试 230 项 100% 通过

---

## 目录

- [1. 系统架构总览](#1-系统架构总览)
- [2. 四层软硬件分层设计](#2-四层软硬件分层设计)
- [3. 控制链路与通信协议](#3-控制链路与通信协议)
- [4. 运动学与笛卡尔空间控制算法](#4-运动学与笛卡尔空间控制算法)
- [5. LeRobot 具身智能与策略训练集成](#5-lerobot-具身智能与策略训练集成)
- [6. 安全状态机与多级看门狗](#6-安全状态机与多级看门狗)

---

## 1. 系统架构总览

```text
┌─────────────────────────────────────────────────────────────────────────────┐
│                       【应用层 / 协同遥操 / AI 策略】                         │
│   • 22-DOF 臂-手协同视觉遥操 (Co_Teleop/run_teleop.py)                       │
│   • SmolVLA 具身多模态策略推理 (SmolVLAPolicy / PyTorch)                     │
│   • USB 游戏手柄遥控 (scripts/control/joystick_control.py)                   │
│   • 笛卡尔键盘精密示教 (scripts/control/cartesian_keyboard.py)               │
└──────────────────────────────────────┬──────────────────────────────────────┘
                                       │ 笛卡尔指令: 线速度 v, 角速度 w (50ms 周期)
┌──────────────────────────────────────▼──────────────────────────────────────┐
│                    【运动学与笛卡尔控制层 (Cartesian Level)】                 │
│   • CartesianController (MDH 正向运动学 + 空间雅可比计算)                    │
│   • 阻尼最小二乘法 (Damped Least Squares, DLS) 逆解: dq = J† · twist          │
│   • 奇异点阻尼重整化 + 单步 30° 最大跳变限幅                                  │
└──────────────────────────────────────┬──────────────────────────────────────┘
                                       │ 6 轴目标相对转角增量 (dq1 ~ dq6)
┌──────────────────────────────────────▼──────────────────────────────────────┐
│                    【ZDT 驱动与状态机层 (Driver & Safety)】                  │
│   • ZdtController: 状态机 (SAFE_IDLE → ARMED → TELEOP → STOPPED)           │
│   • 50ms 通信看门狗心跳检测 + 电流/堵转过载保护                              │
│   • 0xFD 相对位置脉冲生成 (65536 脉冲/圈) + 0x1F 状态回读                    │
└──────────────────────────────────────┬──────────────────────────────────────┘
                                       │ Linux 原生 SocketCAN (can0 @ 500 kbps)
┌──────────────────────────────────────▼──────────────────────────────────────┐
│                    【底层硬件执行层 (Physical Actuators)】                  │
│   • 6 × ZDT / Emm42 V5.0 步进闭环驱动器 (CAN ID 0x02 ~ 0x07)                 │
│   • 减速箱: J1~J4/J6 (51:1 减速比), J5 (27:1 减速比)                         │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. 四层软硬件分层设计

### Layer 0: 物理执行与驱动层 (Hardware & Actuation)
- 6 个闭环步进驱动器并联在 500kbps CAN 总线上；
- 硬件端集成 FOC 闭环磁编码器，实现失步自动补偿与堵转过流检测。

### Layer 1: 总线与协议驱动层 (SocketCAN & Frame Driver)
- `can_transport.py`：基于 Linux 原生 SocketCAN 协议栈收发 8-byte 扩展帧；
- `zdt_driver.py` & `zdt_bus.py`：驱动器二进制帧编解码与多圈绝对角度读取。

### Layer 2: 运动学与空间控制层 (Kinematics & Cartesian)
- 基于修正 DH（MDH）参数构建 6 轴空间链；
- 采用微分运动学雅可比逆解算法，将末端 6DOF 速度矢量映射到 6 个关节轴，确保运动平顺性。

### Layer 3: 具身智能与交互应用层 (Embodied AI & Teleop)
- 封装为 LeRobot 标准 `Robot` 插件（`MassageRobot`）；
- 支持多模态手柄/键盘/视觉遥控与 SmolVLA 数据集采集、模型训练。

---

## 3. 控制链路与通信协议

1. **PC SocketCAN 直连（默认主力链路）**：
   - 接口：`can0` @ 500 kbps；
   - 周期：50ms（20Hz 控制闭环）；
   - 报文：`0xFD` 相对位置运动，`0x1F` / `0x36` 角度状态回读。
2. **PC-STM32 UART 串口（向下兼容链路）**：
   - 接口：`USART1` @ 115200 bps；
   - 协议：行文本 ASCII 协议（`get_state`, `set_joints`, `remote_event`）。

---

## 4. 运动学与笛卡尔空间控制算法

- **正向运动学 (FK)**：通过齐次变换矩阵逐步乘积计算末端在基坐标系下的位姿 $T_0^6 \in SE(3)$；
- **雅可比矩阵 (Jacobian)**：计算 $6 \times 6$ 几何雅可比矩阵 $\boldsymbol{J}(\boldsymbol{q})$；
- **DLS 阻尼最小二乘逆解**：
  $$\Delta \boldsymbol{q} = \boldsymbol{J}^T (\boldsymbol{J}\boldsymbol{J}^T + \lambda^2 \boldsymbol{I})^{-1} \boldsymbol{v}_{twist} \Delta t$$
- 阻尼因子 $\lambda$ 在接近奇异点时动态自适应增大，保证逆解矩阵条件数良好，彻底杜绝关节突变飞车。

---

## 5. LeRobot 具身智能与策略训练集成

- **注册机制**：通过 `lerobot_robot_massage/config_massage_robot.py` 动态注册为 LeRobot 机器人；
- **观测空间 (Observation)**：`observation.state` (6-DOF 关节角/角速度) + `observation.images.phone` (相机 RGB 画面)；
- **动作空间 (Action)**：`action` (6-DOF 目标关节角度或末端位姿增量)；
- **模型支持**：支持 SmolVLA、ACT、Diffusion Policy 等流匹配模型直接加载训练。

---

## 6. 安全状态机与多级看门狗

```text
       ┌────────────────┐
       │ UNINITIALIZED  │
       └───────┬────────┘
               │ connect() (总线连接, 扭矩释放)
       ┌───────▼────────┐
       │   SAFE_IDLE    │
       └───────┬────────┘
               │ arm(gravity_confirmed=True) (闭环上电)
       ┌───────▼────────┐
       │     ARMED      │ ◄──────────┐
       └───────┬────────┘            │
               │ enter_teleop()      │ exit_teleop()
       ┌───────▼────────┐            │
       │     TELEOP     ├────────────┘
       └───────┬────────┘
               │ 通信超时 (>50ms) / 急停广播 (0x0000)
       ┌───────▼────────┐
       │ STOPPED/FAULT  │
       └────────────────┘
```
- **重力关节安全确认**：机械臂上电时必须显式传入 `gravity_confirmed=True`，防止大臂/小臂掉落；
- **通信心跳看门狗**：任意时刻若连续 50ms 未收到上位机心跳包，驱动器强制进入制动状态。
