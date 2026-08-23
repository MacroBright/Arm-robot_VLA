# PC 直连 CAN 控制 + 视觉遥操 + 香橙派边缘部署 设计方案

> 日期：2026-08-15 | 状态：设计定稿，待实现
> 范围：机械臂（6×ZDT X系列V2 闭环步进驱动）PC 直连 CAN 控制；Leap_Hand 腕部关键点视觉遥操接入；LeRobot 集成；最终部署到香橙派 AI Pro 边缘节点。
> 依据：`docs/2026-08-14-zdt-driver-setup.md`、`docs/SERIAL_COMMANDS.md`、`docs/ARCHITECTURE.md`、参考固件 `armbot_example/zero-robotic-arm-master`、Leap_Hand SDK、LeRobot BYOH 文档。
> 调研（2026-08-15 GitHub）：`timessage/zeroarm-ros2-can`（SocketCAN 直连 Emm_V5）、`zhanyinan150/cantest`（ZDT 协议 + S_FLAG=0x3A）、`unitreerobotics/unitree_lerobot`（臂+手 LeRobot 流程）、`hexchip/lerobot-on-ascend`（Orange Pi AIpro=昇腾310B 跑 LeRobot）、`imitrob/teleop_gesture_toolbox`（WristTracker 参考来源）。

---

## 0. 决策摘要

| # | 决策 | 理由 |
|---|------|------|
| D1 | **PC 永久直连 CAN 控制机械臂，绕过 STM32** | 用户决策：去掉串口瓶颈。**取代 ADR-001**（STM32 作为唯一控制网关） |
| D2 | CAN 收发器选 **SocketCAN 兼容器**（CANable/candleLight 类，`gs_usb`） | Linux 内核原生支持，python-can socketcan 后端，两端（x86/aarch64）一致 |
| D3 | 波特率 **500k**（扩展帧） | 参考固件寄存器算得：42MHz / (14 × (1+4+1)) = 500kHz；与用户确认一致 |
| D4 | 首版范围：**关节级 + 笛卡尔遥操** | 用户选择。含 set_joints/get_state/set_torque/rel_rotate/e_stop/soft_reset + remote_event/end_event |
| D5 | IK 用**参考固件解析式闭式解**（移植 `robot_kinematics.c`），**不搬** MuJoCo DLS | 真机物理几何 ground truth，闭式解无迭代抖动；MuJoCo DLS 仅供仿真 |
| D6 | 遥操接入点 = **`ArmClient` CAN 后端**（`CanArmClient`），`demo_arm_teleop.py` 零改动 | Leap_Hand 视觉遥操只依赖 `ArmClient` 抽象，接口与 CAN 控制器一一对应 |
| D7 | 目标架构：**PC=训练+推理，香橙派=实时控制+采集+执行** | 香橙派无 CUDA/NPU 训练 VLM 能力；SmolVLA 训练必须留 PC/GPU。部署=PC 推理+action chunk 流式下发 |
| D8 | 传输层隔离，保证 **PC→香橙派 零重写移植** | 全纯 Python + SocketCAN，无 x86 专属依赖；移植=重装依赖+改配置 |

---

## 1. 架构分层与目录结构

### 1.1 分层

```
┌──────────────────────────────────────────────────────────────┐
│ 应用层（不变）                                                │
│   MassageRobot(LeRobot) / joystick_control / record / hub     │
│   ↑ 接口等价: get_state set_joints set_torque e_stop          │
│              rel_rotate soft_reset remote_event end_event     │
├──────────────────────────────────────────────────────────────┤
│ 控制器/语义层（PC 新增，替代 STM32 robot.c 的角色）            │
│   ZdtController: 限位clamp · e_stop · 看门狗 · 电流力控        │
│   remote_event 语义（复用 remote_semantics.py 纯函数）         │
│   kinematics.py: FK + 解析式IK（移植 robot_kinematics.c）       │
│   get_state 聚合: 角度/速度/电流                              │
├──────────────────────────────────────────────────────────────┤
│ ZDT 驱动层（新增）                                            │
│   帧编解码: 扩展帧 ID(Addr<<8|seq) · 0x6B校验 · >7B数据拆包     │
│   命令集: enable/stop/pos(0xFB)/vel(0xF6)/read_pos(0x36)/     │
│           read_current(0x27)/zero(0x93)/home(0x9A)/param      │
│   超时重试 + 响应状态机 + 0xfd 到位事件                        │
├──────────────────────────────────────────────────────────────┤
│ 传输层                                                        │
│   SocketCAN can0 (500k, 扩展帧) ← gs_usb USB-CAN 适配器        │
│   python-can (socketcan backend)                              │
└──────────────────────────────────────────────────────────────┘
```

### 1.2 目录结构

```
lerobot_robot_massage/
├── massage_robot.py          # LeRobot Robot 子类（transport: can|serial 可切）
├── serial_protocol.py        # 保留 —— STM32 并行路径（A/B 对比用）
├── config_massage_robot.py   # 增加 transport 字段
└── zdt/
    ├── can_transport.py      # SocketCAN 封装（python-can, can0）
    ├── zdt_driver.py         # ZDT 帧编解码 + 命令集 + 拆包/校验/重试
    ├── controller.py         # ZdtController: 限位/e_stop/看门狗/电流力控/状态聚合
    ├── kinematics.py         # 纯Python FK + 解析式IK（移植 robot_kinematics.c）
    └── config.py             # 波特率/ID映射/限位表/增益
scripts/
├── joystick_control.py       # 加 --transport can
└── zdt_bringup.py            # 新增: bring-up 验证 CLI（单轴步进/回零/限位扫掠）
```

### 1.3 数据流

```
控制: joystick/LeRobot/遥操 → controller(限位+看门狗+力控) → zdt_driver(编码/拆包)
      → can_transport → can0 → ZDT 驱动
状态: ZDT 回帧 → can_transport → zdt_driver(校验/组包) → controller(聚合)
      → get_state 输出 (角度/速度/电流) / FK → 末端位姿
到位: 0xfd 到位帧 → 事件钩子（替代固定延时轮询）
```

> **serial_protocol.py 保留**：永久直连下串口路径仅用于 A/B 验证和 STM32 调试，不承担生产路径。

---

## 2. ZDT 驱动层（帧格式已核实）

### 2.1 组帧约定（参考固件 `can.c` + `Emm_V5.c` 核实，权威）

参考固件 `armbot_example/zero-robotic-arm-master/2. Software/robot/Core/Src/can.c:161` `can_SendCmd()`：

```c
can.CAN_TxMsg.ExtId = ((uint32_t)cmd[0] << 8) | (uint32_t)packNum;  // cmd[0]=地址 → 帧ID高字节
can.txData[0] = cmd[1];                                            // cmd[1]=功能码 → 数据段[0]
// cmd[2:] → 数据段[1:], 末字节 = 0x6B 校验
// k≥8 → 拆包: 每帧 DLC=8, 功能码重复于每包数据[0], 包序号=ID低字节
```

**定稿约定**：

```
扩展帧 ID = (地址 << 8) | 包序号
数据段    = [功能码] [参数...] [0x6B 校验]
规则: cmd[0]=地址→帧ID高字节, cmd[1]=功能码→数据段[0], cmd[2:]=参数, 末字节=0x6B
拆包: 参数部分 >7 字节 → 拆多帧, 每帧 DLC=8, 功能码重复于每包数据[0], 包序号递增
```

> 注：ZDT 文档附录 A 内部"首字节=功能码/地址"两种说法**均已化解**——地址进帧 ID，功能码进数据段首字节，两者不冲突。

### 2.2 关节→CAN 地址映射

| 关节 | 名称 | 角度范围(°) | CAN 地址 | 帧 ID |
|:----:|------|:--------:|:--------:|:-----:|
| 1 | shoulder_pan | 0~360 | 02 | 0x0200 |
| 2 | shoulder_lift | 90~180 | 03 | 0x0300 |
| 3 | elbow_flex | -90~90 | 04 | 0x0400 |
| 4 | wrist_roll | -90~90 | 05 | 0x0500 |
| 5 | wrist_flex | 0~90 | 06 | 0x0600 |
| 6 | gripper | 0~360 | 07 | 0x0700 |

> J4/J5/J6 物理行为以真机实测为准（SERIAL_COMMANDS.md 2026-08-11 标注：J4 末端旋转、J5 末端上下、J6 末端旋转）。

### 2.3 命令集

**核心控制（必需）**：

| 命令 | 功能码 | 载荷 | 对应高层操作 |
|------|:------:|------|--------------|
| 使能/失能 | `0xF3` | `AB 01/00 [snF]` | `set_torque` 底层；失能用于手动示教 |
| 立即停止 | `0xFE` | `98 [snF]` | `e_stop`（广播 `00 FE 98 00`） |
| 直通限速位置 | `0xFB` | `00/01 速度2B 位置3B 0A 同步1B` | `set_joints`/`rel_rotate`/`soft_reset`（位置×10，绝对=01/相对=00，速度 RPM×10） |
| 速度模式 | `0xF6` | `斜率2B 速度2B 同步1B` | `remote_event`/`end_event` 关节速度（RPM×10） |
| 读实时位置 | `0x36` | `-` | `get_state` 角度（×10） |
| 读相电流 | `0x27` | `-` | `get_state` 电流（mA）→ 力控 |
| 读状态标志 | `0x3A` | `-` | `read_flag()` 状态字节：`&0x01=使能 &0x02=到位 &0x04=堵转 &0x08=堵转保护`（2026-08-15 GitHub 调研 `cantest/Emm_V5_CAN.c` 发现；安全层堵转监测直接轮询此命令，优于监听 0xfd 到位帧） |

> **⚠ 位置命令代际差异（2026-08-15 GitHub 调研）**：GitHub 上广泛使用的 Emm_V5 实现（`cantest`/`zeroarm-ros2-can`）位置控制用 **`0xFD`（4 字节脉冲数）**，而 ZDT 文档标注 **`0xFB`（直通限速位置）**。使能 F3/停止 FE/读位置 36 两代一致，**唯位置命令不同**。真机 bring-up 必须先用 candump 对照 STM32 已走通的报文确认本机 X-series V2 用哪个，再定驱动层 `move_abs`/`move_rel` 的载荷布局（plan Task 8 前置）。

**bring-up/标定（工具用，不进核心路径）**：

| 命令 | 功能码 | 说明 |
|------|:------:|------|
| 设单圈零点 | `0x93` | 零点校准 |
| 触发回零 | `0x9A` | 回零 |
| 读写参数 | `0x42`/`0x48` | 逐轴核验 CAN_Baud/ID/Clog 参数（37B 拆包） |

### 2.4 响应状态机

ZDT 回帧带地址（ID 高字节→addr→关节号）。驱动层按 ID 匹配：

```
send(request) → 等待同 ID 回帧
    ├─ 收到 → 校验 0x6B + 长度 → 按功能码解析 → 返回
    ├─ 超时 (timeout_ms=100, 重试×3) → 仍失败 → TimeoutError
    └─ Response=None 配置时无控制回帧 → 依赖显式读命令轮询确认
```

**错误分类**：

| 异常 | 触发 | 上层动作 |
|------|------|----------|
| `ChecksumError` | 末字节 ≠ 0x6B | 丢弃重收 |
| `TimeoutError` | 超时重试仍无回帧 | 控制器记看门狗 + 触发 e_stop |
| `ClogError`（可选） | 读电流突增 + 转速≈0 | 力控层判定堵转 → 停止该轴 |

**0xfd 到位帧**（参考固件 can.c:211）：位置到位时驱动主动回 `[0xfd, 0x9f, 0x6b]`（DLC=3, ID=addr<<8）。PC 端监听做"运动到位"事件，替代固定延时轮询，用于 `soft_reset`/`set_joints` 闭环。

> **与 STM32 时代心智差异**：控制命令不依赖回帧确认（Response=None 减总线流量），PC 侧改为"发命令 → 定期读状态验证到位"。

---

## 3. 安全层（替代 STM32 Layer 1）

| 原 STM32 功能 | PC 端实现 | 说明 |
|------|------|------|
| 关节限位 clamp | `controller.py` 位置命令出口前 clamp | 限位表见 §2.2；真机扫掠后修正 |
| 软急停 | `e_stop()` → 广播 `00 FE 98 00 6B` + 清控制标志 | 全轴广播帧，与固件一致 |
| 通信看门狗 | 主循环记录"最近成功读写时间"，>500ms 无回帧 → 自动 e_stop | 挂在轮询线程上 |
| 力阈值 | 每周期读 6 轴 `0x27` 相电流，超阈值 → e_stop 或单轴停 | 阈值来自 ZDT 文档 §3 标定；初版只告警+记录 |
| 堵转兜底 | 驱动端 `Clog_Pro`（硬件，保留） | ZDT 文档 §3 —— 最后一道物理保护 |
| 命令超时重试 | 驱动层 TimeoutError → 控制器记看门狗 + 触发 e_stop | §2.4 |

**看门狗架构**：主控循环 `controller.tick()` 同时做 ①状态轮询（`get_state`）②看门狗检查 ③力阈值检查。任一触发 → `e_stop()`。

**e_stop 语义对齐**：广播停止后**保持使能**（与固件 `ESTOP` 一致），恢复需显式 `set_torque 1` + 重新下发命令。

**接线前提**：120Ω 终端电阻在总线两端（适配器端 + 末端驱动端）必须接对 —— PC 直连第一优先级硬件项。

---

## 4. 控制器 / IK / 遥操 / LeRobot

### 4.1 ZdtController

```python
class ZdtController:
    def get_state(self) -> (angles, vels, loads)      # 轮询 6×0x36 + 6×0x27
    def set_joints(self, j1..j6)                       # clamp → 6×0xFB 绝对位置
    def rel_rotate(self, joint_id, angle)              # 0xFB 相对 / 读位+绝对
    def set_torque(self, enable: bool)                 # 6×0xF3 AB 01/00
    def e_stop(self)                                   # 广播 00 FE 98 00 6B
    def soft_reset(self)                               # set_joints([90,45,90,90,0,0])
    def remote_event(self, vx,vy,vz,j5,j6=0,j4=0)       # §4.3 (与 ArmClient 命名一致)
    def end_event(self, vx,vy,vz,wx,wy,wz)              # §4.4 末端6DOF
    def get_ee_pose(self) -> (pos_mm3, quat_wxyz)      # FK(J1-J6) 计算
    def get_wrist(self) -> (x,y,z)                     # FK(J1-J3) 腕心
    def tick(self)                                     # 看门狗+力控+轮询
```

### 4.2 kinematics.py（移植参考固件解析式 IK）

真机 DH 参数（`robot.c:25`）：

```c
const float D_H[6][4] = {{0,0,0,π/2}, {0,π/2,0,π/2}, {200,π,0,-π/2},
                         {47.63,-π/2,-184.5,0}, {0,π/2,0,π/2}, {0,π/2,0,0}};
// 列 = [a, α, d, θ_offset]  (长度单位 mm)
```

移植 `robot_kinematics.c` 的解析式闭式逆解（theta1~theta6 多解 + `robot_kinematics_get_optimal_result` 最优选取）+ FK。**不搬** MuJoCo DLS（仿真专用）。

### 4.3 remote_event 语义（复用 remote_semantics.py）

```
remote_event p0..p6
  → parse_remote_event() → v_lin(J1-J3 位置) + j4/j5/j6 系数   [纯函数, 已存在]
  → v_lin × dt → 笛卡尔目标位姿(保持当前姿态) → 解析IK → J1-J3 绝对角
  → j4/j5/j6 系数 → 直接关节通道 (0xF6 速度模式, RPM)
  → 全部走 controller(限位clamp + 看门狗) 出口
```

### 4.4 end_event 6DOF（视觉遥操出口）

```
end_event vx vy vz wx wy wz  (末端线+角速度 ∈[-1,1])
  → 几何 Jacobian（FK 数值差分, 6×6）→ 6 关节速度 (rad/s → RPM)
  → 6×0xF6 速度模式（限位 clamp 出口）
```

### 4.5 MassageRobot：transport 可切换

```
config_massage_robot.py 增加 transport: "can" | "serial"
massage_robot.py 构造时按 transport 选择底层:
  can    → ZdtController (SocketCAN)
  serial → serial_protocol.py (保留)
接口 get_observation/send_action/configure/calibrate 不变
```

### 4.6 线程模型

```
┌─ RX 线程 ──────────────────────┐
│ bus.recv() 阻塞收帧            │
│  → 按 ID 匹配响应/事件         │
│  → 0xfd 到位帧 → 事件回调       │
│  → 状态缓存更新                │
└──────────────┬─────────────────┘
┌─ 主控循环 (controller.tick, 50Hz) ─┐
│ 轮询 get_state (10~20Hz)           │
│ 看门狗检查 (>500ms 无回帧 → e_stop) │
│ 力阈值检查 (相电流超阈值 → e_stop)  │
│ 下发控制命令 (clamp 出口)           │
└───────────────────────────────────┘
```

> RX 收帧独立线程（SocketCAN `recv()` 阻塞，不能与发送串行）；直接 CAN 相比串口，"去掉串口瓶颈"的收益在此兑现。

---

## 5. 视觉遥操接入（Leap_Hand WristTracker）

### 5.1 遥操实现方式

`Leap_Hand/python/gesture_mapping/demo_arm_teleop.py` 的视觉遥操流水线：

```
RealSense → MediaPipe → build_palm_pts(3D关键点)
  → WristTracker.update() → (vx,vy,vz,wx,wy,wz) 6DOF 末端速度
  → cmd_smoother → arm.end_event(*cmd)          ← 唯一控制出口
反馈: arm.get_wrist()    → 腕心位置
      arm.get_ee_pose()  → 末端位姿 (m + wxyz)
      arm.get_state()    → 关节角/速度/电流
支持: remote_enable/disable · e_stop · set_joints · soft_reset · rel_rotate
```

`WristTracker` 控制范式：按住 H 捕获手锚点+末端锚点；按住期间手位置增量→末端位置目标（P 位置环→v_lin），手姿态增量→末端姿态目标（姿态环→w_ang）；松开 H 重锚定（走哪停哪）。

### 5.2 接入方式：`ArmClient` CAN 后端

给 `ArmClient` 增加 CAN 后端（`CanArmClient`，包 `ZdtController` + FK），按 `--port` 前缀分发（`serial:` / `socket://` / `can:`）。**`demo_arm_teleop.py` 零改动**。

| `ArmClient` 方法 | CAN 直连实现 |
|------|------|
| `end_event(vx,vy,vz,wx,wy,wz)` | `ZdtController.end_event()` → 6×6 解析IK/Jacobian → 6×0xF6 |
| `get_ee_pose()` | **PC FK(J1-J6)**（驱动回读角度 → 正解 → 位姿） |
| `get_wrist()` | **PC FK(J1-J3)** 腕心 |
| `get_state()` | 6×0x36 + 6×0x27 聚合 |
| `remote_enable/disable` | no-op（PC 直连无需固件解锁）或映射 set_torque |
| `set_joints/soft_reset/rel_rotate/e_stop` | 直接对应控制器方法 |

### 5.3 解决的问题

当前真机路径是哑的（demo_arm_teleop.py 注释：真机固件无 get_ee_pose/FK → 反馈关闭 → WristTracker 返回全 0）。CAN 直连把 FK 搬到 PC 后：

- `get_ee_pose`/`get_wrist` 变成 PC 纯 FK 计算（驱动回读角度→正解），**无需固件改动**
- `end_event` 的 IK 也在 PC 上
- **视觉遥操第一次在真机可跑** —— 这是比串口→STM32 更实质的跨越（FK 反馈能力）

### 5.4 设计注意点

真机 `get_ee_pose` 用 FK 计算值（非真实位姿传感器），位置环反馈有 DH 参数/装配误差累积。缓解：
1. FK 参数从真机 DH 实测出发，支持运行时微调
2. WristTracker 的 P 位置环（k_pos=0.06）本身有死区+滤波，容忍小误差

---

## 6. 边缘部署（PC 训练/推理 ↔ 香橙派执行/采集）

### 6.1 节点职责

| 节点 | 角色 | 运行内容 |
|------|------|----------|
| **PC (GPU)** | 训练 + 推理 | SmolVLA 训练/微调 · 推理出 action chunk · 评估/量化导出 · 轻量策略(ACT/Diffusion)导出 |
| **香橙派 AI Pro** | 机器人实时节点 | SocketCAN→6×ZDT 机械臂 · Dynamixel→16-DOF 灵巧手 · 相机+MediaPipe+WristTracker 视觉遥操 · LeRobot 本地采集 · 执行远端 action chunk |

> **芯片定案（2026-08-15 GitHub 调研 `hexchip/lerobot-on-ascend`）**：Orange Pi AIpro 20T 为**华为昇腾 310B**（20 TOPS NPU），**非 RK3588**。该仓库已跑通"Orange Pi AIpro + CANN + torch_npu + Python3.11 + PyTorch2.5.1 跑 LeRobot"，是香橙派部署的**直接参考实现**。部署策略用**轻量 ACT/Diffusion**（板子可跑），重型 SmolVLA 训练/推理留 PC。

### 6.2 两条数据流

```
【采集】纯本地，不依赖 PC：
  相机 → MediaPipe → WristTracker → end_event → CAN(机械臂) + Dynamixel(手)
  LeRobot record: (图像, 22-DOF 关节角) → dataset

【部署】PC 推理 → 流式动作：
  香橙派: (相机帧 + 关节状态) → TCP 观测 → PC
  PC:     SmolVLA 推理 → action chunk (22-DOF × N步)
  香橙派: 接收 chunk → 逐帧执行 → CAN + Dynamixel
```

### 6.3 action-chunk 流式协议（部署用）

- **格式**：JSON 二进制（图像 JPEG 压缩 + 关节角 float 数组），chunk 一次性下发
- **传输**：TCP（可靠性优先，按摩动作不允许丢帧）
- **帧率**：采集 30Hz 观测 → PC 3-5Hz 推理 → chunk 内 30Hz 执行（VLA 边执行边预测）
- **失败兜底**：网络断 → 香橙派看门狗 → 全轴急停（沿用 §3 安全层）

### 6.4 可移植性约束（保证 PC→香橙派 零重写）

1. **禁 x86 专属依赖**，全部走有 aarch64 wheel 的包
2. **设备路径/相机后端进配置**（`/dev/ttyUSB*`、can0、RealSense/v4l2 切换）
3. **SocketCAN 两端一致** —— `gs_usb` 兼容 CAN 收发器在香橙派直接插上用
4. **LeRobot 采集代码不加网络依赖** —— 采集纯本地，部署才走流式层

### 6.5 动作空间扩展：22-DOF

原设计 6-DOF 机械臂。接入灵巧手后 action space = **6(臂) + 16(手) = 22 DOF**：
- LeRobot feature 表加 `{finger}.pos` 16 个键
- SmolVLA 输出 head 扩到 22 维
- 数据采集时臂+手关节角一起记录（视觉遥操 WristTracker 控臂，手势/逐指控制手）

---

## 7. bring-up 与验证计划

### 7.1 硬件 bring-up 顺序

1. **120Ω 终端电阻** 确认（适配器端 + 末端驱动端）
2. PC 端 `can0` 起来（`ip link set can0 type can bitrate 500000` + `can0 up`），`candump` 确认总线空载
3. **`zdt_bringup.py` 单轴验证**：使能 → rel_rotate ±N° → 读回位置（用 STM32 串口 master 作为 A/B 基准对比）
4. 逐轴 `rel_rotate` 方向/到位确认 → 6 轴全通
5. 限位扫掠标定（ZDT 文档 §5.2），固件/PC 限位表对齐
6. `soft_reset` / `e_stop` / `Clog_Pro` 触发验证
7. LeRobot 采集链路（MassageRobot CAN transport）
8. 遥操：Leap_Hand WristTracker → `CanArmClient` → 真机视觉遥操
9. 香橙派移植：重装依赖 + 配置 → 跑通采集 + 控制
10. PC 推理 → action chunk → 香橙派执行（部署流）

### 7.2 验证清单

- [ ] 6 轴编码器校准完成（无 `Not Cal`）
- [ ] 6 轴 `P_Serial=CAN1_MAP`、`CAN_Baud`/`ID_Addr`/`Checksum` 逐轴读回一致
- [ ] PC `can0` 500k 扩展帧收发正常（candump 嗅探对比 STM32 报文）
- [ ] **位置命令判定：candump 确认本机 X-series V2 用 `0xFB` 还是 `0xFD`**（两代命令不同，见 §2.3 注）
- [ ] 每轴 `rel_rotate +N° / -N°` 方向正确、到位
- [ ] 零点已设：回零后各轴 `get_state` 与固件初始位一致
- [ ] `e_stop` 立即停、恢复正常
- [ ] `Clog_Pro` 触发测试（手持轻阻电机 → 驱动保护停机）
- [ ] 看门狗：拔适配器 → 500ms 内自动 e_stop
- [ ] 力阈值：超阈值 → 告警/停机
- [ ] 视觉遥操真机跑通（H 离合器跟随 / 松开锚定）
- [ ] 香橙派移植跑通（采集 + 控制）
- [ ] 香橙派芯片确认为昇腾 310B（已由 lerobot-on-ascend 佐证），部署参考该仓库 CANN+torch_npu 流程

### 7.3 环境依赖

```
PC 端:  python-can[socketcan] · numpy · (LeRobot 现有依赖)
香橙派: 同栈 aarch64 wheel + pyrealsense2(版本敏感需验证) + mediapipe(aarch64)
```

---

## 8. 决策记录（ADR 更新）

- **ADR-001 修订**：STM32 不再作为唯一控制网关。PC 直连 CAN 成为主路径；STM32 保留作为 A/B 验证与调试对照（serial_protocol.py 保留），不承担生产路径。
- **ADR-005（新）**：PC 作为 CAN 总线 master，安全层（限位/e_stop/看门狗/力控）整体上移 PC，驱动端 `Clog_Pro` 作为物理兜底。
- **ADR-006（新）**：IK 采用参考固件解析式闭式解移植，不引入 MuJoCo 依赖到真机控制路径。
- **ADR-007（新）**：训练留在 PC/GPU，香橙派为实时控制+采集+执行节点，部署采用"PC 推理 + action chunk 流式下发"。

---

## 9. 风险与待验证项

| 风险 | 等级 | 缓解 |
|------|:----:|------|
| **位置命令代际差异：0xFB(ZDT 文档) vs 0xFD(Emm_V5, GitHub 实测)** | **高** | **真机 candump 对照 STM32 已走通报文（plan Task 8 Step 3）是任何运动命令的硬前置**；驱动层 move_abs/move_rel 载荷布局做成易切换配置 |
| ZDT 帧载荷字段细节（0xFB 的 00/0A/同步位）可能需微调 | 低 | 参考固件 + candump 对比核实；bring-up 第 3 步验证 |
| `set_torque 0` 后闭环步进能否被反拖（free-wheel）手动示教 | 中 | 真机验证；不行则示教改用 rel_rotate/视觉遥操 |
| 香橙派 pyrealsense2 aarch64 版本兼容 | 中 | 提前验证；备选 v4l2/gstreamer 后端 |
| 真机 FK 反馈误差（DH vs 装配） | 低 | DH 运行时微调 + 位置环死区容忍 |
| 22-DOF 训练改动（SmolVLA/ACT/Diffusion） | 中 | action space 扩展为配置项；先跑通 6-DOF 再扩手；边缘部署用轻量 ACT/Diffusion（lerobot-on-ascend 已验证板子可跑） |
