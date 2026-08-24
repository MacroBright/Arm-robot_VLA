# Zero Arm VLA — 项目总纲 (CLAUDE.md)

> 基于 VLA（Vision-Language-Action）的机械臂灵巧手按摩系统。
> 将 zero-robotic-arm (STM32F407 + Emm_V5 CAN 步进电机) 集成到 LeRobot 生态系统。

---

## 一、项目愿景

开发一套**安全、智能、可泛化**的机器人按摩系统，能够：
1. 通过视觉识别人体背部穴位位置
2. 理解自然语言指令（如"按揉大椎穴"）
3. 自主规划并执行按摩动作（按、揉、推、捏等）
4. 适应不同体型、不同体位的受试者

---

## 二、技术栈

| 层级 | 技术 | 说明 |
|------|------|------|
| **VLA 模型** | SmolVLA-450M (PyTorch) | 轻量视觉-语言-动作模型，消费级 GPU 可训练 |
| **框架** | LeRobot (Hugging Face) | 数据采集、训练、推理管线 |
| **上位机** | Python 3.10+, pyserial, OpenCV | PC 端适配层与视觉处理 |
| **下位机** | STM32F407VET6, FreeRTOS, CAN | 实时安全控制 |
| **电机** | Emm_V5 CAN 总线步进闭环驱动器 ×6 | 6-DOF 机械臂关节 |
| **灵巧手** | AmazingHand (设计中) | 8-DOF 开源机械手，Feetech SCS0009 |
| **相机** | USB 摄像头 (OpenCV) | 可扩展 RGB-D |

---

## 三、架构边界

```
┌──────────────────────────────────────────────┐
│            Layer 3: 语义决策层 (PC)            │
│  SmolVLA: 图像 + 指令 → 目标关节角度序列        │
├──────────────────────────────────────────────┤
│            Layer 2: 视觉遥操 / 感知层 (PC)     │
│  RealSense + HandTracker + WristTracker      │
│  → VisionWatchdog (分级: DECAY/STOP/ESTOP)   │
├──────────────────────────────────────────────┤
│            Layer 1: 笛卡尔/控制器层 (PC)      │
│  RealArmAdapter → CartesianController        │
│  (6DOF 雅可比 DLS + 测量 dt + 单调陈旧命令看门狗) │
├──────────────────────────────────────────────┤
│            底层: ZDT CAN 驱动层 (PC 直连)     │
│  ZdtController (RobotStateMachine 门禁)      │
│  → ZdtDriver → SocketCAN (0xFD + multi_sync) │
│  → Emm_V5 / ZDT 步进闭环驱动器 ×6              │
└──────────────────────────────────────────────┘
```

**关键边界规则：**
- PC 端通过 USB-CAN (SocketCAN) 直连 6 轴电机总线，执行统一安全链控制
- 安全包络由 PC 层严格守卫：`RobotStateMachine` 门禁 (ARMED/TELEOP 显式确认) + 0x36 真实位置软限位 + 枚举硬不变式
- 通信超时或异常时，执行广播急停 (0x0000 0xFE/0xF7) 并切入 SAFE_IDLE / DISCONNECTED

---

## 四、开发规范

### 代码风格
- **Python**: PEP 8, 类型注解, docstring (Google style)
- **C (STM32)**: 遵循现有 zero-robotic-arm 代码风格, 不引入 C++ 特性
- **文件命名**: lowercase_with_underscores
- **文档语言**: 中文为主，代码注释和技术术语可用英文

### Git 规范
- 提交信息: Conventional Commits (`feat:`, `fix:`, `docs:`, `refactor:`, `test:`, `safety:`)
- 功能分支开发，合并前测试

### 安全准则
1. 所有电机操作必须可被急停中断 (e_stop / CAN 广播急停)
2. 重力关节 (J2/J3) 必须显式确认 (`arm(gravity_confirmed=True)`) 后方可使能扭矩
3. 初期测试必须在虚拟环境 (`FakeTransport` / `FakeMuJoCoServer`) 验证
4. 按摩测试先用硅胶假体，禁止直接在人体上测试未验证的模型

---

## 五、架构决策记录 (ADR)

### ADR-001: STM32 作为安全网关 (历史记录)
- **决策**: 初期设计 STM32 保留作为 PC 与电机之间的通信桥梁
- **演进**: 后续架构演进为 ADR-005 PC 直连 USB-CAN

### ADR-002: 文本协议而非二进制协议
- **决策**: PC↔STM32 使用行文本协议 (ASCII, `\n` 分隔)
- **原因**: 与现有固件风格一致；可调试性强；30fps 场景下带宽不是瓶颈
- **影响**: 串口波特率 115200，与现有固件一致

### ADR-003: 自定义 Robot 子类而非 MotorsBus
- **决策**: `MassageRobot` 直接在 `Robot` 子类中管理通信
- **原因**: 保持机械臂运动学与安全策略内聚
- **来源**: LeRobot BYOH 文档推荐的 Approach A

### ADR-004: Emm_V5 CAN 电机 vs 知识库中的 Feetech STS3215
- **事实**: zero-robotic-arm 使用 Emm_V5 CAN 步进电机，不是 STS3215 RS485 舵机
- **影响**: 知识库中关于 `FeetechMotorsBus` 的直接使用方案不适用，但 LeRobot BYOH 流程不变
- **灵巧手**: AmazingHand 使用 SCS0009 (Feetech 协议)，届时可扩展独立总线

### ADR-005: PC 直连 USB-CAN (SocketCAN) 架构
- **决策**: STM32 网关已移除，PC 直连 USB-CAN (SocketCAN)。
- **原因**: 消除串口协议栈往返延迟与双层状态不同步风险，利用 Linux SocketCAN 原生确定性广播。
- **安全**: 安全包络由 PC 端执行：`RobotStateMachine` 门禁 + 0x36 真实位置软限位 + 枚举硬不变式。

### ADR-006: 内部姿态表示 = SO(3)/轴角
- **决策**: 笛卡尔控制器内部姿态与误差表示采用 SO(3)/轴角 (`log_so3`)，禁止 Euler 累加。
- **原因**: 避免欧拉角万向锁与奇异多解跳变。
- **安全**: RPY 仅在可选相对边界安全约束中做投影限幅。

### ADR-007: CartesianController 唯一笛卡尔运动入口与控制层单调看门狗
- **决策**: `CartesianController` 是唯一笛卡尔运动入口；陈旧命令判定归控制层 (`step(cmd_ts)` 单调期限)。
- **原因**: 视觉层只负责视觉信号健康分级 (DECAY/STOP/ESTOP)；控制层负责单调时间看门狗与关节运动安全，职责严格解耦。

---

## 六、当前阶段

**Phase 1**: LeRobot 适配与数据采集验证
- [x] 硬件架构理解完成
- [x] 项目文档搭建
- [ ] STM32 固件扩展（新增 `get_state`, `set_joints`, `set_torque`, `e_stop`）
- [ ] `MassageRobot` LeRobot 适配器
- [ ] 手动示教数据采集跑通
- [ ] 小规模训练验证 (SmolVLA)

> **环境重装提示**：Ubuntu conda 环境（`smolvla`，envs 在 E盘 NTFS）的安装步骤与
> pip 见 `scripts/setup_dev.sh`（自动处理 editable install）。

## 七、多窗口并行工作流

本项目按技术层拆分为 4 个独立会话窗口。如需自动拆分其他项目，使用 skill:
- [split-project-workstreams](https://github.com/MacroBright/claude_skills-autoworkstreams)，每个窗口专注一类任务：

| 窗口 | 简报文件 | 职责 |
|------|----------|------|
| 🎮 仿真与数据 | `.claude/workstreams/01-simulation-data.md` | MuJoCo 仿真、手柄遥操作、数据采集 |
| 🧠 模型训练 | `.claude/workstreams/02-model-training.md` | SmolVLA 训练、评估、远程部署 |
| 🔧 固件与硬件 | `.claude/workstreams/03-firmware-hardware.md` | STM32 固件、MassageRobot 适配器 |
| 📝 文档与知识 | `.claude/workstreams/04-docs-knowledge.md` | 文档、ADR、实验记录、知识库 |

**用法**: 新开一个 Claude Code 窗口，输入：
```
加载 .claude/workstreams/0X-xxx.md，当前任务：[描述具体任务]
```

**依赖关系**: 窗口1 → 窗口2 (数据流) ，窗口3 ↔ 窗口1 (协议对齐)，窗口4 → 所有窗口 (记录)。

## 八、参考

- 项目索引: `E:\Liang\Documents\Bright的知识库\基于VLA的机械臂设计\项目索引与笔记地图.md`
- 参考硬件: `d:\robo arm\software\zero-robotic-arm`
- LeRobot 文档: https://github.com/huggingface/lerobot
- SmolVLA 论文: https://huggingface.co/papers/2506.01844
