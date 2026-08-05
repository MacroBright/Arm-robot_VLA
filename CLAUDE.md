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
│            Layer 2: 穴位感知层 (PC)            │
│  Keypoint Detection + 3D 定位                 │
├──────────────────────────────────────────────┤
│            Layer 1: 动作执行层 (PC)            │
│  LeRobot MassageRobot → 串口协议 → STM32      │
├──────────────────────────────────────────────┤
│          底层: 力控安全层 (STM32)              │
│  CAN → Emm_V5 电机, 限位开关, 急停, PID       │
└──────────────────────────────────────────────┘
```

**关键边界规则：**
- STM32 是唯一直接控制电机的设备，PC 不能绕过 STM32
- 安全相关逻辑（急停、限位、力阈值）必须在 STM32 上独立运行
- PC↔STM32 通信中断时，STM32 必须自动停止所有电机

---

## 四、开发规范

### 代码风格
- **Python**: PEP 8, 类型注解, docstring (Google style)
- **C (STM32)**: 遵循现有 zero-robotic-arm 代码风格, 不引入 C++ 特性
- **文件命名**: lowercase_with_underscores
- **文档语言**: 中文为主，代码注释和技术术语可用英文

### Git 规范
- 提交信息: Conventional Commits (`feat:`, `fix:`, `docs:`, `refactor:`)
- 功能分支开发，合并前测试

### 安全准则
1. 所有电机操作必须可被急停中断
2. 力/扭矩上限在 STM32 固件中硬编码
3. 初期测试必须在无人的安全区域进行
4. 按摩测试先用硅胶假体，禁止直接在人体上测试未验证的模型

---

## 五、架构决策记录 (ADR)

### ADR-001: STM32 作为安全网关
- **决策**: STM32 保留作为 PC 与电机之间的唯一通信桥梁
- **原因**: 安全隔离——PC 崩溃或 LeRobot 错误不能绕过 STM32 的力/限位保护
- **影响**: 不能直接使用 LeRobot 的 `FeetechMotorsBus`（RS485 直连模式）

### ADR-002: 文本协议而非二进制协议
- **决策**: PC↔STM32 使用行文本协议 (ASCII, `\n` 分隔)
- **原因**: 与现有固件风格一致；可调试性强；30fps 场景下带宽不是瓶颈
- **影响**: 串口波特率 115200，与现有固件一致

### ADR-003: 自定义 Robot 子类而非 MotorsBus
- **决策**: `MassageRobot` 直接在 `Robot` 子类中管理串口通信
- **原因**: STM32 已经抽象了电机层，不必强行适配 `MotorsBus` 的 register 读写模式
- **来源**: LeRobot BYOH 文档推荐的 Approach A

### ADR-004: Emm_V5 CAN 电机 vs 知识库中的 Feetech STS3215
- **事实**: zero-robotic-arm 使用 Emm_V5 CAN 步进电机，不是 STS3215 RS485 舵机
- **影响**: 知识库中关于 `FeetechMotorsBus` 的直接使用方案不适用，但 LeRobot BYOH 流程不变
- **灵巧手**: AmazingHand 使用 SCS0009 (Feetech 协议)，届时可扩展独立总线

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
> pip editable `.pth` 修复见 `README.md` §2.5。

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
