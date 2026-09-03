# Arm-robot_VLA 技术文档中心 (Documentation Hub)

欢迎查阅 `Arm-robot_VLA` 机械臂子系统技术文档。本目录收录了关于机械臂系统架构、底层硬件总线通信、正逆运动学解析、仿真孪生与具身智能数据集集成的完整技术白皮书。

---

## 📑 核心技术文档索引

| 文档名称 | 主要内容与受众 | 链接 |
| :--- | :--- | :---: |
| **系统架构设计** | 机械臂 5 层解耦架构、状态机模型、时钟域与多线程隔离设计 | [ARCHITECTURE.md](ARCHITECTURE.md) |
| **底层硬件 CAN 规范** | Emm42 V5.0 步进闭环 CAN 协议、0xFD 脉冲、0x36 编码器读数校验 | [HARDWARE_CAN_SPEC.md](HARDWARE_CAN_SPEC.md) |
| **笛卡尔键盘遥操指引** | 键盘 6-DOF 交互遥操、微步调试与姿态预设切换快捷键 | [KEYBOARD_TELEOP_GUIDE.md](KEYBOARD_TELEOP_GUIDE.md) |
| **MuJoCo 仿真与数字孪生** | MJCF 建模参数、TCP 通信协议、点云与多相机虚拟推流配置 | [MUJOCO_SIM.md](MUJOCO_SIM.md) |
| **LeRobot 具身集成方案** | 机械臂 + 灵巧手 22-DOF 接入 LeRobot v0.4+ 标准数据集与 SmolVLA | [tuinadex_to_lerobot.md](tuinadex_to_lerobot.md) |
| **研发与发布工作流** | 代码规范、TDD 测试准则、Git 提交与固件版本发布规范 | [WORKFLOW.md](WORKFLOW.md) |

---

## 🚀 快速跳转

- 🔙 返回机械臂主项目首页：[README.md](../README.md)
- 🤖 查阅灵巧手子项目文档：[Leap_Hand 文档](../../Leap_Hand/README.md)
- 🕹️ 查阅协同遥操顶层文档：[Co_Teleop 文档](../../Co_Teleop/README.md)
