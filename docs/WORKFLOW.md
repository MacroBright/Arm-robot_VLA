# 项目完整工作流程

> 从固件开发到模型推理的端到端管线，含每个流程涉及的文件地址。
> 版本: 0.2.0 | 日期: 2026-07-14

---

## 总览

```
Phase 1              Phase 2             Phase 3            Phase 4
+----------+       +----------+        +----------+       +----------+
| STM32    |       | 数据采集  |        | 模型训练  |       | 推理部署  |
| 固件开发  |  --> | (record) |  -->   | (train)  |  --> | (replay) |
+----------+       +----------+        +----------+       +----------+
                        |
                        +-- 手动示教（torque off + 手拖臂）
                        +-- 手柄遥操作（pygame + remote_event）
```

---

## Phase 1: STM32 固件开发

### 流程
1. STM32CubeIDE 打开工程并配置时钟树
2. 编写/修改应用代码
3. CubeIDE 编译 + 烧录
4. 串口助手验证命令

### 涉及文件

| 文件 | 路径 | 说明 |
|------|------|------|
| CubeIDE 工程 | `D:\robo arm\software\zero_arm\robot\` | 主工程目录 |
| 时钟配置 | `robot/Core/Src/main.c` | HSE 25MHz -> PLL -> 168MHz |
| 机器人控制 | `robot/Core/Src/robot.c` | PID、逆运动学、事件处理 |
| 命令解析 | `robot/Core/Src/robot_cmd.c` | 串口命令表 + LeRobot 4 条命令 |
| 电机驱动 | `robot/Core/Src/Emm_V5.c` | Emm_V5 CAN 步进闭环驱动 |
| 串口驱动 | `robot/Core/Src/usart.c` | USART1 中断接收 + DMA |
| FreeRTOS 配置 | `robot/Core/Inc/FreeRTOSConfig.h` | 堆内存 30KB |
| 命令文档 | `docs/SERIAL_COMMANDS.md` | 14 条串口命令完整参考 |

---

## Phase 2: 数据采集（两种方式）

### 方式 A: 手动示教（Torque Off + 手拖臂）

#### 流程
1. STM32 上电、电机上电
2. `set_torque 0` -> 电机自由转动
3. 操作员手动拖拽机械臂演示动作
4. LeRobot 定时 `get_state` + 相机帧记录轨迹
5. `set_torque 1` 恢复力矩

#### 命令
```powershell
.venv\Scripts\Activate.ps1

# 安装依赖
pip install lerobot
pip install -e lerobot_robot_massage

# 运行手动示教录制（待实现完整 lerobot-record 管线）
python -c "
from lerobot_robot_massage import SerialProtocol
from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig

# 连接
proto = SerialProtocol('COM5'); proto.connect()
cam = OpenCVCamera(OpenCVCameraConfig(index_or_path=1, fps=30, width=640, height=480))
cam.connect()

# 自由模式
proto.set_torque(False)
print('扭矩已关闭，可手动拖拽...')

# 录制循环
import time
for i in range(600):  # 20s @ 30fps
    angles, _, _ = proto.get_state()
    frame = cam.read_latest()
    # 这里写入 parquet 或自定义存储格式
    time.sleep(1/30)

proto.set_torque(True)
cam.disconnect(); proto.disconnect()
"
```

#### 涉及文件

| 文件 | 路径 | 说明 |
|------|------|------|
| LeRobot 插件包 | `lerobot_robot_massage/` | `lerobot_robot_` 前缀自动发现 |
| 配置类 | `lerobot_robot_massage/config_massage_robot.py` | `MassageRobotConfig` |
| 适配器 | `lerobot_robot_massage/massage_robot.py` | `MassageRobot(Robot)` |
| 串口协议 | `lerobot_robot_massage/serial_protocol.py` | `SerialProtocol` |
| 配置 | `configs/massage_robot.yaml` | YAML 配置 |
| 命令参考 | `docs/SERIAL_COMMANDS.md` | get_state/set_joints 等 |

---

### 方式 B: 手柄遥操作（推荐用于采集精细轨迹）

#### 流程

```
+------------------+          +------------------+
| 窗口 1: 手柄控制  |          | 窗口 2: 轨迹录制  |
| (joystick_control)|          | (record_         |
|  .py)             |          |  trajectory.py)  |
|                   |          |                   |
| 摇杆 -> remote_   |          | get_state(30fps)  |
|  event            |          | camera.read()     |
| 关节 -> rel_rotate|          | --> 写入 data.json|
| 急停 -> Y 键      |          | --> 保存 png 帧   |
+------------------+          +------------------+
        |                               |
        v                               v
    [STM32 执行]                  [datasets/raw/]
```

#### 启动步骤

**终端 1 - 手柄控制：**
```powershell
.venv\Scripts\Activate.ps1
python scripts\joystick_control.py --port COM5 --camera 1
```

**终端 2 - 轨迹录制（`scripts/record_trajectory.py`，需新建）：**
```powershell
.venv\Scripts\Activate.ps1
python scripts\record_trajectory.py --port COM5 --camera 1 --duration 20
```

#### 手柄安全操作流程

```
1. 给 STM32 上电（启动完成）
2. 给所有电机上 24V 电源
3. 运行手柄脚本
4. 按 A -> remote_enable（软复位到初始姿态）
5. 按 X -> set_torque 1（使能电机）
6. 开始摇杆控制
7. 急停：按 Y（e_stop）
8. 退出：按 B（remote_disable）-> 按 BACK
```

#### 手柄按键映射

| 手柄操作 | 作用 | 串口命令 |
|---------|------|---------|
| 左摇杆 | 末端 XY 平移 | `remote_event vx vy ...` |
| 右摇杆 | 末端旋转 | `remote_event ... rx ry` |
| L2/R2 | Z 升降 | `remote_event ... vz_up vz_down` |
| A | 进入遥控 | `remote_enable` |
| B | 退出遥控 | `remote_disable` |
| Y | 紧急停止 | `e_stop` |
| X | 扭矩切换 | `set_torque 0/1` |
| L1/R1 | 切换关节模式 | (内部状态切换) |
| 十字键 ↑↓ | 选中关节正/反转 | `rel_rotate joint delta` |
| 十字键 ←→ | 退出关节模式 | (内部状态切换) |
| START | 当前位置归零 | `zero` |
| BACK | 退出脚本 | (内部) |

#### 涉及文件

| 文件 | 路径 | 说明 |
|------|------|------|
| 手柄控制 | `scripts/joystick_control.py` | pygame 读取手柄 -> 串口控制 |
| 轨迹录制 | `scripts/record_trajectory.py` | 定时读取状态 + 相机（待创建）|
| 串口协议 | `lerobot_robot_massage/serial_protocol.py` | 底层串口封装 |
| 数据集 | `datasets/raw/` | 录制结果存储 |

#### 采集建议

录制 50+ 条不同轨迹以获得可用训练效果：

| 轨迹 | 动作 | 时长 | 手法 |
|:----:|------|:----:|------|
| 1-10 | 空载平移/画圆 | 10s | 基础运动 |
| 11-25 | 模拟按揉 | 15s | 按压 + 旋转 |
| 26-40 | 推压动作 | 20s | 线性推 + 保持 |
| 41-50 | 复合手法 | 25s | 多种组合 |

---

## Phase 3: 模型训练

### 流程
1. 将 `datasets/` 数据转换为 LeRobot 兼容格式
2. 配置训练参数（policy、batch_size、steps）
3. 运行 `train.py` 微调 SmolVLA
4. 导出训练好的模型

### 涉及文件

| 文件 | 路径 | 说明 |
|------|------|------|
| 训练配置 | `configs/train_smolvla.yaml` | batch_size=64, steps=200000 |
| 数据集 | `datasets/massage_v1/` | 采集的演示数据 |
| 模型输出 | `outputs/train/` | 训练结果 (gitignored) |
| LeRobot 训练脚本 | `lerobot/scripts/train.py` | (LeRobot 源码) |

### 训练命令示例

```powershell
# 在带 GPU 的机器上运行
pip install 'lerobot[all]'

python lerobot/scripts/train.py ^
  --policy.type=smolvla ^
  --policy.path=lerobot/smolvla_base ^
  --dataset.repo_id=./datasets/massage_v1 ^
  --batch_size=64 ^
  --steps=200000 ^
  --output_dir=outputs/train/smolvla_massage
```

---

## Phase 4: 推理部署

### 流程
1. 加载训练好的模型
2. STM32 上电、电机上电
3. 运行 `replay` 或自定义推理循环
4. 模型输入：相机帧 + 当前关节状态 + 指令
5. 模型输出：目标关节角度 -> `send_action()` -> STM32

### 涉及文件

| 文件 | 路径 | 说明 |
|------|------|------|
| 训练好的模型 | `outputs/train/smolvla_massage/` | 模型权重 |
| 推理配置 | `configs/massage_robot.yaml` | 同采集配置 |
| LeRobot 推理脚本 | `lerobot/scripts/control_robot.py replay` | (LeRobot 源码) |

### 推理命令示例

```powershell
python lerobot/scripts/control_robot.py replay ^
  --robot.type=massage_robot ^
  --robot.port=COM5 ^
  --policy.path=outputs/train/smolvla_massage/checkpoints/last/pretrained_model
```

---

## 跨阶段参考文档

| 文档 | 路径 | 用途 |
|------|------|------|
| 项目总纲 | `CLAUDE.md` | AI 开发指南、架构决策 |
| 项目说明 | `README.md` | 硬件清单、快速开始 |
| 系统架构 | `docs/ARCHITECTURE.md` | 四层架构、数据流 |
| 部署教程 | `docs/DEPLOYMENT.md` | 从 0 到训练的完整步骤 |
| 命令参考 | `docs/SERIAL_COMMANDS.md` | 14 条串口命令 |
| 项目工作流 | `docs/WORKFLOW.md` | 本文件 |
| 实验记录 | `experiments/` | 训练日志 |

---

## 完整目录结构

```
E:\Arm-robot_VLA/                         <- 本项目 (LeRobot 集成)
+-- CLAUDE.md
+-- README.md
+-- .gitignore
+-- docs/
|   +-- ARCHITECTURE.md
|   +-- SERIAL_COMMANDS.md
|   +-- DEPLOYMENT.md
|   +-- WORKFLOW.md                       <- 本文件
+-- lerobot_robot_massage/                <- LeRobot 插件包
|   +-- pyproject.toml
|   +-- __init__.py
|   +-- config_massage_robot.py
|   +-- massage_robot.py
|   +-- serial_protocol.py
+-- configs/
|   +-- massage_robot.yaml
|   +-- train_smolvla.yaml
+-- scripts/
|   +-- joystick_control.py               <- USB 手柄遥控
|   +-- record_trajectory.py              <- 轨迹录制 (待创建)
|   +-- verify_interface.py               <- 接口规范验证
|   +-- test_serial.py                    <- 串口测试
+-- firmware/                             <- STM32 固件修改指南
|   +-- README.md
|   +-- src/
+-- datasets/                             <- 采集数据 (gitignored)
+-- outputs/                              <- 训练模型 (gitignored)
+-- experiments/                          <- 实验记录

D:\robo arm\software\zero_arm\robot\     <- STM32 固件工程 (CubeIDE)
+-- Core/Src/robot.c
+-- Core/Src/robot_cmd.c
+-- Core/Src/main.c
+-- Core/Src/Emm_V5.c
+-- Core/Inc/robot.h
+-- Core/Inc/FreeRTOSConfig.h
+-- ...
```
