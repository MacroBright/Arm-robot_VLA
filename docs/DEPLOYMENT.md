# Zero Arm VLA — 部署与使用教程

> 适用硬件：STM32F407VET6 + Emm_V5 CAN 步进闭环电机 ×6
> 软件版本：LeRobot 0.6.0 | Python 3.12 | 项目路径: `E:\Arm-robot_VLA`

---

## 目录

1. [项目概述](#1-项目概述)
2. [硬件准备](#2-硬件准备)
3. [Python 环境搭建（本地 LeRobot 部署）](#3-python-环境搭建本地-lerobot-部署)
4. [LeRobot 插件安装与验证](#4-lerobot-插件安装与验证)
5. [串口通信验证](#5-串口通信验证)
6. [USB 相机接入](#6-usb-相机接入)
7. [机械臂联通验证](#7-机械臂联通验证)
8. [手柄遥操作（Joystick 遥控）](#8-手柄遥操作joystick-遥控)
9. [数据采集：用手柄录训练数据](#9-数据采集用手柄录训练数据)
10. [模型训练（SmolVLA）](#10-模型训练smolvla)
11. [模型推理部署](#11-模型推理部署)
12. [故障排查](#12-故障排查)

---

## 1. 项目概述

### 1.1 系统架构

```
PC (LeRobot + MassageRobot)
    │ 文本协议 UART 115200bps
STM32F407VET6 (FreeRTOS + robot_cmd)
    │ CAN 1Mbps
Emm_V5 步进闭环电机 ×6 (CAN ID 2-7)
```

### 1.2 控制边界

- **STM32** 是唯一直接控制电机的设备，PC 不可绕过 STM32
- 安全逻辑（急停、限位、力阈值）在 STM32 固件中独立运行
- PC↔STM32 通信中断时，STM32 自动停止所有电机

### 1.3 数据采集原理

机械臂的数据采集用于**模仿学习**：通过记录人类操作员的操作轨迹（关节角度序列 + 相机画面），让 SmolVLA 模型学会自主完成相同任务。

两种采集方式：

| 方式 | 原理 | 适用场景 |
|------|------|---------|
| **手动示教**（Manual Puppeting） | 扭矩关闭 → 手拖机械臂演示 → LeRobot 定时读状态 | 需要直接演示按摩手法，没有手柄 |
| **手柄遥操作**（Joystick Teleop） | 手柄实时控制机械臂 → LeRobot 同步记录 | 精确控制，可做远程操作，操作员更舒适 |

---

## 2. 硬件准备

### 2.1 硬件清单

| 组件 | 规格 | 数量 | 备注 |
|------|------|:----:|------|
| STM32 主控板 | STM32F407VET6 | 1 | FreeRTOS 已移植 |
| ST-LINK V2 | 调试/烧录器 | 1 | 固件烧录用 |
| USB-TTL 模块 | 3.3V 电平，CH340G | 1 | 串口通信 |
| USB 摄像头 | 640×480@30fps | 1 | 视觉输入 |
| Emm_V5 驱动器 | CAN 总线步进闭环 | 6 | CAN ID 2-7 (关节1=ID 2) |
| 24V 电源 | 至少 10A | 1 | 电机供电 |
| USB 游戏手柄 | Xbox/通用 USB 手柄 | 1 | 遥操作（可选） |

> 关节 4 (CAN ID 5) 的电机可能有相线问题（有电流声但不转），不影响其他 5 个关节的数据采集。

### 2.2 接线图

#### ST-LINK 烧录接线
```
ST-LINK V2            STM32 板
  SWDIO  ──────────→  PA13
  SWCLK  ──────────→  PA14
  GND    ──────────→  GND
  3.3V   ──────────→  VCC (板子无外部供电时)
```

#### USB-TTL 串口接线
```
USB-TTL               STM32 板
  TX     ──────────→  PA10 (USART1 RX)
  RX     ──────────→  PA9  (USART1 TX)
  GND    ──────────→  GND
```
> TX→RX、RX→TX 必须**交叉**。

#### CAN 电机接线
```
STM32 CAN1_H ──→ 所有 Emm_V5 CAN_H 并联
STM32 CAN1_L ──→ 所有 Emm_V5 CAN_L 并联
24V+  ──→ 所有 Emm_V5 驱动器供电+
GND   ──→ 所有 Emm_V5 驱动器供电-
```

---

## 3. Python 环境搭建（本地 LeRobot 部署）

### 3.1 前置条件

| 软件 | 版本要求 | 说明 |
|------|---------|------|
| Python | **≥3.12** | LeRobot 0.6 要求 Python 3.12+ |
| pip | 最新版 | 用于安装包 |

> 本机已安装 Python 3.12.10。不要求 CUDA（训练可在带 GPU 的机器上进行，采集可在任何 Windows PC 上进行）。

### 3.2 创建虚拟环境

```powershell
cd E:\Arm-robot_VLA
python -m venv .venv
```

创建后在 **每次新开终端** 时都需要激活：

```powershell
.venv\Scripts\Activate.ps1
```

> 如果 PowerShell 提示"在此系统上禁止执行脚本"，先以管理员身份运行：
> ```powershell
> Set-ExecutionPolicy -Scope CurrentUser -ExecutionPolicy RemoteSigned
> ```

激活后终端提示符前面会显示 `(.venv)`：

```
(.venv) E:\Arm-robot_VLA>
```

### 3.3 安装 LeRobot 核心库

确保 `.venv` 已激活后：

```powershell
pip install lerobot
```

这会自动安装 PyTorch 2.11、torchvision、numpy、opencv-python-headless、huggingface-hub 等依赖。

> ⏱ 安装耗时 5-20 分钟（取决于网络速度），PyTorch 约 800MB。进度看终端输出即可，看到 **Successfully installed lerobot-0.6.0** 即完成。

### 3.4 验证核心库安装

```powershell
python -c "import lerobot; print(f'lerobot {lerobot.__version__}'); import torch; print(f'torch {torch.__version__}')"
```

预期输出：
```
lerobot 0.6.0
torch 2.11.0
```

### 3.5 安装额外依赖（手柄 + 相机）

```powershell
pip install pygame opencv-python pyserial
```

| 包 | 用途 |
|---|------|
| `pygame` | 读取 USB 游戏手柄输入 |
| `opencv-python` | **完整版** OpenCV（非 headless），支持相机窗口显示 |
| `pyserial` | 串口通信（已自动安装） |

---

## 4. LeRobot 插件安装与验证

### 4.1 插件包结构

本项目插件包位于 `E:\Arm-robot_VLA\lerobot_robot_massage\`，遵循 LeRobot BYOH 规范：

| 文件 | 用途 |
|------|------|
| `pyproject.toml` | 包配置（包名 `lerobot_robot_massage`，以 `lerobot_robot_` 前缀自动发现） |
| `__init__.py` | 导出 `MassageRobotConfig`、`MassageRobot` |
| `config_massage_robot.py` | `MassageRobotConfig` 配置类（串口参数、关节名、相机） |
| `massage_robot.py` | `MassageRobot(Robot)` 子类（核心适配器） |
| `serial_protocol.py` | `SerialProtocol` 串口协议封装（get_state/set_joints/set_torque/e_stop） |

### 4.2 以可编辑模式安装

```powershell
cd E:\Arm-robot_VLA
.venv\Scripts\python.exe -m pip install -e lerobot_robot_massage
```

> `-e` 表示可编辑模式，修改插件代码后无需重新安装即可生效。

### 4.3 验证插件安装

```powershell
.venv\Scripts\python.exe -c "
from lerobot_robot_massage import MassageRobotConfig, MassageRobot
print(f'Config: {MassageRobotConfig.__name__}')
print(f'Robot:  {MassageRobot.__name__}')
print('OK — 插件已安装')
"
```

### 4.4 验证 LeRobot 自动发现

LeRobot 的 CLI 工具（`lerobot-record`、`lerobot-teleop`）通过包名前缀 `lerobot_robot_` 自动发现自定义机器人：

```powershell
.venv\Scripts\python.exe -c "
from lerobot.robots import RobotConfig
cls = RobotConfig.get_choice_class('massage_robot')
print(f'Registry resolves massage_robot → {cls.__name__}')
# 预期: Registry resolves massage_robot → MassageRobotConfig
"
```

### 4.5 验证接口规范（不接硬件）

```powershell
.venv\Scripts\python.exe -X utf8 scripts\verify_interface.py
```

预期输出 11 项全部 `[PASS]` 并以 `全部验证通过` 结束。

---

## 5. 串口通信验证

### 5.1 确认串口号

把 USB-TTL 模块插上电脑后，在设备管理器中查看串口号（本例为 COM5）：

```
设备管理器 → 端口（COM 和 LPT） → USB Serial Port (COM5)
```

> 如果找不到，检查 CH340G 驱动是否安装，或换一个 USB 口。

### 5.2 验证固件启动

用串口助手（如 Putty、Arduino 串口监视器）连接 COM5，115200-8-N-1，然后给 STM32 上电：

```
[BOOT] UART1 PA9 OK
[INIT] robot_init OK
```

出现上述启动信息表示固件运行正常。

### 5.3 测试 LeRobot 命令（关掉串口助手后）

```powershell
.venv\Scripts\python.exe -c "
from lerobot_robot_massage import SerialProtocol
p = SerialProtocol('COM5')
p.connect()
angles, vels, loads = p.get_state()
print(f'angles:  {[round(a,1) for a in angles]}')
print(f'vels:    {[round(v,1) for v in vels]}')
print(f'loads:   {[round(l,1) for l in loads]}')
p.disconnect()
"
```

预期输出 (6 个关节的角度、速度、负载)：
```
angles:  [90.0, 90.0, 270.0, 0.0, 90.0, 0.0]
```

### 5.4 测试扭矩控制

```powershell
.venv\Scripts\python.exe -c "
from lerobot_robot_massage import SerialProtocol
from time import sleep
p = SerialProtocol('COM5')
p.connect()
print('扭矩 OFF → 机械臂可手动拖拽')
p.set_torque(False)
sleep(3)
print('扭矩 ON → 机械臂锁定')
p.set_torque(True)
p.disconnect()
"
```

> 发送 `set_torque 0` 后电机断电、机械臂可自由转动（手动示教模式）。
> 发送 `set_torque 1` 后电机锁定位置。

---

## 6. USB 相机接入

### 6.1 确认相机索引

```powershell
.venv\Scripts\python.exe -c "
import cv2
for i in range(5):
    cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            print(f'Camera {i}: {frame.shape[1]}x{frame.shape[0]}')
        cap.release()
"
```

预期输出（取决于你的相机）：
```
Camera 1: 640x480
```

本例中相机索引为 **1**（已在 `config_massage_robot.py` 和 `configs/massage_robot.yaml` 中预设）。

### 6.2 LeRobot 相机测试

```powershell
.venv\Scripts\python.exe -c "
from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig
cam = OpenCVCamera(OpenCVCameraConfig(index_or_path=1, fps=30, width=640, height=480))
cam.connect()
frame = cam.read_latest()
print(f'Frame shape: {frame.shape}')
cam.disconnect()
"
```

---

## 7. 机械臂联通验证

关掉串口助手，运行联调测试（同时验证串口 + 相机）：

```powershell
.venv\Scripts\python.exe -c "
from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig
from lerobot_robot_massage import SerialProtocol

proto = SerialProtocol('COM5'); proto.connect()
cam = OpenCVCamera(OpenCVCameraConfig(index_or_path=1, fps=30, width=640, height=480))
cam.connect()

angles, _, _ = proto.get_state()
frame = cam.read_latest()
print(f'STM32 angles: {[round(a,1) for a in angles[:6]]}')
print(f'Camera frame: {frame.shape}')
print('=== 联调通过 ===')

proto.disconnect(); cam.disconnect()
"
```

---

## 8. 手柄遥操作（Joystick 遥控）

### 8.1 安装手柄驱动

本脚本使用 `pygame` 读取 USB 游戏手柄，支持常见的手柄（Xbox、PS4、通用 USB 手柄）。一般 Windows 会自动识别。

**首次使用前安装 pygame**（如已从 3.5 安装则可跳过）：
```powershell
pip install pygame
```

### 8.2 启动遥控

```powershell
.venv\Scripts\python.exe scripts\joystick_control.py --port COM5 --camera 1
```

- `--port`：STM32 串口号（默认 COM5）
- `--camera`：USB 摄像头索引（默认 1）

### 8.3 手柄按键映射

```
┌──────────────────────────────────────────────────┐
│  左摇杆        →  末端 XY 平移                   │
│  右摇杆        →  末端旋转 (RX/RY)               │
│  L2 / R2      →  末端 Z 升降                    │
│  A 键          →  remote_enable（进入遥控模式）   │
│  B 键          →  remote_disable（退出遥控模式）  │
│  Y 键          →  e_stop（急停）                  │
│  X 键          →  set_torque 切换                 │
│  十字键 ↑↓     →  逐关节控制（J1-J6）            │
│  L1 / R1      →  切换当前关节                    │
│  十字键 ←→    →  退出关节模式，回到笛卡尔遥控     │
│  START        →  zero（当前位置归零）             │
│  BACK         →  退出脚本                        │
└──────────────────────────────────────────────────┘
```

### 8.4 控制模式

遥控脚本支持两种控制模式，可在运行中切换：

**笛卡尔模式（默认）**：通过摇杆控制机械臂末端在笛卡尔空间运动。左摇杆→XY平移，L2/R2→Z升降，右摇杆→旋转。适合大范围粗调。

**关节模式**：按 L1/R1 进入，可选中单个关节（J1-J6，显示高亮），用十字键 ↑↓ 步进旋转。适合微调校准。按 ←/→ 退出回到笛卡尔模式。

### 8.5 安全操作流程

```
1. 给 STM32 上电 → 等待启动完成
2. 给所有电机上 24V 电源
3. 运行手柄脚本: python scripts/joystick_control.py
4. 按 A 键 → remote_enable（软复位到初始姿态）
5. 按 X 键 → set_torque 1（使能电机）
6. 开始摇杆控制
7. 急停：按 Y 键（e_stop）
8. 退出：先按 B → remote_disable，再按 BACK 退出
```

> ⚠️ **安全须知**：
> - `remote_enable` 要求所有电机上电，否则 CAN timeout
> - 首次使用请确保急停按钮（Y）可用
> - 遥控模式下操作员必须始终在急停按钮旁边

### 8.6 相机 HUD

运行手柄脚本后，会弹出一个相机窗口，实时显示关节角度和模式状态：

```
左侧面板:             右侧面板:
J1:  90.0  ◀         REMOTE          ← 遥控模式
J2:  45.0             TORQUE:ON      ← 电机使能
J3: -20.0             FPS: 29.8      ← 帧率
J4:   0.0
J5:  95.0             ◀ 标记 = 当关节选中
J6:  40.0
```

---

## 9. 数据采集：用手柄录训练数据

### 9.1 两种采集方式对比

| 方式 | 操作 | 工具 | 适用场景 |
|------|------|------|---------|
| **手动示教** | `set_torque 0` → 手拖臂 → LeRobot record | `lerobot-record`（手动托拽） | 没有手柄时 |
| **手柄遥操作采集** | 手柄控制 + 同步记录 | 本项目的`record_with_joystick.py` | 精确平滑轨迹、远程操作 |

### 9.2 方法 A：手动示教（扭矩关闭 + 录轨迹）

**原理**：关闭电机扭矩，用手拖拽机械臂演示动作，LeRobot 定时读取关节角度 + 相机画面记录轨迹。

**步骤：**

1. 运行记录脚本：
```powershell
.venv\Scripts\python.exe -c "
from lerobot_robot_massage import SerialProtocol, MassageRobot, MassageRobotConfig
from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig
import time, json
from pathlib import Path

# 连接
proto = SerialProtocol('COM5'); proto.connect()
cam = OpenCVCamera(OpenCVCameraConfig(index_or_path=1, fps=30, width=640, height=480))
cam.connect()

# 进入自由模式
proto.set_torque(False)
print('扭矩已关闭，可手动拖拽机械臂。录制期间按 Ctrl+C 停止')

# 开始录制
timestamps = []
states = []
for i in range(600):  # 20秒 x 30fps
    angles, vels, loads = proto.get_state()
    frame = cam.read_latest()
    timestamps.append(time.time())
    states.append({'angles': angles, 'frame': frame})
    time.sleep(1/30)

# 退出
proto.set_torque(True)
cam.disconnect(); proto.disconnect()
print(f'录制完成，共 {len(states)} 帧')
"
```

### 9.3 方法 B：手柄遥控采集（推荐）

由于 `lerobot-record` 需要一个已注册的 Teleoperator 插件才能从手柄获取动作，而我们目前还未创建该插件，推荐用以下**组合方式**：

1. 在**一个窗口**运行手柄脚本控制机械臂
2. 在**另一个窗口**运行记录脚本来捕获状态

**分步操作：**

**窗口 1 — 手柄控制**（保持运行）：
```powershell
.venv\Scripts\python.exe scripts\joystick_control.py --port COM5
```
→ 按 A 使能遥控、按 X 使能扭矩，开始摇杆控制。

**窗口 2 — 录制观察数据**（采集关节 + 相机数据）：
> 此脚本会每秒 30 帧记录关节角度和相机图像，生成 `datasets/` 下的 CHUNK 文件。

由于 LeRobot 的标准 record 管线需要 Teleoperator 插件来产生动作，我们再用一个**自定义录制脚本来做数据采集（无需 Teleoperator 插件）**。

### 9.4 录制一条训练轨迹

把下面的内容保存为 `scripts/record_trajectory.py`：

```python
"""手柄遥控 + 同步轨迹记录。

运行方式：
  1. 本机先插好手柄
  2. STM32 上电、电机上电
  3. 在一个终端跑手柄控制:   python scripts/joystick_control.py
  4. 在另一个终端跑本脚本:    python scripts/record_trajectory.py

本脚本只负责记录（get_state + 相机帧），机械臂控制由手柄脚本负责。
"""

import time
import json
import argparse
from pathlib import Path

import cv2
import numpy as np

from lerobot_robot_massage import SerialProtocol


def record_episode(port: str, camera_index: int,
                   duration_s: int = 20, fps: int = 30):
    """录制一段轨迹到 datasets/raw/ 目录"""
    proto = SerialProtocol(port)
    proto.connect()

    cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print(f"⚠ 无法打开相机 {camera_index}，将只录制关节数据")

    output_dir = Path("datasets/raw")
    output_dir.mkdir(parents=True, exist_ok=True)

    episode_id = len(list(output_dir.glob("episode_*"))) + 1
    episode_dir = output_dir / f"episode_{episode_id:04d}"
    episode_dir.mkdir(parents=True)

    # 等待 5 秒准备，给操作员时间切换到手柄控制
    print("准备开始录制，请在 5 秒内切换到手柄窗口...")
    for i in range(5, 0, -1):
        print(f"{i}...")
        time.sleep(1)

    start = time.time()
    frame_count = 0
    interval = 1.0 / fps
    data = []

    print(f"🎬 开始录制 Episode {episode_id}（{duration_s}s @ {fps}fps）")
    print("  按 Ctrl+C 提前停止录制")

    try:
        while time.time() - start < duration_s:
            frame_time = time.time()

            # 读关节状态
            angles, vels, loads = proto.get_state()
            if not angles:
                print("  ⚠ get_state 超时")
                continue

            # 读相机画面
            frame = None
            if cap and cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    frame = None

            # 保存
            timestamp = frame_time - start
            data.append({
                "timestamp": round(timestamp, 3),
                "angles": [round(a, 2) for a in angles[:6]],
                "velocities": [round(v, 2) for v in vels[:6]],
                "loads": [round(l, 2) for l in loads[:6]],
            })

            # 每帧保存相机图像
            if frame is not None:
                cv2.imwrite(str(episode_dir / f"frame_{frame_count:06d}.png"), frame)

            frame_count += 1

            # 精确控制帧率
            elapsed = time.time() - frame_time
            if elapsed < interval:
                time.sleep(interval - elapsed)

    except KeyboardInterrupt:
        print("\n⏹ 用户中断录制")

    # 保存 JSON 数据
    meta = {
        "episode_id": episode_id,
        "duration_s": round(time.time() - start, 2),
        "frames": frame_count,
        "joint_names": [
            "shoulder_pan", "shoulder_lift", "elbow_flex",
            "wrist_flex", "wrist_roll", "gripper"
        ],
        "fps": fps,
    }
    with open(episode_dir / "data.json", "w", encoding="utf-8") as f:
        json.dump({"meta": meta, "frames": data}, f, indent=2, ensure_ascii=False)

    print(f"✅ Episode {episode_id} 录制完成")
    print(f"  帧数: {frame_count}")
    print(f"  位置: {episode_dir}")

    if cap:
        cap.release()
    proto.disconnect()


def main():
    parser = argparse.ArgumentParser(description="手柄遥控 + 轨迹记录")
    parser.add_argument("--port", default="COM5")
    parser.add_argument("--camera", type=int, default=1)
    parser.add_argument("--duration", type=int, default=20)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()

    record_episode(args.port, args.camera, args.duration, args.fps)


if __name__ == "__main__":
    main()
```

**使用方法：**
```powershell
.venv\Scripts\python.exe scripts\record_trajectory.py --port COM5 --camera 1 --duration 20 --fps 30
```

该脚本会在 `datasets/raw/` 下生成以下结构：
```
datasets/raw/
├── episode_0001/
│   ├── data.json         ← 关节角度 + 时间戳
│   ├── frame_000000.png  ← 相机画面（帧对齐）
│   ├── frame_000001.png
│   └── ...
├── episode_0002/
└── ...
```

### 9.5 录制多条轨迹的建议

| 轨迹编号 | 动作内容 | 时长 | 说明 |
|:--------:|---------|:----:|------|
| 1 | 空载平移（不做接触） | 10s | 验证机械臂基本运动 |
| 2 | 画圆（俯视视角） | 10s | 测试末端轨迹平滑度 |
| 3 | 模拟按揉（绕 Z 轴） | 15s | 简单按摩手法 |
| 4 | 推压 + 放松交替 | 20s | 复合动作 |
| 5 | 全流程按摩演示 | 30s | 多种手法组合 |

> 录制轨迹数建议 50+ 条以获得可用的训练效果。每条轨迹应在 `duration_s` 秒内自然完成，不要过快或中断。

---

## 10. 模型训练（SmolVLA）

### 10.1 训练前的数据准备

把手采集数据（`datasets/raw/`）转换为 LeRobot 兼容的数据集格式。参考 LeRobot 文档中的 `lerobot/scripts/control_robot.py record` 工具或自定义转换脚本。

当前最简单的训练路径：
1. 用 `lerobot-record --robot.type=massage_robot` 录制（使用手动示教模式）
2. 或用上面的 `record_trajectory.py` 采集后，自行编写转换脚本

在带 GPU 的训练机上：

```powershell
# 安装训练依赖
pip install 'lerobot[all]'

# 运行训练（调整参数）
python lerobot/scripts/train.py ^
  --policy.type=smolvla ^
  --policy.path=lerobot/smolvla_base ^
  --dataset.repo_id=./datasets/massage_v1 ^
  --batch_size=64 ^
  --steps=200000 ^
  --output_dir=outputs/train/smolvla_massage
```

训练配置参考 `configs/train_smolvla.yaml`。

---

## 11. 模型推理部署

训练完成后，将推理模型部署回数据采集 PC：

```powershell
python lerobot/scripts/control_robot.py replay ^
  --robot.type=massage_robot ^
  --robot.port=COM5 ^
  --policy.path=outputs/train/smolvla_massage/checkpoints/last/pretrained_model
```

推理时数据流：
```
[相机帧 + 关节状态]
       │
[SmolVLA 模型]
       │ 输出目标关节角度
[MassageRobot.send_action()]
       │ set_joints 命令 (空格分隔)
[STM32 → CAN → 电机执行]
```

---

## 12. 故障排查

### 环境与安装

| 现象 | 原因 | 解决 |
|------|------|------|
| `pip install lerobot` 慢 | PyTorch 包大（~800MB） | 等待，确保网络稳定 |
| `import lerobot` 报错 | Python 版本 <3.12 | 升级到 Python 3.12+ |
| 手柄脚本报 `No module named 'pygame'` | 未安装 pygame | `pip install pygame` |
| 手柄检测不到 | 手柄未插或驱动未装 | 进"游戏控制器"面板确认 |

### 串口问题

| 现象 | 原因 | 解决 |
|------|------|------|
| `SerialException: could not open port` | COM 被占用（串口助手） | 关掉串口助手 |
| `get_state` 返回空 | 串口堵塞或 STM32 忙 | 检查接线，重启 STM32 |
| `Failed to read state from STM32` | 串口无响应 | 确认 USB-TTL 接线 TX/RX 交叉 |
| LOG 消息洪水 | CAN 电机离线 | 给所有电机上电 |

### 机械臂控制

| 现象 | 原因 | 解决 |
|------|------|------|
| 关节不动 | 电机未使能 | 发送 `set_torque 1` |
| 关节 4（J5硬件但 CAN ID 5）有电流声但不转 | 电机可能相线问题 | 跳过该关节，其他 5 个正常 |
| 发送 `set_joints` 后某些关节归零 | ~~逗号格式解析错误~~ | ✅ 已修复为空格分隔（2026-07-14） |
| `remote_enable` CAN timeout 洪流 | 电机 24V 未上电 | 打开电机电源，或不要用 `remote_enable` |
| 卡住不动 | 超出限位 | 发送 `soft_reset` 回到初始姿态 |

### 相机问题

| 现象 | 原因 | 解决 |
|------|------|------|
| Camera `Failed to open` | 索引不对 | 用前文 cv2 检测命令确认实际索引 |
| `Cannot find camera` | Index 被其他应用占用 | 换一个 index 或关闭其他软件 |
| 相机画面卡顿 | USB 带宽不足 | 降低分辨率到 640×480，不和其他 USB 3.0 设备共用控制器 |

---

## 附录 A：常用命令速查

```powershell
# 激活环境
.venv\Scripts\Activate.ps1

# 启动手柄遥控
python scripts\joystick_control.py --port COM5

# 录制轨迹
python scripts\record_trajectory.py --port COM5 --duration 20

# 串口测试
python scripts\test_serial.py --port COM5

# 接口验证（不接硬件）
python scripts\verify_interface.py

# 更新插件（修改后无需重装，-e 模式自动生效）
# 只需在环境激活状态下重新导入即可
```

## 附录 B：文件索引

| 文件 | 用途 |
|------|------|
| `lerobot_robot_massage/` | LeRobot 插件包（核心代码） |
| `scripts/joystick_control.py` | 手柄遥控脚本 |
| `scripts/record_trajectory.py` | 轨迹录制脚本（建议新建） |
| `scripts/verify_interface.py` | 接口规范验证（不接硬件） |
| `scripts/test_serial.py` | 串口通信测试 |
| `configs/massage_robot.yaml` | 机器人硬件配置 |
| `configs/train_smolvla.yaml` | SmolVLA 训练配置 |
| `docs/SERIAL_COMMANDS.md` | 14 条串口命令完整参考 |
| `docs/ARCHITECTURE.md` | 系统架构设计 |
| `docs/WORKFLOW.md` | 端到端工作流 |
| `firmware/README.md` | STM32 固件修改指南 |
