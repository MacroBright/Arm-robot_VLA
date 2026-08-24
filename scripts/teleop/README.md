# 机械臂 6DOF 视觉遥操系统使用教程

> 本文档针对 `Arm-robot_VLA/scripts/teleop/` 目录下的机械臂视觉遥操模块，详细说明系统架构、脚本功能、前置准备、运行指令、安全操作及故障排查。

---

## 1. 脚本与核心文件路径

所有视觉遥操相关脚本均位于目录：
📂 **`Arm-robot_VLA/scripts/teleop/`**

| 文件名 | 职责与定位 | 说明 |
|---|---|---|
| **[`real_arm_teleop.py`](real_arm_teleop.py)** | **真机 6DOF 视觉遥操主入口** | RealSense D455 + 手势识别 + 6DOF 笛卡尔控制器 + CAN 直连 |
| **[`demo_arm_teleop.py`](demo_arm_teleop.py)** | **仿真 / 前代遥操入口** | 适用于 MuJoCo 仿真环境测试或双目/单目算法验证 |
| **[`arm_adapter.py`](arm_adapter.py)** | **臂控制适配层 (Adapter)** | 提供 `RealArmAdapter` 与 `SimulationArmAdapter` 统一接口 |
| **[`watchdog.py`](watchdog.py)** | **视觉安全看门狗** | 负责对丢失手部、低置信度、深度无效、腕心跳变进行分级安全防护 |
| **[`handeye_calib.py`](handeye_calib.py)** | **手眼标定与坐标变换** | 相机系到机械臂基坐标系 (Base) 空间旋转变换与标定生成 |
| **[`handeye_calib.json`](handeye_calib.json)** | **手眼标定外参配置** | 存储手眼标定矩阵 $R_{cam \to base}$ |
| **[`test_real_arm_teleop.py`](test_real_arm_teleop.py)** | **遥操管线无硬件单元测试** | Fake Provider 驱动验证按键、陈旧超时、看门狗升级与录制 |
| **[`test_sim_regression.py`](test_sim_regression.py)** | **6DOF 仿真全栈闭环回归** | `FakeMuJoCoServer` + 6DOF 空间运动位姿积分验证 |

---

## 2. 系统控制流与安全链路

```
[RealSense D455 相机 (RGB-D)]
               │
               ▼
[HandTracker / WristTracker (手势与手腕 6D 位姿提取)]
               │
               ▼
[VisionWatchdog 视觉看门狗] ─── (手部丢失/跳变/低置信度) ──► [DECAY / STOP / ESTOP]
               │
               ▼ (输出规范化 CartesianCommand)
[RealArmAdapter 机械臂适配器]
               │
               ▼
[CartesianController 6DOF 闭环控制器]
   ├─ 1. ARMED / TELEOP 状态机门禁
   ├─ 2. 测量 monotonic dt (单调时钟计算实际周期)
   ├─ 3. 单调陈旧命令看门狗 (超过 250ms 无新帧自动归零)
   ├─ 4. 工作空间边界限幅 (Workspace Box Bounding)
   ├─ 5. 奇异度指标监控与自适应阻尼 (Adaptive Damping DLS)
   ├─ 6. 预测关节限位渐进减速 (_scale_toward_limits)
   ├─ 7. 速度 / 加速度硬上限裁剪
   └─ 8. 实时 0x36 软限位守卫 (check_limits_real)
               │
               ▼ (输出安全关节角度 q_target)
[ZdtController (生命周期状态机) ──► ZdtDriver ──► SocketCAN (can0)]
               │
               ▼
[6 轴 Emm_V5 / ZDT 步进闭环驱动器]
```

同时，所有运行数据由 **`EpisodeRecorder`** 同步落盘为标准 JSONL 格式并记录相机帧图片，为后续 VLA / 具身模型训练提供数据集。

---

## 3. 运行前置准备

### 3.1 硬件连接
1. 将 **USB-CAN 分析仪** (如 USBCAN-UCP100) 连接至电脑 USB 端口，并连接机械臂 CAN 总线。
2. 将 **Intel RealSense D455** 深度相机通过 USB 3.0 数据线连接至电脑。
3. 机械臂供电电源开启（确保急停物理按钮处于可触及位置）。

### 3.2 启用 SocketCAN 接口
在终端执行以下命令将 `can0` 接口配置为 **500 kbps** 波特率并启动：
```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```
*验证接口状态*：
```bash
ip link show can0
# 正常应显示: <NOARP,UP,LOWER_UP,ECHO>
```

### 3.3 依赖安装
确保当前 Python 环境（如 `uv` 或 conda 环境）具备以下依赖：
- `python-can`
- `pyrealsense2`
- `opencv-python`
- `mediapipe`
- `numpy`, `scipy`

---

## 4. 运行使用指南

### 4.1 启动真机视觉遥操
进入主项目目录：
```bash
cd ~/win_office/ubantu_files/project/TuinaDex/Arm-robot_VLA
```

执行真机遥操命令（**必须携带 `-y` 确认重力关节使能**）：
```bash
python scripts/teleop/real_arm_teleop.py --iface can0 -y
```

### 4.2 常用命令行参数说明

| 参数 | 默认值 | 作用说明 |
|---|---|---|
| `--iface` | `can0` | 指定 SocketCAN 接口名称 |
| `-y`, `--gravity-confirm` | `False` | **必填**。二次确认使能重力关节（J2/J3）扭矩 |
| `--calib` | `scripts/teleop/handeye_calib.json` | 指定手眼标定矩阵文件路径 |
| `--out` | `datasets/teleop_real` | 指定录制轨迹与画面的输出文件夹 |
| `--no-drive` | `False` | 只做视觉追踪与显示计算，不向 CAN 总线发送驱动指令（用于空跑演练） |

### 4.3 遥操过程交互与快捷键

在弹出的 OpenCV 画面窗口中，支持以下键盘交互（按键与键盘遥操完全统一）：

- **`R` 键 (回准备姿态 Ready)**：
  - 各关节安全同步运动至按摩准备姿态（`[0°, 60°, 50°, 0°, 120°, 0°]`），速度 **100.0 RPM** 同步。
- **`H` 键 / `O` 键 / `0` 键 (回上电姿态 Home)**：
  - 各关节安全同步运动至上电全零姿态（`[0°, 0°, 0°, 0°, 0°, 0°]`），速度 **100.0 RPM** 同步。
- **`Y` 键 (E-Stop 紧急制动)**：
  - 立即向 CAN 总线广播 `0x0000 0xFE` 抱闸停机，状态机切入 `STOPPED` 保护。
- **`Q` 键 / `ESC` 键 (安全退出)**：
  - 停止发送速度，平稳断开连接并保存当前录制数据后安全退出。

---

## 5. 安全机制说明

1. **显式使能门禁 (Explicit Arm Gate)**：
   - 启动时 `connect()` 仅建立连接并校验 6 轴电机通信不变式，停留在 `SAFE_IDLE` 状态（不产生力矩）。
   - 只有用户通过命令行参数 `-y` 显式确认后，才会进入 `ARMED` 并过渡到 `TELEOP`。
2. **分级视觉看门狗 (Vision Watchdog)**：
   - **`OK`**（手部存在且置信度正常）：正常执行 6DOF 速度跟随。
   - **`DECAY`**（手部短暂离开或深度短暂抖动 $<0.4s$）：线速度与角速度指数级平滑衰减至 0（禁止无脑保持上一帧速度）。
   - **`STOP`**（手部持续丢失 $>0.4s$ 或手腕跳变 $>150mm$）：速度立即归零。
   - **`ESTOP`**（手部完全丢失 $>1.0s$）：直接触发紧急停机。
3. **控制层单调看门狗**：
   - 控制器周期性检查 `cmd_ts`，若超过 250ms 未收到新视觉指令，立即将笛卡尔命令强制归零，防止进程阻塞或网络卡顿造成失控。
4. **工作空间与关节限位预测**：
   - 工作空间被限制在安全长方体盒内（$X \in [-300, 300]$, $Y \in [-300, 300]$, $Z \in [0, 400]$ mm）。
   - 接近单轴软限位边缘时，自动按剩余安全边际比例渐进降速（Limit Deceleration Scaling）。

---

## 6. 无硬件/离线验证命令

在没有实体机械臂或相机的环境下，可通过以下命令测试整套控制管线与算法闭环：

### 6.1 运行单元测试（无硬件/Mock）
```bash
# 测试视觉看门狗分级逻辑
uv run python scripts/teleop/test_watchdog.py

# 测试 Adapter 接口与安全性
uv run python scripts/teleop/test_adapter.py

# 测试遥操管线 run_once (无相机纯逻辑驱动)
uv run python scripts/teleop/test_real_arm_teleop.py
```

### 6.2 运行 6DOF 仿真全栈闭环回归
```bash
# 启动 FakeMuJoCoServer 验证 6DOF 位姿积分与闭环响应
uv run python scripts/teleop/test_sim_regression.py
```

---

## 7. 常见问题排查 (Troubleshooting)

### Q1: 运行报错 `OSError: [Errno 19] No such device`
- **原因**：SocketCAN `can0` 尚未创建或未启动。
- **解决**：检查 USB-CAN 是否插好，执行 `sudo ip link set can0 type can bitrate 500000 && sudo ip link set can0 up`。

### Q2: 提示 `遥操前必须 -y/--gravity-confirm 确认重力关节 (J2/J3)`
- **原因**：为了防止误触直接使能电机导致重力下坠，必须传入 `-y` 参数。
- **解决**：在命令末尾加上 `-y`。

### Q3: 提示 `未检测到 RealSense (D455) 相机`
- **原因**：USB 接口松动或相机未被识别。
- **解决**：检查 `rs-enumerate-devices`，确保使用 USB 3.0 接口连接相机。

### Q4: 画面卡顿或机械臂动作迟滞
- **原因**：MediaPipe 手部识别计算占用过高或 CAN 发送频率受阻。
- **解决**：降低输入图像分辨率，或使用 `--no-drive` 排查纯视觉帧率是否稳定在 30fps。
