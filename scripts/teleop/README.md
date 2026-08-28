# 机械臂 6DOF 视觉遥操系统使用教程

> 本文档针对 `Arm-robot_VLA/scripts/teleop/` 目录下的机械臂视觉遥操模块，详细说明系统架构、核心解耦算法、手眼标定向导、实机运行指令、安全操作阶梯及故障排查。

---

## 1. 核心架构与文件索引

所有视觉遥操核心代码均位于目录：
📂 **`Arm-robot_VLA/scripts/teleop/`**

| 文件名 | 职责与定位 | 核心技术要点 |
|---|---|---|
| **[`real_arm_teleop.py`](real_arm_teleop.py)** | **真机 6DOF 视觉遥操主入口** | RealSense D455 + 3D 刚体掌骨姿态解耦 + 6DOF 闭环笛卡尔控制器 + CAN 直驱 + HUD 仪表盘 |
| **[`handeye_calib.py`](handeye_calib.py)** | **手眼标定与可视化向导** | 交互式 3 轴挥手向导，Procrustes SVD 正交求解相机系到机械臂基座系 $R_{cam \to base}$ |
| **[`handeye_calib.json`](handeye_calib.json)** | **手眼外参矩阵存储** | 实时存储 $3\times 3$ 手眼正交旋转外参矩阵 |
| **[`arm_adapter.py`](arm_adapter.py)** | **机械臂控制适配层 (Adapter)** | 提供 `RealArmAdapter` 与 `NoDriveArmAdapter` 统一控制与状态回读接口 |
| **[`watchdog.py`](watchdog.py)** | **四级视觉安全看门狗** | 负责对丢失手部、低置信度、深度无效、腕心跳变进行 `OK / DECAY / STOP / ESTOP` 分级防护 |
| **[`demo_arm_teleop.py`](demo_arm_teleop.py)** | **仿真 / 算法验证入口** | 适用于 MuJoCo 仿真环境测试或纯算法验证 |
| **[`test_real_arm_teleop.py`](test_real_arm_teleop.py)** | **遥操管线单元测试** | Fake Provider 验证按键、超时、看门狗升级与数据录制 |
| **[`test_handeye_calib.py`](test_handeye_calib.py)** | **手眼标定单元测试** | 验证正交约束、欧拉角转换与 3 轴映射解算 |
| **[`test_sim_regression.py`](test_sim_regression.py)** | **6DOF 仿真全栈闭环回归** | `FakeMuJoCoServer` + 6DOF 空间运动位姿积分验证 |

---

## 2. 核心算法特性 (Key Highlights)

```
                    【MediaPipe 3D 神经网络】
                               │
                               ▼
        【提取 3D World Landmarks (米制 3D 刚体骨骼)】
                               │
          ┌────────────────────┴────────────────────┐
          ▼                                         ▼
【解剖学刚体掌骨 0-5-17】                      【纯手腕深度 Point 0】
  Wrist(0), IndexMCP(5), PinkyMCP(17)         位于手腕褶皱处，绝不受手指弯曲影响
  (此三点为刚体手掌骨架，抓握/屈指完全解耦)                 │
          │                                         ▼
          ▼                               【纯手腕位置与线速度 v_lin】
【纯正 SO(3) 手掌姿态 R_palm】
          │
          ▼
【SVD 正交重整化 + 一欧元滤波】
          │
          ▼
【输出摇杆偏角速率 dPitch / dRoll / w_ang (5° 死区, 回平锁定)】
```

1. **解剖学刚体掌骨解耦（Metacarpal Frame Decoupling）**：
   - 传统方案在手指弯曲抓握时会因手指遮挡指根导致深度误采样，产生 $20^\circ \sim 30^\circ$ 的伪俯仰/翻滚。
   - 本系统直接基于 MediaPipe `world_landmarks` 提取**手腕（Point 0）、食指指根（Point 5）、小指指根（Point 17）**构成的刚体掌骨平面。无论操作员握拳、屈指还是做抓取动作，掌面朝向稳如泰山。
2. **绝对姿态 1:1 闭环伺服跟踪（Absolute Orientation Tracking）**：
   - 彻底废除传统“倾斜持续转、回平找死区”的摇杆累赘模式；
   - 采用李代数闭环伺服：**手倾斜多少度，机械臂末端旋转多少度；手回平则机械臂瞬间回平**！推拿数据采集零负担。
3. **推拿专属 4 大姿态解耦模态（Tuina Task Modes）**：
   - **模式 1: 垂直点按揉捏模式 (`KNEAD` / 姿态全锁定)**：末端姿态强制垂直朝下锁定（$\vec{\omega}=0$），空间移动自由，手指任意进行 16-DOF 揉捏/点按，末端姿态 100% 稳如泰山！
   - **模式 2: 滚法一指禅推法模式 (`ROLL_ONLY` / 单轴Roll / 锁Pitch)**：锁定 Pitch/Yaw，仅开放沿臂前进轴的 Roll 滚转，专为推拿“滚法”标准动作设计；
   - **模式 3: 俯仰角度调节模式 (`PITCH_ONLY` / 单轴Pitch / 锁Roll)**：锁定 Roll/Yaw，仅开放垂直横向轴的 Pitch 俯仰，专为推拿叩法/点穴/切入角度调整设计；
   - **模式 4: 全 6-DOF 自由遥操模式 (`FULL_6DOF`)**：开放 3 轴全部自由度，满足复杂全自由度手法需求。
   - **按键支持**：在运行时随时按键盘 **`M`**（循环切换 1 $\to$ 2 $\to$ 3 $\to$ 4 $\to$ 1）或 **`1` / `2` / `3` / `4`** 直达对应模式。
4. **线速度平滑与低速安全透传**：
   - EMA 指数滑动滤波 ($\alpha=0.30$) 消除采样微震，死区经过单轴独立解耦与跨轴抑制，支持低至 5%（$1 \sim 4\text{ mm/s}$）的超低速安全遥操。
5. **全状态 6 轴电机 HUD 仪表盘**：
   - 画面右下角实时展示 6 轴电机 `ALL STATUS`（绝对角度、实时相电流、驱动器状态标志）。

---

## 3. 运行前置准备

### 3.1 硬件连接与环境建议
1. **USB-CAN 分析仪**：插入工控机 USB 3.0 接口，CAN-H / CAN-L 接驳机械臂总线；
2. **Intel RealSense D455 深度相机**：
   - **最佳操作距离**：**`60 cm ~ 80 cm`**（人手臂半伸展的自然舒适位置）；
   - **盲区警示**：人手距离镜头不得近于 **`40 cm`**（近于 40cm 会进入双目深度盲区导致丢帧）；
3. **机械臂动力电源**：开启 24V/48V 动力电源（确保物理急停拍钮处于操作员左手可及范围）。

### 3.2 启用 SocketCAN 接口
每次开机后在终端执行一次：
```bash
sudo ip link set can0 up type can bitrate 500000
```
*验证接口状态*：
```bash
ip link show can0
# 正常应显示: <NOARP,UP,LOWER_UP,ECHO> state UP
```

---

## 4. (可选) 可视化交互手眼标定向导

如果调整了 RealSense 相机的安装角度或高低位置，建议运行 30 秒可视化标定向导：

```bash
cd ~/win_office/ubantu_files/project/TuinaDex/Arm-robot_VLA
python scripts/teleop/handeye_calib.py
```

```
┌─────────────────────────────────────────────────────────────┐
│ [Step 1/3: 请沿机械臂 +X 方向 (右方) 平移手掌约 15cm]         │
│ Press [SPACE] to start recording motion vector              │
└─────────────────────────────────────────────────────────────┘
```

1. **Step 1/3 (右移 +X)**：按 **`SPACE`** 键开始录制，将手掌向**右方平移 15~20cm**，再次按 **`SPACE`** 保存；
2. **Step 2/3 (前推 +Y)**：按 **`SPACE`** 键开始录制，将手掌向**前方推移 15~20cm**，再次按 **`SPACE`** 保存；
3. **Step 3/3 (上抬 +Z)**：按 **`SPACE`** 键开始录制，将手掌向**上方抬高 15~20cm**，再次按 **`SPACE`** 保存；
4. 系统自动通过 SVD 正交分析解出最优 $R_{cam \to base}$ 矩阵并保存至 `handeye_calib.json`。

---

## 5. 运行使用与分级安全测试指南

### 5.1 启动命令与常用参数

进入主项目目录：
```bash
conda activate leap_hand
cd ~/win_office/ubantu_files/project/TuinaDex/Arm-robot_VLA
```

| 场景需求 | 执行命令 | 说明 |
|---|---|---|
| **纯视觉空跑演练 (无硬件)** | `python scripts/teleop/real_arm_teleop.py --no-drive` | 仅做视觉识别、HUD 显示与按键测试，不连 CAN 总线 |
| **真机 5% 超低速安全初测** | `python scripts/teleop/real_arm_teleop.py --iface can0 -y --speed-scale 0.05` | **首次真机必选**。限速 5% ($v \le 4\text{ mm/s}$)，防冲防撞 |
| **真机 20% 中速推拿测试** | `python scripts/teleop/real_arm_teleop.py --iface can0 -y --speed-scale 0.20` | 推拿标准速度 ($v \le 16\text{ mm/s}$)，适合精细作业 |
| **真机 100% 全速模式** | `python scripts/teleop/real_arm_teleop.py --iface can0 -y` | 全速跟随 ($v \le 80\text{ mm/s}$)，适合大幅度轨迹采集 |

#### 常用命令行参数说明：
- `--iface can0`：指定 SocketCAN 接口名称（默认 `can0`）；
- `-y` 或 `--gravity-confirm`：**必须携带**，显式二次确认使能重力关节（J2/J3）扭矩；
- `--speed-scale <float>`：平移线速度缩放系数（`0.01` ~ `1.0`，默认 `1.0`）；
- `--ang-scale <float>`：旋转角速度独立缩放系数（`0.01` ~ `1.0`，默认自动解耦为高灵敏度 `0.8`）；
- `--calib <path>`：指定手眼标定矩阵文件路径（默认 `scripts/teleop/handeye_calib.json`）；
- `--out <path>`：指定多模态训练数据集录制目录（默认 `datasets/teleop_real`）；
- `--no-drive`：空跑模式开关（无硬件，仅开 OpenCV 遥操画面）。

---

### 5.2 遥操键盘交互快捷键

在弹出的 OpenCV 画面窗口中，支持以下全局快捷键：

- **`SPACE` 空格键 (离合器 Clutch Toggle 开关)**：
  - **启动默认态**：系统启动默认处于 **`[CLUTCH PAUSED]`（黄色）**，速度强制锁定为 0，防止误动；
  - **按一下空格**：切换为 **`[TELEOP ACTIVE]`（绿色）**，开始实时响应手部动作；
  - **再按一下空格**：随时暂停锁定当前机械臂位姿，操作员可自由休息或抽手。
- **`C` 键 (一键姿态零点重校准 Re-center)**：
  - 手掌放平后按 `C`，立即将当前手部朝向设为中立摇杆零位。
- **`R` 键 (回准备姿态 Ready)**：
  - 各关节安全同步平滑运动至按摩准备姿态（`[0°, 60°, 50°, 0°, 120°, 0°]`）。
- **`H` 键 / `O` 键 / `0` 键 (回上电姿态 Home)**：
  - 各关节安全同步运动至上电全零姿态（`[0°, 0°, 0°, 0°, 0°, 0°]`）。
- **`Y` 键 (E-Stop 紧急制动)**：
  - 立即向 CAN 总线广播抱闸停机，切入 `STOPPED` 保护状态（按 `SPACE` 或 `R` 可尝试复位）。
- **`Q` 键 / `ESC` 键 (安全退出)**：
  - 停止发送速度，断开连接并保存当前录制数据后退出。

---

### 5.3 画面 UI 与 HUD 仪表盘

OpenCV 窗口提供丰富工业级视觉反馈：

```
┌─────────────────────────────────────────────────────────────┐
│ [TELEOP ACTIVE] SPACE: Pause | R: Ready | H: Home | Y: E-Stop│
├────────────────────────────────┬────────────────────────────┤
│ Action: OK | Phase: TELEOP      │ [ALL STATUS - 6 JOINTS]    │
│ v_lin: [+12.5,  -4.2,  +8.0]   │ Jnt CAN  Pos(deg) Cur Flag │
│ w_ang: [ +0.00, +0.12, -0.05]  │ J1  0x02  +0.0°   0mA 0x00 │
│ Joystick Tilt: dPitch / dRoll  │ J2  0x03 +60.0° 120mA 0x00 │
│                                │ J3  0x04 +50.0°  85mA 0x00 │
│   ● 手腕锚点 + 动态速度牵引线   │ J4  0x05  +0.0°   0mA 0x00 │
│                                │ J5  0x06+120.0° 110mA 0x00 │
│                                │ J6  0x07  +0.0°   0mA 0x00 │
└────────────────────────────────┴────────────────────────────┘
```

---

## 6. 安全保护与看门狗机制

1. **四级视觉看门狗 (`watchdog.py`)**：
   - **`OK`**：手部正常追踪，速度全额下发；
   - **`DECAY`**（短暂丢帧 $<0.4s$）：速度指数平滑衰减，维持 3 帧时域惯性缓冲，防电机顿挫；
   - **`STOP`**（手部丢失 $>0.4s$ 或位置跳变 $>150mm$）：速度立即归零；
   - **`ESTOP`**（完全丢失 $>1.0s$）：直接触发急停停机。
2. **控制层单调看门狗**：
   - 超过 250ms 无有效视觉新帧，自动强制归零，防止进程阻塞或网络掉包。
3. **软硬件多重限幅**：
   - 末端线速度硬截断：$|v| \le 80\text{ mm/s}$（电机转速 $\le 113\text{ RPM}$）；
   - 末端角速度硬截断：$|\omega| \le 0.8\text{ rad/s}$（输出角速度 $\le 45.8^\circ/\text{s}$）；
   - 单控制周期最大关节角变：$|\Delta q| \le 2.0^\circ$。

---

## 7. 自动化测试套件

在没有实体硬件时，可执行以下命令验证全部 44 项单元测试：

```bash
PYTHONPATH=. pytest scripts/teleop/
```
*测试覆盖内容*：
- `test_adapter.py`: Real / Simulation 统一适配器接口与状态机验证；
- `test_arm_client.py`: 驱动器网络通信与协议打包；
- `test_handeye_calib.py`: 手眼 SVD Procrustes 算法与欧拉角转换；
- `test_real_arm_teleop.py`: 离合器、死区、按键与多模态录制链路；
- `test_sim_regression.py`: 6DOF 空间运动位姿闭环回归；
- `test_watchdog.py`: 视觉看门狗四级状态转移与衰减测试。

---

## 8. 常见故障排查 (Troubleshooting)

### Q1: 运行提示 `ModuleNotFoundError: No module named 'can'`
- **解决**：在当前 Python 环境安装依赖 `pip install python-can`。

### Q2: 报错 `timeout addr=02 func=36` 或 `[Errno 105] 没有可用的缓冲区空间`
- **原因**：电机动力电源未开启，或 USB-CAN 的 CAN-H / CAN-L 接线断开，导致报文无法收到硬件 ACK。
- **解决**：检查 24V/48V 电机供电及 CAN 接线，确保终端电阻（120Ω）正常。

### Q3: 画面显示为绿色 `[TELEOP ACTIVE]`，但手移动时速度一直为 0
- **原因**：检查是否由于环境光照过暗导致 RealSense 丢失深度（画面中手腕圆环变红/灰）。
- **解决**：确保手部在相机前 60~80cm 范围，且室内光线均匀无强逆光。

### Q4: 机械臂动作方向与人手直觉相反（如手向前但臂向后）
- **解决**：运行 `python scripts/teleop/handeye_calib.py` 重新完成一次 30 秒手眼标定。
