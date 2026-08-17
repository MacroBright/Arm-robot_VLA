# MuJoCo 机械臂仿真使用说明

> 版本: 0.1.0 | 日期: 2026-07-22 | 依赖: MuJoCo 3.10+

---

## 一、概述

在机械臂硬件未组装完成时，使用 MuJoCo 物理引擎 + SolidWorks 导出的 6-DOF 机械臂模型，通过 TCP 文本协议模拟 STM32 固件行为，实现：

1. **手柄遥操作** — 在仿真环境中用手柄操控虚拟机械臂
2. **数据录制** — 同步记录关节轨迹 + 渲染帧，生成训练数据集
3. **实物无缝切换** — 硬件就位后，仅需改 `--port` 参数即可从仿真切换到真机

### 架构

```
joystick_control.py ──socket:5555──→ mujoco_sim.py (MuJoCo 物理引擎)
MassageRobot         ──socket:5555──→   ├─ TCP 14 条命令解析
record_sim.py        ──SharedMemory──→  ├─ mj_step @ 50Hz
                                        ├─ 离屏渲染 → SharedMemory
                                        └─ MuJoCo viewer (可选)
```

### 仿真 vs 实物对照

```
仿真:  joystick_control.py → socket://localhost:5555 → mujoco_sim.py → MuJoCo
实物:  joystick_control.py → COM5                  → STM32         → 电机
```

---

## 二、环境准备

### 2.1 安装依赖

```bash
# 激活 conda 环境
conda activate smolvla          # Linux (conda)
# .venv\Scripts\Activate.ps1     # Windows

# 核心依赖
pip install mujoco                # MuJoCo 物理引擎 (3.10+)
pip install opencv-python         # 帧图像保存
pip install pyserial              # 串口协议 (socket:// 模式不需要真串口)

# 手柄遥控 (仅桌面环境)
pip install pygame                # USB 手柄读取

# LeRobot 插件 (可选, 用于 MassageRobot 集成; 见 scripts/setup_dev.sh)
pip install -e lerobot_robot_massage
```

### 2.2 验证安装

```bash
python -c "import mujoco; print('MuJoCo', mujoco.__version__)"
# 预期: MuJoCo 3.10.0
```

### 2.3 相关文件

```
scripts/
├── mujoco_scene/
│   ├── scene.xml              ← 改编的 MJCF 模型 (关节限位 + 位置伺服)
│   └── meshes/                ← STL 网格文件 (7 个)
├── mujoco_sim.py              ← MuJoCo 仿真 TCP 服务
├── record_sim.py              ← 仿真数据录制脚本
├── joystick_control.py        ← USB 手柄遥控 (不改动)
└── fake_stm32.py              ← 简单运动模拟器 (被 mujoco_sim.py 取代)
```

---

## 三、启动仿真

### 3.1 基本启动 (带 3D 可视化)

```bash
python scripts/simulation/mujoco_sim.py
```

启动后弹出 MuJoCo viewer 窗口，鼠标可旋转/缩放视角。

**命令行参数:**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `-p, --port` | `5555` | TCP 监听端口 |
| `--no-viewer` | 关 | 无头模式 (不弹窗口) |
| `--scene` | `scripts/mujoco_scene/scene.xml` | 自定义场景文件 |

### 3.2 无头模式 (服务器 / 无 GUI)

```bash
python scripts/simulation/mujoco_sim.py --no-viewer
```

不启动 viewer，TCP 服务和离屏渲染仍正常运行。

> 若渲染帧全黑，安装虚拟显示:
> ```bash
> sudo apt install xvfb
> xvfb-run python scripts/simulation/mujoco_sim.py --no-viewer
> ```

---

## 四、手柄遥操作

### 4.1 启动

确保 `mujoco_sim.py` 已在另一个终端运行：

```bash
# 先激活 conda 环境
conda activate smolvla
python scripts/control/joystick_control.py --port socket://localhost:5555 --camera 0
```

### 4.2 按键映射

```
┌──────────────────────────────────────────────────┐
│  左摇杆        →  末端 XY 平移                   │
│  右摇杆        →  末端旋转 (RX/RY)               │
│  L2 / R2      →  末端 Z 升降                    │
│  A 键          →  remote_enable（进入遥控模式）   │
│  B 键          →  remote_disable（退出遥控模式）  │
│  Y 键          →  e_stop（急停）                  │
│  X 键          →  set_torque 切换                 │
│  十字键 ↑↓     →  逐关节控制 (J1-J6)             │
│  L1 / R1      →  切换当前关节                    │
│  十字键 ←→    →  退出关节模式                    │
│  START        →  zero（当前位置归零）             │
│  BACK         →  退出脚本                        │
└──────────────────────────────────────────────────┘
```

### 4.3 操作流程

```
1. 终端 1: python scripts/simulation/mujoco_sim.py
2. 终端 2: python scripts/control/joystick_control.py --port socket://localhost:5555
3. 按 A → 进入遥控 (机械臂回 soft_reset 姿态)
4. 按 X → 使能扭矩
5. 推摇杆控制机械臂
6. 急停: 按 Y
7. 退出: 按 B → 按 BACK
```

---

## 五、数据录制

### 5.1 录制仿真轨迹

确保 `mujoco_sim.py` 在另一个终端运行：

```bash
python scripts/record_sim.py --duration 20 --fps 30
```

**参数:**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--port` | `socket://localhost:5555` | 仿真 TCP 地址 |
| `--duration` | `20` | 录制时长 (秒) |
| `--fps` | `30` | 录制帧率 |
| `--output` | `datasets/sim_v1` | 输出根目录 |

### 5.2 输出结构

```
datasets/sim_v1/
└── episode_0001/
    ├── data.json            ← 关节角度 + 速度 + 负载 + 时间戳
    ├── frame_000000.png     ← 渲染帧 (640x480)
    ├── frame_000001.png
    └── ...
```

### 5.3 data.json 格式

```json
{
  "meta": {
    "episode_id": 1,
    "duration_s": 20.1,
    "frames": 600,
    "fps_target": 30,
    "joint_names": ["shoulder_pan","shoulder_lift","elbow_flex",
                    "wrist_flex","wrist_roll","gripper"],
    "frame_width": 640,
    "frame_height": 480,
    "recorded_at": "2026-07-22T10:30:00"
  },
  "frames": [
    {
      "timestamp": 0.0,
      "angles": [90.0, 90.0, -90.0, 0.0, 90.0, 0.0],
      "velocities": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      "loads": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      "shm_frame": 0
    }
  ]
}
```

### 5.4 完整录制流程 (3 终端)

```
终端 1: python scripts/simulation/mujoco_sim.py                  ← 仿真后端
终端 2: python scripts/control/joystick_control.py \          ← 手柄控制
             --port socket://localhost:5555
终端 3: python scripts/record_sim.py --duration 20    ← 数据录制

步骤:
1. 启动终端 1
2. 启动终端 2, 按 A 进遥控, 按 X 使能扭矩
3. 启动终端 3, 立即切回终端 2 演示动作
4. 录制结束后检查 datasets/sim_v1/
```

### 5.5 采集建议

| 轨迹编号 | 动作 | 时长 | 说明 |
|:--------:|------|:----:|------|
| 1-10 | 空载平移/画圆 | 10s | 基础运动 |
| 11-25 | 模拟按揉 (绕 Z 轴) | 15s | 按压 + 旋转 |
| 26-40 | 推压动作 | 20s | 线性推 + 保持 |
| 41-50 | 复合手法 | 25s | 多种手法组合 |

> 建议录制 50+ 条轨迹。仿真数据可用于 Sim-to-Real: 预训练 + 实物数据微调。

---

## 六、模型参数

### 6.1 关节限位 (与固件 robot.c 一致)

| 关节 | 名称 | 角度范围 | CAN ID |
|:----:|------|:--------:|:------:|
| J1 | shoulder_pan | 0° ~ 360° | 2 |
| J2 | shoulder_lift | 90° ~ 180° | 3 |
| J3 | elbow_flex | -90° ~ 90° | 4 |
| J4 | wrist_flex | -90° ~ 90° | 5 |
| J5 | wrist_roll | 0° ~ 90° | 6 |
| J6 | gripper | 0° ~ 360° | 7 |

### 6.2 初始姿态 (soft_reset)

| J1 | J2 | J3 | J4 | J5 | J6 |
|:--:|:--:|:--:|:--:|:--:|:--:|
| 90° | 90° | -90° | 0° | 90° | 0° |

### 6.3 物理参数

| 参数 | 值 |
|------|-----|
| 步进频率 | 50 Hz |
| Actuator | position (位置伺服) |
| PD 增益 | kp=100, kv=20 |
| 力矩限幅 | ±20 Nm |
| 遥控速度增益 | 30°/s per unit |
| 遥控超时 | 0.3s |

---

## 七、协议命令速查

仿真支持全部 14 条命令 (与真固件一致)：

| # | 命令 | 格式 | 响应 | 说明 |
|:--|------|------|------|------|
| 1 | `get_state` | `get_state` | `STATE:j1..j6,v1..v6,l1..l6` | 读取关节状态 |
| 2 | `set_joints` | `set_joints j1..j6` | `OK` | 目标角度(°) |
| 3 | `set_torque` | `set_torque 0/1` | `OK` / `OK:FREE` | 扭矩开关 |
| 4 | `e_stop` | `e_stop` | `ESTOP` | 急停 |
| 5 | `remote_enable` | `remote_enable` | — | 进入远程 |
| 6 | `remote_disable` | `remote_disable` | — | 退出远程 |
| 7 | `remote_event` | `remote_event vx vy ry rx vz_up vz_down` | — | 笛卡尔速度 |
| 8 | `rel_rotate` | `rel_rotate joint_id angle` | — | 关节旋转 |
| 9 | `auto` | `auto x y z` | — | IK 定位(stub) |
| 10 | `zero` | `zero` | — | 归零 |
| 11 | `hard_reset` | `hard_reset` | — | 限位归零 |
| 12 | `soft_reset` | `soft_reset` | — | 回预设角度 |
| 13 | `stream_start` | `stream_start` | — | 开数据流(无操作) |
| 14 | `stream_stop` | `stream_stop` | — | 关数据流(无操作) |

---

## 八、手动测试命令

无需手柄，用 raw socket 直接测试：

```bash
# 终端 1: 启动仿真
python scripts/simulation/mujoco_sim.py --no-viewer

# 终端 2: 发送命令
python -c "
import socket, time

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('127.0.0.1', 5555))
s.settimeout(1.0)

def cmd(c):
    s.sendall(c.encode() + b'\n')
    time.sleep(0.05)
    try: return s.recv(4096).decode().strip()
    except: return None

print('get_state:', cmd('get_state')[:100])
print('torque on:', cmd('set_torque 1'))
print('move:', cmd('set_joints 100 120 -45 0 45 30'))
time.sleep(1)
print('after 1s:', cmd('get_state')[:100])
print('e_stop:', cmd('e_stop'))
cmd('soft_reset')
time.sleep(1)
print('reset:', cmd('get_state')[:100])
s.close()
"
```

---

## 九、故障排查

| 现象 | 原因 | 解决 |
|------|------|------|
| `No module named 'mujoco'` | 未安装 | `pip install mujoco` |
| Viewer 闪退 | 无 GPU | `--no-viewer` 无头模式 |
| `OpenGL error 0x502` | 软件渲染 | 可忽略, 帧仍正常 |
| 渲染帧全黑 | 无 GL 上下文 | `xvfb-run python scripts/simulation/mujoco_sim.py --no-viewer` |
| SharedMemory 不存在 | sim 未启动 | 先启动 mujoco_sim.py |
| `No module named 'pygame'` | 未安装 | `pip install pygame` (需桌面) |
| J2 关节超调 | PD 偏高 | 编辑 scene.xml 降 kp |
| SharedMemory 泄漏 warning | 进程被 kill | 正常, 下次启动自动覆盖 |

---

## 十、实物切换

从仿真切换到真机只需改 `--port`:

```bash
# 仿真
python scripts/control/joystick_control.py --port socket://localhost:5555

# 实物 (STM32 接 COM5)
python scripts/control/joystick_control.py --port COM5
```

仿真数据用途:
- **Sim-to-Real**: 仿真预训练 + 实物微调
- **模型验证**: 先在仿真中验证 SmolVLA 能否复现轨迹
- **算法调试**: 无硬件风险下快速迭代

---

## 十一、参考

- [系统架构](ARCHITECTURE.md) — 四层架构与数据流
- [串口命令参考](SERIAL_COMMANDS.md) — 14 条命令完整规范
- [部署教程](DEPLOYMENT.md) — 从 0 到训练
- [工作流](WORKFLOW.md) — 端到端管线
- [仿真方案设计](SIMULATION_PLAN.md) — 技术决策与路线图
