# MuJoCo 手柄遥操作仿真方案

> 状态：已评审通过（2026-07-16），待实施
> 预计工期：~半天（依赖现成 SolidWorks 导出模型）

---

## 1. 背景与目标

目前机械臂硬件未组装完成，需要一套仿真系统来实现：

1. **手柄遥操作**：在 MuJoCo 物理仿真中用手柄操控虚拟机械臂
2. **数据录制**：同步记录关节轨迹 + 渲染相机帧，生成 LeRobot 兼容训练集
3. **实物切换**：硬件就位后，仅需改 `--port` 参数即可从仿真切换到真机

## 2. 已有基础（复用资产）

### 2.1 软件链路（已跑通，不动）

```
joystick_control.py ──socket:5555──→ SerialProtocol ──→ STM32 文本协议
MassageRobot         ──socket:5555──→ SerialProtocol ──→ LeRobot 数据集
```

| 文件 | 状态 | 说明 |
|------|:----:|------|
| [serial_protocol.py](../lerobot_robot_massage/serial_protocol.py) | ✅ 不动 | 已支持 `socket://` URL, 线程安全 (RLock) |
| [joystick_control.py](../scripts/control/joystick_control.py) | ✅ 不动 | 手柄遥控, `--port socket://localhost:5555` 直连仿真 |
| [massage_robot.py](../lerobot_robot_massage/massage_robot.py) | ✅ 不动 | LeRobot Robot 子类, `get_observation()` + `send_action()` |
| [fake_stm32.py](../scripts/fake_stm32.py) | ✅ 保留 | 简单恒速模型, 快速验证用 |

### 2.2 MuJoCo 机械臂模型（已存在！）

**来源**：`d:\robo arm\software\zero-robotic-arm\5. Deep_LR\`

| 文件 | 内容 |
|------|------|
| `robot_arm_mujoco.xml` | SolidWorks 导出的完整 MJCF 模型（关节、质量、惯量） |
| `meshes/*.stl` | 各关节 STL 网格文件 |

**现有模型参数**：

| 属性 | 值 |
|------|-----|
| 总质量 | ~1.0 kg |
| 关节数 | 6 (revolute) + 末端 |
| Link 长度 | 底座 166mm, 上臂 200mm, 前臂 47.63/184.5mm, 末端 125mm |
| 现有 actuator | torque 模式, `forcerange="-20 20"` (Nm) |

**待修改**（见 §4.1）：
- 关节限位：当前为近似值，需更新为固件精确值
- actuator 类型：torque → position（更适合手柄控制）
- 新增虚拟相机用于渲染数据采集

## 3. 固件物理参数（权威来源）

来源：[robot.c](../firmware/src/robot.c) 和 [robot.h](../firmware/src/robot.h)

### 3.1 DH 参数（Modified DH Convention）

```
const float D_H[6][4] = {{a, alpha, d, theta_offset}}:

Joint 1 (shoulder_pan):  a=0,      alpha=0,       d=0,       theta=+pi/2
Joint 2 (shoulder_lift): a=0,      alpha=pi/2,    d=0,       theta=+pi/2
Joint 3 (elbow_flex):    a=200.0,  alpha=pi,      d=0,       theta=-pi/2
Joint 4 (wrist_roll):    a=47.63,  alpha=-pi/2,   d=-184.5,  theta=0
Joint 5 (wrist_flex):    a=0,      alpha=pi/2,    d=0,       theta=+pi/2
Joint 6 (gripper):       a=0,      alpha=pi/2,    d=0,       theta=0
```

### 3.2 关节角度限位

```c
// robot.c:44-51 — g_joints_init
// {初始角, 电机方向, 减速比, 限位GPIO, 限位Pin, 最小角, 最大角, 复位方向}
{90,   CCW, 50.00, ...  0,   360},  // J1: shoulder_pan
{90,   CW,  50.89, ... 90,   180},  // J2: shoulder_lift
{-90,  CW,  50.89, ... -90,   90},  // J3: elbow_flex
{0,    CW,  51.00, ... -90,   90},  // J4: wrist_roll
{90,   CCW, 26.85, ...  0,    90},  // J5: wrist_flex
{0,    CW,  51.00, ...  0,   360},  // J6: gripper
```

### 3.3 复位姿态（soft_reset）

| J1 | J2 | J3 | J4 | J5 | J6 |
|:--:|:--:|:--:|:--:|:--:|:--:|
| 90° | 90° | -90° | 0° | 90° | 0° |

### 3.4 电机参数

| 项 | 值 |
|----|-----|
| 驱动器 | Emm_V5.0 闭环步进 (CAN 1Mbps) |
| 编码器 | 16-bit (65536 cnt/rev) |
| 微步 | 16x |
| J1-J4,J6 电机 | NEMA 17 (42x42mm, L40mm), ~0.45 Nm |
| J5 电机 | NEMA 14 (35x35mm, L28mm), ~0.18 Nm |
| J1-J4,J6 减速比 | ~50:1 → 关节扭矩 ~22 Nm |
| J5 减速比 | 26.85:1 → 关节扭矩 ~5 Nm |
| 关节默认速度 | 10 RPM |
| 遥操作最大速度 | 20 mm/s (末端), 5 RPM (关节) |

### 3.5 末端复位位置

```c
// T_0_6_reset → 末端坐标
x=0, y=-47.63, z=15.5 (mm, 相对底座)
```

## 4. 架构设计

### 4.1 数据流

```
┌────────────────────────── 零改动 ──────────────────────────┐
│                                                             │
│  joystick_control.py  ──socket:5555──→  mujoco_sim.py      │
│  MassageRobot          ──socket:5555──→  (MuJoCo 后端)      │
│                                                             │
└─────────────────────────────────────────────────────────────┘
                                              │
                                         SharedMemory
                                       "mujoco_frame_0"
                                              │
                                         record_sim.py
                                              │
                                         datasets/sim_v1/
```

### 4.2 mujoco_sim.py 内部架构

```
TCP socket (5555)
    │ accept / recv
    v
handle_line(line: str) ─── 14 条命令解析 (同 fake_stm32.py)
    │
    │  get_state → 读 mjData.qpos[0:6]
    │  set_joints → 写 actuator ctrl[0:6]
    │  remote_event → ctrl[i] += velocity[i] * dt (夹限位)
    │  e_stop → 强制 qvel = 0
    │
    v
mj_step(model, data)      ← 物理步进 (每收到命令后)
    │
    ├─ mjr_render          → MuJoCo viewer 窗口 (实时可视化)
    └─ mjr_render (离屏)   → SharedMemory (帧缓冲, 供录制)
```

### 4.3 相机管线

| 用途 | 方式 | 说明 |
|------|------|------|
| **仿真可视化** | MuJoCo viewer 窗口 (`mj_passive_viewer`) | 实时交互, 手柄操作时观察手臂姿态 |
| **数据录制** | 离屏渲染 + `SharedMemory` | 640x480 BGR, 30fps, 零拷贝, ~27MB/s |

**为什么用共享内存**：
- 640x480x3 BGR = 921KB/帧
- 30fps = 27.6 MB/s
- TCP 需序列化/反序列化, 共享内存零拷贝延迟 <1ms
- Python `multiprocessing.shared_memory` 原生支持 (3.8+), 无需额外依赖

## 5. 实现步骤

### Step 1: 安装 MuJoCo

```powershell
pip install mujoco
```

MuJoCo 3.x 通过 pip 原生安装，无需额外 SDK 或编译。验证：

```powershell
python -c "import mujoco; print(mujoco.__version__)"
```

### Step 2: 复制并改编机械臂模型

```powershell
mkdir scripts\mujoco_scene\meshes
copy "d:\robo arm\software\zero-robotic-arm\5. Deep_LR\robot_arm_mujoco.xml" scripts\mujoco_scene\scene.xml
xcopy "d:\robo arm\software\zero-robotic-arm\5. Deep_LR\meshes\*" scripts\mujoco_scene\meshes\
```

**scene.xml 改编清单**：

1. **更新关节限位**：将每个 `<joint>` 的 `range` 改为 §3.2 的固件值
   - J1: `range="0 6.2832"` (0-360°, 连续旋转)
   - J2: `range="1.5708 3.1416"` (90-180°)
   - J3: `range="-1.5708 1.5708"` (-90-90°)
   - J4: `range="-1.5708 1.5708"` (-90-90°)
   - J5: `range="0 1.5708"` (0-90°)
   - J6: `range="0 6.2832"` (0-360°, 连续旋转)

2. **切换 actuator 类型**：`motor` → `position`
   ```xml
   <actuator>
     <position name="j1" joint="joint1" ctrlrange="0 6.2832"
               kp="100" kv="20" forcerange="-20 20"/>
     <!-- 其余 5 个关节同理 -->
   </actuator>
   ```
   - `kp=100, kv=20`：PD 增益, 提供平滑的伺服行为
   - `forcerange="-20 20"`：Max torque 饱和 (Nm), 保留现有值

3. **添加虚拟相机**（在 `<worldbody>` 内、机械臂上方）：
   ```xml
   <camera name="cam_top" pos="0.2 0 0.5" xyaxes="-1 0 0 0 -1 0"
           fovy="60" resolution="640 480"/>
   ```

4. **初始 qpos**：设 `keyframe` 或 `<joint>` 默认值为 soft_reset 姿态：
   ```
   qpos = [90°, 90°, -90°, 0°, 90°, 0°] = [1.5708, 1.5708, -1.5708, 0, 1.5708, 0] (rad)
   ```

5. **添加地面**（如果原模型没有）：
   ```xml
   <geom type="plane" size="1 1 0.1" rgba="0.3 0.3 0.4 1"/>
   <light directional="true" diffuse="0.8 0.8 0.8" pos="0 0 2"/>
   ```

### Step 3: 创建 `scripts/simulation/mujoco_sim.py`

**核心思路**：完全复用 `fake_stm32.py` 的命令解析和 TCP 框架，只替换物理引擎。

**关键类 `MuJoCoArm`**（替换原有的 `FakeArm`）：

```python
class MuJoCoArm:
    def __init__(self, scene_path: str):
        self.model = mujoco.MjModel.from_xml_path(scene_path)
        self.data = mujoco.MjData(self.model)
        # 设置初始 qpos 为 soft_reset 姿态
        self.data.qpos[:] = [1.5708, 1.5708, -1.5708, 0, 1.5708, 0]
        mujoco.mj_forward(self.model, self.data)  # 初始化运动学
        # 共享内存帧缓冲
        self._shm = SharedMemory(create=True, size=..., name="mujoco_frame_0")

    def step(self):
        """步进物理 + 渲染一帧到 SharedMemory"""
        mujoco.mj_step(self.model, self.data)
        # 离屏渲染到共享内存
        ...
```

**命令解析**（`handle_line` 方法, 14 条命令, 与 fake_stm32 完全相同）：

| 命令 | MuJoCo 实现 |
|------|------------|
| `get_state` | `qpos[0:6]` → 度数; `qvel[0:6]` → 度/s; `qfrc_actuator[0:6]` → 负载 |
| `set_joints` | `ctrl[0:6] = targets` (position actuator 自动趋近) |
| `set_torque 0` | `ctrl = qpos` (锁当前位置, 模拟自由模式) |
| `set_torque 1` | 恢复 `ctrl` 跟随度逻辑 |
| `e_stop` | `ctrl = qpos`; `qvel[0:6] = 0` (瞬间停止) |
| `remote_event` | `ctrl[i] += vel[i] * dt`, clamp 到 `ctrlrange` |
| `rel_rotate` | `ctrl[joint] += delta` (1-based joint_id -1) |
| `soft_reset` | `ctrl = [90, 90, -90, 0, 90, 0]` (度) |
| 其余 | 与 [fake_stm32.py](../scripts/fake_stm32.py) 一致 |

**主循环架构**：
```python
def main():
    arm = MuJoCoArm("scripts/mujoco_scene/scene.xml")
    srv = socket.socket(...); srv.bind(("127.0.0.1", 5555)); srv.listen(1)

    with mujoco.viewer.launch_passive(arm.model, arm.data) as viewer:
        while True:
            conn, addr = srv.accept()
            viewer.sync()  # 更新 viewer
            while True:
                line = read_line(conn)
                if not line: break
                responses = arm.handle_line(line)
                arm.step()  # mj_step + 渲染 → SharedMemory
                viewer.sync()
                send_responses(conn, responses)
```

### Step 4: 创建 `scripts/record_sim.py`

自定义录制脚本，不依赖 `lerobot-record`（它需要 Teleoperator 插件）。

```python
"""仿真数据采集 — 同时记录 MuJoCo 关节角 + 渲染帧"""

from multiprocessing import shared_memory
from lerobot_robot_massage import MassageRobot, MassageRobotConfig
import numpy as np, cv2, time, json
from pathlib import Path

def record_episode(port, duration_s=20, fps=30, output="datasets/sim_v1"):
    # 1. 连接 MassageRobot → 关节状态
    config = MassageRobotConfig(port=port, sim_mode=True)
    robot = MassageRobot(config); robot.connect()

    # 2. 打开共享内存 → MuJoCo 渲染帧
    shm = shared_memory.SharedMemory(name="mujoco_frame_0")
    frame_buf = np.ndarray((480, 640, 3), dtype=np.uint8, buffer=shm.buf[64:])

    # 3. 录制循环
    episode_dir = Path(output) / f"episode_{...}"
    for i in range(duration_s * fps):
        obs = robot.get_observation()         # 关节角
        frame = frame_buf.copy()              # 从 SharedMemory 取帧
        cv2.imwrite(episode_dir / f"frame_{i:06d}.png", frame)
        # 写 JSON/parquet
        ...
```

**录制流程**（3 个终端）：
```powershell
# 终端 1: 启动 MuJoCo 仿真
python scripts/simulation/mujoco_sim.py

# 终端 2: 手柄遥控（此时 MuJoCo viewer 中手臂可见）
python scripts/control/joystick_control.py --port socket://localhost:5555 --camera 0

# 终端 3: 数据录制
python scripts/record_sim.py --duration 20 --output datasets/sim_v1
```

## 6. 验证清单

| # | 验证项 | 命令/方法 | 预期结果 |
|:--:|--------|-----------|----------|
| 1 | MuJoCo 安装 | `python -c "import mujoco"` | 3.x 版本 |
| 2 | viewer 初始姿态 | 启动 `mujoco_sim.py` | 手臂 = soft_reset [90,90,-90,0,90,0] |
| 3 | 协议握手 | `SerialProtocol("socket://localhost:5555").get_state()` | 正常返回 6 角度 |
| 4 | 手柄遥操 | 运行 joystick_control → 推摇杆 | viewer 中手臂实时运动 |
| 5 | set_joints | `set_joints [100,120,-45,0,45,30]` | 手臂有惯性效果地平滑动到位 |
| 6 | e_stop | `e_stop` | 手臂瞬间停止, viewer 中 qvel=0 |
| 7 | 渲染帧 | 读取 SharedMemory | `(480,640,3)` uint8, 非全黑 |
| 8 | 数据录制 | record_sim 录制 10s | episode 目录, 帧对齐, Joint 数据可读 |

## 7. 技术决策记录

| 决策 | 选项 | 选择 | 原因 |
|------|------|:--:|------|
| 模型格式 | URDF vs MJCF | **MJCF** | MuJoCo 原生, 已有 SolidWorks 导出 |
| Actuator 类型 | torque vs position | **position** | 手柄输出速度指令, 适合位置目标增量 |
| 帧传输 | TCP vs SharedMemory | **SharedMemory** | 零拷贝, <1ms 延迟, 无序列化开销 |
| 协议层 | 改写 vs 复用 | **复用** | 仿真/实物切换只改 `--port` |
| 录制方式 | lerobot-record vs 自定义脚本 | **自定义脚本** | LeRobot 需要 Teleoperator 插件(我们没有), 自定义更可控 |

## 8. 后续路线图

- **Phase 2**: 多回合采集 50+ 条轨迹, 训练 SmolVLA (仿真 → 仿真推理验证)
- **Phase 3**: 硬件到货后, 切换 `--port COM5` 做实物采集 (仿真数据可做预训练 + 实物数据微调 = Sim-to-Real)
- **Phase 4**: 替换 MJCF 中简化几何体为精确 STL → 提升碰撞检测/渲染精度
- **Phase 5**: 接入 `lerobot-record` 正式管线 (需要实现 Teleoperator 插件)
