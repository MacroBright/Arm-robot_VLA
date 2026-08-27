# MuJoCo 物理仿真与数字孪生使用指南 (MUJOCO_SIM)

> **物理引擎**：MuJoCo 3.10+
> **仿真程序**：`scripts/simulation/mujoco_sim.py`
> **场景与模型**：`scripts/simulation/scene.xml` 与 `scripts/simulation/meshes/`

---

## 目录

- [1. 仿真系统架构与设计](#1-仿真系统架构与设计)
- [2. 环境准备与依赖安装](#2-环境准备与依赖安装)
- [3. 快速启动与模式选择](#3-快速启动与模式选择)
- [4. 手柄遥操作仿真 (Joystick Teleop)](#4-手柄遥操作仿真-joystick-teleop)
- [5. 多视角共享内存相机渲染 (Camera Server)](#5-多视角共享内存相机渲染-camera-server)
- [6. 仿真闭环示教数据录制 (Record Sim)](#6-仿真闭环示教数据录制-record-sim)
- [7. 自动化仿真回归测试](#7-自动化仿真回归测试)

---

## 1. 仿真系统架构与设计

MuJoCo 仿真环境完整模拟了 6-DOF 机械臂的物理动力学、关节限位、阻尼以及视觉渲染，并对外提供与真机完全兼容的通信接口（支持 TCP Socket 模式），实现算法在仿真与真机之间的**无缝零代码切换**。

```text
┌────────────────────────────────────────────────────────┐
│  控制器 (joystick_control.py / LeRobot / 键盘控制)      │
└──────────────────────────┬─────────────────────────────┘
                           │ TCP Socket (socket://127.0.0.1:5555)
┌──────────────────────────▼─────────────────────────────┐
│  mujoco_sim.py (MuJoCo 物理引擎主循环 @ 50Hz)           │
│  • 14 条控制协议指令解析                                │
│  • 阻尼最小二乘 (DLS) 数值逆解积分                      │
│  • 碰撞检测、动力学重力补偿与物理约束                   │
├──────────────────────────┬─────────────────────────────┤
│ 离屏渲染 (Offscreen)     │ 实时物理可视化 (Viewer)      │
│ → 共享内存 SharedMemory │ → 交互式 3D 渲染窗口        │
└──────────────────────────┴─────────────────────────────┘
```

---

## 2. 环境准备与依赖安装

```bash
conda activate smolvla
pip install mujoco opencv-python pygame pyserial
```

验证安装：
```bash
python -c "import mujoco; print('MuJoCo 版本:', mujoco.__version__)"
```

---

## 3. 快速启动与模式选择

### 3.1 启动带 GUI 交互窗口的仿真
```bash
python scripts/simulation/mujoco_sim.py
```
- 默认监听 TCP 端口 `5555`；
- 可在 3D 窗口中自由旋转视点、查看关节坐标系与末端轨迹。

### 3.2 启动无头模式 (Headless / 服务器无显卡)
```bash
xvfb-run python scripts/simulation/mujoco_sim.py --no-viewer
```

---

## 4. 手柄遥操作仿真 (Joystick Teleop)

在另一个终端启动手柄遥控程序，连接至仿真 TCP 端口：

```bash
python scripts/control/joystick_control.py --port socket://localhost:5555
```
- 手柄左摇杆控制虚拟机械臂 XY 平移，LT/RT 控制升降，右摇杆控制姿态自旋；
- 行为与真机完全一致。

---

## 5. 多视角共享内存相机渲染 (Camera Server)

仿真内建了 3 路虚拟相机（前视、俯视、侧视），通过 POSIX 共享内存以零拷贝方式向算法推流：

```bash
# 启动相机共享内存服务
python scripts/simulation/camera_server.py
```
- 数据格式：`RGB 640x480 @ 30fps`；
- 供 LeRobot 数据集录制器与 SmolVLA 策略在线推理直接读取。

---

## 6. 仿真闭环示教数据录制 (Record Sim)

在无实物硬件时，直接在仿真中录制标准的 LeRobot 训练轨迹：

```bash
python scripts/simulation/record_sim.py \
    --output-dir datasets/sim_massage_dataset \
    --num-episodes 20
```

---

## 7. 自动化仿真回归测试

运行全栈仿真回归测试套件（闭环验证 kinematics $\to$ adapter $\to$ watchdog $\to$ fake server）：

```bash
pytest ../Co_Teleop/tests/test_sim_regression.py
```
