# Arm-robot_VLA 项目迁移指南

> 从一台 Ubuntu 电脑无缝迁移到另一台 Ubuntu 电脑的完整步骤。

---

## 一、源环境概况

| 项目 | 详情 |
|------|------|
| **OS** | Ubuntu 26.04 LTS (Resolute Raccoon) |
| **内核** | Linux 7.0.0-28-generic x86_64 |
| **Python** | 3.10 (conda 环境 `smolvla`) |
| **虚拟环境** | conda `smolvla` (lerobot 0.4.4, torch 2.10.0+cu128) |
| **MuJoCo** | 3.10.0 (Apache 2.0, 无需 license key) |
| **手柄** | Nintendo Switch Pro Controller (USB, 057e:2009) |
| **相机** | USB 摄像头 (/dev/video0) |
| **串口** | /dev/ttyUSB0 或 /dev/ttyACM0 (STM32) |

---

## 二、目标机器需要安装的系统依赖

### 2.1 必须安装 (MuJoCo + OpenGL 渲染)

```bash
sudo apt update
sudo apt install -y \
    libgl1 libglx0 libegl1 \
    libx11-6 libxrandr2 libxinerama1 libxi6 libxcursor1 \
    libglvnd0 mesa-libgallium
```

> Python 由 conda 提供（环境 `smolvla`），无需系统 python-venv。若目标机器尚未装 Miniconda，先按 https://docs.conda.io/en/latest/miniconda.html 安装。

### 2.2 无头模式额外依赖 (服务器 / 无显示器)

如果目标机器没有显示器（headless），还需要：

```bash
sudo apt install -y xvfb
```

启动时使用：
```bash
xvfb-run python scripts/simulation/mujoco_sim.py --no-viewer
```

### 2.3 可选 (GPU 加速渲染)

如果目标机器有 NVIDIA GPU：

```bash
sudo apt install -y nvidia-driver-550
```

---

## 三、迁移步骤

### Step 1: 复制项目文件夹

```bash
# 在目标机器上
scp -r user@source:/home/bright/office/Arm-robot_VLA ~/office/Arm-robot_VLA
# 或使用 U 盘 / 移动硬盘复制
```

### Step 2: 创建 conda 环境

> **用 conda 建环境，不建项目内 `.venv`**。本机旧 `.venv` 已退役并备份至 `backups/venv-20260805/`（勿删勿改）。

```bash
# 若目标机器没有 Miniconda，先安装并初始化
#   https://docs.conda.io/en/latest/miniconda.html
source ~/miniconda3/etc/profile.d/conda.sh

cd ~/office/Arm-robot_VLA

# 创建并激活环境 (Python 3.10)
conda create -n smolvla python=3.10 -y
conda activate smolvla
```

> 依赖安装与验证见 `scripts/setup_dev.sh`，或参考 README「快速开始」章节。

### Step 3: 安装 Python 依赖

```bash
# 确保在 smolvla 环境激活状态
conda activate smolvla
pip install --upgrade pip

# 核心依赖
pip install mujoco==3.10.0         # 物理仿真引擎
pip install opencv-python          # 图像采集与处理
pip install pygame                 # 手柄输入 (SDL2)
pip install numpy                  # 数值计算
pip install glfw                   # OpenGL 窗口
pip install pillow                 # 图像处理
pip install PyOpenGL               # OpenGL 绑定
pip install pyserial               # 串口通信 (STM32)

# LeRobot 插件 (见 scripts/setup_dev.sh)
pip install -e lerobot_robot_massage
```

> 或者从当前机器导出精确版本：
> ```bash
> # 在源机器上
> pip freeze --local > /tmp/requirements_freeze.txt
> # 拷贝到目标机器后
> pip install -r requirements_freeze.txt
> ```

### Step 4: 验证安装

```bash
python -c "import mujoco; print('MuJoCo', mujoco.__version__)"
# 预期: MuJoCo 3.10.0

python -c "import cv2; print('OpenCV', cv2.__version__)"
# 预期: OpenCV 5.0.0+

python -c "import pygame; print('pygame', pygame.version.ver)"
# 预期: pygame 2.6.1

python -c "import glfw; print('GLFW OK')"
# 预期: GLFW OK
```

### Step 5: 连接硬件外设

```bash
# 1. 插入 USB 手柄 (Nintendo Switch Pro Controller 或类似)
ls /dev/input/js*          # 应该看到 /dev/input/js0

# 2. 插入 USB 摄像头
ls /dev/video*             # 应该看到 /dev/video0

# 3. (如果有真机) 连接 STM32
ls /dev/ttyUSB*            # 应该看到 /dev/ttyUSB0
ls /dev/ttyACM*            # 或 /dev/ttyACM0
```

> **手柄兼容性**: pygame 支持大多数 USB 手柄 (Xbox、PS4/PS5、Switch Pro、第三方手柄)。不同手柄的按键编号可能不同，首次使用时建议运行：
> ```bash
> python scripts/test_joystick_map.py
> ```

### Step 6: 首次启动测试

```bash
# 先用 MuJoCo viewer 测试 (需要显示器)
MUJOCO_GL=glfw python scripts/simulation/mujoco_sim.py --viewer

# 如果弹出了 3D 窗口 → 渲染正常
# 按 Ctrl+C 退出
```

如果 Viewer 闪退或报 OpenGL 错误，使用 CPU 渲染：
```bash
export MUJOCO_GL=glfw
export LIBGL_ALWAYS_SOFTWARE=1
python scripts/simulation/mujoco_sim.py --viewer
```

---

## 四、完整启动 (一键脚本)

```bash
cd ~/office/Arm-robot_VLA
bash scripts/startup.sh
```

该脚本会自动：
1. 清理旧进程和共享内存
2. 启动 MuJoCo 仿真 (后台)
3. 启动手柄控制 (后台)
4. 启动控制台 Hub (前台)
5. 退出时自动清理所有后台进程

---

## 五、环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `MUJOCO_GL` | `glfw` | MuJoCo 渲染后端 (glfw / egl / osmesa) |
| `LIBGL_ALWAYS_SOFTWARE` | 不设 | 设为 `1` 强制软件渲染 (无 GPU 时) |

---

## 六、文件结构 (仅需迁移的部分)

```
Arm-robot_VLA/
├── CLAUDE.md                     # 项目总纲
├── README.md
├── .gitignore
├── .claude/                      # Claude Code 配置 (可选)
├── scripts/                      # ★ 核心 — 所有运行脚本
│   ├── startup.sh                # 一键启动
│   ├── mujoco_sim.py             # MuJoCo 仿真服务
│   ├── joystick_control.py       # 手柄遥控
│   ├── control_hub.py            # 控制台 Hub
│   ├── record_sim.py             # 数据录制
│   ├── camera_server.py          # 相机服务
│   ├── mujoco_scene/             # MJCF 场景 + STL 网格
│   │   ├── scene.xml
│   │   └── meshes/
│   └── ...
├── lerobot_robot_massage/        # LeRobot 适配器
│   ├── pyproject.toml
│   └── ...
├── firmware/                     # STM32 固件 (可选项)
├── configs/                      # 训练/机器人配置
├── docs/                         # 文档
├── backups/                      # 备份版本
├── datasets/                     # 录制数据 (按需迁移)
└── outputs/                      # 训练输出 (按需迁移)
```

> datasets/ 和 outputs/ 按需迁移——它们可能很大，且包含仿真录制数据和模型权重。

---

## 七、常见问题

### Q: 目标机器 Python 版本不一致怎么办？

项目声明 `requires-python = ">=3.10"`，用 conda 创建对应版本的环境即可（conda 自带 python，无需系统安装）：

```bash
# 例如创建 Python 3.10 环境
conda create -n smolvla python=3.10 -y
conda activate smolvla
```

### Q: MuJoCo Viewer 窗口是黑色的？

```bash
# 尝试软件渲染
export LIBGL_ALWAYS_SOFTWARE=1
MUJOCO_GL=glfw python scripts/simulation/mujoco_sim.py --viewer

# 或者直接无头模式
python scripts/simulation/mujoco_sim.py --no-viewer
```

### Q: pygame 找不到手柄？

```bash
# 检查手柄是否被系统识别
ls /dev/input/js*
sudo apt install joystick
jstest /dev/input/js0

# 确认用户在 input 组
sudo usermod -a -G input $USER
# 重新登录后生效
```

### Q: 仿真 vs 真机如何切换？

只需修改 `--port` 参数：

```bash
# 仿真
python scripts/control/joystick_control.py --port socket://localhost:5555

# 真机 (STM32 串口)
python scripts/control/joystick_control.py --port /dev/ttyUSB0
```

### Q: MuJoCo 需要 license 吗？

**不需要。** MuJoCo 3.x 使用 Apache 2.0 开源协议，免费用于商用和研究。

---

## 八、迁移检查清单

- [ ] 系统依赖已安装 (libgl, libx11 等)
- [ ] conda 环境 `smolvla` 已创建并激活
- [ ] `pip install` 所有依赖成功
- [ ] `import mujoco` 通过
- [ ] `import cv2` 通过
- [ ] `import pygame` 通过
- [ ] `pip install -e lerobot_robot_massage/` 通过
- [ ] MuJoCo Viewer 正常弹出 (有显示器时)
- [ ] USB 手柄已连接并被 `/dev/input/js*` 识别
- [ ] `/dev/video0` 相机可用
- [ ] `bash scripts/startup.sh` 一键启动正常

---

## 九、参考

- [MuJoCo 仿真使用说明](MUJOCO_SIM.md) — 详细操作文档
- [系统架构](ARCHITECTURE.md) — 四层架构
- [部署教程](DEPLOYMENT.md) — 从 0 到训练
