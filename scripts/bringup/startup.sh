#!/bin/bash
# Zero Arm VLA 一键启动: MuJoCo 仿真 → 手柄控制 → 控制台 Hub

echo "=== 清理旧进程和共享内存 ==="
kill $(lsof -ti:5555) 2>/dev/null && echo "  端口 5555 已释放" || echo "  端口 5555 空闲"
pkill -f camera_server.py 2>/dev/null && echo "  相机子进程已清理" || true
rm -f /dev/shm/mujoco_frame_0 /dev/shm/mujoco_frame_ee 2>/dev/null
echo "  共享内存已清理"

echo ""
echo "=== 启动 MuJoCo 仿真 (后台) ==="
cd /home/bright/office/Arm-robot_VLA
MUJOCO_GL=glfw /home/bright/win_office/conda/envs/smolvla/bin/python scripts/mujoco_sim.py --ik --trail 500 --viewer &
SIM_PID=$!

# 等 mujoco + camera_server 初始化
echo "  等待 MuJoCo 初始化 (4s)..."
sleep 4
if ! kill -0 $SIM_PID 2>/dev/null; then
    echo "  错误: MuJoCo 仿真启动失败, 请手动运行查看原因:"
    echo "  /home/bright/win_office/conda/envs/smolvla/bin/python scripts/mujoco_sim.py --ik --viewer"
    exit 1
fi
echo "  MuJoCo 运行中 (PID $SIM_PID)"

echo ""
echo "=== 启动手柄控制 (后台) ==="
/home/bright/win_office/conda/envs/smolvla/bin/python scripts/joystick_control.py --port socket://localhost:5555 --camera 0 &
JOY_PID=$!
echo "  手柄控制运行中 (PID $JOY_PID)"

echo ""
echo "=== 启动控制台 Hub ==="
echo "  操作: 鼠标点击按钮 | R=开始/停止录制 | Q=退出"
echo ""
/home/bright/win_office/conda/envs/smolvla/bin/python scripts/control_hub.py --port 5555 --fps 25

# Hub 退出后清理
echo ""
echo "=== Hub 已退出, 清理后台进程 ==="
kill $JOY_PID 2>/dev/null
kill $SIM_PID 2>/dev/null
pkill -f camera_server.py 2>/dev/null
echo "  全部进程已终止"
