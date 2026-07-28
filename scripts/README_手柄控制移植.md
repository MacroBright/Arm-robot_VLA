# 手柄控制移植指南

## 需要复制的文件

```
lerobot_robot_massage/
├── __init__.py               # 包导入
└── serial_protocol.py        # TCP 通信协议

scripts/
├── joystick_control.py       # 手柄遥控主脚本
└── test_joystick_map.py      # 按键诊断工具（首次运行）
```

## 依赖

```bash
pip install pygame pyserial opencv-python
```

## 首次使用 — 诊断按键映射

不同手柄按钮编号不同，先跑诊断：

```bash
python scripts/test_joystick_map.py
```

逐个按 A/B/X/Y/LB/RB/L2/R2/BACK/START，记录显示的按钮编号。

## 修改映射

编辑 `joystick_control.py` 顶部常量，对齐你的手柄：

```python
BTN_A = 0       # 改成实际编号
BTN_B = 1
BTN_Y = 2
BTN_X = 3
BTN_LB = 5
BTN_RB = 6
BTN_LT = 7      # L2 扳机
BTN_RT = 8      # R2 扳机
BTN_BACK = 9
BTN_START = 10

AXIS_LX = 0     # 左摇杆 X
AXIS_LY = 1     # 左摇杆 Y
AXIS_RX = 2     # 右摇杆 X
AXIS_RY = 3     # 右摇杆 Y
```

## 启动

```bash
python scripts/joystick_control.py --port socket://localhost:5555 --camera 0
```

仿真必须先启动，手柄脚本通过 TCP 连接仿真后端。

## 按键功能

| 按键 | 功能 |
|------|------|
| A | 进入遥控 (remote_enable) |
| B | 退出遥控 (remote_disable) |
| Y | 急停 (e_stop) |
| X | 扭矩切换 |
| LB/RB | 关节模式/切换关节 |
| 十字键 | 关节步进 |
| L2/R2 | 末端 Z 升降 |
| START | 归零 |
| BACK | 退出脚本 |
| 左摇杆 | 末端 XY 平移 |
| 右摇杆 | 末端旋转 |
