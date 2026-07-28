# 串口命令参考手册

> STM32F407VET6 | USART1 (PA9/PA10) | 115200-8-N-1 | 换行: LF (`\n`)

---

## 关节编号

| 关节 | 名称 | 说明 | 角度范围 | CAN ID |
|:----:|------|------|:--------:|:------:|
| **1** | shoulder_pan | 底座旋转 | 0° ~ 360° | **2** |
| **2** | shoulder_lift | 肩部抬升 | 90° ~ 180° | **3** |
| **3** | elbow_flex | 肘部 | -90° ~ 90° | **4** |
| **4** | wrist_flex | 腕部俯仰 | -90° ~ 90° | **5** |
| **5** | wrist_roll | 腕部旋转 | 0° ~ 90° | **6** |
| **6** | gripper | 末端/夹爪 | 0° ~ 360° | **7** |

> **注意**: 串口 `joint_id` 参数为 **1-based**：`1 = 关节1`, `2 = 关节2`, ..., `6 = 关节6`。
> 内部 CAN ID = `joint_id + 1`，即关节1→CAN 2, 关节2→CAN 3, ..., 关节6→CAN 7。
> `set_joints` 和 `get_state` 直接按顺序 1→6 排列，无需偏移。

---

## 命令总览 (当前固件中烧录的 14 条命令)

### LeRobot 数据采集命令 (新增)

| # | 命令 | 格式 | 响应 | 说明 |
|:--|------|------|------|------|
| 1 | `get_state` | `get_state` | `STATE:j1,...,j6,v1,...,v6,l1,...,l6` | 读取全部关节状态 |
| 2 | `set_joints` | `set_joints j1 j2 j3 j4 j5 j6` | `OK` | 设置全部关节目标角度(°) |
| 3 | `set_torque` | `set_torque 0` 或 `set_torque 1` | `OK` / `OK:FREE` | 电机扭矩禁用/使能 |
| 4 | `e_stop` | `e_stop` | `ESTOP` | 紧急停止 |

### 遥操作命令 (已有)

| # | 命令 | 格式 | 响应 | 说明 |
|:--|------|------|------|------|
| 5 | `remote_enable` | `remote_enable` | — | 使能远程控制 + 软复位 |
| 6 | `remote_disable` | `remote_disable` | — | 禁用远程控制 + 软复位 |
| 7 | `remote_event` | `remote_event vx vy vz rx ry rz` | — | 笛卡尔速度控制 (手柄) |

### 关节运动命令 (已有)

| # | 命令 | 格式 | 响应 | 说明 |
|:--|------|------|------|------|
| 8 | `rel_rotate` | `rel_rotate joint_id angle` | — | 关节相对旋转 (joint_id: 1=关节1, ..., 6=关节6) |
| 9 | `auto` | `auto x y z` | — | 逆运动学自动定位 (末端坐标 mm) |

### 系统命令 (已有)

| # | 命令 | 格式 | 响应 | 说明 |
|:--|------|------|------|------|
| 10 | `zero` | `zero` | — | 当前位置设为机械零位 |
| 11 | `hard_reset` | `hard_reset` | — | 限位开关归零复位 |
| 12 | `soft_reset` | `soft_reset` | — | 软件复位到预设初始角度 |
| 13 | `stream_start` | `stream_start` | — | 启动 USB 数据流 |
| 14 | `stream_stop` | `stream_stop` | — | 停止 USB 数据流 |

---

## 命令详解

### `get_state` — 读取关节状态

```
发送: get_state
响应: STATE:j1,j2,j3,j4,j5,j6,v1,v2,v3,v4,v5,v6,l1,l2,l3,l4,l5,l6

j1~j6 : 关节1~6 角度 (°)
v1~v6 : 关节1~6 速度 (°/s)
l1~l6 : 关节1~6 相电流 (mA, 负载参考)
```

**示例:**
```
发送:  get_state
响应:  STATE:90.00,90.00,270.00,0.00,90.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00
        关节1 关节2 关节3 关节4 关节5 关节6 |---速度---|---负载---|
```

**30fps 可行性**: 单次读取 6 关节 × 2 CAN 寄存器 ≈ 10~20ms，满足 30fps (33ms 周期)。

---

### `set_joints` — 设置全部关节目标角度

```
发送: set_joints j1 j2 j3 j4 j5 j6
响应: OK

j1~j6 : 关节1(底座)~关节6(夹爪) 目标角度 (°)
```

**示例:**
```
发送:  set_joints 90 45 -20 0 90 0
响应:  OK
→ 关节1→90°, 关节2→45°, 关节3→-20°, 关节4→0°, 关节5→90°, 关节6→0°
```

---

### `set_torque` — 电机扭矩控制

```
发送:  set_torque 1      →  OK        (使能, 电机保持力矩)
发送:  set_torque 0      →  OK:FREE   (禁用, 电机自由转动)
```

**手动示教流程:**
```
set_torque 0          → 进入自由模式, 可手动拖拽机械臂
(手动演示按摩动作, 同时定时 get_state 记录位姿)
set_torque 1          → 恢复力矩
```

---

### `e_stop` — 紧急停止

```
发送:  e_stop
响应:  ESTOP
```

立即停止所有 6 个关节, 清除远程控制标志, 保持力矩。

---

### `remote_enable` / `remote_disable` — 远程控制开关

```
发送:  remote_enable     → 使能远程控制 + 自动软复位所有关节到初始角度
发送:  remote_disable    → 退出远程控制 + 自动软复位所有关节
```

**⚠️ 注意事项：**

| 条件 | 说明 |
|------|------|
| **需要电机 24V 电源打开** | `remote_enable` 会执行 `soft_reset`，通过 CAN 总线控制所有电机复位。如电机未上电 → CAN timeout 洪流 |
| **LeRobot 采集不需要它** | `get_state`、`set_torque`、`rel_rotate`、`e_stop` 均可独立使用，不依赖 `remote_enable` |
| **只有 `remote_event` 需要它** | 手柄笛卡尔速度控制必须先发 `remote_enable` 解锁，否则 `remote_event` 被固件静默忽略 |

**常见错误：**
```
发送: remote_enable
返回: CAN timeout:0 / CAN timeout:1 / ... (无限循环)
原因: 电机 24V 电源未打开 → 关掉电源不等于能复位
解决: 打开电机电源，或不要使用 remote_enable（LeRobot 采集不需要）
```

`remote_event` 仅在 `remote_enable` 之后生效。

---

### `remote_event` — 笛卡尔空间速度控制

```
发送:  remote_event vx vy vz rx ry rz

vx, vy, vz : 末端线速度比例 (-1.0 ~ +1.0), 实际 = 输入 × 20 mm/s
rx, ry, rz : 末端角速度比例 (-1.0 ~ +1.0), 实际 = 输入 × 5 rpm
```

---

### `rel_rotate` — 关节相对旋转

```
发送:  rel_rotate joint_id angle

joint_id : 关节编号 (0=关节1/底座, 1=关节2, ..., 5=关节6)
angle    : 相对旋转角度 (°)
```

**示例:**
```
发送:  rel_rotate 0 45       → 关节1(底座)相对旋转 +45°
发送:  rel_rotate 2 -15      → 关节3(肘部)相对旋转 -15°
```

---

### `auto` — 逆运动学定位

```
发送:  auto x y z
```

通过逆运动学计算关节角度, 自动移动末端到指定坐标 (mm)。

---

### `zero` — 当前位置归零

```
发送:  zero
```

将当前所有电机位置清零, 设为新的零位参考。

---

### `hard_reset` — 硬件复位

```
发送:  hard_reset
```

每个关节向限位开关方向运动, 触发限位后该关节归零。需连接限位开关。

---

### `soft_reset` — 软件复位

```
发送:  soft_reset
```

所有关节恢复到预设初始角度:

| 关节1 | 关节2 | 关节3 | 关节4 | 关节5 | 关节6 |
|:-----:|:-----:|:-----:|:-----:|:-----:|:-----:|
| 90° | 90° | -90° | 0° | 90° | 0° |

---

### `stream_start` / `stream_stop` — USB 数据流

```
发送:  stream_start    → 启动高频位姿数据流 (USB CDC)
发送:  stream_stop     → 停止数据流
```

---

## 典型工作流程

### A. 手动示教数据采集

```
remote_enable
set_torque 0                          # 自由拖拽
(手动静置演示按摩动作)
while 采集:
    get_state                         # 读取关节角度
    camera.read()                     # 读取相机帧
    → 组成 (图像, 关节角) 数据对
set_torque 1                          # 恢复力矩
```

### B. 模型推理控制

```
remote_enable
set_torque 1
get_state                             # 读取当前状态
set_joints j1 j2 j3 j4 j5 j6          # 模型输出 → 关节角度
(循环 get_state → 推理 → set_joints)
```

### C. 紧急停止恢复

```
e_stop                                # 急停
(排查问题)
remote_enable                         # 恢复
```

### D. 手柄遥操作

```
remote_enable
(循环读取手柄)
remote_event vx vy vz rx ry rz
remote_disable
```

---

## 通信参数

| 参数 | 值 |
|------|-----|
| 接口 | USART1 |
| 引脚 | PA9 (TX), PA10 (RX) |
| 波特率 | 115200 bps |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验位 | 无 |
| 换行符 | `\n` (LF, 0x0A) |

---

## 快速测试

- [ ] `get_state` → 返回 `STATE:` 开头 + 18 个逗号分隔数值
- [ ] `set_torque 0` → `OK:FREE`, 机械臂可手动拖拽
- [ ] `set_torque 1` → `OK`, 电机保持位置
- [ ] `e_stop` → `ESTOP`
- [ ] `remote_enable` → 无报错
- [ ] `soft_reset` → 机械臂回到初始姿态
