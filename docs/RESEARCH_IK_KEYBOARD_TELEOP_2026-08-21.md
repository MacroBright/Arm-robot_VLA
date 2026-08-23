# 调研报告 + 开发计划：键盘遥操 IK 笛卡尔控制

> **日期**：2026-08-21（第二版，连杆长度已与源项目一致，撤销第一版误差分析）
> **参考**：源项目 `/home/bright/win_office/ubantu_files/project/armbot_example/zero-robotic-arm-master`
> **前置**：[RESEARCH_ABS_ENCODER_AND_FW_COMPARE_2026-08-21.md](RESEARCH_ABS_ENCODER_AND_FW_COMPARE_2026-08-21.md)（pos→(k,b)→anchor 体系）

---

## 一、源项目 IK 实现（已完整读源）

### 1.1 运动学模块 `robot_kinematics.c`（498 行，解析解）

文件位置：`zero-robotic-arm-master/2. Software/robot/Core/Src/robot_kinematics.c`

**DH 参数**（`#define a2 D_H[2][0]`, `a3 D_H[3][0]`, `d4 D_H[3][2]`）：

| 行号 | a (mm) | α | d (mm) | θ offset |
|------|--------|---|--------|----------|
| J1 | 0 | π/2 | 0 | π/2 |
| J2 | 0 | π/2 | 0 | π/2 |
| J3 | **200** | π | 0 | -π/2 |
| J4 | **47.63** | -π/2 | **-184.5** | 0 |
| J5 | 0 | π/2 | 0 | π/2 |
| J6 | 0 | π/2 | 0 | 0 |

**求解链**（依赖顺序，`robot_kinematics_calc()` :272-290）：

```
calc_theta3()  → 由目标位置 px,py,pz 解 2 组 θ3  (atan², sqrt)
calc_theta2()  → 每组 θ3 解 2 组 θ2            (4 组)
calc_theta1()  → 每组 (θ2,θ3) 解 θ1            (4 组)
calc_theta5()  → 由 θ1,θ2,θ3 + 旋转矩阵解 θ5    (奇异: θ5≈0 或 π)
calc_theta4()  → 由 θ1,θ2,θ3,θ5 解 θ4
calc_theta6()  → 由前 5 角解 θ6
```

共 4 组候选解（`ROBOT_KINEMATICS_RESULT_NUM=4`，θ3 二值 × θ2 二值）。

**后处理**（`robot_kinematics_inverse()` :443-474）：
1. 弧度 → 0-360°（`radians_to_degrees_0_360`）
2. 关节限位映射（`joint_angle_map` :382-412）：角度 < min 时 +360、> max 时 −360 折叠，仍越界则标记该解无效
3. 最优解选择（`get_optimal_result` :345-378）：`Σ |解[i][j] − 当前角[j]| × joint_weight[j]` 最小者。`joint_weight = {5,3,3,1,1,1}`（J1/J2/J3 权重高）
4. 当前角由 `robot_kinematics_joint_angle_update()` 每路径点回写作种子（robot.c:394/406/448/461/717）

**入口** `robot_kinematics_inverse(float *T_target, float *result, int show)`：
- 输入 4×4 齐次变换矩阵 T（行优先 float[16]）
- 返回 0 成功 / -1 无有效解

**`robot_kinematics_cal_T`**（:477-491）：仅更新 T 的平移列 `T_out[i][3] = pos->x/y/z + T_out[i][3]` —— **当前固件 IK 只支持位置，不支持姿态旋转**（`struct rotate` 定义了但未接入）。姿态固定用复位矩阵 `T_0_6_reset`。

### 1.2 遥操链路（固件侧）

**入口** `robot_cmd.c:103-134 robot_remote_event_handle`：

```c
float vx = -param[0] * ROBOT_REMOTE_MAX_VELOCITY;  // ±20 mm/s
float vy =  param[1] * ROBOT_REMOTE_MAX_VELOCITY;
float vz = (param[4] - param[5]) / 2 * ROBOT_REMOTE_MAX_VELOCITY;  // L2/R2 差动
float rx = -param[3] * ROBOT_REMOTE_MAX_RPM;       // ±5 rpm
float ry =  param[2] * ROBOT_REMOTE_MAX_RPM;
```

**`robot_pid_remote`**（robot.c:688-741，每 `ROBOT_REMOTE_TIME_RESOLUTION=50ms`）：
1. 速度积分得目标位置：`g_pos = cur_pos + v×50/1000`
2. `cal_T(T_0_6_reset, T, &g_pos)` → `inverse()` 求 6 关节角
3. IK 失败：累错 ≥10 次 → 停全部关节（`robot_joint_stop`）
4. J1-J4：软件 PID 跟踪（KP=10, KI=0.002, KD=0）逐路径点；J5/J6：直接速度控制 rx/ry
5. 更新 `cur_pos` = 积分后位置（命令积分，非编码器回读）

**上位机 `robot_joystick.py`**（97 行，pygame 手柄）：
- `remote_enable\n` → 每 100ms 发 `remote_event <a0> <a1> <a2> <a3> <a4> <a5>\n` → Q 键退出发 `remote_disable\n`
- 纯串口文本协议，无 IK 计算（IK 全在固件）

### 1.3 URDF / MuJoCo 几何（三处一致）

| 参数 | 固件 D_H | URDF | 我们 scene.xml |
|------|---------|------|----------------|
| 底座高 | (隐含) | 0.166m | 0.166m |
| 上臂 J2→J3 | 200mm | link2 几何吻合 | 0.2m |
| 腕偏距 | 47.63mm | 吻合 | 0.047631m |
| 前臂 J3→腕心 | d4=-184.5mm | 吻合 | 0.1845m |
| 末端 J5→ee | — | ee_link | 0.125m |

**结论：连杆长度已与源项目一致（用户实测确认），IK 直接用源项目 DH 表即可，无需误差补偿。**

---

## 二、本项目落地方案：键盘遥操 IK 笛卡尔控制

### 2.1 与源项目遥操的差异点

| 维度 | 源项目（固件遥操） | 本项目（PC 直连 CAN） |
|------|-------------------|----------------------|
| IK 位置 | STM32 固件 | **PC 端 Python**（新写） |
| 末端状态 | 命令积分 cur_pos | 0x36 真实位置（经 k,b anchor） |
| 安全限位 | 固件 GPIO 限位开关 | 上位机 `check_limits_real` |
| 输入设备 | 手柄 | **键盘**（用户指定） |
| 通信 | 串口文本 | CAN 直接 |

### 2.2 架构

```
键盘 (pygame/终端)
   ↓ 按键状态 → twist 速度 vx/vy/vz mm/s + rx/ry
CartesianController (新)
   ↓ 每帧: 读 0x36 真实位置 → anchor 真实角 → FK 得当前末端
   ↓       速度积分 → 目标末端 → IK → 目标关节角
   ↓ 安全: clamp 到限位 + check_limits_real 每帧守卫
ZdtController.set_joints_safe → CAN 0xFD 相对运动
   ↓ 循环 ~20-30Hz
```

**关键决策**（2026-08-21 用户确认）：

| 决策 | 选择 | 理由 |
|------|------|------|
| IK 方法 | 移植源项目**解析解**（Python+numpy） | 确定性、实时、无迭代问题；源码完整可移植 |
| 末端反馈 | **0x36 真实位置经 (k,b) anchor**（已确认） | 漂移后可自愈，不同于固件命令积分 |
| 键盘形式 | **pygame 窗口**（已确认） | 可实时显示末端坐标/关节角/速度 |
| 循环频率 | **20Hz**（已确认） | 与源项目 50ms 同量级，重力关节下稳 |
| 姿态控制 | **仅位置 3DOF，姿态锁 T_0_6_reset**（已确认） | 与源项目 cal_T 能力对齐，YAGNI |
| 速度上限 | 20mm/s（同源项目） | 按摩安全范围 |
| 安全层 | `check_limits_real` 每帧 + watchdog | 复用现有 C-task |
| 坐标系 | 基座系 z 向上 | 与 DH 一致 |

### 2.3 待新建/修改文件

| 文件 | 类型 | 内容 |
|------|------|------|
| `lerobot_robot_massage/zdt/kinematics.py` | 新 | Modified DH FK + 解析 IK（移植源项目）+ 多解加权选择 + 限位折叠 |
| `lerobot_robot_massage/zdt/cartesian.py` | 新 | `CartesianController`: 速度积分→FK 反馈→IK→set_joints_safe 循环 |
| `lerobot_robot_massage/zdt/test_kinematics.py` | 新 | FK/IK 往返精度 + 边界用例 |
| `lerobot_robot_massage/zdt/test_cartesian.py` | 新 | 运动学闭环 + 限位守卫用例 |
| `scripts/control/cartesian_keyboard.py` | 新 | 键盘输入（pygame 窗口，Q 退出/急停，R 回 ready） |
| `lerobot_robot_massage/zdt/config.py` | 改 | 新增 `KINEMATICS`（DH 表）+ `CARTESIAN_SPEED_LIMIT` |
| `lerobot_robot_massage/massage_robot.py` | 改 | 增 `cartesian_step(vx,vy,vz)` 桥接（可选，供 LeRobot 遥操） |

### 2.4 开发分期

**P0 — kinematics 模块**（纯函数，可无硬件测试）
- [ ] `fk_mdh(dh, q)` Modified DH 正解，用源项目 DH 表
- [ ] FK 对拍：与源项目 `robot_kinematics.m`（Simulink）和 URDF 输出对比
- [ ] `ik_analytic(dh, T)` 移植源项目 θ3→θ2→θ1→θ5→θ4→θ6 解析解
- [ ] 多解加权选择 + 关节限位折叠 + 0-360 归一化（对齐源项目逻辑）
- [ ] 单元测试：FK(IK(T))=T 往返、workspace 边界、奇异点（θ5≈0/π）

**P1 — CartesianController**（依赖 P0）
- [ ] 速度积分目标位置（20mm/s 上限，可配）
- [ ] 每帧 FK 由 0x36 anchor 真实角反馈当前末端（区别于命令积分）
- [ ] IK → `set_joints_safe` 下发 + 限位 clamp
- [ ] IK 无解 → 停车不误动；`check_limits_real` 每帧守卫
- [ ] 看门狗集成（tick → e_stop）

**P2 — 键盘输入**（依赖 P1）
- [ ] pygame：W/S=+vx/-vx, A/D=+vy/-vy, Q/E=+vz/-vz, Shift 倍率
- [ ] R=回 ready（复用 `controller.ready()`），Space=急停，Esc=退出
- [ ] 屏幕显示当前末端坐标 + 关节角

**P3 — 集成与验收**
- [ ] 真机：键盘 → 末端沿直线/圆弧平滑运动，越限自动停
- [ ] 与 `ready()` / `check_limits_real` / watchdog 联调
- [ ] 数据采集：键盘遥操录制末端轨迹（评估后续 SmolVLA 接入）

### 2.5 安全设计

- IK 无解：`return None` + 保持原位置，不上报错误角度
- 速度钳制：单轴 + 末端双重限幅（J2/J3 重力关节更严）
- `check_limits_real`：每帧目标越界 → 单轴 stop
- watchdog：`>watchdog_s` 无 CAN IO → e_stop 广播
- 急停：Space 立即 `e_stop()`
- **明确不做**：姿态（rx/ry/rz）控制——源项目 `cal_T` 也只动平移，姿态固定复位矩阵。后续需要再扩展。

---

## 三、实施前置条件

设计方案已定（2026-08-21 用户确认四项：pygame / 20Hz / 仅位置 3DOF / 0x36 anchor 反馈）。

进入实现计划前确认：
1. **实现起点**：从 P0（kinematics 纯函数模块）开始——不依赖硬件，可立即开发 + 单测。
2. **源项目对拍基准**：P0 用 `robot_kinematics.m`（Simulink 符号推导）和 URDF 输出做 FK/IK 精度对拍。
3. **真机验收**：P3 需 CAN 在线（`zdt_anchor.py` 先确认 6 轴在线），并先跑 `ready()` 回准备姿态再开始键盘遥操。
