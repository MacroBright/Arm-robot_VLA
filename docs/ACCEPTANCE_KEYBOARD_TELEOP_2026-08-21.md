# 键盘遥操 IK 笛卡尔控制 — P3 集成验收清单

> **日期**：2026-08-21
> **关联**：[RESEARCH_IK_KEYBOARD_TELEOP_2026-08-21.md](RESEARCH_IK_KEYBOARD_TELEOP_2026-08-21.md)（P0-P2 设计）、[RESEARCH_ABS_ENCODER_AND_FW_COMPARE_2026-08-21.md](RESEARCH_ABS_ENCODER_AND_FW_COMPARE_2026-08-21.md)（anchor 体系）
> **前置**：P0 kinematics / P1 CartesianController / P2 键盘脚本已完成（153 测试全绿, `pytest lerobot_robot_massage/zdt/ -q`）

---

## 一、真机验收前置条件

| 项 | 要求 | 检查方法 |
|----|------|---------|
| CAN 链路 | `can0` up, 500kbit | `sudo ip link set can0 type can bitrate 500000 && sudo ip link set can0 up` |
| 6 轴在线 | 全部在线 | `python scripts/bringup/zdt_anchor.py` 无离线 |
| 开机姿态 | 摆固定姿态后上电, 6 轴 pos≈0 | anchor 输出各轴 |真实| < 1° |
| **坐标帧验证** | 开机姿态 == 源项目复位姿态 (手臂竖直向上) | anchor 6 轴应≈0; 若≠0 需更新 `kinematics.SOURCE_TO_ANCHOR_OFFSET` |
| 安全区 | 无人 / 假体, 急停可及 | 操作前确认 |
| 模块测试 | P0-P2 测试全绿 | `pytest lerobot_robot_massage/zdt/ -q` |

---

## 二、验收步骤 (每步记录结果)

### Step 1 — ready 准备姿态

**命令**: 交互工具 `prod ready` 或键盘脚本启动自动回 ready。

| 检查点 | 通过标准 |
|--------|---------|
| 6 轴同步运动 | 各轴同时启动 (多机同步广播) |
| 慢速 | 全程 ~10 RPM 电机轴 (无急冲) |
| 到位 | `all status` 的 anchor 列 ≈ [0,60,50,0,120,0] (±1°) |
| 限位不误报 | 过程无 "限位告警" 输出 |

### Step 2 — 末端坐标零点

**命令**: 键盘脚本启动后观察屏幕 "末端" 行 (FK 反馈)。

| 检查点 | 通过标准 |
|--------|---------|
| 末端坐标 | ≈ FK(ready anchor 角) 的理论值 (先记录, 供 Step 3 对照) |
| 数值稳定 | 静止时末端坐标变化 < 0.5mm (无抖动/漂移) |

### Step 3 — 笛卡尔平移

**命令**: W/S/A/D/Q/E 单轴 + 组合轴。

| 检查点 | 通过标准 |
|--------|---------|
| 单轴响应 | 按住 W → 末端 x 单调增大, 松手停止 |
| 方向正确 | W=+x, S=-x, A=+y, D=-y, Q=+z, E=-z (对照屏幕坐标) |
| 慢档速度 | 单轴 ≈20 mm/s (实测位移/时间) |
| 快档 | Shift+W ≈60 mm/s |
| 平滑 | 无抖动/步进顿挫, 轨迹连续 |
| 无越界 | 持续推向单一方向 → 到 workspace 边界自动停 (限位守卫 `limit_alarm`) |

### Step 4 — 急停与看门狗

**命令**: SPACE / Ctrl+C / 拔 CAN。

| 检查点 | 通过标准 |
|--------|---------|
| SPACE 急停 | 立即停止, 屏幕提示, 机械臂保持力矩 (不掉落) |
| 退出清理 | Ctrl+C → 先 e_stop 再断开 (无残留使能) |
| 看门狗 | 拔 CAN 线 >0.5s → 自动 e_stop (日志 "watchdog") |

### Step 5 — 回 ready 与限位恢复

**命令**: R 键 (运动中按)。

| 检查点 | 通过标准 |
|--------|---------|
| 任意位回 ready | 从 workspace 内任意位置回 [0,60,50,0,120,0] |
| 限位内运动 | 全程无越界告警 |

### Step 6 — 漂移自愈 (关键)

**命令**: 运动中手动轻推末端。

| 检查点 | 通过标准 |
|--------|---------|
| FK 反馈自愈 | 外力搬动后松开 → 下一帧末端坐标反映真实位置 |
| 无累积漂移 | 连续操作 3 分钟, 松手静止, 末端坐标回稳 (漂移 < 1mm) |

---

## 三、数据采集评估 (P4 预备)

| 项 | 现状 | 评估目标 |
|----|------|---------|
| 键盘遥操录轨迹 | 尚无采集脚本 | 键盘 → 末端轨迹平滑度 (转折角/抖动) |
| SmolVLA 起点 | `MassageRobot.reset()` 已接入 ready | 每 episode 从 READY_POSE 开始 |
| 评估 | 仿真 evaluate_policy 已有 | 键盘采集数据替换手柄数据后训练对比 |

---

## 四、已知边界与风险

| 项 | 说明 | 处置 |
|----|------|------|
| 坐标帧假设 | `SOURCE_TO_ANCHOR_OFFSET = RESET_POSE_DEG` 待真机验证 | Step 1 前置条件强制验证 |
| 姿态锁定 | 第一版仅位置 (3DOF), 姿态锁 T_0_6_RESET | 按摩主要需位置; 需要再扩 |
| J2/J3 重力关节 | 失电即下坠 | 全程使能, 急停只断命令不断电 |
| J5 通信失联 | HANDOFF 有记录待查 | 自动运动前先 anchor 确认在线 |
| workspace | 源项目限位 vs 本项目限位不同 | CartesianController 用雅可比数值 IK + anchor 帧 FIRMWARE 限位, 已双限 |

---

## 五、验收签名

```
日期: ________  操作者: ________  环境: 仿真 / 真机
P0 测试全绿: [ ]  P1 测试全绿: [ ]  P2 脚本启动: [ ]
Step1 ready: [ ]  Step2 零点: [ ]  Step3 平移: [ ]
Step4 急停: [ ]   Step5 回ready: [ ]  Step6 漂移自愈: [ ]
结论: 通过 / 不通过 (不通过项: ________)
```

---

## 六、下一步

- [ ] Step 1-6 真机验收, 记录实测末端坐标与理论 FK 值对照
- [ ] 若 `SOURCE_TO_ANCHOR_OFFSET` 不符 → 更新常量 + 重跑 P0 测试
- [ ] P4: 键盘遥操采集脚本 (复用 `record_sim.py` 模式) → SmolVLA 数据
