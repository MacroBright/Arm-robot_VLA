# 真机 6DOF 视觉遥操 — 增量设计文档

> 日期：2026-08-23
> 分支：`feat/arm-visual-teleop`
> 上游规范：`/home/bright/下载/TuinaDex_6DOF_视觉遥操实施Plan.md`（35 TASK，权威）
> 本文档只记录**增量设计决策**（上游 Plan 已锁定的不重复），并固化用户 2026-08-23 确认的三点澄清。

---

## 0. 范围与本设计定位

本会话实现**全部可代码化的差距**（上游 Plan TASK-01~26、TASK-35 中无需硬件部分）。
TASK-27~34（真机 bring-up / 无动力 / 低速 / 视觉真机测试）**暂不执行**，属硬件验证路径，
实现后在 Plan 中标记 `pending: 真机验证`。

现有基线：171 个测试全绿（`leap_hand` conda env，`pytest lerobot_robot_massage/zdt scripts/teleop/`）。

---

## 1. 架构澄清（用户确认，不复议）

### 1.1 两个安全层共存，通过 controller 协作

| 状态机 | 职责 | 生命周期 |
|---|---|---|
| **RobotStateMachine**（新，safety.py） | 整臂生命周期门禁 | DISCONNECTED → CONNECTED → ENUMERATED → SAFE_IDLE → ARMED → TELEOP → FAULT/STOPPED（手动 re_arm 恢复） |
| **SafetyMachine**（现有，保留） | 单电机枚举 / 选择 / step / joint-limit clamp | 面板级 bring-up 调试 |

- **不合并成一个枚举**，不互相复制状态。
- 共享底层安全能力：`JointModel` / `MotorState` / `MovePlan` / `clamp_delta_real` / `dir_byte_for` / `pulses_for`。
- `ZdtController` 持有 RobotStateMachine 作为整臂门禁；面板工具继续使用 SafetyMachine。
- RobotStateMachine 的 `ARMED` 前置条件 = 全部 6 电机在线 + 关节槽映射完整 + 用户显式确认（重力关节 J2/J3 需二次确认）。

### 1.2 CartesianController 是唯一笛卡尔运动入口

```text
RealArmAdapter → CartesianController → ZdtController → ZdtDriver → CAN
```

- 视觉遥操 / Joystick / VLA **不得绕过** CartesianController 直接发 joint/CAN 命令。
- `step()` = 统一实时接口（速度命令，6DOF）。
- `step_pose()` = 目标位姿接口（内部 SE(3) 误差 → twist → step()）。

### 1.3 6DOF 安全链严格顺序

```text
real FK (0x36 → anchor → source → fk_mdh)
→ workspace limiter
→ orientation safety
→ singularity_metrics
→ adaptive damping (λ + velocity scaling 实际参与，非仅 telemetry)
→ weighted DLS
→ predicted joint-limit scaling (margin 渐进)
→ velocity / acceleration limiting
→ set_joints_safe
→ CAN (0xFD + snF + multi_sync)
```

**纪律：`singularity_metrics()` 必须实际参与 λ 与速度缩放，否则只算检测不算规避。**

---

## 2. 新文件

| 路径 | 职责 | 对应 TASK |
|---|---|---|
| `lerobot_robot_massage/zdt/workspace.py` | `BoxWorkspace` + `CartesianVelocityLimiter` | TASK-13 |
| `lerobot_robot_massage/zdt/types.py` | `CartesianCommand` / `JointState` / `EEPose` | TASK-06/24 |
| `lerobot_robot_massage/zdt/recording.py` | `EpisodeRecorder`（JSONL，观察/动作分离） | TASK-35 |
| `scripts/teleop/arm_adapter.py` | `SimulationArmAdapter` + `RealArmAdapter` 统一接口 | TASK-02/24 |
| `scripts/teleop/real_arm_teleop.py` | 真机视觉遥操入口 | TASK-23 |

---

## 3. kinematics.py 扩展（纯函数）

```python
def log_so3(R: np.ndarray) -> np.ndarray:
    """SO(3) → 轴角向量 ∈ R³ (rad). 归一化/小角退化处理.  → TASK-25"""

def singularity_metrics(J: np.ndarray) -> dict:
    """np.linalg.svd(J) → {sigma_min, sigma_max, condition_number, manipulability}."""
    # condition_number = sigma_max / sigma_min (sigma_min→0 时 inf, 调用方处理)
    # manipulability = prod(sigma)

def adaptive_damping(metrics: dict, base_lam: float,
                     near_ratio: float = 0.3, sing_ratio: float = 0.1,
                     lam_max: float | None = None) -> tuple[float, float]:
    """返回 (λ, velocity_scale).
    三档: NORMAL λ=base_lam · scale=1.0
          NEAR_SINGULAR λ↑ · scale↓
          SINGULAR λ=lam_max · scale→0 (拒绝/停车)
    阈值 near_ratio/sing_ratio 为 sigma_min/sigma_max 之比；初值可配，
    真机 bring-up 实测校准 (Plan TASK-11 纪律: 不拍死最终阈值).
    """
```

---

## 4. CartesianController 升级

### 4.1 `step()` — 统一实时接口（6DOF）

```python
def step(self, vx, vy, vz, wx=0.0, wy=0.0, wz=0.0) -> dict:
    """6DOF 笛卡尔速度闭环. 向后兼容 3DOF 调用 (ω 默认 0).
    返回 {moved, reason?, sigma_min?, condition?, lambda?, scale?, alarms?, target_xyz?}"""
```

管线（严格顺序，见 §1.3）：
1. `_read_current_ee()`：0x36 真实角（anchor 帧）→ source 帧 → FK → 当前末端 (p_act, R_act)。
2. **workspace limiter**：`p_des = p_act + v·dt` 是否越盒 → 越界分量速度缩放/拒绝。
3. **orientation safety**：速度输入模式下 clamp `|ω| ≤ max_ang_rad_s`（位置优先、姿态平稳）；
   `step_pose()` 额外在进入 twist 前对目标 R 相对 anchor 的 roll/pitch/yaw 范围 clamp。
4. **singularity_metrics(J)**。
5. **adaptive_damping(metrics, ik_lambda)** → (λ, scale)；`twist *= scale`。
6. **weighted DLS**：`dq = damped_ls(J, twist*dt, λ, weights)`（姿态权重 = orient_weight）。
7. **predicted joint-limit scaling**：`q_next = q_src + dq`，margin 渐进——越近限位越缩 `dq`；目标越界 → reject。
8. **velocity/acceleration limiting**：`|dq| ≤ max_dq_deg`、`|dq/dt| ≤ max_joint_vel`、帧间 dq 差分 ≤ max_joint_acc。
9. **set_joints_safe(q_anchor_target)**（0x36 真实位置限位 + 限位守卫）。
10. 返回遥测。

### 4.2 `step_pose()` — 目标位姿接口（SE(3)）

```python
def step_pose(self, p_des, R_des) -> dict:
    """位置误差 e_p = p_des - p_act → v = Kp·e_p
       姿态误差 R_err = R_des @ R_act.T → e_R = log_so3(R_err) → ω = Kr·e_R
       → twist → step(). 禁止 Euler 累加.  → TASK-08/25"""
```

新增配置参数（构造器 + config）：`max_ang_rad_s`、`max_joint_vel_deg_s`、`max_joint_acc_deg_s2`、`joint_limit_margin_deg`、`near/singular` 阈值、`kp_pos`、`kr_ori`。

---

## 5. ZdtController 状态机 + 安全

### 5.1 RobotStateMachine（safety.py 新增）

```python
class RobotPhase(Enum):
    DISCONNECTED, CONNECTED, ENUMERATED, SAFE_IDLE, ARMED, TELEOP, FAULT, STOPPED

class RobotStateMachine:
    def __init__(self, num_joints=6): ...
    def on_connected(self): CONNECTED
    def on_enumerated(self, motors): ENUMERATED   # 校验 6 轴在线 + 槽映射完整
    def on_safe_idle(self): SAFE_IDLE            # 已读状态 + sync
    def arm(self, gravity_confirmed=False): SAFE_IDLE→ARMED (set_torque 由 controller 执行)
    def enter_teleop(self): ARMED→TELEOP
    def exit_teleop(self): TELEOP→ARMED
    def e_stop(self): *→STOPPED (闩锁)
    def fault(self, reason): *→FAULT→STOPPED (闩锁)
    def re_arm(self, confirmed): STOPPED→ENUMERATED (需显式确认 + 重新枚举)
    def assert_armed(self): 门禁
    def assert_teleop(self): 门禁
```

### 5.2 `connect()` 改（TASK-20）

```text
connect(): CAN open → on_connected → scan/verify motors → on_enumerated
           → read state → sync real position → on_safe_idle
           ⚠ 不再 set_torque(True)
arm():    assert SAFE_IDLE → gravity 确认 → set_torque(True) → on_armed
```

### 5.3 `get_real_state()`（TASK-18）

```python
def get_real_state(self) -> dict:
    """{q: 0x36 真实角(anchor, use_kb=True),
         velocity: 滤波有限差分 dq (低通, 默认窗口/α 可配; 驱动器实时速度读取留待后续),
         current: 0x36... (read_current),
         flags: read_flag,   # 使能/到位/堵转/堵转保护
         status: RobotPhase.name}"""
```

现有 `get_state()` 保留（SerialProtocol 兼容），内部不再作为 VLA observation 唯一来源。

### 5.4 调用方适配

| 调用方 | 改动 |
|---|---|
| `scripts/control/cartesian_keyboard.py` | connect 后打印状态 → 显式 `--arm`/回车确认 arm → 才进入运动循环 |
| `scripts/bringup/zdt_bringup.py` | `--state on/off` 改走 `arm()/disarm()`（语义不变） |
| `lerobot_robot_massage/massage_robot.py` | 显式 set_torque 路径保持；接入 RobotStateMachine 门禁 |

---

## 6. 遥操层

### 6.1 `arm_adapter.py`

```python
class SimulationArmAdapter:   # MuJoCo socket (复用 ArmClient), 保持仿真能力
    def connect(); def disconnect()
    def get_joint_state(); def get_ee_pose()   # JointState / EEPose
    def move_cartesian_velocity(cmd: CartesianCommand)
    def reset(); def e_stop()
class RealArmAdapter:          # 封装 CartesianController → ZdtController → ... 
    def connect(); def disconnect()
    def get_joint_state(); def get_real_joint_angles(); def get_ee_pose()
    def move_cartesian_velocity(cmd: CartesianCommand)
    def step_pose(p_des, R_des)
    def reset(); def e_stop(); def state()
```

**RealArmAdapter 不实现 CAN 协议、不重复 IK、不直接操作电机帧。** 视觉层只产
`CartesianCommand`，绝不经 UART `remote_enable/end_event`（TASK-22 移除该依赖）。

### 6.2 `real_arm_teleop.py`

- RealSense(D455) + HandTracker + WristTracker（复用 Leap_Hand 共享模块，同 demo）
- clutch（H 按住跟随 / 松开重锚，**保留** TASK-05）
- handeye 旋转（K 校准向导 / C 重载，复用现有 handeye_calib）
- **VisionWatchdog**（TASK-16）：hand confidence / depth invalid / wrist jump / hand lost / stale command 分级——
  短暂丢失 → 速度衰减（hold with decay）；持续丢失 → 停止；严重 → e_stop。**禁止无限保持上一帧命令。**
- 视觉输出：`CartesianCommand{linear_velocity mm/s, angular_velocity rad/s, timestamp}` → RealArmAdapter。
- 按键：H=clutch，R=reset/ready（显式），Y=e_stop，Q/ESC=安全退出。

---

## 7. 数据记录（TASK-35）

`zdt/recording.py` — `EpisodeRecorder`（JSONL per episode，时间戳 + 序号文件名）。

每条记录：

```json
{
  "timestamp": float,
  "observation": {
    "q": [..6], "dq": [..6], "current": [..6],
    "ee_pose": {"position": [x,y,z], "quaternion": [w,x,y,z]},
    "hand_pose": {"position": [...], "orientation": [...], "confidence": float},
    "camera_ts": float
  },
  "action": {
    "cartesian_command": {"linear_velocity": [..3], "angular_velocity": [..3], "timestamp": float},
    "commanded_joint_target": [..6]
  },
  "safety": {"phase": "...", "sigma_min": float, "condition": float, "workspace_ok": bool}
}
```

**observation / action 明确分离**，直接服务后续 ACT / Diffusion Policy / SmolVLA / VLA。

---

## 8. 测试计划

- 现有 171 测试保持绿。
- 新增单元测试（FakeTransport 注入，复用 `zdt/testutil.py`）：
  - `test_kinematics.py` 增：`test_log_so3`、`test_singularity_metrics`、`test_adaptive_damping`
  - 新 `test_workspace.py`：盒越界 / 限幅器
  - `test_cartesian.py` 增：6DOF step、step_pose、安全链顺序、奇异点参与缩放（非仅 telemetry）
  - 新 `test_robot_state.py`：状态转移 + 非法转移拒绝 + STOPPED 闩锁 + re_arm 需确认
  - `test_controller.py` 增：connect 不再使能力矩；arm 才使能；get_real_state 字段
  - 新 `test_recording.py`：字段 schema + observation/action 分离
  - 新 `test_adapter.py`：Simulation/Real Adapter 接口 + Real 不直接操作 CAN
  - 新 `test_watchdog.py`：分级策略
- 真机验证路径（TASK-27~34）→ Plan 标记 `pending: 真机验证`，本会话不执行。

---

## 9. 文档

1. `Arm-robot_VLA/CLAUDE.md`：更新为当前 USB-CAN 架构（无 STM32 网关；ADR/边界规则改写）。
2. 上游 Plan 文档：实施记录勾选完成项 + 标注真机 pending（Plan §29 纪律要求）。

---

## 10. 明确不做（YAGNI / 留待后续）

- 驱动器实时速度读取（0x35）直读 —— 本次用滤波有限差分，0x35 留待真机验证阶段（TASK-18 优先级 2 已允许）。
- 工作空间扩展（Sphere/Cylinder/Patient ROI）—— 首版盒状（Plan TASK-13 明确）。
- 6DOF 冗余自由度 null-space 奇异规避 —— 本臂 6DOF 非冗余（Plan §9 已说明）。
- 真机 bring-up / 运动验证（TASK-27~34）。
