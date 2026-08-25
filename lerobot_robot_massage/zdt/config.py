"""ZDT 驱动常量与配置.

组帧约定 (参考固件 can.c can_SendCmd, spec §2.1):
  扩展帧 ID = (地址<<8) | 包序号
  数据段    = [功能码, 参数..., 0x6B]
  参数 >7 字节 → 拆多帧, 每帧重复功能码
"""
from typing import Optional
from dataclasses import dataclass, field

# 关节→CAN 地址 (J1→02 ... J6→07), 帧 ID 高字节
# ⚠ 固件 robot.c 按 joint_id+1 寻址 (J1=0x01..J6=0x06); PC 侧配置用 0x02..0x07.
#   bring-up 面板 (zdt_panel) 通过枚举实测裁决 (见 scan.py), 不要盲信任一侧.
JOINT_ADDRS: list[int] = [0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
FIRMWARE_JOINT_ADDRS: list[int] = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06]

# 各关节正方向对应 0xFD dir 字节 (固件 robot.c g_joints_init postive_direction:
#   J1/J5=CCW(dir=1), 其余 CW(dir=0); 负角度取反 = 1-dir_byte)
JOINT_DIR_POS_BYTE: list[int] = [1, 0, 0, 0, 1, 0]

# 开机姿态即全零期望位: 手动摆固定姿态 → 上电 → 驱动器自动 pos=0 → anchor
# (经 CALIB 换算出真实角 ≈0). soft_reset / scan 跟踪初始化 / zdt_anchor --expected
# 默认值均以此为基准. 旧值 [90,90,-90,0,90,0] 为固件出厂角, 与本方案冲突已弃用.
JOINT_INIT_ANGLE_DEG: list[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# 按摩准备姿态 (真实输出角度, 与 anchor 同基准) — controller.ready() 目标.
# 2026-08-23 真机调整: [0,60,50,0,120,0] (原 [0,45,45,0,165,0]).
# 各值须在 FIRMWARE_JOINT_LIMITS 内 (J2 60∈[-1,150] J3 50∈[-1,120] J5 120∈[-1,180]).
READY_POSE_DEG: list[float] = [0.0, 60.0, 50.0, 0.0, 120.0, 0.0]
# ready 专用速度 (电机轴 RPM), 仅该命令生效, 不改全局 speed_rpm.
# 2026-08-23: 修复 0xFD 速度字段 ×10 编码 bug 后, 直接设真实 RPM (100 = 保持修复前手感).
READY_SPEED_RPM: float = 100.0

# 重力关节 (0-based 索引): J2 肩抬 + J3 肘 = 承载重量, 禁止轻易失电
GRAVITY_JOINTS: set[int] = {1, 2}

# 各关节机械限位 (度) — 真机 anchor 实测 (关节真实角度, 2026-08-20)
# 原为固件表抄值, 现以真机实测 anchor 值为权威 (spec §2.2)
# 下界 -1.0 = 开机姿态锚定余量: 开机 pos=0, 经 CALIB(k,b) 换算出 -b/k 的小负角
#   (如 J2 b=2.02 → (0-2.02)/50.89 = -0.040°), 因此下限必须含小负值, 否则
#   开机姿态落在限位外会误报. 360° 关节 J1/J6 允许 -1.0 无碍 (可回转).
FIRMWARE_JOINT_LIMITS: list[tuple[float, float]] = [
    (-1.0, 360.0),   # J1 shoulder_pan (360° 旋转)
    (-1.0, 150.0),   # J2 shoulder_lift (真机实测 anchor; 上限 120→150 实测提升)
    (-1.0, 120.0),   # J3 elbow_flex (真机实测 anchor)
    (-90.0, 90.0),   # J4 wrist_roll (腕翻转)
    (-1.0, 180.0),   # J5 wrist_flex (腕俯仰, 真机实测 anchor)
    (-1.0, 360.0),   # J6 gripper (360° 旋转)
]

# 帧校验字节 (数据段末字节固定)
CHECKSUM: int = 0x6B

# ZDT 功能码
F_ENABLE: int = 0xF3      # 使能/失能
F_STOP: int = 0xFE        # 立即停止
F_POS: int = 0xFB         # 直通限速位置 (ZDT 新代手册格式)
F_LEGACY_POS: int = 0xFD  # 固件/旧代 Emm_V5 位置命令 (脉冲计数) — 真机验证常用
F_VEL: int = 0xF6         # 速度模式
F_READ_POS: int = 0x36    # 读实时位置
F_READ_CUR: int = 0x27    # 读相电流
F_ARRIVED: int = 0xFD     # 到位帧 (数据段[0])

# 换算倍率
POS_SCALE: float = 10.0   # 位置 ×10
# 2026-08-23: 0xFD/0xF6 速度字段 = RPM 直传 (手册 0x05DC=1500RPM), 非 ×10.
# 0x35 读实时转速字段才是 ×10 (scan.py 内联 ÷10, 与本常量无关).
VEL_SCALE: float = 1.0    # 速度 RPM 直传

# 限位与初始位姿: 单一来源 = FIRMWARE_JOINT_LIMITS / JOINT_INIT_ANGLE_DEG
# (旧 DEFAULT_LIMITS 纯副本与假初始位 INIT_POSE_DEG[J2=45°] 已删除, 见 git 历史)

# 各关节减速比 (输出轴 1° = 电机轴 N°), 取自已跑通真机的固件 robot.c g_joints_init.
# 位置命令换算: 脉冲 = 输出角度 × reduction_ratios[i] × pulses_per_rev / 360.
# 单机调试装外置减速器 (如 51:1) 时必须按实测 override (交互工具 calib 标定).
DEFAULT_REDUCTION_RATIOS: list[float] = [50.0, 50.89, 50.89, 51.0, 27.0, 51.0]

# 0x36 pos 刻度标定表 (0x36 4字节读数 → 输出轴角度 的线性换算).
# 索引 = 关节槽 (0=J1..5=J6), None = 未标定. 公式: 输出角度 = (pos - b) / k.
# 0x36 返回带符号真实位置 (Emm42 V5.0 说明书 §0x36 + 固件 robot.c:1045 实测确认),
# 位置字段 4字节, 单位已是电机轴度 (value × 360/65536). k ≈ 减速比 (电机轴/输出轴),
#   b = 零点偏置 (电机轴度). 旧 k≈3.62 基于错误的 3字节÷10 解码, 已作废.
# 用途: 上电锚定 (interactive `anchor`) / 0x36 真实位置限位 / pos 漂移守卫.
# 标定方法: zdt_interactive.py 的 `cal` 扫点拟合得 (k, b), 填入对应槽位.
# ⚠ (k,b) 每台电机独有, 勿跨关节套用.
# 2026-08-21: decode_pos4 修正后真机重新标定 (详见 HANDOFF §六).
#   J1 复测修正: k=48.01 → 50.00 (拟合偏差修正，与50:1标称一致)；
#   J3 初版 k=38.77 偏差较大, 复测后修正为 k=50.90 (43分立+51:1, 与 J2 一致).
CALIB: list = [
    (50.0014, 0.03),     # J1
    (50.8886, 2.02),      # J2
    (50.8992, 0.02),      # J3
    (51.0041, -1.55),     # J4
    (27.0117, -0.83),     # J5 (真机 cal 扫点标定, 2026-08-21)
    (51.009, 0.01),       # J6
]

# 0x0A 6D 清零偏置表 (A 任务主路线, 简单标定).
# 索引 = 关节槽, None = 未标定. 公式: 真实输出角度 = (0x36读数 / 减速比) - offset.
# 标定流程 (方案 b): 人工摆到已知姿态 → 读 0x36 → offset = (0x36/减速比) - 期望角度
#                   → 发 0x0A 6D 清零 → 把 offset 存入此表.
# 与 CALIB(k,b) 的关系: CALIB_OFFSETS 是简单偏置 (减速比用 DEFAULT_REDUCTION_RATIOS),
#                       CALIB 是精确 (k,b) 拟合 (可校准减速比误差). 两者互斥, 用其一即可.
CALIB_OFFSETS: list = [None] * 6


@dataclass
class ZdtConfig:
    channel: str = "can0"
    bitrate: int = 500_000
    timeout_s: float = 0.1
    retries: int = 3
    # 0xFD 位置命令电机速度 (RPM, 修复 ×10 bug 后直传). 遥操三档需 ~150 RPM 上限.
    speed_rpm: float = 150.0
    watchdog_s: float = 0.5
    joint_addrs: list[int] = field(default_factory=lambda: list(JOINT_ADDRS))
    limits: list[tuple[float, float]] = field(
        default_factory=lambda: list(FIRMWARE_JOINT_LIMITS))
    # 0xFD 位置命令参数 (固件 Emm_V5_Pos_Control 兼容)
    pulses_per_rev: int = 3200      # 16 细分下 3200 脉冲 = 电机轴一圈 (固件约定)
    # 加速度档位 (0-255, 越高加速越快: (256-acc)*50µs/+1RPM). 遥操 50ms 帧内
    # 需快速加速, 220 → 1.8ms/RPM, 50RPM 约 90ms 到位 (修复"爬行"). 初始值待真机调.
    position_acc: int = 220
    reduction_ratios: list[float] = field(
        default_factory=lambda: list(DEFAULT_REDUCTION_RATIOS))
    # 0x36 pos → 真实输出角度 标定表 (默认整机 CALIB). 经 (pos - b) / k 换算,
    # 与 interactive `anchor` / zdt_anchor.py 结论一致. 测试可传 [(1.0, 0.0)]*6
    # 保持 1:1 直观换算; None = 退化为纯减速比 (未标定参考).
    calib: list = field(default_factory=lambda: list(CALIB))

    # ── 遥操/笛卡尔 (2026-08-23, spec §4) ──
    max_vel_mm_s: float = 600.0
    max_ang_rad_s: float = 10.0
    max_joint_vel_deg_s: float = 540.0
    max_joint_acc_deg_s2: float = 2000.0
    joint_limit_margin_deg: float = 2.0
    kp_pos: float = 2.0
    kr_ori: float = 2.0
    ik_near_ratio: float = 0.15
    ik_sing_ratio: float = 0.03
    workspace_min: Optional[list] = None      # None = 不启用盒约束 (待真机标定)
    workspace_max: Optional[list] = None
    dt_min_factor: float = 0.5
    dt_max_factor: float = 3.0
    stale_cmd_max_s: float = 0.25
    vel_filter_alpha: float = 0.2
