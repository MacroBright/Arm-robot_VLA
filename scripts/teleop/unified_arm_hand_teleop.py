"""scripts/teleop/unified_arm_hand_teleop.py — 机械臂-灵巧手协同视觉遥操统一主控管线 (Arm-Hand Unified Teleoperation Pipeline).

【架构特性】
1. 单目视觉复用 (Single Perception Stream):
   - 单台 RealSense D455 相机, 单次 MediaPipe 3D 手势检测;
   - 同时解算手腕 3D 宏观轨迹 (驱动 6DOF 机械臂) 与五指 21 关节点微观几何 (驱动 16DOF 灵巧手);
2. 臂-手解耦控制 (Decoupled Arm-Hand Kinematics):
   - 手腕位移 + 掌面倾角 -> 机械臂空间平移与推拿模态 (点按/滚法/俯仰/全自由);
   - 五指屈伸 -> 灵巧手 16 舵机抓握与揉捏;
3. 统一安全看门狗与离合器 (Unified Watchdog & Clutch):
   - 丢帧/遮挡时: 机械臂平稳减速悬停, 灵巧手执行 relax_step 平滑回全开位 (OPEN_POSE);
4. 一体化综合 HUD 监控看板 (Single Unified Dashboard):
   - 单窗口 960x720: 手部骨骼 + 虚拟摇杆 + 机械臂 6 轴状态表 + 灵巧手 16 舵机弯曲柱状图;
5. 集中配置即改即用: 自动加载 teleop_config.yaml 并执行起飞前安全检查.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from enum import Enum, auto
from pathlib import Path
import sys
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np

# 导入工程路径
REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT.parent / "Leap_Hand" / "python"))

from scripts.teleop.arm_adapter import NoDriveArmAdapter, RealArmAdapter  # noqa: E402
from lerobot_robot_massage.zdt.types import CartesianCommand, EEPose, JointState  # noqa: E402
from scripts.teleop.hand_adapter import LeapHandAdapter, NoDriveHandAdapter  # noqa: E402
from scripts.teleop.teleop_config import DEFAULT_TELEOP_CONFIG, TeleopConfig, build_gear_configs  # noqa: E402
from scripts.teleop.watchdog import VisionWatchdog, WatchdogAction  # noqa: E402

# 灵巧手算法组件
from gesture_mapping import Calibrator, FingerIdentifier, HandTracker, JointMapper  # noqa: E402
from gesture_mapping.camera import open_realsense  # noqa: E402
from gesture_mapping.filter import OneEuroFilter  # noqa: E402
from gesture_mapping.hamer_3d import HaMeR3D, hand_bbox_from_landmarks  # noqa: E402
from gesture_mapping.wrist_tracker import build_palm_pts  # noqa: E402


# ── 推拿模态定义 ─────────────────────────────────────────────────────────────
MODE_KNEAD = 1   # 点按/揉捏模式 (姿态完全锁定)
MODE_ROLL  = 2   # 滚法推拿模式 (单轴 Roll 摇杆翻滚)
MODE_PITCH = 3   # 俯仰推拿模式 (单轴 Pitch 摇杆摆动)
MODE_FULL  = 4   # 全自由 6-DOF (全姿态跟随)

MODE_NAMES = {
    MODE_KNEAD: "1. 揉捏模式 (姿态锁定)",
    MODE_ROLL:  "2. 滚法模式 (Roll 摇杆)",
    MODE_PITCH: "3. 俯仰模式 (Pitch 摇杆)",
    MODE_FULL:  "4. 全 6-DOF (全姿态跟随)",
}
MODE_MAP = {"knead": MODE_KNEAD, "roll": MODE_ROLL, "pitch": MODE_PITCH, "full": MODE_FULL}

# 灵巧手手指与电机对应标签
FINGER_NAMES = ["食指", "中指", "无名", "拇指"]
SOURCE_NAMES = {0: "HAMER 3D", 1: "WORLD 3D", 2: "MP PSEUDO-3D"}
_MIRRORED_LABEL = {"right": "left", "left": "right"}


def _smoothed_frame(pts, smoother):
    """计算掌心参考系并对 normal/mid_dir/lateral 进行时域正交平滑."""
    wrist, normal, mid_dir, lateral = JointMapper._palm_frame(pts)
    fvec = smoother(np.concatenate([normal, mid_dir, lateral]))
    normal, mid_dir, lateral = fvec[:3], fvec[3:6], fvec[6:9]
    for v in (normal, mid_dir):
        n = np.linalg.norm(v)
        if n > 1e-9:
            v /= n
    lateral = lateral - np.dot(lateral, mid_dir) * mid_dir
    n = np.linalg.norm(lateral)
    if n > 1e-9:
        lateral /= n
    else:
        lateral = np.cross(normal, mid_dir)
        n = np.linalg.norm(lateral)
        if n > 1e-9:
            lateral /= n
    return (wrist, normal, mid_dir, lateral)


def _draw_unified_dashboard(frame: np.ndarray,
                            arm_out: dict,
                            joint_state: Optional[JointState],
                            hand_angles: np.ndarray,
                            hand_bent: Optional[List[bool]],
                            hand_state_str: str,
                            clutch_active: bool,
                            arm_mode: int,
                            arm_gear: int,
                            gear_configs: dict,
                            source_name: str,
                            fps: float,
                            no_drive_arm: bool,
                            no_drive_hand: bool) -> None:
    """在 1280x720 画面上绘制高透明度、无阴影重叠的机械臂 + 灵巧手一体化 HUD 监控看板 (含末端姿态/速度面板与手腕运动箭头)."""
    h, w = frame.shape[:2]
    gear_info = gear_configs.get(arm_gear, gear_configs[2])

    # 1. 统一构建高透明度玻璃态背景图层 (Single-pass Semi-transparent HUD Layer)
    overlay = frame.copy()

    # 顶部状态栏底色
    cv2.rectangle(overlay, (0, 0), (w, 46), (16, 18, 24), -1)

    # 左侧末端运动方向与姿态面板 (Motion & Attitude Panel)
    lx0, ly0 = 14, 56
    l_box_w, l_box_h = 390, 160
    cv2.rectangle(overlay, (lx0, ly0), (lx0 + l_box_w, ly0 + l_box_h), (16, 18, 24), -1)

    # 右侧机械臂状态面板
    box_w = 330
    rx0, ry0 = w - box_w - 14, 56
    box_h = 160
    cv2.rectangle(overlay, (rx0, ry0), (rx0 + box_w, ry0 + box_h), (16, 18, 24), -1)

    # 右侧灵巧手舵机面板
    hx0, hy0 = rx0, ry0 + box_h + 12
    h_box_h = 280
    cv2.rectangle(overlay, (hx0, hy0), (hx0 + box_w, hy0 + h_box_h), (16, 18, 24), -1)

    # 底部快捷键提示栏底色
    cv2.rectangle(overlay, (0, h - 30), (w, h), (16, 18, 24), -1)

    # 高透明度混合: 35% 黑色遮罩 + 65% 相机原画 (用户手部清晰透见)
    cv2.addWeighted(overlay, 0.35, frame, 0.65, 0, frame)

    # 2. 绘制面板边框线
    cv2.line(frame, (0, 46), (w, 46), (60, 70, 80), 1)
    cv2.line(frame, (0, h - 30), (w, h - 30), (60, 70, 80), 1)
    cv2.rectangle(frame, (lx0, ly0), (lx0 + l_box_w, ly0 + l_box_h), (0, 220, 255), 1)
    cv2.rectangle(frame, (rx0, ry0), (rx0 + box_w, ry0 + box_h), (0, 220, 255), 1)
    cv2.rectangle(frame, (hx0, hy0), (hx0 + box_w, hy0 + h_box_h), (0, 255, 180), 1)

    # 3. 顶部 Header 状态栏内容
    if clutch_active:
        cv2.rectangle(frame, (10, 8), (145, 38), (0, 180, 80), -1)
        cv2.putText(frame, "[SPACE] RUN", (18, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    else:
        cv2.rectangle(frame, (10, 8), (145, 38), (40, 40, 200), -1)
        cv2.putText(frame, "[SPACE] PAUSE", (15, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)

    # 机械臂模式
    cv2.putText(frame, f"ARM: {MODE_NAMES[arm_mode]}", (160, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 2)

    # 档位徽标
    badge_col = gear_info.get("color", (0, 220, 255))
    cv2.rectangle(frame, (530, 8), (620, 38), (30, 30, 30), -1)
    cv2.rectangle(frame, (530, 8), (620, 38), badge_col, 2)
    cv2.putText(frame, f"[S] {gear_info.get('badge', 'MID')}", (538, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, badge_col, 2)

    # 灵巧手状态
    hand_col = (0, 255, 120) if "POWERED" in hand_state_str else (160, 160, 160)
    cv2.putText(frame, f"HAND: {hand_state_str}", (640, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, hand_col, 2)
    cv2.putText(frame, f"3D: {source_name}", (870, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 200, 255), 1)
    cv2.putText(frame, f"{fps:3.0f} FPS", (w - 90, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 2)

    # 4. 左上末端运动方向与姿态面板 (Motion & Attitude Panel)
    cv2.putText(frame, "MOTION & ATTITUDE", (lx0 + 10, ly0 + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 230, 255), 2)

    vel = arm_out.get("v", np.zeros(3))
    ang = arm_out.get("w", np.zeros(3))
    if clutch_active:
        x_dir = "左" if vel[0] > 4 else ("右" if vel[0] < -4 else "-")
        y_dir = "后" if vel[1] > 4 else ("前" if vel[1] < -4 else "-")
        z_dir = "上" if vel[2] > 4 else ("下" if vel[2] < -4 else "-")
        spd_txt = f"v_lin: [X:{vel[0]:+4.0f}({x_dir}), Y:{vel[1]:+4.0f}({y_dir}), Z:{vel[2]:+4.0f}({z_dir})] mm/s"
        cv2.putText(frame, spd_txt, (lx0 + 10, ly0 + 52), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 120), 1)

        p_tag = "低头" if ang[0] < -0.04 else ("抬头" if ang[0] > 0.04 else "")
        r_tag = "左滚" if ang[1] > 0.04 else ("右滚" if ang[1] < -0.04 else "")
        tags = [t for t in (p_tag, r_tag) if t]
        rot_tag = f" [{' '.join(tags)}]" if tags else (" [锁定]" if arm_mode == 1 else "")
        ang_txt = f"w_ang: [Pitch:{ang[0]:+4.2f}, Roll:{ang[1]:+4.2f}] rad/s{rot_tag}"
        cv2.putText(frame, ang_txt, (lx0 + 10, ly0 + 82), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 230, 0), 1)
    else:
        cv2.putText(frame, "v_lin: [ +0.0,  +0.0,  +0.0] mm/s [PAUSED]", (lx0 + 10, ly0 + 52), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 180, 255), 1)
        cv2.putText(frame, "w_ang: [ +0.00,  +0.00] rad/s [PAUSED]", (lx0 + 10, ly0 + 82), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 180, 255), 1)

    # 掌面虚拟摇杆角度 (Tilt Angles)
    roll_deg = float(arm_out.get("d_roll_deg", 0.0))
    pitch_deg = float(arm_out.get("d_pitch_deg", 0.0))
    h_roll_tag = "左倾" if roll_deg > 4.0 else ("右倾" if roll_deg < -4.0 else "平")
    h_pitch_tag = "下压" if pitch_deg < -4.0 else ("上抬" if pitch_deg > 4.0 else "平")
    tilt_txt = f"Tilt: Roll:{roll_deg:+5.1f}°({h_roll_tag}) | Pitch:{pitch_deg:+5.1f}°({h_pitch_tag})"
    cv2.putText(frame, tilt_txt, (lx0 + 10, ly0 + 112), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 230, 255), 1)

    # 看门狗与安全状态
    act_str = str(arm_out.get("action", "OK"))
    act_col = (0, 255, 120) if act_str in ("OK", "MOVE") else ((0, 160, 255) if act_str == "DECAY" else (0, 0, 255))
    status_txt = f"Watchdog: {act_str} | Scale: {float(arm_out.get('wd_scale', 1.0))*100:.0f}% | Gear: {gear_info.get('name', 'MID')}"
    cv2.putText(frame, status_txt, (lx0 + 10, ly0 + 142), cv2.FONT_HERSHEY_SIMPLEX, 0.40, act_col, 1)

    # 5. 右上机械臂 6 轴状态表
    arm_title = "ARM (6-DOF ZDT) [SIM]" if no_drive_arm else "ARM (6-DOF ZDT) [REAL]"
    cv2.putText(frame, arm_title, (rx0 + 10, ry0 + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 230, 255), 2)

    if joint_state is not None:
        for idx in range(6):
            col = idx % 2
            row = idx // 2
            x = rx0 + 10 + col * 160
            y = ry0 + 48 + row * 38
            q_val = float(joint_state.q[idx]) if idx < len(joint_state.q) else 0.0
            cur_val = float(joint_state.current_ma[idx]) if idx < len(joint_state.current_ma) else 0.0
            cv2.putText(frame, f"J{idx+1}: {q_val:6.1f} deg", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
            cv2.putText(frame, f"    {cur_val:4.0f} mA", (x, y + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (160, 220, 160), 1)

    # 6. 右下灵巧手 16 关节弯曲条形图
    hand_title = "LEAP HAND (16-DOF) [SIM]" if no_drive_hand else "LEAP HAND (16-DOF) [REAL]"
    cv2.putText(frame, hand_title, (hx0 + 10, hy0 + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 180), 2)

    for f_idx in range(4):
        fy = hy0 + 46 + f_idx * 56
        f_name = FINGER_NAMES[f_idx]
        is_bent = hand_bent[f_idx] if hand_bent and f_idx < len(hand_bent) else False
        f_col = (0, 255, 120) if is_bent else (200, 200, 200)
        cv2.putText(frame, f"{f_name} {'[BENT]' if is_bent else ''}", (hx0 + 10, fy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, f_col, 1)

        for j_idx in range(4):
            motor_id = f_idx * 4 + j_idx
            angle_val = float(hand_angles[motor_id]) if motor_id < len(hand_angles) else 0.0
            bar_w = int(np.clip(angle_val / 2.0, 0.0, 1.0) * 60)
            bx = hx0 + 58 + j_idx * 68
            by = fy + 8
            cv2.rectangle(frame, (bx, by), (bx + 60, by + 10), (45, 50, 55), -1)
            cv2.rectangle(frame, (bx, by), (bx + bar_w, by + 10), (0, 220, 255), -1)
            cv2.putText(frame, f"M{motor_id}", (bx, by - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (180, 180, 180), 1)

    # 7. 手腕锚定标记与动态运动引导箭头 (Wrist Movement Arrow)
    wrist_px = arm_out.get("wrist_px")
    if wrist_px is not None:
        u, v = int(wrist_px[0]), int(wrist_px[1])
        cv2.circle(frame, (u, v), 8, (0, 255, 0), -1)
        cv2.circle(frame, (u, v), 12, (255, 255, 255), 2)

        v_mag = float(np.linalg.norm(vel))
        if clutch_active and v_mag > 4.0:
            # 根据真实平移速度方向绘制黄色导引箭头
            dx = int(np.clip(vel[0] * 1.5, -80, 80))
            dy = int(np.clip(vel[1] * 1.5, -80, 80))
            if abs(dx) > 3 or abs(dy) > 3:
                cv2.arrowedLine(frame, (u, v), (u + dx, v + dy), (0, 255, 255), 3, tipLength=0.25)
                cv2.putText(frame, f"{v_mag:.0f} mm/s", (u + dx + 6, v + dy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)

    # 8. 底部操作提示栏
    cv2.putText(frame, "SPACE: Arm Pause/Resume | Z: Wrist Zero | W: Watchdog Reset | K: Hand Calib | P: Hand Power | S/TAB: Gear | M: Mode | Q: Quit",
                (12, h - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 200, 200), 1)


def main():
    ap = argparse.ArgumentParser(description="真机 6DOF 机械臂 + 16DOF 灵巧手视觉遥操统一管线")
    ap.add_argument("--iface", default="can0", help="SocketCAN 接口 (机械臂)")
    ap.add_argument("--hand-port", default=None, help="灵巧手串口路径 (默认从配置文件读取或自动扫描)")
    ap.add_argument("--config", default=str(Path(__file__).parent / "teleop_config.yaml"),
                    help="集中配置文件路径 (默认 teleop_config.yaml)")
    ap.add_argument("--calib", default=str(Path(__file__).parent / "handeye_calib.json"),
                    help="手眼标定矩阵路径")
    ap.add_argument("-y", "--gravity-confirm", action="store_true",
                    help="确认重力关节 J2/J3 二次确认 (机械臂真机驱动必须)")
    ap.add_argument("--no-drive-arm", action="store_true", help="机械臂空跑测试 (不连 CAN 总线)")
    ap.add_argument("--no-drive-hand", action="store_true", help="灵巧手空跑测试 (不连 Dynamixel 串口)")
    ap.add_argument("--mode", choices=["knead", "roll", "pitch", "full"], default="roll",
                    help="推拿遥操姿态模式: knead, roll, pitch, full")
    args = ap.parse_args()

    if not args.gravity_confirm and not args.no_drive_arm:
        sys.exit("遥操前必须 -y/--gravity-confirm 确认重力关节 (J2/J3) (空跑测试请加 --no-drive-arm)")

    # 1. 载入并验证全局集中配置 (起飞前安全检查)
    teleop_cfg = TeleopConfig.load(args.config)
    gear_configs = build_gear_configs(teleop_cfg)

    # 2. 机械臂适配器初始化
    joint_factors = teleop_cfg.joint_factor.as_list()
    if args.no_drive_arm:
        arm_adapter = NoDriveArmAdapter()
        print("[机械臂] 已启用 --no-drive-arm 空跑测试模式 (不发送 CAN 指令)")
    else:
        from lerobot_robot_massage.zdt.config import ZdtConfig
        from lerobot_robot_massage.zdt.controller import ZdtController
        ctrl = ZdtController(ZdtConfig(channel=args.iface,
                                       speed_rpm=teleop_cfg.motor.speed_rpm,
                                       position_acc=teleop_cfg.motor.position_acc,
                                       joint_speed_factors=joint_factors,
                                       max_vel_mm_s=teleop_cfg.motor.max_vel_mm_s,
                                       max_ang_rad_s=teleop_cfg.motor.max_ang_rad_s,
                                       max_joint_vel_deg_s=teleop_cfg.motor.max_joint_vel_deg_s,
                                       max_joint_acc_deg_s2=teleop_cfg.motor.max_joint_acc_deg_s2))
        arm_adapter = RealArmAdapter(ctrl, max_dq_deg=teleop_cfg.motor.max_dq_deg,
                                     joint_factors=joint_factors,
                                     ready_pose=teleop_cfg.pose.ready_pose_deg,
                                     home_pose=teleop_cfg.pose.home_pose_deg)

    # 3. 灵巧手适配器与映射器初始化
    if args.no_drive_hand:
        hand_adapter = NoDriveHandAdapter()
        hand_adapter.connect()
        print("[灵巧手] 已启用 --no-drive-hand 空跑测试模式 (不连接串口)")
    else:
        target_port = args.hand_port or teleop_cfg.hand.port
        hand_adapter = LeapHandAdapter(port=target_port)
        print(f"[灵巧手] 实体驱动就绪 (按 SPACE 延迟上电): port={target_port}")

    mapper = JointMapper()
    calibrator = Calibrator(mapper)
    finger_id = FingerIdentifier(mapper, bend_threshold=teleop_cfg.hand.bend_threshold)
    h3d = HaMeR3D()

    # 4. 视觉感知与手眼标定
    cam = open_realsense()
    if cam is None:
        sys.exit("[错误] 未检测到 RealSense D455 深度相机")
    tracker = HandTracker(max_num_hands=1, min_detection_confidence=0.5)

    calib_path = Path(args.calib)
    if calib_path.exists():
        from scripts.teleop.handeye_calib import load_calib
        r_cam_to_base = load_calib(calib_path)
    else:
        r_cam_to_base = np.eye(3)

    # 5. 滤波器与安全看门狗
    pts_filter = OneEuroFilter(n_joints=3, min_cutoff=teleop_cfg.vision.pts_min_cutoff, beta=teleop_cfg.vision.pts_beta)
    rot_filter = OneEuroFilter(n_joints=9, min_cutoff=teleop_cfg.vision.rot_min_cutoff, beta=teleop_cfg.vision.rot_beta)
    hand_angle_filter = OneEuroFilter(n_joints=16, min_cutoff=teleop_cfg.hand.filter_min_cutoff, beta=teleop_cfg.hand.filter_beta)
    pseudo_smoother = OneEuroFilter(n_joints=63, min_cutoff=1.5, beta=0.004)
    world_smoother = OneEuroFilter(n_joints=63, min_cutoff=1.2, beta=0.004)
    frame_smoother = OneEuroFilter(n_joints=9, min_cutoff=1.0, beta=0.005)

    watchdog = VisionWatchdog()

    # 6. 控制循环状态变量
    current_mode = [MODE_MAP.get(args.mode, MODE_ROLL)]
    current_gear = [teleop_cfg.gear.default_gear]
    clutch_active = [True]
    source_mode = [teleop_cfg.hand.source_mode]

    last_wrist = [None]
    last_t = [None]
    anchor_r_hand = [None]
    smooth_v_base = [np.zeros(3)]
    smooth_w_base = [np.zeros(3)]

    cached_joint_state = [None]
    last_joint_poll = [0.0]
    hand_angles = np.zeros(16, dtype=np.float64)
    hand_bent = [False, False, False, False]
    show_diag = False

    prev_loop_t = time.monotonic()
    fps = 0.0

    WIN_NAME = "TuinaDex — Arm & LeapHand Unified Visual Teleoperation"
    cv2.namedWindow(WIN_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN_NAME, 1280, 720)

    print("\n" + "=" * 75)
    print("  TuinaDex 机械臂-灵巧手协同视觉遥操统一系统 (全视野 1280x720 宽屏 HUD)")
    print("  SPACE: 暂停/跟随 | Z: 姿态回零 | W: 看门狗复位 | K: 灵巧手校准 | P: 灵巧手上电 | S/TAB: 换档 | M: 模式 | Q: 退出")
    print("=" * 75 + "\n")

    try:
        while True:
            now = time.monotonic()
            dt = now - prev_loop_t
            prev_loop_t = now
            fps = 0.9 * fps + 0.1 * (1.0 / max(dt, 1e-6))

            # ── 1. 单目视觉采集与感知 ────────────────────────────────
            ok, bgr, depth, K = cam.read_with_depth()
            if not ok or bgr is None:
                time.sleep(0.01)
                continue

            # 同步镜像翻转 BGR 与 Depth，确保全局视野深度反投影完全对齐 (不再局限于中心区域)
            frame = cv2.flip(bgr, 1)
            depth_flipped = cv2.flip(depth, 1) if depth is not None else None

            # 内参主点 cx 对应水平镜像变换: cx_mirrored = depth_w - cx
            if K is not None and depth is not None:
                fx, fy, cx, cy = K
                K_mirrored = (fx, fy, float(depth.shape[1] - cx), cy)
            else:
                K_mirrored = K

            if frame.shape[1] != 1280 or frame.shape[0] != 720:
                frame = cv2.resize(frame, (1280, 720), interpolation=cv2.INTER_LINEAR)
            h, w = frame.shape[:2]
            results = tracker.detect(frame)

            hand_detected = False
            arm_action = "MOVE"
            scaled_v = np.zeros(3)
            scaled_w = np.zeros(3)
            roll_deg = 0.0
            pitch_deg = 0.0
            wrist_px = None

            if results:
                # 匹配物理操作手 (默认右手)
                target_hand_label = _MIRRORED_LABEL.get(teleop_cfg.hand.hand_type, teleop_cfg.hand.hand_type)
                matched_hand = None
                for r in results:
                    if teleop_cfg.hand.hand_type == "first" or r.handedness.lower() == target_hand_label:
                        matched_hand = r
                        break

                if matched_hand is not None:
                    hand_detected = True
                    wrist_px = (int(matched_hand.landmarks[0].x * w), int(matched_hand.landmarks[0].y * h))
                    hand_adapter.reset_loss_state()
                    frame = tracker.draw_landmarks(frame, [matched_hand])
                    mp_pts = tracker.landmark_xy(matched_hand, (h, w))

                    # ── 2. 机械臂手腕 6DOF 解算 ──
                    palm_pts = build_palm_pts(matched_hand, depth_flipped, K_mirrored)
                    if palm_pts is not None:
                        palm_wrist_mm = palm_pts[0]
                        # 3D 腕部位置滤波与速度积分
                        filt_wrist = pts_filter(palm_wrist_mm)
                        if last_wrist[0] is not None and last_t[0] is not None:
                            v_dt = max(1e-4, now - last_t[0])
                            v_cam = (filt_wrist - last_wrist[0]) / v_dt
                            v_base = r_cam_to_base @ v_cam

                            # 档位比例与增益
                            curr_g = gear_configs[current_gear[0]]
                            v_scale = curr_g["lin_scale"]
                            v_norm = float(np.linalg.norm(v_base))
                            if v_norm < teleop_cfg.vision.deadband_vel_mm_s:
                                v_base = np.zeros(3)
                            else:
                                if v_norm > 150.0:
                                    v_base = v_base * curr_g["gain_xyz"]
                                v_base = v_base * v_scale

                            smooth_v_base[0] = 0.65 * smooth_v_base[0] + 0.35 * v_base
                        last_wrist[0] = filt_wrist
                        last_t[0] = now

                        # 掌面倾角解算姿态摇杆角速度
                        p_wrist = palm_pts[0]
                        p_idx = palm_pts[5]
                        p_mid = palm_pts[9]
                        p_pky = palm_pts[17]
                        v_mid = p_mid - p_wrist
                        v_lat = p_pky - p_idx
                        n_mid = np.linalg.norm(v_mid)
                        n_lat = np.linalg.norm(v_lat)
                        if n_mid > 1e-6 and n_lat > 1e-6:
                            y_dir = v_mid / n_mid
                            z_norm = np.cross(v_lat / n_lat, y_dir)
                            nz = np.linalg.norm(z_norm)
                            if nz > 1e-6:
                                z_norm /= nz
                                x_dir = np.cross(y_dir, z_norm)
                                r_raw = np.column_stack([x_dir, y_dir, z_norm])
                                r_filt = rot_filter(r_raw.reshape(-1)).reshape(3, 3)
                                u_mat, _, vt_mat = np.linalg.svd(r_filt)
                                r_palm = u_mat @ vt_mat

                                # 姿态回零与相对旋转基准 (Wrist Attitude Zero Calibration)
                                if anchor_r_hand[0] is None or not clutch_active[0]:
                                    anchor_r_hand[0] = r_palm.copy()
                                    r_rel = np.eye(3)
                                else:
                                    r_rel = r_palm @ anchor_r_hand[0].T

                                # 提取相对于中立基准的偏角 (度) — 修复符号调换: 下压为负, 上抬为正
                                roll_deg = float(np.degrees(np.arctan2(r_rel[2, 0], r_rel[2, 2])))
                                pitch_deg = - float(np.degrees(np.arctan2(r_rel[2, 1], r_rel[2, 2])))

                                # 模式角度解算
                                curr_g = gear_configs[current_gear[0]]
                                max_omega_val = curr_g["max_omega"]
                                deadband_deg = teleop_cfg.vision.deadband_angle_deg

                                # 虚拟摇杆速率响应
                                def _joy_rate(ang_deg: float) -> float:
                                    abs_a = abs(ang_deg)
                                    if abs_a <= deadband_deg:
                                        return 0.0
                                    ratio = min(1.0, (abs_a - deadband_deg) / max(1.0, 28.0 - deadband_deg))
                                    return float(np.sign(ang_deg) * (ratio ** 1.4) * max_omega_val)

                                w_cmd = np.zeros(3)
                                if current_mode[0] == MODE_ROLL:
                                    w_cmd[0] = _joy_rate(roll_deg)
                                elif current_mode[0] == MODE_PITCH:
                                    w_cmd[1] = _joy_rate(pitch_deg)
                                elif current_mode[0] == MODE_FULL:
                                    w_cmd[0] = _joy_rate(roll_deg)
                                    w_cmd[1] = _joy_rate(pitch_deg)

                                smooth_w_base[0] = 0.70 * smooth_w_base[0] + 0.30 * (r_cam_to_base @ w_cmd)

                    # ── 3. 灵巧手五指 16DOF 关节角解算 ──
                    if source_mode[0] == 1 and matched_hand.world_landmarks is not None:
                        wpts = np.array([[lm.x, lm.y, lm.z] for lm in matched_hand.world_landmarks], dtype=np.float64)
                        pts = world_smoother(wpts.reshape(-1)).reshape(21, 3)
                        p_frame = _smoothed_frame(pts, frame_smoother)
                        raw_angles = calibrator.map_points(pts, frame=p_frame)
                        hand_bent, _ = finger_id.identify_points(pts)
                    else:
                        # 默认 MediaPipe 伪 3D 模式 (跟手性与握拳最佳)
                        npts = np.array([[lm.x, lm.y, lm.z] for lm in matched_hand.landmarks], dtype=np.float64)
                        pts = pseudo_smoother(npts.reshape(-1)).reshape(21, 3)
                        p_frame = _smoothed_frame(pts, frame_smoother)
                        raw_angles = calibrator.map_points(pts, frame=p_frame)
                        hand_bent, _ = finger_id.identify_points(pts)

                    hand_angles = hand_angle_filter(raw_angles)

                    # 下发灵巧手舵机目标
                    hand_adapter.set_angles(hand_angles)

            # ── 4. 手部丢失/遮挡保护 ──────────────────────────────────
            if not hand_detected:
                last_wrist[0] = None
                last_t[0] = None
                smooth_v_base[0] = np.zeros(3)
                smooth_w_base[0] = np.zeros(3)
                pseudo_smoother.reset()
                world_smoother.reset()
                frame_smoother.reset()
                hand_angle_filter.reset()

                # 灵巧手平滑回全开安全位
                hand_adapter.relax_step(now)

            # ── 5. 看门狗检测与机械臂下发 ────────────────────────────
            action, wd_scale = watchdog.update(
                hand_present=hand_detected,
                hand_confidence=1.0 if hand_detected else 0.0,
                depth_valid=True if hand_detected else False,
                wrist_mm=last_wrist[0],
                now=now,
            )

            if clutch_active[0] and action != WatchdogAction.STOP and arm_adapter.state() != "STOPPED":
                scaled_v = smooth_v_base[0] * wd_scale
                scaled_w = smooth_w_base[0] * wd_scale
                arm_cmd = CartesianCommand(
                    (float(scaled_v[0]), float(scaled_v[1]), float(scaled_v[2])),
                    (float(scaled_w[0]), float(scaled_w[1]), float(scaled_w[2])),
                    timestamp=now,
                )
                arm_adapter.move_cartesian_velocity(arm_cmd)
            elif action == WatchdogAction.ESTOP:
                arm_adapter.e_stop()
                clutch_active[0] = False

            # ── 6. 状态轮询与界面渲染 ────────────────────────────────
            if now - last_joint_poll[0] >= 0.08:
                try:
                    cached_joint_state[0] = arm_adapter.get_joint_state()
                except Exception:
                    pass
                last_joint_poll[0] = now

            _draw_unified_dashboard(
                frame=frame,
                arm_out={
                    "action": action.name,
                    "v": scaled_v,
                    "w": scaled_w,
                    "d_roll_deg": roll_deg,
                    "d_pitch_deg": pitch_deg,
                    "wrist_px": wrist_px,
                    "wd_scale": wd_scale,
                },
                joint_state=cached_joint_state[0],
                hand_angles=hand_angles,
                hand_bent=hand_bent,
                hand_state_str=hand_adapter.state(),
                clutch_active=clutch_active[0],
                arm_mode=current_mode[0],
                arm_gear=current_gear[0],
                gear_configs=gear_configs,
                source_name=SOURCE_NAMES.get(source_mode[0], "PSEUDO-3D"),
                fps=fps,
                no_drive_arm=args.no_drive_arm,
                no_drive_hand=args.no_drive_hand,
            )

            cv2.imshow(WIN_NAME, frame)

            # ── 7. 统一键盘交互分发 ──────────────────────────────────
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), ord("Q"), 27):
                print("[系统] 用户请求退出，安全停机...")
                break
            elif k == ord(" "):
                # SPACE: 机械臂遥操暂停 / 恢复跟随 (Clutch Toggle)
                clutch_active[0] = not clutch_active[0]
                anchor_r_hand[0] = None
                smooth_v_base[0] = np.zeros(3)
                smooth_w_base[0] = np.zeros(3)
                print(f"[机械臂遥操] {'已恢复跟随 (RUN)' if clutch_active[0] else '已暂停锁定 (PAUSE)'}")
            elif k in (ord("z"), ord("Z")):
                # Z: 机械臂手腕姿态回零校准 (将当前姿态设为 0° 中立基准)
                anchor_r_hand[0] = None
                smooth_w_base[0] = np.zeros(3)
                print("\n  *** 机械臂手腕姿态已回零校准 (当前手势设为 0° 中立基准)! ***\n")
            elif k in (ord("w"), ord("W")):
                # W: 视觉看门狗手动复位并重新使能 (Watchdog Manual Reset & Re-arm)
                watchdog.reset()
                anchor_r_hand[0] = None
                smooth_v_base[0] = np.zeros(3)
                smooth_w_base[0] = np.zeros(3)
                last_wrist[0] = None
                if arm_adapter.state() == "STOPPED":
                    arm_adapter.arm(gravity_confirmed=True)
                print("\n  *** 视觉看门狗已手动复位并重新使能 (Watchdog Reset & Re-armed)! ***\n")
            elif k in (ord("k"), ord("K")):
                # K: 灵巧手张开全开校准
                if hand_detected and 'pts' in locals() and 'p_frame' in locals():
                    calibrator.calibrate_points(pts, frame=p_frame)
                    hand_angle_filter.reset()
                    print("\n  *** 灵巧手五指全开校准完成! ***\n")
            elif k in (ord("p"), ord("P")):
                # P: 灵巧手延迟上电
                if not args.no_drive_hand and not hand_adapter.is_connected():
                    hand_adapter.connect()
            elif k in (ord("s"), ord("S"), 9):  # TAB or S: 切换机械臂档位
                current_gear[0] = (current_gear[0] % 3) + 1
                anchor_r_hand[0] = None
                smooth_v_base[0] = np.zeros(3)
                smooth_w_base[0] = np.zeros(3)
                print(f"[档位切换] 机械臂灵敏度: {gear_configs[current_gear[0]]['name']}")
            elif k in (ord("m"), ord("M")):
                current_mode[0] = (current_mode[0] % 4) + 1
                anchor_r_hand[0] = None
                smooth_w_base[0] = np.zeros(3)
                print(f"[模式切换] 机械臂推拿模式: {MODE_NAMES[current_mode[0]]}")
            elif k in (ord("c"), ord("C"), ord("f"), ord("F")):
                clutch_active[0] = not clutch_active[0]
                anchor_r_hand[0] = None
                smooth_v_base[0] = np.zeros(3)
                smooth_w_base[0] = np.zeros(3)
                print(f"[离合切换] 离合器状态: {'已激活 (CLUTCH ON)' if clutch_active[0] else '已冻结 (FREEZE)'}")
            elif k in (ord("r"), ord("R")):
                print("[姿态] 机械臂安全运动至按摩准备姿态 (READY)，灵巧手张开...")
                arm_adapter.ready()
                hand_adapter.set_open()
            elif k in (ord("h"), ord("H"), ord("o"), ord("O")):
                print("[姿态] 机械臂安全运动至上电初始姿态 (HOME)，灵巧手张开...")
                arm_adapter.home()
                hand_adapter.set_open()

    except KeyboardInterrupt:
        print("\n[系统] 检测到 Ctrl+C 中断信号，正在优雅停机...")
    finally:
        try:
            arm_adapter.disconnect()
        except Exception:
            pass
        try:
            hand_adapter.disconnect()
        except Exception:
            pass
        tracker.close()
        cam.release()
        cv2.destroyAllWindows()
        print("[系统] 机械臂与灵巧手已完全安全断开，系统退出完成。")


if __name__ == "__main__":
    main()
