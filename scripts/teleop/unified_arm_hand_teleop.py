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
    """在一帧 960x720 画面上绘制机械臂 + 灵巧手一体化综合 HUD 监控看板."""
    h, w = frame.shape[:2]
    gear_info = gear_configs.get(arm_gear, gear_configs[2])

    # 1. 顶部半透明全局状态栏 (Header Bar)
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (w, 52), (18, 18, 24), -1)
    cv2.addWeighted(overlay, 0.75, frame, 0.25, 0, frame)

    # 离合器标签
    if clutch_active:
        cv2.rectangle(frame, (10, 10), (135, 42), (0, 180, 80), -1)
        cv2.putText(frame, "CLUTCH ON", (18, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    else:
        cv2.rectangle(frame, (10, 10), (135, 42), (40, 40, 180), -1)
        cv2.putText(frame, "FREEZE (F)", (15, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

    # 机械臂模式与档位
    cv2.putText(frame, f"ARM: {MODE_NAMES[arm_mode]}", (150, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 220, 0), 2)
    badge_col = gear_info.get("color", (0, 220, 255))
    cv2.rectangle(frame, (430, 10), (510, 42), (30, 30, 30), -1)
    cv2.rectangle(frame, (430, 10), (510, 42), badge_col, 2)
    cv2.putText(frame, gear_info.get("badge", "MID"), (440, 33), cv2.FONT_HERSHEY_SIMPLEX, 0.6, badge_col, 2)

    # 灵巧手状态与 3D 源
    hand_col = (0, 255, 100) if "POWERED" in hand_state_str else (120, 120, 120)
    cv2.putText(frame, f"HAND: {hand_state_str}", (530, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.55, hand_col, 2)
    cv2.putText(frame, f"3D: {source_name}", (710, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 200, 255), 1)
    cv2.putText(frame, f"{fps:3.0f}FPS", (w - 75, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 2)

    # 2. 右侧机械臂 6 轴状态表 (Top-Right Subpanel)
    box_w, box_h = 295, 140
    rx0, ry0 = w - box_w - 12, 62
    cv2.rectangle(overlay, (rx0, ry0), (rx0 + box_w, ry0 + box_h), (15, 15, 20), -1)
    cv2.addWeighted(overlay, 0.70, frame, 0.30, 0, frame)
    cv2.rectangle(frame, (rx0, ry0), (rx0 + box_w, ry0 + box_h), (80, 80, 80), 1)

    arm_title = "ARM (6-DOF ZDT) [SIM]" if no_drive_arm else "ARM (6-DOF ZDT) [REAL]"
    cv2.putText(frame, arm_title, (rx0 + 8, ry0 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 230, 255), 1)

    if joint_state is not None:
        for idx in range(6):
            col = idx % 2
            row = idx // 2
            x = rx0 + 8 + col * 142
            y = ry0 + 38 + row * 34
            q_val = float(joint_state.q[idx]) if idx < len(joint_state.q) else 0.0
            cur_val = float(joint_state.current_ma[idx]) if idx < len(joint_state.current_ma) else 0.0
            cv2.putText(frame, f"J{idx+1}: {q_val:6.1f} deg", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (220, 220, 220), 1)
            cv2.putText(frame, f"    {cur_val:4.0f} mA", (x, y + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (160, 200, 160), 1)

    # 3. 右侧灵巧手 16 关节弯曲条形图 (Bottom-Right Subpanel)
    hx0, hy0 = w - box_w - 12, ry0 + box_h + 10
    h_box_h = 240
    cv2.rectangle(overlay, (hx0, hy0), (hx0 + box_w, hy0 + h_box_h), (15, 15, 20), -1)
    cv2.addWeighted(overlay, 0.70, frame, 0.30, 0, frame)
    cv2.rectangle(frame, (hx0, hy0), (hx0 + box_w, hy0 + h_box_h), (80, 80, 80), 1)

    hand_title = "LEAP HAND (16-DOF) [SIM]" if no_drive_hand else "LEAP HAND (16-DOF) [REAL]"
    cv2.putText(frame, hand_title, (hx0 + 8, hy0 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 180), 1)

    # 绘制 4 根手指 16 关节柱状图 (每指 4 个关节)
    for f_idx in range(4):
        fy = hy0 + 38 + f_idx * 48
        f_name = FINGER_NAMES[f_idx]
        is_bent = hand_bent[f_idx] if hand_bent and f_idx < len(hand_bent) else False
        f_col = (0, 230, 100) if is_bent else (180, 180, 180)
        cv2.putText(frame, f"{f_name} {'[BENT]' if is_bent else ''}", (hx0 + 8, fy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, f_col, 1)

        for j_idx in range(4):
            motor_id = f_idx * 4 + j_idx
            angle_val = float(hand_angles[motor_id]) if motor_id < len(hand_angles) else 0.0
            # 柱状图条 (0.0 ~ 2.0 rad 归一化)
            bar_w = int(np.clip(angle_val / 2.0, 0.0, 1.0) * 55)
            bx = hx0 + 48 + j_idx * 60
            by = fy + 6
            cv2.rectangle(frame, (bx, by), (bx + 55, by + 10), (50, 50, 50), -1)
            cv2.rectangle(frame, (bx, by), (bx + bar_w, by + 10), (0, 200, 255), -1)
            cv2.putText(frame, f"M{motor_id}", (bx, by - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.28, (150, 150, 150), 1)

    # 4. 底部操作提示栏
    cv2.putText(frame, "SPACE:Calib/Power | S/TAB:Gear | M:Mode | R:Ready | H:Home | C:Clutch | Q:Quit",
                (12, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (180, 180, 180), 1)


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
    cv2.resizeWindow(WIN_NAME, 960, 720)

    print("\n" + "=" * 65)
    print("  TuinaDex 机械臂-灵巧手协同视觉遥操统一系统已就绪")
    print("  SPACE: 校准灵巧手/上电 | S/TAB: 换档 | M: 换模式 | C: 离合 | Q: 退出")
    print("=" * 65 + "\n")

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

            frame = cv2.flip(bgr, 1)  # 镜像
            h, w = frame.shape[:2]
            results = tracker.detect(frame)

            hand_detected = False
            arm_action = "MOVE"
            scaled_v = np.zeros(3)
            scaled_w = np.zeros(3)

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
                    hand_adapter.reset_loss_state()
                    frame = tracker.draw_landmarks(frame, [matched_hand])
                    mp_pts = tracker.landmark_xy(matched_hand, (h, w))

                    # ── 2. 机械臂手腕 6DOF 解算 ──
                    palm_wrist_mm, palm_pts = build_palm_pts(matched_hand, depth, K, w)
                    if palm_wrist_mm is not None:
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
                        if palm_pts is not None and len(palm_pts) >= 4:
                            p_wrist, p_idx, p_mid, p_pky = palm_pts[:4]
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
                                    u, _, vt = np.linalg.svd(r_filt)
                                    r_palm = u @ vt

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

                                    roll_deg = float(np.degrees(np.arctan2(r_palm[2, 0], r_palm[2, 2])))
                                    pitch_deg = float(np.degrees(np.arctan2(r_palm[2, 1], r_palm[2, 2])))

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
            action, wd_scale = watchdog.check(
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
                arm_out={"action": action.name, "v": scaled_v, "w": scaled_w},
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
                # SPACE: 张开手全开校准 + 灵巧手延迟上电
                if not args.no_drive_hand and not hand_adapter.is_connected():
                    hand_adapter.connect()
                if hand_detected and 'pts' in locals() and 'p_frame' in locals():
                    calibrator.calibrate_points(pts, frame=p_frame)
                    hand_angle_filter.reset()
                    print("\n  *** 灵巧手全开校准完成! ***\n")
            elif k in (ord("s"), ord("S"), 9):  # TAB or S: 切换机械臂档位
                current_gear[0] = (current_gear[0] % 3) + 1
                smooth_v_base[0] = np.zeros(3)
                smooth_w_base[0] = np.zeros(3)
                print(f"[档位切换] 机械臂灵敏度: {gear_configs[current_gear[0]]['name']}")
            elif k in (ord("m"), ord("M")):
                current_mode[0] = (current_mode[0] % 4) + 1
                smooth_w_base[0] = np.zeros(3)
                print(f"[模式切换] 机械臂推拿模式: {MODE_NAMES[current_mode[0]]}")
            elif k in (ord("c"), ord("C"), ord("f"), ord("F")):
                clutch_active[0] = not clutch_active[0]
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
