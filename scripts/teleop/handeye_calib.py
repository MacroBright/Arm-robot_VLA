"""手眼标定：相机系→机器人基座系的旋转 R（差分遥操只需旋转）。

方式 A: 直接填相机安装欧拉角 → rot_from_euler。
方式 B: N 点 Procrustes（≥4 非共面）：手到已知物理位置 + 臂端对应位置。
方式 C: 轴对齐向导 (Arm 侧 demo_arm_teleop.py 中通过 K 键流程调用)。

历史路径: 此文件原位于 Leap_Hand/python/gesture_mapping/handeye_calib.py,
2026-08 迁移到本仓. apply_rotation() 函数已被内联到
Leap_Hand/python/gesture_mapping/wrist_tracker.py (那里只有它被共享
调用, 不再需要跨仓 sys.path 注入).
"""
import json
from pathlib import Path
from typing import Union

import numpy as np

_Path = Union[str, Path]


def rot_from_euler(rx_deg: float, ry_deg: float, rz_deg: float) -> np.ndarray:
    """绕 X→Y→Z（相机系）的旋转矩阵 R(3,3)。列向量应用: v_base = R @ v_cam。"""
    rx, ry, rz = np.radians([rx_deg, ry_deg, rz_deg])
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def procrustes_rotation(src_pts, dst_pts) -> np.ndarray:
    """最小化 Σ||R@p_i − q_i||² 的旋转 R。src/dst: (N,3)。返回 R(3,3)。"""
    src = np.asarray(src_pts, float).T   # (3,N)
    dst = np.asarray(dst_pts, float).T   # (3,N)
    H = src @ dst.T
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    return R


def apply_rotation(R: np.ndarray, pts) -> np.ndarray:
    """R(3,3) 作用于 (N,3) 点集（每行一个列向量）。"""
    pts = np.asarray(pts, float)
    return (R @ pts.T).T


def save_calib(path: _Path, R: np.ndarray) -> None:
    Path(path).write_text(json.dumps({"R": np.asarray(R).tolist()}))


def load_calib(path: _Path) -> np.ndarray:
    data = json.loads(Path(path).read_text())
    return np.array(data["R"])


# 基座方向码: 1=+X 2=-X 3=+Y 4=-Y 5=+Z(上) 6=-Z(下)
_BASE_DIR_CODES = {
    1: np.array([1.0, 0.0, 0.0]), 2: np.array([-1.0, 0.0, 0.0]),
    3: np.array([0.0, 1.0, 0.0]), 4: np.array([0.0, -1.0, 0.0]),
    5: np.array([0.0, 0.0, 1.0]), 6: np.array([0.0, 0.0, -1.0]),
}


def solve_handeye(cam_dirs, base_codes):
    """从 (相机系单位方向, 基座方向码) 配对解手眼旋转 R。

    cam_dirs: (N,3) 相机系单位方向（操作者沿某方向挥手的位移方向）
    base_codes: 长度 N 的 int 列表，操作者指定"臂应去"的基座方向码(1-6)
    返回 R(3,3) 使 R @ cam_dir_i ≈ base_dir(base_codes[i])。
    """
    src = np.asarray(cam_dirs, float)
    dst = np.array([_BASE_DIR_CODES[c] for c in base_codes], float)
    return procrustes_rotation(src, dst)


def main():
    import argparse
    import sys
    import cv2

    sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "Leap_Hand" / "python"))
    from gesture_mapping.camera import open_realsense
    from gesture_mapping.hand_tracker import HandTracker
    from gesture_mapping.wrist_tracker import build_palm_pts

    ap = argparse.ArgumentParser(description="手眼标定向导 (相机系 -> 机器人基座系 R 旋转标定)")
    ap.add_argument("--out", default=str(Path(__file__).parent / "handeye_calib.json"),
                    help="标定输出文件路径")
    args = ap.parse_args()

    print("=" * 60)
    print("【TuinaDex 手眼标定向导】")
    print("目标: 求解 RealSense D455 相机系到机械臂基座系的 3D 旋转矩阵 R")
    print("=" * 60)

    cam = open_realsense()
    if cam is None:
        sys.exit("错误: 未检测到 RealSense 相机，请检查 USB 连接。")

    tracker = HandTracker(max_num_hands=1)
    win_name = "Hand-Eye Calibration Wizard"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, 960, 720)

    steps = [
        ("Step 1/3: 请沿机械臂 +X 方向 (右方) 平移手掌约 15cm", 1),
        ("Step 2/3: 请沿机械臂 +Y 方向 (前方) 平移手掌约 15cm", 3),
        ("Step 3/3: 请沿机械臂 +Z 方向 (上方) 平移手掌约 15cm", 5),
    ]

    current_step = 0
    calib_cam_dirs = []
    calib_codes = []
    history_pts = []
    is_recording = False

    print("\n按 SPACE 开始记录当前步骤的手部轨迹，平移手掌约 15~20cm 后再次按 SPACE 确认。")
    print("按 Q 或 ESC 随时退出。\n")

    try:
        while current_step < len(steps):
            ok, bgr, depth, K = cam.read_with_depth()
            if not ok or bgr is None:
                continue

            hands = tracker.detect(bgr)
            wrist_cam = None
            if hands:
                bgr = tracker.draw_landmarks(bgr, hands)
                pts = build_palm_pts(hands[0], depth, K)
                if pts is not None:
                    wrist_cam = pts[0]

            h, w = bgr.shape[:2]

            # 顶部 HUD 提示
            step_title, target_code = steps[current_step]
            cv2.rectangle(bgr, (0, 0), (w, 55), (30, 30, 30), -1)
            cv2.putText(bgr, f"[{step_title}]", (15, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 255), 2)

            if not is_recording:
                hint_txt = "Press [SPACE] to start recording motion vector"
                cv2.putText(bgr, hint_txt, (15, 48),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            else:
                hint_txt = f"RECORDING... Move Hand now! Samples: {len(history_pts)} | Press [SPACE] to save"
                cv2.putText(bgr, hint_txt, (15, 48),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                if wrist_cam is not None:
                    history_pts.append(wrist_cam)

            cv2.imshow(win_name, bgr)
            k = cv2.waitKey(1) & 0xFF

            if k in (ord("q"), ord("Q"), 27):
                print("[退出] 标定已取消。")
                return

            if k == ord(" "):
                if not is_recording:
                    history_pts = []
                    is_recording = True
                    print(f"[{step_title}] 开始记录轨迹，请平移手部...")
                else:
                    is_recording = False
                    if len(history_pts) < 5:
                        print("采样点过少，请重新按 SPACE 录制。")
                    else:
                        d = history_pts[-1] - history_pts[0]
                        norm = np.linalg.norm(d)
                        if norm < 25.0:
                            print(f"手部位移过小 ({norm:.1f} mm < 25 mm)，请重新大幅度平移手部。")
                        else:
                            unit_dir = d / norm
                            calib_cam_dirs.append(unit_dir)
                            calib_codes.append(target_code)
                            print(f"[{step_title}] 采集成功! 相机系位移向量: {np.round(unit_dir, 3)}, 距离: {norm:.1f} mm")
                            current_step += 1

        # 3 个方向均采集完成，解算 R 矩阵
        R = solve_handeye(calib_cam_dirs, calib_codes)
        save_calib(args.out, R)
        print("\n" + "=" * 60)
        print("【标定成功！】手眼旋转矩阵 R 已保存至:", args.out)
        print("R (相机系 -> 机器人基座系):\n", np.round(R, 4))
        print("=" * 60)

        done_img = np.zeros((720, 960, 3), dtype=np.uint8)
        cv2.putText(done_img, "CALIBRATION SUCCESSFUL!", (200, 250),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
        cv2.putText(done_img, f"Matrix R saved to: {args.out}", (150, 320),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(done_img, "Press any key to exit", (320, 420),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 220, 255), 2)
        cv2.imshow(win_name, done_img)
        cv2.waitKey(2000)

    finally:
        cv2.destroyAllWindows()
        tracker.close()


if __name__ == "__main__":
    main()
