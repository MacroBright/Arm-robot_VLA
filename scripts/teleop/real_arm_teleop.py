"""scripts/teleop/real_arm_teleop.py — 真机 6DOF 视觉遥操入口 (spec TASK-23).

管线: RealSense + HandTracker + WristTracker → CartesianCommand → VisionWatchdog
(分级) → RealArmAdapter → CartesianController → ZdtController → CAN.
控制层是陈旧命令最终权威 (step 的 cmd_ts 单调期限); 本入口只做视觉分级 + 组装.
按键: H=clutch, R=reset/ready, Y=e_stop, Q/ESC=安全退出.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from arm_adapter import RealArmAdapter  # noqa: E402
from lerobot_robot_massage.zdt.recording import EpisodeRecorder  # noqa: E402
from lerobot_robot_massage.zdt.types import CartesianCommand  # noqa: E402
from watchdog import VisionWatchdog, WatchdogAction  # noqa: E402


class RealArmTeleop:
    """一帧遥操逻辑 (可无相机单测): hand_provider → cmd → watchdog → clutch/ramp → adapter."""

    def __init__(self, adapter, watchdog, recorder, hand_provider, key_provider,
                 stale_cmd_max_s: float = 0.25,
                 ramp_up_s: float = 0.3,
                 deadband_mm_s: float = 3.0,
                 default_clutch_active: bool = True):
        self.adapter = adapter
        self.watchdog = watchdog
        self.recorder = recorder
        self.hand_provider = hand_provider      # () -> dict | None
        self.key_provider = key_provider        # () -> key or None
        self.stale_cmd_max_s = stale_cmd_max_s
        self.ramp_up_s = ramp_up_s
        self.deadband_mm_s = deadband_mm_s
        self.clutch_active = default_clutch_active
        self._clutch_on_time = -1e9             # 初始默认已完成热身, toggle 后从当前时间起算
        self._cmd = CartesianCommand((0.0, 0.0, 0.0))

    def toggle_clutch(self, now: float) -> bool:
        """切换离合器状态 (True=接合运动, False=暂停离合)."""
        self.clutch_active = not self.clutch_active
        if self.clutch_active:
            self._clutch_on_time = now
        return self.clutch_active

    def run_once(self, cmd_ts: float, now: float) -> dict:
        """跑一帧: 返回 {action, cmd, phase, clutch}. 调用方提供时间 (单调)."""
        key = self.key_provider()
        if key in (ord("y"), ord("Y")):
            self.adapter.e_stop()
            return {"action": "ESTOP", "cmd": self._cmd,
                    "phase": self.adapter.state(),
                    "clutch": self.clutch_active}
        if key in (ord("q"), ord("Q"), 27):
            return {"action": "QUIT", "cmd": self._cmd,
                    "phase": self.adapter.state(),
                    "clutch": self.clutch_active}
        if key in (ord("r"), ord("R")):
            self.adapter.ready()
            return {"action": "READY", "cmd": CartesianCommand((0.0, 0.0, 0.0)),
                    "phase": self.adapter.state(),
                    "clutch": self.clutch_active}
        if key in (ord("o"), ord("O"), ord("0"), ord("h"), ord("H")):
            self.adapter.home()
            return {"action": "HOME", "cmd": CartesianCommand((0.0, 0.0, 0.0)),
                    "phase": self.adapter.state(),
                    "clutch": self.clutch_active}
        if key == 32:  # SPACE bar: Toggle clutch
            self.toggle_clutch(now)

        if not self.clutch_active:
            scaled = CartesianCommand((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), timestamp=cmd_ts)
            self._record(None, scaled, "PAUSED")
            return {"action": "PAUSED", "cmd": scaled,
                    "phase": self.adapter.state(),
                    "clutch": False}

        if (now - cmd_ts) > self.stale_cmd_max_s:
            scaled = CartesianCommand((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), timestamp=cmd_ts)
            self._record(None, scaled, "STOP")
            return {"action": "STOP", "cmd": scaled,
                    "phase": self.adapter.state(),
                    "clutch": True}

        hand = self.hand_provider()
        cmd = self._build_command(hand, cmd_ts)
        action, wd_scale = self.watchdog.update(
            hand_present=bool(hand and hand.get("hand_present")),
            hand_confidence=float(hand.get("confidence", 0.0)) if hand else 0.0,
            depth_valid=bool(hand and hand.get("depth_valid")),
            wrist_mm=hand.get("wrist_mm") if hand else None,
            now=now)

        # 开启缓起动 (Ramp-up) 线性增益
        dt_on = max(0.0, now - self._clutch_on_time)
        ramp_scale = min(1.0, dt_on / self.ramp_up_s) if self.ramp_up_s > 0 else 1.0
        total_scale = wd_scale * ramp_scale

        scaled = CartesianCommand(
            tuple(float(v) * total_scale for v in cmd.linear_velocity),
            tuple(float(w) * total_scale for w in cmd.angular_velocity),
            timestamp=cmd.timestamp)
        if action == WatchdogAction.ESTOP:
            self.adapter.e_stop()
        elif action != WatchdogAction.STOP:
            self.adapter.move_cartesian_velocity(scaled)

        self._record(hand, scaled, action.name)
        return {"action": action.name, "cmd": scaled,
                "phase": self.adapter.state(),
                "clutch": True}

    def _build_command(self, hand, cmd_ts: float) -> CartesianCommand:
        """由手部信息合成 CartesianCommand (速度增量直接积分模式 + 死区过滤)."""
        if hand is None:
            return CartesianCommand((0.0, 0.0, 0.0), timestamp=cmd_ts)
        v = list(hand.get("velocity") or (0.0, 0.0, 0.0))
        w = list(hand.get("angular_velocity") or (0.0, 0.0, 0.0))
        v_norm = (v[0]**2 + v[1]**2 + v[2]**2)**0.5
        if v_norm < self.deadband_mm_s:
            v = [0.0, 0.0, 0.0]
        return CartesianCommand(tuple(float(x) for x in v),
                                tuple(float(x) for x in w),
                                timestamp=cmd_ts)

    def _record(self, hand, cmd: CartesianCommand, action: str) -> None:
        obs = {
            "q": [0.0] * 6, "dq": [0.0] * 6, "current": [],
            "ee_pose": {"position": [0.0, 0.0, 0.0],
                        "quaternion": [1.0, 0.0, 0.0, 0.0]},
            "hand_pose": ({"position": list(hand.get("wrist_mm") or (0, 0, 0)),
                           "orientation": [0.0, 0.0, 0.0],
                           "confidence": hand.get("confidence", 0.0)}
                          if hand else {"position": [], "orientation": [],
                                        "confidence": 0.0}),
        }
        act = {"cartesian_command": {"linear_velocity": list(cmd.linear_velocity),
                                     "angular_velocity": list(cmd.angular_velocity),
                                     "timestamp": cmd.timestamp},
               "commanded_joint_target": [0.0] * 6}
        saf = {"phase": self.adapter.state(), "action": action}
        self.recorder.add_record(obs, act, saf)


def _draw_overlay(bgr, out: dict, hand_info: dict | None, clutch_active: bool) -> None:
    """在 OpenCV 画面上绘制丰富图元 (状态横幅 + 离合徽标 + 锚定球 + 速度矢量)."""
    import cv2
    h, w = bgr.shape[:2]
    # 顶部状态横幅
    if clutch_active:
        cv2.rectangle(bgr, (0, 0), (w, 42), (0, 150, 0), -1)
        txt = " [TELEOP ACTIVE]  SPACE: Pause | R: Ready | H: Home | Y: E-Stop"
        cv2.putText(bgr, txt, (15, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
    else:
        cv2.rectangle(bgr, (0, 0), (w, 42), (0, 140, 220), -1)
        txt = " [CLUTCH PAUSED]  Press SPACE to Engage Teleop"
        cv2.putText(bgr, txt, (15, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

    # 动作状态
    act_str = f"Action: {out.get('action', 'N/A')}"
    cv2.putText(bgr, act_str, (15, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # 绘制手腕锚点与速度矢量线
    if hand_info and hand_info.get("px_coord") is not None:
        u, v = hand_info["px_coord"]
        cv2.circle(bgr, (int(u), int(v)), 10, (0, 255, 0) if clutch_active else (0, 165, 255), -1)
        cv2.circle(bgr, (int(u), int(v)), 14, (255, 255, 255), 2)
        vel = out.get("cmd").linear_velocity if out.get("cmd") else (0, 0, 0)
        end_pt = (int(u + vel[0] * 3.0), int(v - vel[1] * 3.0))
        cv2.arrowedLine(bgr, (int(u), int(v)), end_pt, (255, 0, 0), 3, tipLength=0.3)


def main():
    ap = argparse.ArgumentParser(description="真机 6DOF 视觉遥操")
    ap.add_argument("--iface", default="can0", help="SocketCAN 接口")
    ap.add_argument("--calib", default=str(Path(__file__).parent / "handeye_calib.json"))
    ap.add_argument("--out", default="datasets/teleop_real", help="录制输出目录")
    ap.add_argument("-y", "--gravity-confirm", action="store_true",
                    help="确认重力关节 J2/J3 二次确认 (必须)")
    ap.add_argument("--no-drive", action="store_true", help="只显示不发送")
    args = ap.parse_args()
    if not args.gravity_confirm:
        sys.exit("遥操前必须 -y/--gravity-confirm 确认重力关节 (J2/J3)")

    sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "Leap_Hand" / "python"))
    import cv2  # noqa: E402
    from gesture_mapping.camera import open_realsense  # noqa: E402
    from gesture_mapping.hand_tracker import HandTracker  # noqa: E402

    from lerobot_robot_massage.zdt.config import ZdtConfig  # noqa: E402
    from lerobot_robot_massage.zdt.controller import ZdtController  # noqa: E402

    cam = open_realsense()
    if cam is None:
        sys.exit("未检测到 RealSense (D455) 相机")
    tracker = HandTracker(max_num_hands=1)
    ctrl = ZdtController(ZdtConfig(channel=args.iface))
    adapter = RealArmAdapter(ctrl, gravity_confirmed=True)
    watchdog = VisionWatchdog()
    recorder = EpisodeRecorder(args.out)

    from gesture_mapping.wrist_tracker import build_palm_pts  # noqa: E402

    latest_frame = [None]
    latest_hand = [None]

    def hand_provider():
        ok, bgr, depth, K = cam.read_with_depth()
        if not ok or bgr is None:
            latest_frame[0] = None
            latest_hand[0] = None
            return None
        latest_frame[0] = bgr
        hands = tracker.detect(bgr)
        if not hands:
            latest_hand[0] = None
            return None
        pts = build_palm_pts(hands[0], depth, K)
        px = (int(hands[0].landmarks[0].x * bgr.shape[1]),
              int(hands[0].landmarks[0].y * bgr.shape[0]))
        if pts is None:
            info = {"hand_present": True, "confidence": 0.0,
                    "depth_valid": False, "wrist_mm": None, "px_coord": px}
            latest_hand[0] = info
            return info
        info = {"hand_present": True, "confidence": 0.9, "depth_valid": True,
                "wrist_mm": tuple(float(v) for v in pts[0]), "px_coord": px}
        latest_hand[0] = info
        return info

    curr_key = -1

    teleop = RealArmTeleop(adapter, watchdog, recorder, hand_provider,
                           key_provider=lambda: curr_key)
    # P0-①: connect → SAFE_IDLE → 显式 arm (已 -y 确认重力) → TELEOP → ready (初始准备位)
    adapter.connect()
    adapter.arm(gravity_confirmed=True)
    adapter.enter_teleop()
    adapter.ready()
    recorder.start_episode()
    try:
        while True:
            now = time.monotonic()
            out = teleop.run_once(cmd_ts=now, now=now)
            if out["action"] == "QUIT":
                break
            if out["action"] == "ESTOP":
                print("[急停] e_stop 已触发")
                break
            if out["action"] == "READY":
                print("[姿态] 正在安全同步运动至按摩准备姿态 (READY)...")
            elif out["action"] == "HOME":
                print("[姿态] 正在安全同步运动至上电初始姿态 (HOME)...")

            if latest_frame[0] is not None:
                _draw_overlay(latest_frame[0], out, latest_hand[0], teleop.clutch_active)
                cv2.imshow("RealArmTeleop 6DOF", latest_frame[0])

            k = cv2.waitKey(1) & 0xFF
            curr_key = k if k != 255 else -1
    finally:
        try:
            adapter.e_stop()
        except Exception:  # noqa: BLE001
            pass
        recorder.finish_episode()
        adapter.disconnect()
        cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
