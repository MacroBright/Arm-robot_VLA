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
    """一帧遥操逻辑 (可无相机单测): hand_provider → cmd → watchdog → adapter."""

    def __init__(self, adapter, watchdog, recorder, hand_provider, key_provider,
                 stale_cmd_max_s: float = 0.25):
        self.adapter = adapter
        self.watchdog = watchdog
        self.recorder = recorder
        self.hand_provider = hand_provider      # () -> dict | None
        self.key_provider = key_provider        # () -> key or None
        self.stale_cmd_max_s = stale_cmd_max_s
        self._cmd = CartesianCommand((0.0, 0.0, 0.0))

    def run_once(self, cmd_ts: float, now: float) -> dict:
        """跑一帧: 返回 {action, cmd, phase}. 调用方提供时间 (单调)."""
        key = self.key_provider()
        if key in (ord("y"), ord("Y")):
            self.adapter.e_stop()
            return {"action": "ESTOP", "cmd": self._cmd,
                    "phase": self.adapter.state()}
        if key in (ord("q"), ord("Q"), 27):
            return {"action": "QUIT", "cmd": self._cmd,
                    "phase": self.adapter.state()}
        if key in (ord("r"), ord("R")):
            self.adapter.ready()
            return {"action": "READY", "cmd": CartesianCommand((0.0, 0.0, 0.0)),
                    "phase": self.adapter.state()}
        if key in (ord("o"), ord("O"), ord("0"), ord("h"), ord("H")):
            self.adapter.home()
            return {"action": "HOME", "cmd": CartesianCommand((0.0, 0.0, 0.0)),
                    "phase": self.adapter.state()}

        if (now - cmd_ts) > self.stale_cmd_max_s:
            scaled = CartesianCommand((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), timestamp=cmd_ts)
            self._record(None, scaled, "STOP")
            return {"action": "STOP", "cmd": scaled,
                    "phase": self.adapter.state()}

        hand = self.hand_provider()
        cmd = self._build_command(hand, cmd_ts)
        action, scale = self.watchdog.update(
            hand_present=bool(hand and hand.get("hand_present")),
            hand_confidence=float(hand.get("confidence", 0.0)) if hand else 0.0,
            depth_valid=bool(hand and hand.get("depth_valid")),
            wrist_mm=hand.get("wrist_mm") if hand else None,
            now=now)   # 陈旧判定归控制层 (adapter.move_cartesian_velocity → step(cmd_ts))

        scaled = CartesianCommand(
            tuple(float(v) * scale for v in cmd.linear_velocity),
            tuple(float(w) * scale for w in cmd.angular_velocity),
            timestamp=cmd.timestamp)
        if action == WatchdogAction.ESTOP:
            self.adapter.e_stop()
        elif action != WatchdogAction.STOP:
            self.adapter.move_cartesian_velocity(scaled)

        self._record(hand, scaled, action.name)
        return {"action": action.name, "cmd": scaled,
                "phase": self.adapter.state()}

    def _build_command(self, hand, cmd_ts: float) -> CartesianCommand:
        """由手部信息合成 CartesianCommand (遥操产线可替换实现)."""
        if hand is None:
            return CartesianCommand((0.0, 0.0, 0.0), timestamp=cmd_ts)
        v = hand.get("velocity") or (0.0, 0.0, 0.0)
        w = hand.get("angular_velocity") or (0.0, 0.0, 0.0)
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

    # 真机依赖装配 (与 demo_arm_teleop 共用 Leap_Hand 共享模块)
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

    def hand_provider():
        ok, bgr, depth, K = cam.read_with_depth()
        if not ok or bgr is None:
            return None
        hands = tracker.detect(bgr)
        if not hands:
            return None
        pts = build_palm_pts(hands[0], depth, K)
        if pts is None:
            return {"hand_present": True, "confidence": 0.0,
                    "depth_valid": False, "wrist_mm": None}
        return {"hand_present": True, "confidence": 0.9, "depth_valid": True,
                "wrist_mm": tuple(float(v) for v in pts[0])}

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
