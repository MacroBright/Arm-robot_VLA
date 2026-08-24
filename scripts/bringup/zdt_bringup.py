#!/usr/bin/env python3
"""ZDT 直连 CAN bring-up CLI (spec §7.1).

用法:
  python scripts/zdt_bringup.py status            # 使能+读6轴角度/电流
  python scripts/zdt_bringup.py step <j> <deg>    # 关节相对旋转 (j=1..6)
  python scripts/zdt_bringup.py reset             # soft_reset
  python scripts/zdt_bringup.py torque <0|1>
  python scripts/zdt_bringup.py estop             # 广播急停
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from lerobot_robot_massage.zdt.config import JOINT_INIT_ANGLE_DEG, ZdtConfig
from lerobot_robot_massage.zdt.controller import ZdtController
from lerobot_robot_massage.zdt.zdt_driver import ZdtDriverError


def _print_state(ctrl: ZdtController) -> None:
    try:
        angles, vels, loads = ctrl.get_state()
    except ZdtDriverError as exc:
        print(f"[status] 读取失败: {exc}")
        return
    if not angles:
        print("[status] 读取失败 (CAN 超时?)")
        return
    line = "  ".join(f"J{i+1}:{a:7.1f}° cur:{int(l):4d}mA"
                     for i, (a, l) in enumerate(zip(angles, loads)))
    print(f"[status] {line}")


def main() -> None:
    ap = argparse.ArgumentParser(description="ZDT 直连 CAN bring-up")
    ap.add_argument("--iface", default="can0", help="SocketCAN 接口 (默认 can0)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p_status = sub.add_parser("status")
    p_status.add_argument("--n", type=int, default=1, help="读取次数")

    p_step = sub.add_parser("step")
    p_step.add_argument("joint", type=int, help="关节 1-6")
    p_step.add_argument("deg", type=float, help="相对角度")

    sub.add_parser("reset")
    p_torque = sub.add_parser("torque")
    p_torque.add_argument("state", type=int, choices=[0, 1])
    sub.add_parser("estop")
    args = ap.parse_args()

    cfg = ZdtConfig(channel=args.iface)
    ctrl = ZdtController(cfg)

    try:
        ctrl.connect()
        if args.cmd == "status":
            for _ in range(args.n):
                _print_state(ctrl)
                ctrl.tick()      # 看门狗每轮巡检
        elif args.cmd == "step":
            if not 1 <= args.joint <= 6:
                raise SystemExit("joint 需在 1-6")
            ctrl.rel_rotate(args.joint, args.deg)
            ctrl.tick()
            print(f"[step] J{args.joint} {args.deg:+.1f}°")
        elif args.cmd == "reset":
            ctrl.soft_reset()
            ctrl.tick()
            print(f"[reset] soft_reset → {JOINT_INIT_ANGLE_DEG}")
        elif args.cmd == "torque":
            if args.state == 1:
                ctrl.arm(gravity_confirmed=True)
                print("[torque] 已臂置 (set_torque on + ARMED)")
            else:
                ctrl.disarm()
                print("[torque] 已解除 (set_torque off + SAFE_IDLE)")
        elif args.cmd == "estop":
            ctrl.e_stop()
            ctrl.tick()
            print("[estop] 已广播急停")
    finally:
        ctrl.disconnect()


if __name__ == "__main__":
    main()
