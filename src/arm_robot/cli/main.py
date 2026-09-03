"""Unified CLI entrypoint for Arm-robot_VLA (arm-robot)."""

from __future__ import annotations

import argparse
import sys
from typing import Sequence


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="arm-robot",
        description="🤖 Arm-robot_VLA: 6-DOF 机械臂与具身智能统一命令行工具 (Unified CLI)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
常用示例:
  arm-robot teleop --no-drive       # 启动纯视觉手势空跑遥操 (无需机械臂)
  arm-robot sim --viewer            # 启动 MuJoCo 3D 物理仿真数字孪生
  arm-robot calib --sandbox         # 启动手眼标定实时 6DOF 沙盒
  arm-robot panel --iface can0      # 查看 6 轴电机状态遥测面板
  arm-robot bringup status          # 探查 CAN 总线电机在线与物理角度
  arm-robot test                    # 运行全套 285 项自动化单元测试

提示: 您也可以直接使用独立快捷命令:
  arm-teleop, arm-sim, arm-panel, arm-bringup, arm-calib, arm-control, arm-joystick
""",
    )

    subparsers = parser.add_subparsers(dest="subcommand", title="可用子命令", metavar="<command>")

    subparsers.add_parser("teleop", help="6-DOF 视觉手势遥操主程序 (支持 --no-drive 空跑)", add_help=False)
    subparsers.add_parser("sim", help="MuJoCo 机械臂物理动力学仿真与 TCP 服务", add_help=False)
    subparsers.add_parser("control", help="交互式 6 轴电机底层协议与标定调试控制台 (zdt_interactive)", add_help=False)
    subparsers.add_parser("interactive", help="交互式 6 轴电机底层协议与标定调试控制台 (zdt_interactive 别名)", add_help=False)
    subparsers.add_parser("panel", help="交互式 6 轴电机底层协议与标定调试控制台 (zdt_interactive 别名)", add_help=False)
    subparsers.add_parser("bringup", help="底层硬件快速拉起与极性自检", add_help=False)
    subparsers.add_parser("calib", help="30 秒手眼标定向导与 6DOF 实时沙盒", add_help=False)
    subparsers.add_parser("keyboard", help="键盘 W/A/S/D/Q/E 6-DOF 笛卡尔遥操 (支持 --sim 与真机)", add_help=False)
    subparsers.add_parser("joystick", help="Xbox / USB 游戏手柄 6 轴遥控", add_help=False)
    subparsers.add_parser("test", help="运行全套自动化测试套件 (pytest)", add_help=False)

    return parser



def main(argv: Sequence[str] | None = None) -> int:
    if argv is None:
        argv = sys.argv[1:]

    parser = build_parser()

    if not argv or argv[0] in ("-h", "--help"):
        parser.print_help()
        return 0

    subcommand = argv[0]
    sub_args = list(argv[1:])

    # Dispatcher
    if subcommand == "teleop":
        from arm_robot.cli.teleop import main as teleop_main
        sys.argv = ["arm-teleop"] + sub_args
        return teleop_main() or 0

    elif subcommand == "sim":
        from arm_robot.cli.sim import main as sim_main
        sys.argv = ["arm-sim"] + sub_args
        return sim_main() or 0

    elif subcommand in ("control", "interactive", "panel"):
        from arm_robot.cli.control import main as control_main
        sys.argv = ["arm-control"] + sub_args
        return control_main() or 0


    elif subcommand == "bringup":
        from arm_robot.cli.bringup import main as bringup_main
        sys.argv = ["arm-bringup"] + sub_args
        return bringup_main() or 0

    elif subcommand == "calib":
        from arm_robot.cli.calib import main as calib_main
        sys.argv = ["arm-calib"] + sub_args
        return calib_main() or 0

    elif subcommand == "control":
        from arm_robot.cli.control import main as control_main
        sys.argv = ["arm-control"] + sub_args
        return control_main() or 0

    elif subcommand == "keyboard":
        from arm_robot.cli.keyboard import main as keyboard_main
        sys.argv = ["arm-keyboard"] + sub_args
        return keyboard_main() or 0

    elif subcommand == "joystick":
        from arm_robot.cli.joystick import main as joystick_main
        sys.argv = ["arm-joystick"] + sub_args
        return joystick_main() or 0


    elif subcommand == "test":
        import pytest
        return pytest.main(["tests/"] + sub_args)

    else:
        parser.print_help()
        print(f"\n未知子命令: '{subcommand}'", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
