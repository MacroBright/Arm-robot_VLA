#!/usr/bin/env python3
"""ZDT 上电自动 anchor CLI — 建立 6 轴真实角度基准 (PC 方案A).

背景: 本机驱动器固件 (版本 0x05 / HW 0x82) 不支持 CAN 触发回零 (0x9A 无响应、
0x4C 0xAE 写入不持久化), 无法依赖驱动器"断电记忆回零"。改用 PC 端上电自动
锚定: 读 0x36 真实位置 (经 CALIB(k,b) 换算) 建立基准。

⚠ 本脚本是纯读取 (不使能电机、不发运动), 与 zdt_interactive.py 的 anchor 一致;
每次上电或外力搬动后运行一次, 得到各关节真实角度作为基准/限位参考.

用法:
  python scripts/bringup/zdt_anchor.py                    # 读 6 轴真实角度 (显示)
  python scripts/bringup/zdt_anchor.py --expected "90 90 -90 0 90 0"
      # 给定期望初始位, 计算 offset = 真实 - 期望 (显示, 不写入)
"""
import argparse
import sys
import time
from pathlib import Path

import can

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from lerobot_robot_massage.zdt.config import (
    CALIB, DEFAULT_REDUCTION_RATIOS, JOINT_ADDRS, JOINT_INIT_ANGLE_DEG,
)
from lerobot_robot_massage.zdt.frames import decode_pos4, encode_frame
from lerobot_robot_massage.zdt.zdt_driver import CommunicationError


def raw_send_recv(bus: can.Bus, addr: int, payload: bytes, func: int,
                  timeout_s: float = 0.3):
    """发送负载并等待 (addr, func) 匹配回帧 (同 zdt_interactive)."""
    for frame in encode_frame(addr, payload):
        bus.send(can.Message(arbitration_id=frame.arbitration_id,
                             is_extended_id=True, data=frame.data))
    t0 = time.monotonic()
    deadline = t0 + timeout_s
    while time.monotonic() < deadline:
        resp = bus.recv(timeout=max(0, deadline - time.monotonic()))
        if resp is None:
            continue
        if resp.arbitration_id >> 8 == addr and len(resp.data) > 0 \
                and resp.data[0] == func:
            return bytes(resp.data)
    return None


def read_real_angle(bus: can.Bus, addr: int, slot: int) -> float:
    """读 0x36 pos → 真实输出角度 (CALIB(k,b) 优先, 否则纯减速比)."""
    data = raw_send_recv(bus, addr, bytes([0x36, 0x6B]), 0x36, timeout_s=0.3)
    if data is None or len(data) < 6:
        raise CommunicationError(f"J{slot+1} 0x{addr:02X} 读 pos 超时")
    sbyte = -1 if data[1] == 0x01 else 1
    pos = decode_pos4(data[2:6], sbyte)
    kb = CALIB[slot]
    if kb is not None:
        k, b = kb
        real = (pos - b) / k if abs(k) > 1e-9 else pos
    else:
        real = pos / DEFAULT_REDUCTION_RATIOS[slot]
    return real


def main() -> None:
    ap = argparse.ArgumentParser(description="ZDT 上电自动 anchor (PC 方案A)")
    ap.add_argument("--iface", default="can0", help="SocketCAN 接口 (默认 can0)")
    ap.add_argument("--expected", default=None,
                    help="期望初始位 6 角度 (如 '90 90 -90 0 90 0'); 缺省用 g_joints_init")
    args = ap.parse_args()

    expected = ([float(x) for x in args.expected.split()]
                if args.expected else list(JOINT_INIT_ANGLE_DEG))

    bus = can.Bus(interface="socketcan", channel=args.iface, bitrate=500_000)
    try:
        real = [0.0] * 6
        for i, addr in enumerate(JOINT_ADDRS):
            try:
                real[i] = read_real_angle(bus, addr, i)
            except CommunicationError as exc:
                print(f"  ⚠ {exc}")
                real[i] = float("nan")
    finally:
        try:
            bus.shutdown()
        except Exception:  # noqa: BLE001
            pass

    offsets = [real[i] - expected[i] for i in range(6)]
    print(f"[anchor] 6 轴真实角度 (0x36 + CALIB(k,b)), 期望初始位:")
    print(f"  期望: {expected}")
    print(f"  {'关节':<4}{'CAN':<5}{'真实°':>9}{'期望°':>9}{'offset':>9}")
    for i in range(6):
        r = real[i]
        rs = f"{r:>9.2f}" if r == r else f"{'离线':>9}"
        print(f"  J{i+1:<3}0x{JOINT_ADDRS[i]:02X}{rs}"
              f"{expected[i]:>9.2f}{offsets[i]:>9.2f}")
    print("\n  [提示] 每次上电/外力搬动后运行此脚本, 得到各轴真实角度作基准;")
    print("         读数实时反映当前位, 限位判断 (check_limits_real) 已直接用它.")


if __name__ == "__main__":
    main()
