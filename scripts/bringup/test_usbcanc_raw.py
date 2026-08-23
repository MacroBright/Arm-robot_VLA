#!/usr/bin/env python3
"""USBCAN-UCP100 裸发测试 — 发一帧 + 监听回帧.

用法:
  1. 先确保 can0 已 up: sudo ip link set can0 type can bitrate 500000 && sudo ip link set can0 up
  2. python test_usbcanc_raw.py          # 单次发送+监听
  3. python test_usbcanc_raw.py --listen  # 只监听 5 秒
"""
import argparse
import can
import time

INTERFACE = "can0"
BITRATE = 500_000


def test_send_and_recv():
    """发送一帧到 CAN 地址 0x02 (关节1), 读实时位置 (0x36)."""
    bus = can.Bus(interface="socketcan", channel=INTERFACE, bitrate=BITRATE)
    print(f"[OK] SocketCAN opened on {INTERFACE} @ {BITRATE}")

    # ZDT 读位置帧: 扩展帧 ID = (0x02 << 8) | 0x00 = 0x200
    # 数据段: [0x36, 0x6B] = F_READ_POS + checksum
    ext_id = 0x02 << 8 | 0x00  # addr=0x02, seq=0
    msg = can.Message(
        arbitration_id=ext_id,
        is_extended_id=True,
        data=bytes([0x36, 0x6B]),
    )
    print(f"\n[TX] ID=0x{ext_id:08X}  data={msg.data.hex()}")
    print(f"     → 向关节1 (addr=0x02) 发送 读位置 (0x36)")

    try:
        bus.send(msg)
        print("[TX] 发送成功")
    except can.CanOperationError as e:
        print(f"[TX] 发送失败: {e}")
        bus.shutdown()
        return

    # 监听回帧 2 秒
    print("\n[RX] 监听回帧 (2秒)...")
    deadline = time.monotonic() + 2.0
    got_reply = False
    while time.monotonic() < deadline:
        resp = bus.recv(timeout=0.5)
        if resp is None:
            print("     (超时)")
            continue
        src_addr = resp.arbitration_id >> 8
        seq = resp.arbitration_id & 0xFF
        print(f"     ID=0x{resp.arbitration_id:08X}  addr=0x{src_addr:02X}  seq={seq}  data={resp.data.hex()}")
        if src_addr == 0x02:
            got_reply = True
            if len(resp.data) >= 4:
                func = resp.data[0]
                if func == 0x36:
                    sign = -1 if resp.data[1] == 0x80 else 1
                    pos_raw = (resp.data[2] << 16) | (resp.data[3] << 8) | resp.data[4]
                    pos_deg = pos_raw / 10.0 * sign
                    print(f"     ✓ 关节1 位置: {pos_deg:.1f}° (raw=0x{pos_raw:06X})")
                else:
                    print(f"     功能码: 0x{func:02X} (非 0x36)")
            break

    if not got_reply:
        print("\n[!] 未收到 addr=0x02 的回帧 — 电机可能未上电或 CAN 线未接")

    bus.shutdown()
    print("\n[OK] SocketCAN closed")


def test_listen_only(duration_s: float = 5.0):
    """只监听 CAN 总线上的所有帧."""
    bus = can.Bus(interface="socketcan", channel=INTERFACE, bitrate=BITRATE)
    print(f"[OK] 监听模式 — {INTERFACE} @ {BITRATE}, 持续 {duration_s}s\n")

    deadline = time.monotonic() + duration_s
    count = 0
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.5)
        if msg is None:
            continue
        count += 1
        addr = msg.arbitration_id >> 8
        seq = msg.arbitration_id & 0xFF
        print(f"  [{count:3d}] ID=0x{msg.arbitration_id:08X}  addr=0x{addr:02X}  seq={seq}  "
              f"dlc={msg.dlc}  data={msg.data.hex()}")

    print(f"\n共收到 {count} 帧")
    bus.shutdown()


def main():
    p = argparse.ArgumentParser(description="USBCAN-UCP100 裸发测试")
    p.add_argument("--listen", action="store_true", help="只监听 5 秒")
    p.add_argument("--duration", type=float, default=5.0, help="监听时长 (秒)")
    args = p.parse_args()

    if args.listen:
        test_listen_only(args.duration)
    else:
        test_send_and_recv()


if __name__ == "__main__":
    main()
