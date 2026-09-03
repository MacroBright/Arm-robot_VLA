#!/usr/bin/env python3
"""ZDT 单驱动器读响应延迟探针 (诊断 addr=0x05 不回 0x36 用).

用途: 在**不跑遥操、不使能扭矩**的孤岛条件下, 逐轴发 0x36/0x27/0x3A/0x1F,
      量出每轴的读成功率与响应延迟, 区分"电气/离线" vs "慢" vs "功能特异".

只读不写、不动电机 (纯读帧). 运行前确保 can0 已 up 且驱动器供电:
    sudo ip link set can0 type can bitrate 500000 && sudo ip link set can0 up

用法:
  python scripts/bringup/zdt_probe_drive.py                 # 探测 2..7, 每轴 0x36×20
  python scripts/bringup/zdt_probe_drive.py --addr 5        # 只测 addr=5, 0x36×50
  python scripts/bringup/zdt_probe_drive.py --loop 5        # 持续读 addr=5, 观察间歇性
  python scripts/bringup/zdt_probe_drive.py --full-range    # 探测 1..8 (含扫描边界)
"""
import argparse
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from lerobot_robot_massage.zdt.can_transport import CanTransportError, SocketCanTransport
from lerobot_robot_massage.zdt.zdt_driver import ZdtDriver, ZdtDriverError


def _fmt_ms(vals):
    if not vals:
        return "-"
    p = lambda q: statistics.quantiles(vals, n=100, method="inclusive")[q - 1]
    return (f"p50={p(50):6.1f} p90={p(90):6.1f} "
            f"p99={p(99):6.1f} max={max(vals):6.1f}ms")


def probe_pos(drv, addr, n):
    """0x36 读 n 次, 返回 (成功次数, 延迟ms列表)."""
    ok, lat = 0, []
    for _ in range(n):
        t0 = time.monotonic()
        try:
            drv.read_pos(addr)
            lat.append((time.monotonic() - t0) * 1000.0)
            ok += 1
        except ZdtDriverError:
            pass
    return ok, lat


def probe_func(drv, addr, n, fn):
    """其它读命令 (0x27/0x3A) 成功次数."""
    ok = 0
    for _ in range(n):
        try:
            fn(addr)
            ok += 1
        except ZdtDriverError:
            pass
    return ok


def run_table(iface, addrs, reads, timeout_s):
    t = SocketCanTransport(iface, 500_000)
    t.open()
    drv = ZdtDriver(t, timeout_s=timeout_s, retries=0)
    print(f"== 延迟探针 (timeout={timeout_s}s, retries=0) ==\n")
    print(f"{'addr':>4} {'fw':>6} {'0x36 pos':>10} {'0x36延迟':>34} "
          f"{'0x3A flag':>10} {'0x27 cur':>10}")
    try:
        for addr in addrs:
            ver = drv.read_version(addr, timeout_s=0.3, retries=0)
            fw = f"{ver[0]}.{ver[1]}" if ver else "OFFLINE"
            ok, lat = probe_pos(drv, addr, reads)
            flag_ok = probe_func(drv, addr, 5, drv.read_flag)
            cur_ok = probe_func(drv, addr, 5, drv.read_current)
            flag = f"{flag_ok}/5" if ver else "-"
            cur = f"{cur_ok}/5" if ver else "-"
            mark = " <== 异常" if ver and ok < reads * 0.8 else ""
            print(f"{addr:>4} {fw:>6} {ok:>5}/{reads:>4} {_fmt_ms(lat):>34} "
                  f"{flag:>10} {cur:>10}{mark}")
    finally:
        t.close()


def run_loop(iface, addr, timeout_s):
    t = SocketCanTransport(iface, 500_000)
    t.open()
    drv = ZdtDriver(t, timeout_s=timeout_s, retries=0)
    print(f"== 持续读 addr={addr:#04x} (Ctrl+C 退出) ==\n")
    last = None
    streak = 0
    try:
        while True:
            t0 = time.monotonic()
            try:
                drv.read_pos(addr)
                state, lat = "OK  ", (time.monotonic() - t0) * 1000.0
            except ZdtDriverError:
                state, lat = "TOUT", (time.monotonic() - t0) * 1000.0
            if state != last:
                print(f"{time.strftime('%H:%M:%S')} {state} lat={lat:6.1f}ms "
                      f"(连续 {streak} 次同态)")
                last, streak = state, 1
            else:
                streak += 1
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n[probe] 已退出")
    finally:
        t.close()


def main():
    ap = argparse.ArgumentParser(description="ZDT 单驱动器读响应延迟探针 (只读)")
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--addr", type=int, default=None, help="只测单个地址 (1..8)")
    ap.add_argument("--loop", type=int, default=None, metavar="ADDR",
                    help="持续读某地址观察间歇性")
    ap.add_argument("--reads", type=int, default=20)
    ap.add_argument("--timeout", type=float, default=0.3)
    ap.add_argument("--full-range", action="store_true", help="探测 1..8")
    args = ap.parse_args()

    if args.loop is not None:
        run_loop(args.iface, args.loop, args.timeout)
        return

    addrs = [args.addr] if args.addr is not None else \
        (list(range(1, 9)) if args.full_range else list(range(2, 8)))
    run_table(args.iface, addrs, args.reads, args.timeout)


if __name__ == "__main__":
    main()
