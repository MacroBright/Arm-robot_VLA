"""ZdtBus 线程安全总线单测 (QueueTransport + 后台读线程)."""
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from lerobot_robot_massage.zdt.fakes import QueueTransport
from lerobot_robot_massage.zdt.testutil import run_all
from lerobot_robot_massage.zdt.zdt_bus import ZdtBus

# 手册 §12.3 示例参数数组
MANUAL_PARAMS = bytes.fromhex(
    "0001010202001001000004B008980BB803E8050700010001000807D007D00003")


def _mk_multi_frames():
    """0x42 响应概念帧 → 拆成 CAN 帧列表 (ID=0x0200, func 重复)."""
    payload = bytes([0x42, 37, 24]) + MANUAL_PARAMS + bytes([0x6B])
    frames = []
    for i in range(0, len(payload) - 1, 7):
        frames.append(bytes([0x42]) + payload[i + 1:i + 1 + 7])
    return frames


def _mk_bus(watchdog_s=3.0, on_watchdog=None):
    t = QueueTransport()
    bus = ZdtBus(t, watchdog_s=watchdog_s, on_watchdog=on_watchdog)
    return bus, t


def test_request_matches_and_discards_others():
    bus, t = _mk_bus()
    try:
        bus.open()
        # 先注入一个不匹配帧, 再注入匹配帧 → request 丢弃前者并返回后者
        t.inject(0x03, 0x27, b"\x00\x7b\x6b")           # 不匹配
        t.inject(0x02, 0x36, b"\x01\x00\x00\x64\x6b")   # 匹配
        data = bus.request(0x02, bytes([0x36, 0x6b]), 0x36, timeout_s=0.5)
        assert data is not None
        assert data[0] == 0x36
    finally:
        bus.close()


def test_request_timeout_returns_none():
    bus, t = _mk_bus()
    try:
        bus.open()
        assert bus.request(0x02, bytes([0x36, 0x6b]), 0x36, timeout_s=0.15) is None
    finally:
        bus.close()


def test_request_multi_reassembles_42_block():
    bus, t = _mk_bus()
    try:
        bus.open()
        for f in _mk_multi_frames():
            t.inject(0x02, 0x42, f[1:])          # inject 会前置 func 字节
        data = bus.request_multi(0x02, bytes([0x42, 0x6c, 0x6b]), 0x42,
                                 timeout_s=0.5, gap_s=0.05)
        assert data is not None
        assert data[0] == 0x42 and data[1] == 37 and data[2] == 24
        assert data[3:-1] == MANUAL_PARAMS
        assert data[-1] == 0x6B
    finally:
        bus.close()


def test_estop_broadcast_while_request_in_flight():
    bus, t = _mk_bus()
    try:
        bus.open()
        # request 在后台线程挂起 (无注入会超时), 主线程同时发 e_stop
        result = {}
        def worker():
            result["data"] = bus.request(0x02, bytes([0x36, 0x6b]), 0x36,
                                         timeout_s=0.4)
        th = threading.Thread(target=worker)
        th.start()
        time.sleep(0.1)                 # 让 request 已发出、正在等待
        bus.stop_all()                  # 关键: 在途时 e_stop 必须立即发
        th.join(timeout=1.0)
        assert not th.is_alive()
        # 广播帧 (addr=0x00) 已被发送, 且 request 超时返回 None
        assert any(f.arbitration_id >> 8 == 0x00 for f in t.sent)
        assert result["data"] is None
    finally:
        bus.close()


def test_arrived_frame_routes_to_flag():
    bus, t = _mk_bus()
    try:
        bus.open()
        t.inject(0x02, 0xFD, b"\x00\x6b")          # 到位帧 → 不进收件箱
        time.sleep(0.2)
        assert bus.is_arrived(0x02)
        bus.clear_arrived(0x02)
        assert not bus.is_arrived(0x02)
    finally:
        bus.close()


def test_watchdog_fires_when_armed_and_idle():
    fired = []
    bus, t = _mk_bus(watchdog_s=0.15, on_watchdog=lambda: fired.append(1))
    try:
        bus.open()
        bus.set_watchdog_armed(True)
        time.sleep(0.4)
        assert bus.watchdog_triggered
        assert fired
        # 触发了 stop_all 广播
        assert any(f.arbitration_id >> 8 == 0x00 for f in t.sent)
    finally:
        bus.close()


def test_watchdog_not_armed_no_fire():
    bus, t = _mk_bus(watchdog_s=0.15)
    try:
        bus.open()
        time.sleep(0.4)               # 未武装 → 不触发
        assert not bus.watchdog_triggered
        assert not any(f.arbitration_id >> 8 == 0x00 for f in t.sent)
    finally:
        bus.close()


def test_watchdog_activity_keeps_alive():
    bus, t = _mk_bus(watchdog_s=0.2)
    try:
        bus.open()
        bus.set_watchdog_armed(True)
        for _ in range(6):            # 0.05s×6 = 0.3s, 每次触碰保活
            time.sleep(0.05)
            bus._touch()
        assert not bus.watchdog_triggered
        time.sleep(0.35)              # 停止触碰 → 超时触发
        assert bus.watchdog_triggered
    finally:
        bus.close()


def test_watchdog_fires_on_bus_death_when_armed():
    fired = []
    bus, t = _mk_bus(watchdog_s=5.0, on_watchdog=lambda: fired.append(1))
    try:
        bus.open()
        bus.set_watchdog_armed(True)
        t.make_bus_dead()             # recv 抛 CanTransportError → 立即触发
        time.sleep(0.3)
        assert bus.watchdog_triggered
        assert fired
    finally:
        bus.close()


if __name__ == "__main__":
    run_all(globals())
