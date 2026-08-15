"""ZdtDriver 命令语义单测 (FakeTransport 注入回帧)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from lerobot_robot_massage.zdt.config import CHECKSUM, F_ENABLE, F_POS, F_READ_POS, F_STOP
from lerobot_robot_massage.zdt.fakes import FakeTransport
from lerobot_robot_massage.zdt.testutil import run_all
from lerobot_robot_massage.zdt.zdt_driver import ZdtDriver, TimeoutError


def _last_frame(transport: FakeTransport):
    return transport.sent[-1]


def test_enable_payload():
    t = FakeTransport()
    d = ZdtDriver(t, timeout_s=0.001, retries=0)
    d.enable(0x02, True)
    f = _last_frame(t)
    assert f.arbitration_id == 0x0200
    assert f.data == bytes([F_ENABLE, 0xAB, 0x01, 0x00, CHECKSUM])


def test_stop_all_broadcast():
    t = FakeTransport()
    d = ZdtDriver(t, timeout_s=0.001, retries=0)
    d.stop_all()
    assert _last_frame(t).arbitration_id == 0x0000      # 广播
    assert _last_frame(t).data == bytes([F_STOP, 0x98, 0x00, CHECKSUM])


def test_move_abs_payload_split():
    t = FakeTransport()
    d = ZdtDriver(t, timeout_s=0.001, retries=0)
    d.move_abs(0x05, 90.0, 60.0)
    # 0xFB 命令体 10 字节 → 2 帧, ID=0x0500/0x0501, 功能码 FB 重复
    assert [f.arbitration_id for f in t.sent] == [0x0500, 0x0501]
    assert t.sent[0].data[0] == F_POS
    assert t.sent[1].data[0] == F_POS
    # 位置 90.0×10=900 → 0x000384 (data[3:6])
    assert t.sent[0].data[3:6] == bytes([0x00, 0x03, 0x84])


def test_read_pos_parses():
    t = FakeTransport()
    # 回帧 [36, 符号=0x01, 位置高,中,低, 6B]; 位置 900×? → 90.0°
    t.inject(0x03, F_READ_POS, b"\x01\x00\x03\x84\x6b")
    d = ZdtDriver(t, timeout_s=0.001, retries=0)
    assert abs(d.read_pos(0x03) - 90.0) < 0.001


def test_read_current_parses():
    t = FakeTransport()
    t.inject(0x03, 0x27, b"\x02\x00\x63\x6b")   # mA = 0x0063 = 99
    d = ZdtDriver(t, timeout_s=0.001, retries=0)
    assert abs(d.read_current(0x03) - 99.0) < 0.001


def test_read_flag_parses_status_bits():
    t = FakeTransport()
    t.inject(0x03, 0x3A, b"\x07\x6b")   # 0x07 = 使能|到位|堵转
    d = ZdtDriver(t, timeout_s=0.001, retries=0)
    flag = d.read_flag(0x03)
    assert flag & 0x01          # 使能
    assert flag & 0x02          # 到位
    assert flag & 0x04          # 堵转
    assert not (flag & 0x08)    # 未触发堵转保护


def test_read_timeout_raises_after_retries():
    t = FakeTransport()   # 无注入 → recv 恒 None
    d = ZdtDriver(t, timeout_s=0.001, retries=2)
    try:
        d.read_pos(0x02)
        raise AssertionError("应抛 TimeoutError")
    except TimeoutError:
        pass


def test_arrival_event_dispatches_and_does_not_break_wait():
    t = FakeTransport()
    events = []
    d = ZdtDriver(t, timeout_s=0.001, retries=0)
    d.on_arrived = lambda addr: events.append(addr)
    # 注入: 先到位帧 (addr=0x04), 再位置回帧 (addr=0x02)
    t.inject(0x04, 0xFD, b"\x9f\x6b")
    t.inject(0x02, F_READ_POS, b"\x01\x00\x00\x64\x6b")
    pos = d.read_pos(0x02)
    assert events == [0x04]
    assert pos > 0


def test_set_zero_and_home_payload():
    t = FakeTransport()
    d = ZdtDriver(t, timeout_s=0.001, retries=0)
    d.set_zero(0x02)
    assert _last_frame(t).data == bytes([0x93, 0x88, 0x01, CHECKSUM])
    d.home(0x02)
    assert _last_frame(t).data == bytes([0x9A, 0x00, 0x00, CHECKSUM])


if __name__ == "__main__":
    run_all(globals())
