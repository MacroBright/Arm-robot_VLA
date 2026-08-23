"""ZdtDriver 命令语义单测 (FakeTransport 注入回帧)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from lerobot_robot_massage.zdt.can_transport import CanTransportError
from lerobot_robot_massage.zdt.config import (
    CHECKSUM, F_ENABLE, F_LEGACY_POS, F_READ_POS, F_STOP,
)
from lerobot_robot_massage.zdt.fakes import (
    FailingRecvTransport, FailingSendTransport, FakeTransport,
)
from lerobot_robot_massage.zdt.testutil import run_all
from lerobot_robot_massage.zdt.zdt_driver import (
    TransportError, ZdtDriver, TimeoutError,
)


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


def test_move_pulse_payload_split():
    t = FakeTransport()
    d = ZdtDriver(t, timeout_s=0.001, retries=0)
    # 6 轴视角: addr=0x05, 2222 脉冲 (≈90°@ratio 50), CW, 60RPM, acc=20
    d.move_pulse(0x05, 2222, True, 60.0, 20)
    # 0xFD 命令体 12 字节 → 2 帧, ID=0x0500/0x0501, 功能码 FD 重复
    assert [f.arbitration_id for f in t.sent] == [0x0500, 0x0501]
    assert t.sent[0].data[0] == F_LEGACY_POS
    assert t.sent[1].data[0] == F_LEGACY_POS
    # 首帧: [FD, dir=0(CW), vel_hi, vel_lo, acc, 脉冲3B]
    #   vel=60RPM=0x003C (字段= RPM 直传, 修复 ×10 bug); acc=20=0x14; 2222=0x000008AE
    assert t.sent[0].data[1] == 0x00          # dir CW
    assert t.sent[0].data[2:4] == bytes([0x00, 0x3C])   # 速度 60RPM
    assert t.sent[0].data[4] == 0x14          # 加速度
    assert t.sent[0].data[5:8] == bytes([0x00, 0x00, 0x08])  # 脉冲高3B
    # 尾帧: [FD, 脉冲第4B, 0, 0(相对+同步), 6B]
    assert t.sent[1].data[1] == 0xAE
    assert t.sent[1].data[-1] == CHECKSUM


def test_move_pulse_ccw_sets_dir_bit():
    t = FakeTransport()
    d = ZdtDriver(t, timeout_s=0.001, retries=0)
    d.move_pulse(0x02, 10, False, 30.0)   # CCW
    assert t.sent[0].data[1] == 0x01      # dir 非 0 = CCW


def test_read_pos_parses():
    t = FakeTransport()
    # 回帧 [36, 符号字节, 位置4B, 6B]; 符号 0x00=正 0x01=负 (说明书 §7.4.4)
    # 90° → 90×65536/360 = 16384 = 0x00004000
    t.inject(0x03, F_READ_POS, b"\x00\x00\x00\x40\x00\x6b")   # 正数 +90.0°
    d = ZdtDriver(t, timeout_s=0.001, retries=0)
    assert abs(d.read_pos(0x03) - 90.0) < 0.01


def test_read_pos_negative_sign_byte():
    """符号字节 0x01 = 负数 (固件 robot.c:1045 实测确认).

    旧代码用 data[1]==0x80 判断是错的 — 负位置会被当成正数返回.
    """
    t = FakeTransport()
    # -90° → magnitude=16384=0x4000 → [00,00,40,00]
    t.inject(0x03, F_READ_POS, b"\x01\x00\x00\x40\x00\x6b")   # 负数 -90.0°
    d = ZdtDriver(t, timeout_s=0.001, retries=0)
    assert abs(d.read_pos(0x03) - (-90.0)) < 0.01


def test_read_current_parses():
    t = FakeTransport()
    t.inject(0x03, 0x27, b"\x00\x63\x6b")   # mA = 0x0063 = 99 (无保留字节, data[1:3])
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
    # 注入: 先到位帧 (addr=0x04), 再位置回帧 (addr=0x02, 符号 0x00=正, 4B pos)
    t.inject(0x04, 0xFD, b"\x9f\x6b")
    t.inject(0x02, F_READ_POS, b"\x00\x00\x00\x00\x64\x6b")  # pos≈0.35°(正)
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


def test_send_transport_error_wrapped_as_transport_error():
    # 总线 send 死 → CanTransportError 必须包成 TransportError (ZdtDriverError),
    # 不能泄漏裸 CanTransportError/RuntimeError
    t = FailingSendTransport()
    d = ZdtDriver(t, timeout_s=0.001, retries=0)
    try:
        d.stop_all()
        raise AssertionError("应抛 TransportError")
    except TransportError:
        pass
    except Exception as exc:  # noqa: BLE001
        raise AssertionError(f"泄漏非 TransportError: {type(exc).__name__}: {exc}")


def test_recv_transport_error_wrapped_as_transport_error():
    # read_pos 的 recv 抛 CanTransportError → TransportError; send 已成功 (先注入请求帧)
    t = FailingRecvTransport()
    d = ZdtDriver(t, timeout_s=0.001, retries=0)
    try:
        d.read_pos(0x02)
        raise AssertionError("应抛 TransportError")
    except TransportError:
        pass
    except Exception as exc:  # noqa: BLE001
        raise AssertionError(f"泄漏非 TransportError: {type(exc).__name__}: {exc}")


def test_transport_error_carries_cause():
    # __cause__ 应指向原始 CanTransportError (调试链完整)
    t = FailingSendTransport()
    d = ZdtDriver(t, timeout_s=0.001, retries=0)
    try:
        d.stop_all()
        raise AssertionError("应抛 TransportError")
    except TransportError as exc:
        assert isinstance(exc.__cause__, CanTransportError)


if __name__ == "__main__":
    run_all(globals())
