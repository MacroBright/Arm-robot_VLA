"""CanTransport 抽象契约测试 + SocketCanTransport 惰性导入验证."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from lerobot_robot_massage.zdt.can_transport import (
    CanTransport, CanTransportError, SocketCanTransport,
)
from lerobot_robot_massage.zdt.fakes import FakeTransport
from lerobot_robot_massage.zdt.frames import CanFrame
from lerobot_robot_massage.zdt.testutil import run_all


def test_fake_transport_open_close():
    t = FakeTransport()
    t.open()
    assert t.opened
    t.close()
    assert t.closed


def test_fake_transport_send_recv_roundtrip():
    t = FakeTransport()
    t.inject(0x03, 0x36, b"\x01\x2c\x01\x6b")
    frame = t.recv(0.01)
    assert frame is not None
    assert frame.arbitration_id == 0x0300
    assert frame.data == bytes([0x36, 0x01, 0x2c, 0x01, 0x6b])


def test_fake_transport_recv_empty_returns_none():
    t = FakeTransport()
    assert t.recv(0.001) is None


def test_socketcan_transport_constructs_without_python_can():
    # 构造不抛错; python-can 缺失时应由 open() 抛 ImportError
    t = SocketCanTransport(channel="can0", bitrate=500_000)
    assert t.channel == "can0"
    assert isinstance(t, CanTransport)


def test_can_transport_is_abstract():
    try:
        CanTransport()
        raise AssertionError("CanTransport 应不可实例化")
    except TypeError:
        pass


def test_socketcan_not_open_raises_can_transport_error():
    # 未 open 的 SocketCanTransport: send/recv 必须抛 CanTransportError,
    # 不能泄漏裸 RuntimeError (驱动层单靠 CanTransportError 一种异常)
    t = SocketCanTransport(channel="can0", bitrate=500_000)
    frame = CanFrame(arbitration_id=0x0200, data=b"\x36\x6b")
    try:
        t.send(frame)
        raise AssertionError("send 应抛 CanTransportError")
    except CanTransportError:
        pass
    except Exception as exc:  # noqa: BLE001
        raise AssertionError(f"send 泄漏非 CanTransportError: {type(exc).__name__}")
    try:
        t.recv(0.001)
        raise AssertionError("recv 应抛 CanTransportError")
    except CanTransportError:
        pass
    except Exception as exc:  # noqa: BLE001
        raise AssertionError(f"recv 泄漏非 CanTransportError: {type(exc).__name__}")


if __name__ == "__main__":
    run_all(globals())
