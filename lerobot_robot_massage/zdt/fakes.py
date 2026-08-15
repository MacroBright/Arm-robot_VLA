"""测试用假对象."""
from typing import Optional

from .can_transport import CanTransport, CanTransportError
from .frames import CanFrame


class FakeTransport(CanTransport):
    """记录发送帧 + 可注入回帧 (FIFO). recv 无注入时立即返回 None."""

    def __init__(self) -> None:
        self.sent: list[CanFrame] = []
        self.responses: list[CanFrame] = []
        self.opened = False
        self.closed = False

    def open(self) -> None:
        self.opened = True

    def close(self) -> None:
        self.closed = True

    def send(self, frame: CanFrame) -> None:
        self.sent.append(frame)

    def recv(self, timeout_s: float) -> Optional[CanFrame]:
        if self.responses:
            return self.responses.pop(0)
        return None

    def inject(self, addr: int, func: int, data_body: bytes) -> None:
        """注入一帧回帧: ID=(addr<<8), data=[func]+body."""
        self.responses.append(
            CanFrame(arbitration_id=(addr << 8), data=bytes([func]) + data_body))

    @property
    def sent_ids(self) -> list[int]:
        """已发帧 ID 列表 (断言辅助)."""
        return [f.arbitration_id for f in self.sent]


class FailingSendTransport(FakeTransport):
    """send 抛 CanTransportError (模拟总线发送死亡)."""

    def send(self, frame: CanFrame) -> None:
        raise CanTransportError("simulated send bus death")


class FailingRecvTransport(FakeTransport):
    """recv 在注入耗尽后抛 CanTransportError (模拟总线接收死亡)."""

    def recv(self, timeout_s: float) -> Optional[CanFrame]:
        if self.responses:
            return self.responses.pop(0)
        raise CanTransportError("simulated recv bus death")


class FailingRecvAfterNTransport(FakeTransport):
    """第 n 次 recv 抛 CanTransportError, 之前的 recv 走 FakeTransport 逻辑."""

    def __init__(self, fail_on_recv_n: int) -> None:
        super().__init__()
        self._fail_on_recv_n = fail_on_recv_n
        self._recv_calls = 0

    def recv(self, timeout_s: float) -> Optional[CanFrame]:
        self._recv_calls += 1
        if self._recv_calls >= self._fail_on_recv_n:
            raise CanTransportError(
                f"simulated recv bus death on call #{self._recv_calls}")
        return super().recv(timeout_s)
