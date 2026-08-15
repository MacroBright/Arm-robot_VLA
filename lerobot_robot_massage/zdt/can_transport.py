"""CAN 传输层抽象 + SocketCAN 实现 (spec §1 传输层).

SocketCanTransport 在方法内部惰性导入 python-can, 保证无 python-can
环境也能导入本模块 (驱动层单测依赖).
"""
import logging
from abc import ABC, abstractmethod
from typing import Optional

from .frames import CanFrame

logger = logging.getLogger(__name__)


class CanTransport(ABC):
    """驱动层依赖的传输抽象 (测试用 FakeTransport 注入)."""

    @abstractmethod
    def open(self) -> None:
        """打开总线 (含必要的接口配置)."""

    @abstractmethod
    def close(self) -> None:
        """关闭总线."""

    @abstractmethod
    def send(self, frame: CanFrame) -> None:
        """发送一帧."""

    @abstractmethod
    def recv(self, timeout_s: float) -> Optional[CanFrame]:
        """阻塞接收一帧, 超时返回 None."""


class SocketCanTransport(CanTransport):
    """Linux SocketCAN (can0) 后端.

    接口需预先 up (scripts/can_setup.sh, 需 sudo):
      ip link set can0 type can bitrate 500000 && ip link set can0 up
    """

    def __init__(self, channel: str = "can0", bitrate: int = 500_000):
        self.channel = channel
        self.bitrate = bitrate
        self._bus = None

    def open(self) -> None:
        import can  # 惰性导入: 无 python-can 环境也能 import 本模块
        self._bus = can.Bus(interface="socketcan", channel=self.channel)
        logger.info("SocketCAN opened on %s", self.channel)

    def close(self) -> None:
        if self._bus is not None:
            self._bus.shutdown()
            self._bus = None

    def send(self, frame: CanFrame) -> None:
        import can
        if self._bus is None:
            raise RuntimeError("SocketCAN not open")
        msg = can.Message(
            arbitration_id=frame.arbitration_id,
            is_extended_id=frame.is_extended_id,
            data=frame.data,
        )
        self._bus.send(msg)

    def recv(self, timeout_s: float) -> Optional[CanFrame]:
        import can
        if self._bus is None:
            raise RuntimeError("SocketCAN not open")
        msg = self._bus.recv(timeout=timeout_s)
        if msg is None:
            return None
        return CanFrame(arbitration_id=msg.arbitration_id, data=bytes(msg.data))
