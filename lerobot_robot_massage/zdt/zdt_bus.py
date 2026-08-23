"""线程安全 CAN 总线 — 后台读线程 + 收件箱 + 同步请求 + 看门狗.

背景: 交互面板需要 e_stop 在任意时刻零延迟生效. SocketCAN 发送本身线程安全,
  因此把**唯一的 recv() 调用者**放进后台读线程, 主线程只负责发送与等待匹配回帧,
  任何时刻都能发 e_stop 广播而不被阻塞.

看门狗: 仅当 set_watchdog_armed(True) (电机使能/运动中) 时武装;
  超过 watchdog_s 无任何 CAN IO → 广播 e_stop 并闩锁 (防止 UI 卡死/总线死时
  电机继续受控风险). 发送也计入 IO (UI 1Hz 轮询保活).
"""
import logging
import queue
import threading
import time
from typing import Callable, Optional

from .can_transport import CanTransport, CanTransportError
from .frames import (
    CanFrame, add_checksum, encode_frame, parse_frame, verify_checksum,
)

logger = logging.getLogger(__name__)

F_ARRIVED: int = 0xFD          # 驱动器到位通知帧 (数据段[0])
F_STOP: int = 0xFE             # 立即停止


class ZdtBus(CanTransport):
    """包装任意 CanTransport, 提供面板所需的线程安全请求原语."""

    def __init__(self, transport: CanTransport, watchdog_s: float = 3.0,
                 on_watchdog: Optional[Callable[[], None]] = None):
        self._t = transport
        self.watchdog_s = watchdog_s
        self.on_watchdog = on_watchdog
        self._inbox: "queue.Queue[CanFrame]" = queue.Queue()
        self._arrived: dict[int, bool] = {}
        self._lock = threading.Lock()
        self._last_io_s = time.monotonic()
        self._watchdog_armed = False
        self._watchdog_triggered = False
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    # ── CanTransport ABC ──
    def open(self) -> None:
        self._t.open()
        self._stop.clear()
        self._thread = threading.Thread(target=self._reader_loop,
                                        daemon=True, name="zdt-bus-reader")
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        self._t.close()

    def send(self, frame: CanFrame) -> None:
        self._touch()
        self._t.send(frame)

    def recv(self, timeout_s: float) -> Optional[CanFrame]:
        """仅 ABC 兼容 — 实际读帧在后台线程. 返回收件箱里任意帧."""
        try:
            return self._inbox.get(timeout=timeout_s)
        except queue.Empty:
            return None

    # ── 面板 API ──
    def send_payload(self, addr: int, payload: bytes) -> None:
        for f in encode_frame(addr, payload):
            self.send(f)

    def broadcast(self, payload: bytes) -> None:
        """广播到 addr=0x00 (所有电机)."""
        self.send_payload(0x00, payload)

    def stop_all(self) -> None:
        """广播立即停止 [FE 98 00 6B] (不断电, 电机保持力矩)."""
        self.broadcast(add_checksum(bytes([F_STOP, 0x98, 0x00])))

    def request(self, addr: int, payload: bytes, func: int,
                timeout_s: float = 0.2) -> Optional[bytes]:
        """发送负载并等待 (addr,func) 匹配回帧; 不匹配帧丢弃; 超时 None."""
        self.send_payload(addr, payload)
        deadline = time.monotonic() + timeout_s
        while True:
            frame = self._recv_until(deadline)
            if frame is None:
                return None
            r_addr, _seq, data = parse_frame(frame)
            if r_addr == addr and len(data) > 0 and data[0] == func:
                return data

    def request_multi(self, addr: int, payload: bytes, func: int,
                      timeout_s: float = 0.5,
                      gap_s: float = 0.05) -> Optional[bytes]:
        """发送负载并拼装多帧响应 (0x42 参数块): 首帧全保留, 后续帧去重复功能码.

        以"匹配帧间隙 > gap_s"或总超时判定响应结束.
        """
        self.send_payload(addr, payload)
        parts: list[bytes] = []
        deadline = time.monotonic() + timeout_s
        last_match = time.monotonic()
        while True:
            frame = self._recv_until(deadline)
            if frame is None:
                break
            r_addr, _seq, data = parse_frame(frame)
            if r_addr == addr and len(data) > 0 and data[0] == func:
                if not parts:
                    parts.append(data)
                else:
                    parts.append(data[1:])
                last_match = time.monotonic()
            elif parts:
                break                       # 收集后出现不匹配 → 结束
            else:
                continue                    # 静默丢不匹配
            if time.monotonic() - last_match >= gap_s:
                break
        return b"".join(parts) if parts else None

    def is_arrived(self, addr: int) -> bool:
        with self._lock:
            return self._arrived.get(addr, False)

    def clear_arrived(self, addr: int) -> None:
        with self._lock:
            self._arrived.pop(addr, None)

    def set_watchdog_armed(self, armed: bool) -> None:
        self._watchdog_armed = armed
        self._watchdog_triggered = False
        self._touch()

    @property
    def watchdog_triggered(self) -> bool:
        return self._watchdog_triggered

    # ── 内部 ──
    def _touch(self) -> None:
        with self._lock:
            self._last_io_s = time.monotonic()

    def _recv_until(self, deadline: float) -> Optional[CanFrame]:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        try:
            return self._inbox.get(timeout=min(remaining, 0.1))
        except queue.Empty:
            return None

    def _reader_loop(self) -> None:
        while not self._stop.is_set():
            try:
                frame = self._t.recv(0.1)
            except CanTransportError:
                # 总线死 → 若看门狗武装立即触发 e_stop (物理层干预)
                logger.error("总线接收失败")
                if self._watchdog_armed and not self._watchdog_triggered:
                    self._fire_watchdog()
                continue
            if frame is None:
                if self._watchdog_armed and not self._watchdog_triggered:
                    with self._lock:
                        idle = time.monotonic() - self._last_io_s
                    if idle > self.watchdog_s:
                        self._fire_watchdog()
                continue
            self._touch()
            r_addr, _seq, data = parse_frame(frame)
            if len(data) > 0 and data[0] == F_ARRIVED and r_addr != 0:
                # 到位通知 → 路由到 arrived 标志, 不进收件箱
                with self._lock:
                    self._arrived[r_addr] = True
                continue
            if not verify_checksum(data):
                # 仅告警不丢弃: 多帧响应 (0x42) 的中间帧末字节不是整体校验 0x6B,
                # 丢弃会破坏拼装. 最终校验由 request_multi → parse_42_response 完成.
                logger.warning("校验字节非 0x6B addr=%02X data=%s (多帧中间帧?)",
                               r_addr, data.hex())
            self._inbox.put(frame)

    def _fire_watchdog(self) -> None:
        self._watchdog_triggered = True
        logger.error("WATCHDOG: %.1fs 无 CAN IO → 广播 e_stop", self.watchdog_s)
        try:
            self.stop_all()
        except CanTransportError:
            logger.error("WATCHDOG: e_stop 发送失败, 总线已死 — 需人工物理干预")
        if self.on_watchdog:
            try:
                self.on_watchdog()
            except Exception:  # noqa: BLE001
                logger.exception("on_watchdog 回调失败")
