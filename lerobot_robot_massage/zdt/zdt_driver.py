"""ZDT X系列V2 驱动层: 命令构造 + 响应解析 + 超时重试 (spec §2).

组帧/拆包逻辑在 frames.py; 本层负责功能码语义与错误处理.
单帧响应命令 (位置/电流/到位). 37B 参数读写为多包, bring-up 工具另行处理 (不在本层).
"""
import logging
import time
from typing import Callable, Optional

from .can_transport import CanTransport
from .config import CHECKSUM, F_ENABLE, F_POS, F_READ_CUR, F_READ_POS, F_STOP, F_VEL
from .frames import (
    add_checksum, decode_pos3, encode_frame, encode_pos3, encode_vel2,
    parse_frame, verify_checksum,
)

logger = logging.getLogger(__name__)

ArrivedCallback = Callable[[int], None]  # 参数 = 关节地址


class ZdtDriverError(Exception):
    """驱动层错误基类."""


class TimeoutError(ZdtDriverError):
    """命令超时 (重试耗尽)."""


class ChecksumError(ZdtDriverError):
    """回帧校验失败."""


class ZdtDriver:
    """向 6×ZDT 驱动发命令/读状态. 同步请求-响应, 单线程用."""

    def __init__(self, transport: CanTransport, timeout_s: float = 0.1,
                 retries: int = 3):
        self._t = transport
        self.timeout_s = timeout_s
        self.retries = retries
        self.on_arrived: Optional[ArrivedCallback] = None

    # ── 命令 (fire-and-forget 无回帧) ────────────────────

    def enable(self, addr: int, state: bool) -> None:
        body = bytes([F_ENABLE, 0xAB, 1 if state else 0, 0x00])
        self._request(addr, body, expect_response=False)

    def stop(self, addr: int) -> None:
        body = bytes([F_STOP, 0x98, 0x00])
        self._request(addr, body, expect_response=False)

    def stop_all(self) -> None:
        """广播立即停止 (addr=0x00)."""
        body = bytes([F_STOP, 0x98, 0x00])
        self._request(0x00, body, expect_response=False)

    def move_abs(self, addr: int, pos_deg: float, speed_rpm: float) -> None:
        """直通限速位置, 绝对. 位置(°)×10, 速度(RPM)×10.

        命令体 10 字节 = FB 01 00 + 位置3B + 速度2B + 0A 00
        (test_driver 锁定首帧 data[3:6]=位置3B; bring-up candump 核实, spec §9).
        """
        body = (bytes([F_POS, 0x01, 0x00]) + encode_pos3(pos_deg)
                + encode_vel2(speed_rpm) + b"\x0a\x00")
        self._request(addr, body, expect_response=False)

    def move_rel(self, addr: int, delta_deg: float, speed_rpm: float) -> None:
        """直通限速位置, 相对."""
        body = (bytes([F_POS, 0x00, 0x00]) + encode_pos3(delta_deg)
                + encode_vel2(speed_rpm) + b"\x0a\x00")
        self._request(addr, body, expect_response=False)

    def set_vel(self, addr: int, rpm: float, slope: float = 0.0) -> None:
        """速度模式. 斜率/速度均 ×10."""
        body = (bytes([F_VEL, 0x00]) + encode_vel2(slope)
                + encode_vel2(rpm) + b"\x00")
        self._request(addr, body, expect_response=False)

    def set_zero(self, addr: int) -> None:
        """设单圈零点 (0x93 88 01, 存储)."""
        body = bytes([0x93, 0x88, 0x01])
        self._request(addr, body, expect_response=False)

    def home(self, addr: int) -> None:
        """触发回零 (0x9A 00 00)."""
        body = bytes([0x9A, 0x00, 0x00])
        self._request(addr, body, expect_response=False)

    # ── 读命令 (期待回帧) ─────────────────────────────────

    def read_pos(self, addr: int) -> float:
        """读实时位置 (度). 回帧 [36, 符号, 位置×10 3B, 6B]."""
        data = self._request(addr, bytes([F_READ_POS]), expect_response=True)
        sign = -1 if data[1] == 0x80 else 1
        return decode_pos3(data[2:5], sign)

    def read_current(self, addr: int) -> float:
        """读相电流 (mA). 回帧 [27, 保留字节, mA高, mA低, 6B] — bring-up 核实布局."""
        data = self._request(addr, bytes([F_READ_CUR]), expect_response=True)
        return float((data[2] << 8) | data[3])

    def read_flag(self, addr: int) -> int:
        """读状态标志 (0x3A). 回帧 [3A, 状态字节, 6B].

        状态位: &0x01=使能 &0x02=到位 &0x04=堵转 &0x08=堵转保护
        (2026-08-15 GitHub 调研 cantest/Emm_V5_CAN.c 确认; 安全层堵转监测用)
        """
        data = self._request(addr, bytes([0x3A]), expect_response=True)
        return data[1]

    # ── 内部: 发送 + 同步等回帧 + 重试 ─────────────────────

    def _request(self, addr: int, body: bytes,
                 expect_response: bool) -> Optional[bytes]:
        """发送 (加校验); 期待回帧时等待并返回数据段 (含功能码), 否则 None."""
        payload = add_checksum(body)
        frames = encode_frame(addr, payload)
        for attempt in range(self.retries + 1):
            for f in frames:
                self._t.send(f)
            if not expect_response:
                return None
            resp = self._recv_for(addr, payload[0])
            if resp is not None:
                return resp
            logger.warning("timeout addr=%02X func=%02X attempt=%d/%d",
                           addr, payload[0], attempt, self.retries)
        raise TimeoutError(f"addr={addr:#04x} func={payload[0]:#04x} 超时")

    def _recv_for(self, addr: int, func: int) -> Optional[bytes]:
        """在 deadline 内收帧, 找到 (addr,func) 匹配回帧则返回, 否则 None."""
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            frame = self._t.recv(self.timeout_s)
            if frame is None:
                continue
            r_addr, _seq, data = parse_frame(frame)
            if not verify_checksum(data):
                logger.warning("checksum fail addr=%02X data=%s",
                               r_addr, data.hex())
                continue
            r_func = data[0]
            if r_func == 0xFD and self.on_arrived is not None:
                self.on_arrived(r_addr)
                continue
            if r_addr == addr and r_func == func:
                return data
        return None
