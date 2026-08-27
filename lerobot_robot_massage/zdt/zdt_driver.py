"""ZDT X系列V2 驱动层: 命令构造 + 响应解析 + 超时重试 (spec §2).

组帧/拆包逻辑在 frames.py; 本层负责功能码语义与错误处理.
单帧响应命令 (位置/电流/到位). 37B 参数读写为多包, bring-up 工具另行处理 (不在本层).
"""
import logging
import time
from typing import Callable, Optional

from .can_transport import CanTransport, CanTransportError
from .config import (
    CHECKSUM, F_ENABLE, F_LEGACY_POS, F_READ_CUR, F_READ_POS, F_STOP, F_VEL,
)
from .frames import (
    add_checksum, decode_pos4, encode_frame, encode_pulse4, encode_vel2,
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


class TransportError(ZdtDriverError):
    """传输层异常 (CanTransportError 的驱动层投影).

    保证控制器只依赖 ZdtDriverError 一个异常边界 — 总线死时 send/recv
    抛出的 CanTransportError 在此被包成 ZdtDriverError 子类, 上层
    except ZdtDriverError 即可兜住.
    """


class CommunicationError(ZdtDriverError):
    """CAN 通信整体失败 (一个角度都没读到) — 调用方应 loud fail."""


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

    def move_pulse(self, addr: int, n_pulses: int, dir_cw: bool,
                   speed_rpm: float, acc: int = 20,
                   snF: bool = False) -> None:
        """固件 Emm_V5_Pos_Control 兼容位置命令 (0xFD 脉冲计数).

        2026-08 真机验证: 0xFB 直通限速命令本代驱动器不识别 (无任何响应),
        0xFD 脉冲命令正常执行. 布局:
          数据 = [FD, dir(0=CW/1=CCW), 速度RPM, acc, 脉冲4B, raF=0相对, snF, 6B]
        共 12B → 拆 2 帧 (8B + 4B), 帧 ID=(addr<<8)|seq.

        snF=多机同步标志: True 时电机收到命令但暂不运动, 等 multi_sync()
        广播 (00 FF 66 6B) 触发后才与其余 snF=1 电机同步启动.
        """
        d = 0 if dir_cw else 1
        # 0xFD 速度字段 = RPM 直传 (手册 0x05DC=1500RPM); 修复旧 ×10 bug.
        vel = int(max(1, round(abs(speed_rpm)))) & 0xFFFF
        body = (bytes([F_LEGACY_POS, d, (vel >> 8) & 0xFF, vel & 0xFF, acc])
                + encode_pulse4(n_pulses) + b"\x00\x00")  # raF=0 相对, snF 末字节覆写
        if snF:
            body = body[:-1] + b"\x01"                     # 末字节 0x00→0x01 启用同步
        self._request(addr, body, expect_response=False)

    def multi_sync(self, addrs: Optional[list[int]] = None) -> None:
        """广播多机同步运动 (00 FF 66 6B): 触发所有已带 snF=1 的 0xFD 命令同步启动.

        说明书 §多机通讯及同步控制: 先对各电机发 0xFD(snF=1), 再发本广播 (及逐轴直发),
        确保即使驱动器开启了 CAN ID 硬件地址过滤也能 100% 触发同步启动.
        """
        body = bytes([0xFF, 0x66, CHECKSUM])
        self._request(0x00, body, expect_response=False)
        if addrs:
            for addr in addrs:
                if addr != 0x00:
                    self._request(addr, body, expect_response=False)

    def set_vel(self, addr: int, rpm: float, slope: float = 0.0) -> None:
        """速度模式. 斜率/速度字段 = RPM 直传 (手册 0xF6 0x05DC=1500RPM)."""
        body = (bytes([F_VEL, 0x00]) + encode_vel2(slope)
                + encode_vel2(rpm) + b"\x00")
        self._request(addr, body, expect_response=False)

    def set_zero(self, addr: int) -> None:
        """设单圈零点 (0x93 88 01, 存储).

        ⚠ 这是"单圈回零零点设置", 不是"清零当前位置". 设完后触发 home(0x9A)
        会回到这个位置, 但 0x36 当前读数不变. 要让 0x36 立刻读出 0 用 reset_position.
        """
        body = bytes([0x93, 0x88, 0x01])
        self._request(addr, body, expect_response=False)

    def reset_position(self, addr: int) -> None:
        """清零当前位置 (0x0A 6D): 让 0x36 立刻读出 0.

        固件 robot.c:231/242 + robot_cmd.c:131 已验证可用的标定路线.
        与 set_zero(0x93) 的区别: 0x0A 6D 改变 0x36 当前读数, 但不影响回零目标;
        0x93 88 01 设回零目标但不改变 0x36 当前读数.
        A 任务标定方案 b (人工摆姿态→清零→此后 0x36 读相对偏移) 用本方法.
        """
        body = bytes([0x0A, 0x6D])
        self._request(addr, body, expect_response=False)

    def home(self, addr: int) -> None:
        """触发回零 (0x9A 00 00, 单圈就近模式)."""
        body = bytes([0x9A, 0x00, 0x00])
        self._request(addr, body, expect_response=False)

    # ── 读命令 (期待回帧) ─────────────────────────────────

    def read_pos(self, addr: int) -> float:
        """读实时位置 (度). Emm42 V5.0 回帧 [36, 符号, 位置4B, 6B].

        符号字节: 0x00=正, 0x01=负 (说明书 §7.4.4 + 固件 robot.c:1045 实测确认).
        位置字段4字节, 值×360/65536 = 电机轴角度 (说明书 §0x36 + 固件一致).
        """
        data = self._request(addr, bytes([F_READ_POS]), expect_response=True)
        sign = -1 if data[1] == 0x01 else 1
        return decode_pos4(data[2:6], sign)

    def read_current(self, addr: int) -> float:
        """读相电流 (mA). 回帧 [27, mA高, mA低, 6B] — bring-up 核实布局."""
        data = self._request(addr, bytes([F_READ_CUR]), expect_response=True)
        return float((data[1] << 8) | data[2])

    def read_flag(self, addr: int) -> int:
        """读状态标志 (0x3A). 回帧 [3A, 状态字节, 6B].

        状态位: &0x01=使能 &0x02=到位 &0x04=堵转 &0x08=堵转保护
        (2026-08-15 GitHub 调研 cantest/Emm_V5_CAN.c 确认; 安全层堵转监测用)
        """
        data = self._request(addr, bytes([0x3A]), expect_response=True)
        return data[1]

    # ── 内部: 发送 + 同步等回帧 + 重试 ─────────────────────

    def _request(self, addr: int, body: bytes,
                 expect_response: bool,
                 timeout_s: Optional[float] = None,
                 retries: Optional[int] = None) -> Optional[bytes]:
        """发送 (加校验); 期待回帧时等待并返回数据段 (含功能码), 否则 None.

        timeout_s / retries 为 None 时使用 self 默认值 (向后兼容).
        """
        payload = add_checksum(body)
        frames = encode_frame(addr, payload)
        t_out = timeout_s if timeout_s is not None else self.timeout_s
        r_retries = retries if retries is not None else self.retries
        for attempt in range(r_retries + 1):
            for f in frames:
                try:
                    self._t.send(f)
                except CanTransportError as exc:
                    raise TransportError(
                        f"send 失败 addr={addr:#04x} func={payload[0]:#04x}") from exc
            if not expect_response:
                return None
            resp = self._recv_for(addr, payload[0], timeout_s=t_out)
            if resp is not None:
                return resp
            logger.warning("timeout addr=%02X func=%02X attempt=%d/%d",
                           addr, payload[0], attempt, r_retries)
        raise TimeoutError(f"addr={addr:#04x} func={payload[0]:#04x} 超时")

    def _recv_for(self, addr: int, func: int,
                  timeout_s: Optional[float] = None) -> Optional[bytes]:
        """在 deadline 内收帧, 找到 (addr,func) 匹配回帧则返回, 否则 None.

        timeout_s 为 None 时使用 self 默认值 (向后兼容).
        """
        t_out = timeout_s if timeout_s is not None else self.timeout_s
        deadline = time.monotonic() + t_out
        while time.monotonic() < deadline:
            try:
                frame = self._t.recv(t_out)
            except CanTransportError as exc:
                raise TransportError(
                    f"recv 失败 addr={addr:#04x} func={func:#04x}") from exc
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

    # ── 读版本 (枚举 gate) ──────────────────────────────────

    def read_version(self, addr: int,
                     timeout_s: Optional[float] = None,
                     retries: Optional[int] = None) -> Optional[tuple[int, int]]:
        """探测 0x1F 版本. 超时/无响应 → None (枚举时作为离线判定)."""
        try:
            data = self._request(addr, bytes([0x1F]), expect_response=True,
                                 timeout_s=timeout_s, retries=retries)
        except ZdtDriverError:
            return None
        if data is None or len(data) < 3:
            return None
        return data[1], data[2]
