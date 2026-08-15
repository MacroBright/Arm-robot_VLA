"""ZDT 帧编解码 — 纯函数, 无硬件/无 python-can 依赖 (spec §2.1)."""
from dataclasses import dataclass
from typing import Optional

from .config import CHECKSUM, POS_SCALE, VEL_SCALE


@dataclass
class CanFrame:
    """CAN 帧的轻量表示 (与 python-can Message 解耦)."""
    arbitration_id: int
    data: bytes
    is_extended_id: bool = True


def add_checksum(body: bytes) -> bytes:
    """附加 0x6B 校验字节; 若末字节已是 0x6B 则不重复."""
    if body and body[-1] == CHECKSUM:
        return bytes(body)
    return bytes(body) + bytes([CHECKSUM])


def verify_checksum(data: bytes) -> bool:
    """末字节 == 0x6B?"""
    return len(data) > 0 and data[-1] == CHECKSUM


def payload_chunks(payload: bytes) -> list[bytes]:
    """把 [功能码, 参数..., 0x6B] 拆成多帧数据段.

    每帧 = [功能码 + ≤7 参数], DLC≤8; 参数 >7 字节拆多帧 (功能码重复,
    包序号由 encode_frame 的 ID 低字节编码).
    """
    if not payload:
        return []
    func, rest = payload[0], payload[1:]
    return [bytes([func]) + rest[i:i + 7] for i in range(0, len(rest), 7)]


def encode_frame(addr: int, payload: bytes) -> list[CanFrame]:
    """ZDT 命令 → CAN 帧列表. 扩展帧 ID = (addr<<8)|seq."""
    return [
        CanFrame(arbitration_id=(addr << 8) | seq, data=chunk)
        for seq, chunk in enumerate(payload_chunks(payload))
    ]


def parse_frame(frame: CanFrame) -> tuple[int, int, bytes]:
    """回帧 → (addr, seq, data)."""
    return frame.arbitration_id >> 8, frame.arbitration_id & 0xFF, frame.data


# ── 参数编解码 ──────────────────────────────────────────────

def encode_pos3(pos_deg: float) -> bytes:
    """位置(°)×10 → 3 字节大端, 符号-幅值 (最高位=符号).

    约定需 bring-up candump 核实 (ZDT 文档未明示命令侧符号位, spec §9 风险表).
    """
    q = int(round(pos_deg * POS_SCALE))
    sign = 0x80 if q < 0 else 0x00
    mag = abs(q) & 0x7FFFFF
    return bytes([sign | (mag >> 16) & 0xFF, (mag >> 8) & 0xFF, mag & 0xFF])


def decode_pos3(data3: bytes, sign: Optional[int] = None) -> float:
    """3 字节位置(×10) + 符号 → 度.

    sign=None 时从 data3[0] 最高位推导符号 (与 encode_pos3 的
    符号-幅值约定一致); 调用方传显式 sign (如 driver read_pos 从回帧
    符号字节传) 时保持显式符号. 保证同一约定下负数 round-trip 互逆.
    """
    v = ((data3[0] & 0x7F) << 16) | (data3[1] << 8) | data3[2]
    if sign is None:
        sign = -1 if data3[0] & 0x80 else 1
    return v / POS_SCALE * sign


def encode_vel2(rpm: float) -> bytes:
    """转速(RPM)×10 → 2 字节大端."""
    q = int(round(rpm * VEL_SCALE)) & 0xFFFF
    return bytes([q >> 8, q & 0xFF])


def decode_vel2(data2: bytes) -> float:
    """2 字节速度(×10) → RPM."""
    return ((data2[0] << 8) | data2[1]) / VEL_SCALE
