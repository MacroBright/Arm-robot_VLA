"""ZDT 驱动器参数块编解码 — 0x42 读 / 0x48 写 / 0xAE 改 ID.

协议来源: ZDT_X系列_V2步进闭环驱动说明书Rev1.0.md §7.4.4/§7.4.5/§12.2/§12.3.
CAN 帧约定与 frames.py 一致: 地址在帧 ID 高字节, 负载首字节为功能码, 末字节 0x6B.
0x42 响应为多帧 (37 字节概念帧), 由调用方 (ZdtBus.request_multi) 拼装后传入.

⚠ 手册标注的字节数/参数个数存在笔误 (0x25/0x27 之争): 本模块从响应读 bytecount/
paramcount 字段动态解析, 并在不一致时告警. 真机 candump 核实.
"""
from dataclasses import dataclass, field

# 功能码
F_READ_PARAMS: int = 0x42
F_WRITE_PARAMS: int = 0x48
F_CHANGE_ID: int = 0xAE

# 标签映射 (手册 §12.3: 菜单选项为索引值, 非原始数值)
MSTEP_LABELS: dict[int, str] = {
    0: "256", 1: "1", 2: "2", 4: "4", 8: "8", 16: "16",
    32: "32", 64: "64", 128: "128",
}
CAN_BAUD_LABELS: dict[int, str] = {
    0: "10k", 1: "20k", 2: "50k", 3: "83.3k", 4: "100k",
    5: "125k", 6: "250k", 7: "500k", 8: "800k", 9: "1M",
}
CHECKSUM_LABELS: dict[int, str] = {0: "0x6B", 1: "XOR", 2: "CRC-8", 3: "Modbus"}
RESPONSE_LABELS: dict[int, str] = {0: "None", 1: "Receive", 2: "Reached",
                                   3: "Both", 4: "Other"}
P_SERIAL_LABELS: dict[int, str] = {0: "RxTx_OFF", 1: "ESI_ALO",
                                   2: "UART_FUN", 3: "CAN1_MAP"}
CTRLMODE_LABELS: dict[int, str] = {0: "Pulse", 1: "CR_VFOC"}
POS_TDP_LABELS: dict[int, str] = {0: "×10", 1: "×100"}

# 参数字节块长度 (24 参数 = 32 字节)
PARAMS_LEN: int = 32

# 字段 → 字节偏移 (0-based). 单字节字段 #1-10 连续; #11-14/#21-24 为 2 字节,
# 偏移因前序 2 字节字段累积, 不能用 n-1 直接算.
_BYTE_OFFSET: dict[int, int] = {1: 0, 2: 1, 3: 2, 4: 3, 5: 4, 6: 5,
                                7: 6, 8: 7, 9: 8, 10: 9,
                                15: 18, 16: 19, 17: 20, 18: 21, 19: 22, 20: 23}
_U16_OFFSET: dict[int, int] = {11: 10, 12: 12, 13: 14, 14: 16,
                               21: 24, 22: 26, 23: 28, 24: 30}


def _be16(b: bytes, off: int) -> int:
    """大端 2 字节.

    手册示例均为大端解码 (04 B0 → 0x04B0=1200mA, 5C 6A → 23658mV, 8D 9E → 36254),
    与 read_pos/read_current 等单值读取一致. 真机 candump 复核.
    """
    return (b[off] << 8) | b[off + 1]


@dataclass
class ZdtParamsBlock:
    """0x42 读回的 24 参数块 (32 字节) + 元数据. 索引为 1-based 参数号."""
    params: bytes                       # 32 字节参数数组
    bytecount: int = 37
    paramcount: int = 24
    raw: bytes = b""                    # 原始拼装帧 (含 0x42/bytecount/paramcount/尾 0x6B)
    warning: str = ""                   # bytecount 与预期不符等提示

    # ── 字段读取 (1-based 参数号 → 显式字节偏移) ──
    def _b(self, n: int) -> int:
        return self.params[_BYTE_OFFSET[n]]

    def _u16(self, n: int) -> int:
        return _be16(self.params, _U16_OFFSET[n])

    @property
    def mstep(self) -> int:
        """#7 细分: 0x00=256, 否则 1..128. 默认 16 = 3200 脉冲/圈."""
        v = self._b(7)
        return 256 if v == 0 else v

    @property
    def mstep_is_default(self) -> bool:
        """True=16 细分 (与 3200 脉冲/圈假设一致)."""
        return self.mstep == 16

    @property
    def pulses_per_rev(self) -> int:
        """按实际细分推算脉冲/圈 (200 步电机)."""
        return 200 * self.mstep

    @property
    def p_serial(self) -> int:
        return self._b(4)

    @property
    def can_baud(self) -> int:
        """#16 CAN 波特率索引."""
        return self._b(16)

    @property
    def checksum(self) -> int:
        return self._b(17)

    @property
    def response(self) -> int:
        return self._b(18)

    @property
    def pos_tdp(self) -> int:
        return self._b(19)

    @property
    def ma_limit_ma(self) -> int:
        """#12 闭环最大电流 mA."""
        return self._u16(12)

    @property
    def vm_limit_rpm(self) -> int:
        """#13 最大转速 RPM."""
        return self._u16(13)

    @property
    def clog_pro(self) -> int:
        return self._b(20)

    @property
    def clog_rpm(self) -> int:
        return self._u16(21)

    @property
    def clog_ma(self) -> int:
        return self._u16(22)

    @property
    def clog_ms(self) -> int:
        return self._u16(23)

    @property
    def reach_window_x100_deg(self) -> int:
        return self._u16(24)

    # ── 打补丁 → 新块 (用于 0x48 写) ──
    def patched(self, field_no: int, value: int, width: int = 1) -> "ZdtParamsBlock":
        """克隆并把 1-based 参数 field_no 改成 value (width=1/2, 大端)."""
        data = bytearray(self.params)
        if width == 1:
            data[_BYTE_OFFSET[field_no]] = value & 0xFF
        elif width == 2:
            off = _U16_OFFSET[field_no]
            data[off] = (value >> 8) & 0xFF
            data[off + 1] = value & 0xFF
        else:
            raise ValueError(f"width 仅支持 1/2, got {width}")
        return ZdtParamsBlock(params=bytes(data), bytecount=self.bytecount,
                              paramcount=self.paramcount, raw=self.raw,
                              warning=self.warning)


def parse_42_response(data: bytes) -> ZdtParamsBlock:
    """解析 0x42 多帧拼装后的响应.

    data 布局 (请求方拼装, 保留首帧功能码): [0x42, bytecount, paramcount,
    <参数字节>, 0x6B]. bytecount=37 指概念串行帧 (含被移入帧 ID 的地址字节):
    37 = 地址1+功能1+bytecount1+paramcount1+参数32+校验1 → 参数字节 = bytecount-5.
    paramcount=24 是参数字段数 (非字节数, 8 个 2 字节字段 → 32 字节).
    """
    if not data or data[0] != F_READ_PARAMS:
        raise ValueError(f"非 0x42 响应: {data.hex()}")
    bytecount = data[1]
    paramcount = data[2]
    if data[-1] != 0x6B:
        raise ValueError(f"响应末字节非 0x6B: {data.hex()}")
    params_len = bytecount - 5
    if params_len < 0 or len(data) < 3 + params_len:
        raise ValueError(f"响应长度异常 bytecount={bytecount}: {data.hex()}")
    params = data[3:3 + params_len]
    warning = ""
    if bytecount != 37:
        warning = f"bytecount={bytecount} (手册标注 37, 需 candump 核实)"
    if paramcount != 24:
        warning += (f" paramcount={paramcount} (预期 24)").strip()
    if len(params) < PARAMS_LEN:
        warning += (f" 参数块 {len(params)}B < 32B, 写块将补零").strip()
        params = params + bytes(PARAMS_LEN - len(params))
    elif len(params) > PARAMS_LEN:
        warning += (f" 参数块 {len(params)}B > 32B, 截断").strip()
        params = params[:PARAMS_LEN]
    return ZdtParamsBlock(params=params, bytecount=bytecount,
                          paramcount=paramcount, raw=data, warning=warning.strip())


def encode_48_write(block: ZdtParamsBlock, save: bool = True) -> bytes:
    """编码 0x48 写参数负载 (不含校验, 调用方 add_checksum + 多帧).

    负载 = [0x48, 0xD1, save(1=存储/0=volatile), <32 字节参数>].
    返回的负载会由 frames.encode_frame 拆成多帧 (首帧≤8 字节).
    """
    return bytes([F_WRITE_PARAMS, 0xD1, 1 if save else 0]) + bytes(block.params)


def encode_ae_id_change(new_id: int) -> bytes:
    """编码 0xAE 改 ID 负载 (fire-and-forget). 旧 ID 永久失效, 不可逆."""
    if not 1 <= new_id <= 255:
        raise ValueError(f"ID 需 1-255, got {new_id}")
    return bytes([F_CHANGE_ID, 0x4B, 0x01, new_id])
