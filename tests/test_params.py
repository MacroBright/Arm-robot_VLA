"""ZdtParamsBlock 编解码单测."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from lerobot_robot_massage.zdt.params import (
    CAN_BAUD_LABELS, RESPONSE_LABELS,
    ZdtParamsBlock, encode_48_write, encode_ae_id_change, parse_42_response,
)
from lerobot_robot_massage.zdt.testutil import run_all

# 手册 §12.3 示例参数数组 (32 字节, 字段见 params.py 偏移表)
MANUAL_PARAMS = bytes.fromhex(
    "0001010202001001000004B008980BB803E8050700010001000807D007D00003")


def _mk_response(params: bytes = MANUAL_PARAMS) -> bytes:
    """模拟 ZdtBus.request_multi 重组输出: 概念帧 [42,37,24,params,6B]
    拆成 8B CAN 帧后, 首帧全保留、后续帧去重复功能码字节."""
    payload = bytes([0x42, 37, 24]) + params + bytes([0x6B])
    chunks = []
    for i in range(0, len(payload) - 1, 7):
        chunks.append(bytes([0x42]) + payload[i + 1:i + 1 + 7])
    out = chunks[0]
    for c in chunks[1:]:
        out += c[1:]
    return out


def test_parse_manual_example():
    block = parse_42_response(_mk_response())
    assert isinstance(block, ZdtParamsBlock)
    assert block.paramcount == 24
    assert block.bytecount == 37
    # #7 MStep = 0x10 = 16 → 默认 3200 脉冲/圈
    assert block.mstep == 16
    assert block.mstep_is_default
    assert block.pulses_per_rev == 3200
    # #4 P_Serial / #16 CAN_Baud / #17 Checksum / #18 Response / #19 S_PosTDP
    assert block.p_serial == 2
    assert block.can_baud == 7
    assert CAN_BAUD_LABELS[block.can_baud] == "500k"
    assert block.checksum == 0
    assert block.response == 1
    assert RESPONSE_LABELS[block.response] == "Receive"
    assert block.pos_tdp == 0
    # 2 字节大端字段
    assert block.ma_limit_ma == 0x0898          # 2200
    assert block.vm_limit_rpm == 0x0BB8         # 3000
    assert block.clog_rpm == 8
    assert block.clog_ma == 0x07D0              # 2000
    assert block.clog_ms == 0x07D0
    assert block.reach_window_x100_deg == 3


def test_mstep_256_maps_to_zero_byte():
    # MStep 0x00 = 256 细分
    p = bytearray(MANUAL_PARAMS)
    p[6] = 0x00
    block = parse_42_response(_mk_response(bytes(p)))
    assert block.mstep == 256
    assert not block.mstep_is_default
    assert block.pulses_per_rev == 200 * 256


def _mk_response_with(payload: bytes) -> bytes:
    """通用: 给定概念帧 payload, 模拟 request_multi 重组输出."""
    chunks = []
    for i in range(0, len(payload) - 1, 7):
        chunks.append(bytes([0x42]) + payload[i + 1:i + 1 + 7])
    out = chunks[0]
    for c in chunks[1:]:
        out += c[1:]
    return out


def test_short_paramcount_pads_and_warns():
    # bytecount=25 → 参数字节 20 → 写块补零到 32 + 告警
    payload = bytes([0x42, 25, 20]) + MANUAL_PARAMS[:20] + bytes([0x6B])
    block = parse_42_response(_mk_response_with(payload))
    assert len(block.params) == 32
    assert "补零" in block.warning
    assert "bytecount=25" in block.warning


def test_bad_function_rejected():
    try:
        parse_42_response(b"\x36\x25\x18" + MANUAL_PARAMS + b"\x6b")
        raise AssertionError("应抛 ValueError")
    except ValueError:
        pass


def test_patched_field():
    block = parse_42_response(_mk_response())
    # 改 #18 Response → 2 (Reached)
    p2 = block.patched(18, 2)
    assert p2.response == 2
    # 改 #12 Ma_Limit → 1800 (0x0708)
    p3 = block.patched(12, 1800, width=2)
    assert p3.ma_limit_ma == 1800
    # 原块不受影响
    assert block.response == 1
    assert block.ma_limit_ma == 0x0898


def test_encode_48_write():
    block = parse_42_response(_mk_response())
    payload = encode_48_write(block, save=True)
    assert payload[0] == 0x48 and payload[1] == 0xD1 and payload[2] == 0x01
    assert payload[3:35] == block.params     # 32 字节参数
    assert len(payload) == 35
    # volatile
    payload2 = encode_48_write(block, save=False)
    assert payload2[2] == 0x00


def test_encode_ae_id_change():
    payload = encode_ae_id_change(0x10)
    assert payload == bytes([0xAE, 0x4B, 0x01, 0x10])
    try:
        encode_ae_id_change(0)
        raise AssertionError("ID=0 应抛 ValueError")
    except ValueError:
        pass


if __name__ == "__main__":
    run_all(globals())
