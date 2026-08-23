"""frames 纯函数单测 (直接运行)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from lerobot_robot_massage.zdt.frames import (
    CanFrame, add_checksum, decode_pos3, decode_pos4, decode_vel2,
    encode_frame, encode_pos3, encode_pos4, encode_vel2, parse_frame,
    payload_chunks, verify_checksum,
)


def test_add_checksum():
    assert add_checksum(b"\x36") == b"\x36\x6b"
    # 已以 0x6B 结尾不重复
    assert add_checksum(b"\x36\x6b") == b"\x36\x6b"


def test_verify_checksum():
    assert verify_checksum(b"\x36\x6b") is True
    assert verify_checksum(b"\x36\x00") is False
    assert verify_checksum(b"") is False


def test_payload_chunks_single():
    # 参数 ≤7 → 1 帧
    chunks = payload_chunks(b"\x36\x6b")
    assert chunks == [b"\x36\x6b"]


def test_payload_chunks_multi():
    # 0xFB 命令体: FB + 9 参数 → 2 帧, 功能码 FB 重复
    body = bytes([0xFB, 0x01]) + encode_vel2(60.0) + encode_pos3(90.0) + b"\x0a\x00\x6b"
    chunks = payload_chunks(body)
    assert len(chunks) == 2
    assert chunks[0][0] == 0xFB and len(chunks[0]) == 8
    assert chunks[1][0] == 0xFB and len(chunks[1]) == 3


def test_encode_frame_ids():
    frames = encode_frame(0x05, b"\x36\x6b")
    assert len(frames) == 1
    assert frames[0].arbitration_id == (0x05 << 8)
    assert frames[0].is_extended_id is True

    # 多包: 包序号递增
    body = bytes([0xFB, 0x01]) + encode_vel2(60.0) + encode_pos3(90.0) + b"\x0a\x00\x6b"
    frames = encode_frame(0x05, body)
    assert [f.arbitration_id for f in frames] == [0x0500, 0x0501]


def test_parse_frame_roundtrip():
    f = CanFrame(arbitration_id=(0x03 << 8) | 1, data=b"\xfb\x00")
    assert parse_frame(f) == (0x03, 1, b"\xfb\x00")


def test_pos_roundtrip():
    for deg in (0.0, 90.0, 360.0, 7.5):
        assert abs(decode_pos3(encode_pos3(deg)) - deg) < 0.001, f"deg={deg}"


def test_pos_negative():
    # 负角度: 符号编码进最高位, decode 用符号字节(sign=-1)还原
    assert encode_pos3(-45.0)[0] & 0x80
    assert abs(decode_pos3(encode_pos3(-45.0), -1) + 45.0) < 0.001


def test_pos_negative_roundtrip_same_convention():
    # 同一约定下 (不传 sign, 从 data3[0] 最高位推导) 负数必须互逆
    for deg in (-360.0, -90.0, -45.0, -7.5):
        assert abs(decode_pos3(encode_pos3(deg)) - deg) < 0.001, f"deg={deg}"


# ── Emm42 V5.0 0x36 4字节解码测试 ──────────────────────────

def test_pos4_roundtrip():
    """decode_pos4(encode_pos4(deg), sign=1) ≈ deg (正角度)."""
    for deg in (0.0, 0.0494, 90.0, 180.0, 360.0):
        raw = encode_pos4(deg)
        assert len(raw) == 4
        assert abs(decode_pos4(raw, 1) - deg) < 0.01, f"deg={deg}"


def test_pos4_negative():
    """负角度: sign=-1 还原."""
    raw = encode_pos4(45.0)
    assert abs(decode_pos4(raw, -1) - (-45.0)) < 0.01


def test_pos4_small_angle():
    """真机 J5 证据: pos=9 → 9×360/65536 = 0.0494°."""
    raw = bytes([0x00, 0x00, 0x00, 0x09])
    assert abs(decode_pos4(raw, 1) - 0.0494) < 0.001


def test_pos4_90deg_bytes():
    """90° → 16384 = 0x00004000 → [0x00,0x00,0x40,0x00]."""
    raw = encode_pos4(90.0)
    assert raw == bytes([0x00, 0x00, 0x40, 0x00])


def test_vel_roundtrip():
    assert abs(decode_vel2(encode_vel2(600.0)) - 600.0) < 0.001


if __name__ == "__main__":
    failed = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS {name}")
            except Exception as exc:  # noqa: BLE001
                failed += 1
                print(f"FAIL {name}: {exc}")
    print("ALL PASS" if failed == 0 else f"{failed} FAILED")
    sys.exit(1 if failed else 0)
