"""scan.py 枚举 + scheme 裁决单测."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from lerobot_robot_massage.zdt.fakes import QueueTransport
from lerobot_robot_massage.zdt.scan import read_telemetry, resolve_scheme, scan_bus
from lerobot_robot_massage.zdt.safety import JOINTS, MotorState
from lerobot_robot_massage.zdt.testutil import run_all
from lerobot_robot_massage.zdt.zdt_bus import ZdtBus


# ── resolve_scheme 纯函数 ──
def test_scheme_firmware_full():
    scheme, warns = resolve_scheme({1, 2, 3, 4, 5, 6})
    assert scheme == "firmware" and warns == []


def test_scheme_pc_full():
    scheme, warns = resolve_scheme({2, 3, 4, 5, 6, 7})
    assert scheme == "pc" and warns == []


def test_scheme_too_many():
    scheme, warns = resolve_scheme({1, 2, 3, 4, 5, 6, 7})
    assert scheme is None
    assert any("超过 6" in w for w in warns)


def test_scheme_ambiguous_subset():
    # {2..6} 同时是固件/PC 子集 → 歧义, 不得自动裁决 (防错标关节槽)
    scheme, warns = resolve_scheme({2, 3, 4, 5, 6})
    assert scheme is None
    assert any("模糊" in w for w in warns)


def test_scheme_partial_firmware_missing_joint():
    scheme, warns = resolve_scheme({1, 2, 3, 4, 5})   # 仅固件子集
    assert scheme == "firmware"
    assert any("J6 MISSING" in w for w in warns)


def test_scheme_out_of_range_id_ambiguous():
    # {1..5,9}: 9 不在任何一侧 → 无法自动裁决 (unexpected 分支为防御性死代码,
    # 因为能解析出 scheme 的输入必然全在 expected 内)
    scheme, warns = resolve_scheme({1, 2, 3, 4, 5, 9})
    assert scheme is None
    assert any("模糊" in w for w in warns)


# ── scan_bus 集成 (ZdtBus + QueueTransport) ──
def _mk_bus_with(found_ids):
    t = QueueTransport()
    bus = ZdtBus(t, watchdog_s=5.0)
    bus.open()
    for cid in found_ids:
        t.inject(cid, 0x1F, b"\xc9\x78\x6b")      # 版本 [1F,c9,78,6b]
        t.inject(cid, 0x3A, b"\x03\x6b")          # 标志 0x03
        t.inject(cid, 0x36, b"\x00\x00\x00\x6b")  # 位置
    return bus, t


def test_scan_firmware_scheme_maps_slots():
    bus, t = _mk_bus_with([1, 2, 3, 4, 5, 6])
    try:
        r = scan_bus(bus, id_range=(1, 6), timeout_s=0.05)
        assert r.scheme == "firmware"
        assert len(r.found) == 6
        assert r.found[1].joint_slot == 0
        assert r.found[6].joint_slot == 5
        assert r.found[3].fw_ver == (0xC9, 0x78)
        assert r.found[3].flags == 0x03
    finally:
        bus.close()


def test_scan_pc_scheme_maps_slots():
    bus, t = _mk_bus_with([2, 3, 4, 5, 6, 7])
    try:
        r = scan_bus(bus, id_range=(2, 7), timeout_s=0.05)
        assert r.scheme == "pc"
        assert r.found[2].joint_slot == 0
        assert r.found[7].joint_slot == 5
    finally:
        bus.close()


def test_scan_ambiguous_partial():
    bus, t = _mk_bus_with([2, 3, 4, 5, 6])
    try:
        r = scan_bus(bus, id_range=(2, 6), timeout_s=0.05)
        assert r.scheme is None
        assert any("模糊" in w for w in r.warnings)
        assert r.found[2].joint_slot is None     # 不自动标槽
    finally:
        bus.close()


def test_scan_all_offline_warns():
    bus, t = _mk_bus_with([])
    try:
        r = scan_bus(bus, id_range=(1, 3), timeout_s=0.03)
        assert r.found == {}
        assert any("无响应" in w for w in r.warnings)
    finally:
        bus.close()


def test_scan_forced_scheme_overrides():
    bus, t = _mk_bus_with([1, 2, 3, 4, 5, 6])
    try:
        r = scan_bus(bus, id_range=(1, 6), timeout_s=0.05, forced_scheme="pc")
        assert r.scheme == "pc"
        assert r.found[1].joint_slot == -1      # pc: slot = id-2 → 1-2 = -1 (覆盖即错标, 面板应告警)
        assert any("强制使用" in w for w in r.warnings)
    finally:
        bus.close()


def test_read_telemetry_fills_phase2_fields():
    bus, t = _mk_bus_with([])          # 空枚举, 手动注入遥测
    m = MotorState(can_id=0x02, online=True)
    # 注入: 位置 0x36=+50.0°(500), 速度 0x35=+30.0RPM(300), 电流 0x27=456mA,
    #       温度 0x39=+28°C, 编码器状态 0x3B=0x03(就绪+校准)
    t.inject(0x02, 0x36, b"\x00\x00\x01\xf4\x6b")     # 500 → 50.0°
    t.inject(0x02, 0x35, b"\x00\x01\x2c\x6b")         # 300 → 30.0RPM
    t.inject(0x02, 0x27, b"\x01\xc8\x6b")             # 456
    t.inject(0x02, 0x39, b"\x00\x1c\x6b")             # 28°C
    t.inject(0x02, 0x3B, b"\x03\x6b")                 # 编码器就绪+校准表就绪
    read_telemetry(bus, m)
    assert m.pos_deg is not None and abs(m.pos_deg - 50.0) < 0.1
    assert abs(m.velocity_rpm - 30.0) < 0.1
    assert m.current_ma == 456
    assert m.temp_c == 28.0
    assert m.home_flags == 0x03
    bus.close()


def test_read_telemetry_negative_sign():
    bus, t = _mk_bus_with([])
    m = MotorState(can_id=0x02, online=True)
    t.inject(0x02, 0x36, b"\x01\x00\x01\xf4\x6b")     # sign=01 → -50.0°
    t.inject(0x02, 0x35, b"\x01\x00\xc8\x6b")         # -20.0RPM
    t.inject(0x02, 0x27, b"\x00\x01\x6b")
    t.inject(0x02, 0x39, b"\x01\x05\x6b")             # -5°C
    t.inject(0x02, 0x3B, b"\x00\x6b")
    read_telemetry(bus, m)
    assert m.pos_deg == -50.0
    assert m.velocity_rpm == -20.0
    assert m.temp_c == -5.0
    bus.close()


def test_scan_initializes_tracked_deg_to_joint_init():
    """回归: 枚举裁决后 tracked_deg 应为该关节初始位姿角 (开机姿态, 全 0).

    否则 J2(软限位 [-1,150]) 首次步进 +1° 会被 clamp 放大成 150° 运动
    (电机持续运转, 只能急停). 见 test_safety::test_step_never_amplified_beyond_request.
    """
    bus, t = _mk_bus_with([2, 3, 4, 5, 6, 7])   # pc scheme: J1=0x02..J6=0x07
    try:
        r = scan_bus(bus, id_range=(2, 7), timeout_s=0.05)
        assert r.scheme == "pc"
        for cid, m in r.found.items():
            jm = JOINTS[m.joint_slot]
            assert m.tracked_deg == jm.init_deg, (
                f"0x{cid:02X} tracked={m.tracked_deg}, 期望 {jm.init_deg} (J{m.joint_slot+1})")
        # J2 (slot 1) 特别检查: 开机姿态初始角 0, 落在软限位内
        assert r.found[3].tracked_deg == JOINTS[1].init_deg == 0.0
    finally:
        bus.close()


# ── scan_via_driver (T5a) ──
def test_scan_via_driver_firmware_scheme():
    from lerobot_robot_massage.zdt.zdt_driver import ZdtDriver
    from lerobot_robot_massage.zdt.fakes import FakeTransport
    from lerobot_robot_massage.zdt.scan import scan_via_driver
    t = FakeTransport()
    drv = ZdtDriver(t, timeout_s=0.001, retries=0)
    for addr in range(0x01, 0x07):
        t.inject(addr, 0x1F, bytes([0x01, 0x01]) + b"\x6b")
    res = scan_via_driver(drv, id_range=(1, 8))
    assert res.scheme == "firmware"
    assert len(res.found) == 6
    assert res.found[0x01].joint_slot == 0
    assert res.found[0x06].joint_slot == 5


if __name__ == "__main__":
    run_all(globals())
