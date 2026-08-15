"""ZdtController 单测 (ZdtDriver over FakeTransport, 注入回帧)."""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from lerobot_robot_massage.zdt.config import CHECKSUM, F_READ_CUR, F_READ_POS, ZdtConfig
from lerobot_robot_massage.zdt.controller import ZdtController
from lerobot_robot_massage.zdt.fakes import FakeTransport
from lerobot_robot_massage.zdt.testutil import run_all


def _mk(cfg=None):
    t = FakeTransport()
    ctrl = ZdtController(config=cfg or ZdtConfig(timeout_s=0.001, retries=0),
                         transport=t)
    return ctrl, t


def _inject_all_states(t: FakeTransport):
    addrs = [0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
    for i, addr in enumerate(addrs):
        v = 1000 + i   # 位置×10 = 100.0..100.5°
        t.inject(addr, F_READ_POS, b"\x01" + bytes([v >> 16, (v >> 8) & 0xFF, v & 0xFF]) + b"\x6b")
    for i, addr in enumerate(addrs):
        # 回帧 [27, mA高, mA低, 6B] — mA 位于 data[1:3] (与 driver 解析对齐)
        t.inject(addr, F_READ_CUR, b"\x00" + bytes([50 + i]) + b"\x6b")


def test_get_state_reads_all_joints():
    ctrl, t = _mk()
    _inject_all_states(t)
    angles, vels, loads = ctrl.get_state()
    assert len(angles) == 6
    assert abs(angles[0] - 100.0) < 0.01
    assert abs(loads[5] - 55.0) < 0.01


def test_get_state_empty_on_timeout():
    ctrl, t = _mk()   # 无注入 → read_pos 超时
    angles, vels, loads = ctrl.get_state()
    assert angles == [] and loads == []


def test_set_joints_clamps_and_sends():
    ctrl, t = _mk()
    # J1 限位 [0,360]: 送 720 → clamp 360
    ctrl.set_joints([720.0, 100.0, 0.0, 0.0, 45.0, 0.0])
    # 每轴 0xFB 2 帧 → 共 12 帧; 取 J1 的第 0/1 帧
    assert t.sent_ids[0] == 0x0200 and t.sent_ids[1] == 0x0201
    # 位置 360.0×10=3600 → 0x0E10 (速度2B 之后, data[4:7])
    assert t.sent[0].data[4:7] == bytes([0x00, 0x0E, 0x10])


def test_set_torque_sends_six_enables():
    ctrl, t = _mk()
    ctrl.set_torque(True)
    assert len(t.sent) == 6
    assert t.sent[0].data[:3] == bytes([0xF3, 0xAB, 0x01])


def test_e_stop_broadcasts():
    ctrl, t = _mk()
    ctrl.e_stop()
    assert t.sent[-1].arbitration_id == 0x0000
    assert t.sent[-1].data == bytes([0xFE, 0x98, 0x00, CHECKSUM])


def test_soft_reset_sends_init_pose():
    ctrl, t = _mk()
    ctrl.soft_reset()
    assert len(t.sent) == 12   # 6 轴 × 2 帧
    # J2 (addr 0x03) 45°×10=450 → 0x01C2, 位于其第 0 帧 data[4:7]
    j2 = [f for f in t.sent if f.arbitration_id >> 8 == 0x03]
    assert j2[0].data[4:7] == bytes([0x00, 0x01, 0xC2])


def test_rel_rotate_one_joint():
    ctrl, t = _mk()
    ctrl.rel_rotate(1, 5.0)   # joint_id 1-based
    assert t.sent[0].arbitration_id == 0x0200
    assert t.sent[0].data[1] == 0x00      # 相对标志
    assert t.sent[0].data[6] == 0x32      # 5°×10=50 → 0x32 (位置低字节, 速度2B 之后)


def test_tick_triggers_estop_when_stale():
    cfg = ZdtConfig(timeout_s=0.001, retries=0, watchdog_s=0.05)
    ctrl, t = _mk(cfg)
    ctrl._connected = True
    ctrl._last_io_s = time.monotonic() - 1.0   # 陈旧
    ctrl.tick()
    assert t.sent and t.sent[-1].arbitration_id == 0x0000


def test_tick_noop_when_fresh():
    cfg = ZdtConfig(timeout_s=0.001, retries=0, watchdog_s=0.05)
    ctrl, t = _mk(cfg)
    ctrl._connected = True
    ctrl._last_io_s = time.monotonic()
    ctrl.tick()
    assert t.sent == []


if __name__ == "__main__":
    run_all(globals())
