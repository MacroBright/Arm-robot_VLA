"""ZdtController 单测 (ZdtDriver over FakeTransport, 注入回帧)."""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from lerobot_robot_massage.zdt.config import CHECKSUM, F_READ_CUR, F_READ_POS, ZdtConfig
from lerobot_robot_massage.zdt.controller import ZdtController
from lerobot_robot_massage.zdt.fakes import (
    FailingRecvAfterNTransport, FailingRecvTransport, FailingSendTransport,
    FakeTransport,
)
from lerobot_robot_massage.zdt.testutil import run_all
from lerobot_robot_massage.zdt.zdt_driver import (
    CommunicationError, ZdtDriverError,
)


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


def test_get_state_raises_on_total_failure():
    ctrl, t = _mk()   # 无注入 → read_pos 超时, 一个角度都没读到
    try:
        ctrl.get_state()
        raise AssertionError("应抛 CommunicationError")
    except CommunicationError:
        pass
    except ZdtDriverError as exc:
        raise AssertionError(f"抛了非 CommunicationError: {type(exc).__name__}")
    # 异常应可被 ZdtDriverError 边界捕住 (非裸 RuntimeError)
    try:
        ctrl.get_state()
        raise AssertionError("应抛 ZdtDriverError")
    except ZdtDriverError:
        pass


def test_get_state_empty_on_partial_failure():
    # 部分读到 → 返回空列表 (不 raise, 调用方降级), 但非静默 (logger.warning)
    ctrl, t = _mk()
    addrs = [0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
    for i, addr in enumerate(addrs[:3]):
        v = 1000 + i
        t.inject(addr, F_READ_POS,
                 b"\x01" + bytes([v >> 16, (v >> 8) & 0xFF, v & 0xFF]) + b"\x6b")
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


def test_rel_rotate_invalid_joint_rejected():
    # joint_id 越界 (0/-1/7) 必须显式报错, 不能静默寻到 J5/J6
    for bad in (0, -1, 7, 99):
        ctrl, t = _mk()
        try:
            ctrl.rel_rotate(bad, 5.0)
            raise AssertionError(f"joint_id={bad} 应抛 ValueError")
        except ValueError:
            pass
        assert t.sent == []   # 未发送任何帧


def test_get_state_empty_on_transport_death():
    # 总线死 (recv 抛 CanTransportError) 但已部分读到 → get_state 返回空, 不泄漏
    t = FailingRecvAfterNTransport(fail_on_recv_n=3)
    for i, addr in enumerate([0x02, 0x03]):
        v = 1000 + i
        t.inject(addr, F_READ_POS,
                 b"\x01" + bytes([v >> 16, (v >> 8) & 0xFF, v & 0xFF]) + b"\x6b")
    ctrl = ZdtController(config=ZdtConfig(timeout_s=0.001, retries=0), transport=t)
    angles, vels, loads = ctrl.get_state()
    assert angles == [] and loads == []


def test_get_state_total_transport_death_raises_communication_error():
    # 总线死且一个角度都没读到 → CommunicationError (ZdtDriverError), 非裸 CanTransportError
    t = FailingRecvTransport()
    ctrl = ZdtController(config=ZdtConfig(timeout_s=0.001, retries=0), transport=t)
    try:
        ctrl.get_state()
        raise AssertionError("应抛 CommunicationError")
    except CommunicationError:
        pass
    except ZdtDriverError as exc:
        raise AssertionError(f"抛了非 CommunicationError: {type(exc).__name__}: {exc}")


def test_tick_swallows_transport_error():
    # 总线死时 tick 的 e_stop 发送失败 (TransportError) 必须被吞掉留痕, 不传播
    t = FailingSendTransport()
    ctrl = ZdtController(config=ZdtConfig(timeout_s=0.001, retries=0, watchdog_s=0.05),
                         transport=t)
    ctrl._connected = True
    ctrl._last_io_s = time.monotonic() - 1.0   # 陈旧 → 触发 e_stop 路径
    ctrl.tick()   # 不应抛任何异常


def test_connect_failure_estop_and_close():
    # 第 4 次 read_pos 的 recv 抛 CanTransportError → connect 失败:
    #   已使能轴先 e_stop 广播 (addr=0x00) + transport 已 close + _connected=False
    t = FailingRecvAfterNTransport(fail_on_recv_n=4)
    for i, addr in enumerate([0x02, 0x03, 0x04]):
        v = 1000 + i
        t.inject(addr, F_READ_POS,
                 b"\x01" + bytes([v >> 16, (v >> 8) & 0xFF, v & 0xFF]) + b"\x6b")
    ctrl = ZdtController(config=ZdtConfig(timeout_s=0.001, retries=0), transport=t)
    try:
        ctrl.connect()
        raise AssertionError("connect 应抛错")
    except ZdtDriverError:
        pass
    # e_stop 广播帧 (addr=0x00, 停止命令)
    assert t.sent[-1].arbitration_id == 0x0000
    assert t.sent[-1].data == bytes([0xFE, 0x98, 0x00, CHECKSUM])
    assert t.closed is True
    assert ctrl._connected is False


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
