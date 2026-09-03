"""ZdtDriver 读事务串行化测试 — 修复后台轮询 + 主线程宏回读并发 recv 抢响应.

背景 (2026-08-30 真机): unified_teleop 里 ArmStatePoller(25Hz) 与主线程的
宏回读 (READY/HOME) 会**并发**调用同一个 ZdtDriver。无锁时两线程同时进入
recv() 等回帧，内核把应答交给任意一个线程 → 另一个线程丢回帧 → 伪超时 /
get_real_state 降级 / 宏动作 TimeoutError 崩溃。

本测试用 threading.Barrier 强行让两个并发读线程同时推进到 recv() 内部：
  - 加锁后 (read transaction 串行化) 第二个线程先阻塞在 _read_lock 上，
    不可能与第一个线程同时进入 recv()，max_active 恒为 1；
  - 无锁时两线程同时到达 barrier 并释放，max_active == 2 → 断言失败。
"""
import sys
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from lerobot_robot_massage.zdt.config import F_READ_POS
from lerobot_robot_massage.zdt.fakes import QueueTransport
from lerobot_robot_massage.zdt.testutil import run_all
from lerobot_robot_massage.zdt.zdt_driver import ZdtDriver


class RacyTransport(QueueTransport):
    """QueueTransport + recv 内部 Barrier: 只有两线程真正并发才会同时放行."""

    def __init__(self, gate: threading.Barrier):
        super().__init__()
        self._gate = gate
        self.active = 0
        self.max_active = 0

    def recv(self, timeout_s):
        self.active += 1
        self.max_active = max(self.max_active, self.active)
        try:
            # 2 方栅栏: 若第二个线程在锁外能进入 recv, 则双线程同时到达释放;
            # 若被 _read_lock 挡住, 则只有一方到达, 2s 后 BrokenBarrierError 放行.
            self._gate.wait(timeout=2.0)
        except threading.BrokenBarrierError:
            pass
        finally:
            self.active -= 1
        return super().recv(timeout_s)


def _pos_body(value: int) -> bytes:
    """0x36 回帧数据段 (符号00正 + 4字节低位在前) + 校验 0x6b."""
    return b"\x01" + value.to_bytes(4, "little") + b"\x6b"


def test_read_transactions_are_serialized():
    """核心不变量: 任意时刻进入 recv() 的线程数 ≤ 1 (读事务已串行化).

    无锁时双线程在 barrier 处同时放行 → max_active==2 → 本断言失败;
    加锁后第二个线程被 _read_lock 挡在 recv 之前, max_active 恒为 1.
    """
    gate = threading.Barrier(2)
    t = RacyTransport(gate)
    drv = ZdtDriver(t, timeout_s=0.2, retries=0)
    t.inject(0x02, F_READ_POS, _pos_body(0x64))
    t.inject(0x07, F_READ_POS, _pos_body(0x32))

    results: list = []
    errs: list = []

    def rd(addr):
        try:
            results.append((addr, drv.read_pos(addr)))
        except Exception as exc:  # noqa: BLE001
            errs.append((addr, repr(exc)))

    th_a = threading.Thread(target=rd, args=(0x02,))
    th_b = threading.Thread(target=rd, args=(0x07,))
    th_a.start()
    th_b.start()
    th_a.join(timeout=3.0)
    th_b.join(timeout=3.0)

    assert not th_a.is_alive() and not th_b.is_alive(), "读线程未及时退出"
    assert not errs, f"读事务出现异常: {errs}"
    assert len(results) == 2
    assert t.max_active <= 1, (
        f"两线程并发进入 recv() (max_active={t.max_active}) — response stealing 未修复")


def test_two_thread_read_storm_no_timeout():
    """并发读风暴 (模拟轮询线程 + 主线程宏回读): 全量成功, 零超时."""
    t = QueueTransport()
    drv = ZdtDriver(t, timeout_s=0.2, retries=0)
    n_per_addr = 40
    addrs = (0x02, 0x03, 0x04, 0x05, 0x06, 0x07)
    for addr in addrs:
        for _ in range(n_per_addr):
            t.inject(addr, F_READ_POS, _pos_body(0x64))

    errs: list = []

    def storm(addr):
        for _ in range(n_per_addr):
            try:
                drv.read_pos(addr)
            except Exception as exc:  # noqa: BLE001
                errs.append((addr, repr(exc)))
                return

    ths = [threading.Thread(target=storm, args=(a,)) for a in addrs[:3]]
    for th in ths:
        th.start()
    for th in ths:
        th.join(timeout=10.0)
    assert not errs, f"并发读出现超时: {errs[:5]}"


if __name__ == "__main__":
    run_all(globals())