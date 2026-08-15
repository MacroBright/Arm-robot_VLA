"""ZdtController — 高层控制 + 安全层 (spec §3/§4.1).

API 与 SerialProtocol 对齐, 使 MassageRobot 可直接换协议对象 (Task 7).
安全: 位置 clamp 出口 · e_stop 广播 · 看门狗 (tick) · 电流力控 (tick 预留).
"""
import logging
import time
from typing import Optional

from .can_transport import CanTransport
from .config import INIT_POSE_DEG, ZdtConfig
from .zdt_driver import ZdtDriver, ZdtDriverError

logger = logging.getLogger(__name__)


class ZdtController:
    def __init__(self, config: Optional[ZdtConfig] = None,
                 transport: Optional[CanTransport] = None):
        self.config = config or ZdtConfig()
        self._transport = transport          # None → connect() 构造 SocketCanTransport
        self._driver: Optional[ZdtDriver] = None
        if self._transport is not None:
            # 注入 transport (测试/Fake) → 立即构造 driver, 免 connect
            self._driver = ZdtDriver(self._transport, timeout_s=self.config.timeout_s,
                                     retries=self.config.retries)
        self._connected = False
        self._last_io_s = 0.0                # 看门狗依据

    # ── 连接生命周期 ─────────────────────────────────────

    def connect(self) -> None:
        if self._transport is None:
            from .can_transport import SocketCanTransport
            self._transport = SocketCanTransport(self.config.channel,
                                                 self.config.bitrate)
        self._transport.open()
        self._driver = ZdtDriver(self._transport, timeout_s=self.config.timeout_s,
                                 retries=self.config.retries)
        self.set_torque(True)
        for addr in self.config.joint_addrs:
            self._driver.read_pos(addr)      # 逐轴验证; 超时抛错 → 连接失败
        self._connected = True
        self._last_io_s = time.monotonic()
        logger.info("ZDT CAN connected: %s (6 drives verified)", self.config.channel)

    def disconnect(self) -> None:
        if self._transport is not None:
            self._transport.close()
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    # ── SerialProtocol 兼容接口 ──────────────────────────

    def get_state(self) -> tuple[list[float], list[float], list[float]]:
        """角度°/速度(占位0)/电流mA. 读失败返回空列表 (调用方降级)."""
        angles: list[float] = []
        loads: list[float] = []
        try:
            for addr in self.config.joint_addrs:
                angles.append(self._driver.read_pos(addr))
            for addr in self.config.joint_addrs:
                loads.append(self._driver.read_current(addr))
        except ZdtDriverError:
            return [], [], []
        self._last_io_s = time.monotonic()
        return angles, [0.0] * len(angles), loads

    def set_joints(self, angles: list[float]) -> None:
        """clamp 到限位表 → 6×move_abs."""
        for i, addr in enumerate(self.config.joint_addrs):
            lo, hi = self.config.limits[i]
            a = max(lo, min(hi, float(angles[i])))
            self._driver.move_abs(addr, a, self.config.speed_rpm)
        self._last_io_s = time.monotonic()

    def set_torque(self, enable: bool) -> None:
        for addr in self.config.joint_addrs:
            self._driver.enable(addr, enable)
        self._last_io_s = time.monotonic()

    def e_stop(self) -> None:
        self._driver.stop_all()
        self._last_io_s = time.monotonic()
        logger.warning("EMERGENCY STOP broadcast")

    def zero(self) -> None:
        """逐轴设当前为机械零位 (0x93 88 01 存储)."""
        for addr in self.config.joint_addrs:
            self._driver.set_zero(addr)
        self._last_io_s = time.monotonic()

    # ── ZDT 扩展 ─────────────────────────────────────────

    def rel_rotate(self, joint_id: int, delta_deg: float) -> None:
        """关节相对旋转. joint_id: 1-based (1=关节1)."""
        addr = self.config.joint_addrs[joint_id - 1]
        self._driver.move_rel(addr, delta_deg, self.config.speed_rpm)
        self._last_io_s = time.monotonic()

    def soft_reset(self) -> None:
        # 逐字发送固件初始位姿, 不 clamp — INIT_POSE 为固件定义复位位,
        # 部分关节初始值 (如 J2=45°) 低于其运行限位下界, clamp 会扭曲复位位.
        for i, addr in enumerate(self.config.joint_addrs):
            self._driver.move_abs(addr, INIT_POSE_DEG[i], self.config.speed_rpm)
        self._last_io_s = time.monotonic()
        logger.info("soft_reset → %s", INIT_POSE_DEG)

    # ── 安全: 看门狗 + 力控 (调用方循环 tick) ─────────────

    def tick(self) -> None:
        """看门狗: >watchdog_s 无成功 IO → e_stop. 力控阈值在后续任务接入."""
        if self._connected and time.monotonic() - self._last_io_s > self.config.watchdog_s:
            logger.error("watchdog: no CAN IO for %.1fs → e_stop", self.config.watchdog_s)
            try:
                self.e_stop()
            except ZdtDriverError:
                pass
