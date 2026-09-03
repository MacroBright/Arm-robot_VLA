"""PC ↔ STM32 串口协议封装.

Line-based text protocol over UART at 115200 bps.
Extends the existing zero-robotic-arm command protocol
with LeRobot-specific commands: get_state, set_joints, set_torque, e_stop.
"""

import logging
import threading
import time
from typing import Optional

import serial

logger = logging.getLogger(__name__)

# Protocol constants
BAUDRATE = 115200
TIMEOUT_READ_S = 0.05   # 50ms read timeout
TIMEOUT_WRITE_S = 0.1   # 100ms write timeout
MAX_CONSECUTIVE_FAILS = 5
LINE_TERMINATOR = b"\n"


class SerialProtocolError(Exception):
    """Serial communication error."""


class EmergencyStopError(SerialProtocolError):
    """Emergency stop was triggered."""


class SerialProtocol:
    """Manages serial communication with the STM32 control board.

    Usage::

        proto = SerialProtocol(port="COM3")
        proto.connect()
        angles, velocities, loads = proto.get_state()
        proto.set_joints([90.0, 45.0, -20.0, 0.0, 90.0, 0.0])
        proto.disconnect()
    """

    def __init__(self, port: str, baudrate: int = BAUDRATE):
        self.port = port
        self.baudrate = baudrate
        self._ser: Optional[serial.Serial] = None
        self._fail_count = 0
        # 可重入锁: 保护 "写命令+读响应" 事务, 支持多线程调用
        # (如 joystick_control 的后台 remote_event 线程与主循环 get_state 并发)
        self._io_lock = threading.RLock()

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """Open serial port, flush stale data, and verify communication.

        Accepts both real port names ("COM5") and pyserial URL handlers,
        e.g. "socket://localhost:5555" for the fake STM32 simulator
        (scripts/fake_stm32.py).
        """
        self._ser = serial.serial_for_url(
            self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=TIMEOUT_READ_S,
            write_timeout=TIMEOUT_WRITE_S,
        )
        logger.info("Serial port %s opened at %d bps", self.port, self.baudrate)

        # Flush stale boot messages / leftover LOG output
        import time
        time.sleep(0.3)
        self._ser.reset_input_buffer()

        # Verify STM32 is responding (retry up to 5 times)
        for attempt in range(5):
            self._write_line(b"get_state")
            time.sleep(0.1)
            # Drain lines until we find STATE: or run out of data
            for _ in range(20):
                line = self._read_line()
                if line is None:
                    break
                if line.startswith("STATE:"):
                    angles, _, _ = self._parse_state(line)
                    if angles and len(angles) >= 6:
                        self._fail_count = 0
                        logger.info("STM32 verified on attempt %d", attempt + 1)
                        return
            time.sleep(0.2)

        raise SerialProtocolError(
            f"Failed to read state from STM32 on {self.port}. "
            f"Check wiring and firmware."
        )

    def disconnect(self) -> None:
        """Close serial port."""
        if self._ser and self._ser.is_open:
            self._ser.close()
            logger.info("Serial port %s closed", self.port)

    @property
    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    # ------------------------------------------------------------------
    # Command: get_state
    # ------------------------------------------------------------------

    def get_state(self) -> tuple[list[float], list[float], list[float]]:
        """Request current joint state from STM32.

        STM32 firmware may emit LOG messages before the STATE response.
        We drain lines for up to 500ms waiting for it.

        Returns:
            (angles_deg, velocities_rpm, loads_pct) — each a list of 6 floats.
        """
        import time
        with self._io_lock:
            self._write_line(b"get_state")
            deadline = time.monotonic() + 0.5  # 500ms deadline
            while time.monotonic() < deadline:
                line = self._read_line()
                if line is None:
                    continue
                if line.startswith("STATE:"):
                    self._fail_count = 0
                    return self._parse_state(line)

        self._fail_count += 1
        if self._fail_count >= MAX_CONSECUTIVE_FAILS:
            self._trigger_e_stop()
            raise EmergencyStopError(
                f"Serial communication failed {self._fail_count} times. "
                f"Emergency stop triggered."
            )
        return ([], [], [])

    @staticmethod
    def _parse_state(line: str) -> tuple[list[float], list[float], list[float]]:
        """Parse a STATE response line.

        Format: STATE:j1,j2,j3,j4,j5,j6,v1,...,v6,l1,...,l6
        """
        data = line[6:].strip()  # Remove "STATE:" prefix
        values = [float(v) for v in data.split(",")]

        n = len(values) // 3
        angles = values[0:n]
        velocities = values[n : 2 * n]
        loads = values[2 * n : 3 * n]
        return (angles, velocities, loads)

    # ------------------------------------------------------------------
    # Command: set_joints
    # ------------------------------------------------------------------

    def set_joints(self, angles: list[float]) -> None:
        """Send target joint angles (degrees) to STM32.

        Args:
            angles: List of 6+ target angles in degrees.
        """
        payload = " ".join(f"{a:.2f}" for a in angles)
        cmd = f"set_joints {payload}"
        with self._io_lock:
            self._write_line(cmd.encode())
            response = self._read_until_ok(timeout=2.0)
        if response is None:
            self._fail_count += 1
            logger.warning("set_joints: no OK response")
        else:
            self._fail_count = 0

    # ------------------------------------------------------------------
    # Command: set_torque
    # ------------------------------------------------------------------

    def set_torque(self, enable: bool) -> None:
        """Enable or disable motor torque.

        Args:
            enable: True = torque on, False = torque off (free spin).
        """
        cmd = b"set_torque 1" if enable else b"set_torque 0"
        with self._io_lock:
            self._write_line(cmd)
            response = self._read_until_ok(timeout=1.0)
        if response is None:
            logger.warning("set_torque %s: no OK response", "on" if enable else "off")

    # ------------------------------------------------------------------
    # Command: e_stop (emergency stop)
    # ------------------------------------------------------------------

    def e_stop(self) -> None:
        """Trigger emergency stop — immediately stops all motors."""
        with self._io_lock:
            self._write_line(b"e_stop")
            response = self._read_until_keyword("ESTOP", timeout=1.0)
        if response is None:
            logger.error("Emergency stop may have failed!")
        else:
            logger.warning("EMERGENCY STOP triggered")

    # ------------------------------------------------------------------
    # Command: zero (calibration helper)
    # ------------------------------------------------------------------

    def zero(self) -> None:
        """Set current motor positions as the zero reference."""
        self._write_line(b"zero")
        # zero command has no response in current firmware
        time.sleep(0.1)

    # ------------------------------------------------------------------
    # Command: remote_enable / remote_disable (existing protocol)
    # ------------------------------------------------------------------

    def remote_enable(self) -> None:
        """Enable remote control mode (existing firmware command)."""
        self._write_line(b"remote_enable")

    def remote_disable(self) -> None:
        """Disable remote control mode (existing firmware command)."""
        self._write_line(b"remote_disable")

    def send_command(self, cmd: str) -> None:
        """Send a raw one-line command without waiting for a response.

        Thread-safe. Intended for fire-and-forget commands such as
        remote_event / rel_rotate.

        Args:
            cmd: Command line without trailing newline.
        """
        self._write_line(cmd.encode())

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _write_line(self, data: bytes) -> None:
        """Write a line (data + LF) to the serial port."""
        if not self.is_connected:
            raise SerialProtocolError("Serial port not connected")
        try:
            with self._io_lock:
                self._ser.write(data + LINE_TERMINATOR)
                self._ser.flush()
        except serial.SerialException as exc:
            raise SerialProtocolError(f"Serial write failed: {exc}") from exc

    def _read_line(self) -> Optional[str]:
        """Read a line from the serial port, with timeout."""
        if not self.is_connected:
            raise SerialProtocolError("Serial port not connected")
        try:
            line = self._ser.readline()
            if not line:
                return None
            return line.decode("ascii", errors="replace").strip()
        except serial.SerialException as exc:
            raise SerialProtocolError(f"Serial read failed: {exc}") from exc

    def _read_until_keyword(self, keyword: str, timeout: float = 1.0) -> str | None:
        """Read lines until we find one starting with `keyword`, or timeout.

        STM32 firmware may emit LOG() messages before the command response,
        so we drain all lines until we find the expected response.
        """
        import time
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self._read_line()
            if line is None:
                continue
            if line.startswith(keyword):
                return line
        return None

    def _read_until_ok(self, timeout: float = 1.0) -> str | None:
        """Read lines until we find one starting with 'OK'."""
        return self._read_until_keyword("OK", timeout)

    def _trigger_e_stop(self) -> None:
        """Internal: trigger e_stop on communication failure."""
        try:
            self._write_line(b"e_stop")
        except SerialProtocolError:
            pass  # If we can't write, there's nothing more we can do

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def __enter__(self) -> "SerialProtocol":
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.disconnect()
        return None
