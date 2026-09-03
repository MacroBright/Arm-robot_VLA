"""MassageRobot — LeRobot Robot subclass for the STM32-mediated massage arm.

Implements the Robot abstract interface by communicating with the STM32
control board via serial (not direct RS485), since the STM32 serves as
the safety gateway between PC and Emm_V5 CAN bus stepper motors.

Reference: LeRobot "Bring Your Own Hardware" guide
https://huggingface.co/docs/lerobot/integrate_hardware
"""

import logging
from typing import Any

from lerobot.cameras import make_cameras_from_configs
from lerobot.cameras.opencv import OpenCVCamera
from lerobot.robots import Robot

from .config_massage_robot import MassageRobotConfig
from arm_robot.driver.serial_protocol import SerialProtocol, SerialProtocolError
from arm_robot.driver.zdt_driver import ZdtDriverError

logger = logging.getLogger(__name__)


class MassageRobot(Robot):
    """LeRobot Robot adapter for the zero-robotic-arm massage system.

    Architecture::

        PC (this class) ←─serial 115200─→ STM32 ←─CAN─→ Emm_V5 motors ×6

    Key differences from standard LeRobot robots (e.g., SO-100):
    - Uses custom serial protocol instead of FeetechMotorsBus
    - STM32 handles all motor communication (CAN) and safety logic
    - Supports manual-puppeting data collection (torque-off mode)
    """

    config_class = MassageRobotConfig
    name = "massage_robot"

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def __init__(self, config: MassageRobotConfig):
        super().__init__(config)
        self.config = config
        if config.transport == "can":
            # PC 直连 CAN (ZDT 驱动器), 不再经由 STM32 网关
            from arm_robot.controller.config import ZdtConfig
            from arm_robot.controller.controller import ZdtController
            zdt_cfg = ZdtConfig(
                channel=config.channel, bitrate=config.can_bitrate)
            if config.reduction_ratios:
                zdt_cfg.reduction_ratios = list(config.reduction_ratios)
            self._protocol = ZdtController(zdt_cfg)
        else:
            self._protocol = SerialProtocol(
                port=config.port,
                baudrate=config.baudrate,
            )
        # Build cameras at construction time (NOT in connect) so that
        # observation_features is callable before connecting — required by
        # the LeRobot Robot contract.
        self._cameras: dict[str, OpenCVCamera] = make_cameras_from_configs(config.cameras)

    def connect(self, calibrate: bool = True) -> None:
        """Open serial port to STM32 and connect cameras."""
        if self.is_connected:
            return

        self._protocol.connect()
        logger.info("STM32 connected on %s", self.config.port)

        # Connect the cameras that were created in __init__
        for name, cam in self._cameras.items():
            cam.connect()
            logger.info("Camera '%s' connected", name)

        # Calibration: if not calibrated, zero the current position
        if calibrate and not self.is_calibrated:
            self.calibrate()

        # 推理/评估部署: 连接后自动到按摩准备姿态 (SmolVLA episode 起点目标).
        if self.config.move_to_ready_on_connect:
            self.reset()

    def reset(self) -> None:
        """运动到按摩准备姿态 (READY_POSE_DEG) — episode 边界钩子.

        LeRobot record/eval 的环境重置语义 (0.4.4 仅对 unitree_g1 调用,
        自研采集脚本应显式调用). CAN 直连: 6 轴同步慢速 + 0x36 真实位置限位;
        serial 链路无笛卡尔/多机同步能力, 记日志跳过.
        """
        if self.config.transport != "can":
            logger.info("reset(): transport=%s 不支持自动 ready, 跳过", self.config.transport)
            return
        if not self.config.gravity_confirm:
            raise RuntimeError(
                "reset() 需先显式确认重力关节 (config.gravity_confirm=True) 才能自动 ready")
        self._protocol.arm(gravity_confirmed=True)   # connect 不再自动使能 → 显式 arm
        targets = self._protocol.ready()
        logger.info("reset() → 按摩准备姿态 %s", targets)

    def disconnect(self) -> None:
        """Release all hardware resources."""
        if not self.is_connected:
            return

        # Disable torque before disconnecting (safety)
        try:
            self._protocol.disarm()
        except (SerialProtocolError, ZdtDriverError):
            pass

        self._protocol.disconnect()

        for cam in self._cameras.values():
            cam.disconnect()

    @property
    def is_connected(self) -> bool:
        return self._protocol.is_connected and all(
            cam.is_connected for cam in self._cameras.values()
        )

    @property
    def is_calibrated(self) -> bool:
        # This hardware's "calibration" is a manual zero (optional). Report
        # True so connect() never auto-triggers surprise motion; users call
        # calibrate() explicitly when they want to set the zero reference.
        return True

    # ------------------------------------------------------------------
    # Observation & Action interfaces
    # ------------------------------------------------------------------

    @property
    def _motors_ft(self) -> dict[str, type]:
        """Per-joint proprioceptive features, keyed ``{joint}.pos`` -> float."""
        return {f"{name}.pos": float for name in self.config.joint_names}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        """Camera features, keyed by raw camera name -> (H, W, 3)."""
        return {
            name: (cam.height, cam.width, 3) for name, cam in self._cameras.items()
        }

    @property
    def observation_features(self) -> dict:
        """Describe the observation space.

        Keyed per-joint ``{joint}.pos`` plus one entry per camera. The
        ``observation.state`` / ``observation.images.*`` dataset prefixes are
        added automatically by the LeRobot record pipeline — the robot layer
        must not add them here.

        Must be callable BEFORE connect() — no hardware dependency (cameras
        are built in __init__).
        """
        return {**self._motors_ft, **self._cameras_ft}

    @property
    def action_features(self) -> dict:
        """Describe the action space: target joint angles, keyed ``{joint}.pos``."""
        return self._motors_ft

    def get_observation(self) -> dict:
        """Read current joint state from STM32 and capture camera frames.

        Returns a flat dict matching observation_features, e.g.::

            {
                "shoulder_pan.pos": 90.0,   # degrees
                ...
                "cam_top": np.ndarray (H, W, 3),
            }
        """
        if not self.is_connected:
            raise ConnectionError(f"{self} is not connected.")

        angles, _, _ = self._protocol.get_state()

        n = len(self.config.joint_names)
        if not angles:
            # Communication failure — return zeros as safe fallback
            angles = [0.0] * n
            logger.warning("get_state returned empty, using zeros")
        elif len(angles) != n:
            # Malformed/short STATE line (e.g. serial corruption) — normalise
            # length so the fixed-rate record loop never crashes on IndexError.
            logger.warning(
                "get_state returned %d values, expected %d; normalising",
                len(angles), n,
            )
            angles = (angles + [0.0] * n)[:n]

        obs: dict[str, Any] = {
            f"{name}.pos": float(angles[i])
            for i, name in enumerate(self.config.joint_names)
        }

        # Capture camera frames (raw camera name as key)
        for name, cam in self._cameras.items():
            obs[name] = cam.read_latest()

        return obs

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Send target joint angles (degrees) to STM32.

        Args:
            action: dict keyed ``{joint}.pos`` -> target angle in degrees,
                matching action_features.

        Returns:
            The action actually sent (safety-clipped), same ``{joint}.pos`` keys.
        """
        goal = {
            key.removesuffix(".pos"): float(val)
            for key, val in action.items()
            if key.endswith(".pos")
        }

        # Order strictly by config.joint_names (index i -> firmware joints[i])
        # and clip to a safe range. This preserves byte-for-byte equivalence
        # with the previous positional-array behaviour downstream.
        angles = [
            max(-180.0, min(180.0, goal[name])) for name in self.config.joint_names
        ]
        if self.config.transport == "can":
            # CAN 直连: 用 0x36 真实位置做软限位 + 相对运动 (开机姿态锚定体系).
            # calib (k,b) 由 controller 的 config.calib 默认提供 (与 anchor 一致).
            self._protocol.set_joints_safe(angles, use_kb=True)
        else:
            self._protocol.set_joints(angles)

        return {
            f"{name}.pos": angles[i]
            for i, name in enumerate(self.config.joint_names)
        }

    # ------------------------------------------------------------------
    # Calibration
    # ------------------------------------------------------------------

    def calibrate(self) -> None:
        """Run calibration routine.

        Sends 'zero' command to STM32 to set current positions as the
        reference zero. The arm should be manually placed in the desired
        zero pose before calling this. Left as an explicit action — it is
        never auto-triggered on connect (see is_calibrated).
        """
        logger.info("Calibrating: setting current position as zero...")
        self._protocol.zero()
        logger.info("Calibration complete")

    # ------------------------------------------------------------------
    # Configuration (motor settings)
    # ------------------------------------------------------------------

    def configure(self) -> None:
        """Apply runtime motor configuration.

        Called after connect(). Enables torque for normal operation.
        For manual-puppeting data collection, torque should be OFF
        (call self.set_torque_mode(False) before recording).
        """
        self._protocol.set_torque(True)

    def set_torque_mode(self, enable: bool) -> None:
        """Enable or disable motor torque.

        - enable=True: Motors hold position (normal operation, inference)
        - enable=False: Motors free-spin (manual puppeting, data collection)
        """
        self._protocol.set_torque(enable)
        mode = "ON" if enable else "OFF (free-spin / puppeting)"
        logger.info("Torque %s", mode)

    # ------------------------------------------------------------------
    # Safety
    # ------------------------------------------------------------------

    def emergency_stop(self) -> None:
        """Immediately stop all motors."""
        self._protocol.e_stop()
        logger.critical("EMERGENCY STOP ACTIVATED")

    # ------------------------------------------------------------------
    # Teleoperation step
    # ------------------------------------------------------------------

    def teleop_step(self) -> None:
        """Single step for teleoperation mode.

        Reads current state and publishes it. In record mode, LeRobot
        will call get_observation() at fixed FPS instead.
        """
        pass  # Teleop is handled by the record/teleoperate pipeline
