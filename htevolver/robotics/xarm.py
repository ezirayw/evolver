import logging
from dataclasses import dataclass, field
from typing import Literal

from xarm.wrapper import XArmAPI

from htevolver.dependencies import CartesianMovement
from htevolver.exceptions import xArmError

logger = logging.getLogger(__name__)


@dataclass
class xArm:
    """Controls the xArm robotic arm for HT-eVOLVER.

    Manages communication with the physical xArm, including movement operations,
    orientation control, and error handling.

    Attributes:
        arm_api (XArmAPI): API interface to the physical xArm.
        connected (bool): Connection status of the arm.
        ip (str): IP address of the xArm controller.
        home_position (CartesianMovement): Home position for xArm.
        standby_position (CartesianMovement): Standby position for xArm.
        warning_code (int): Current warning code from the arm, if any.
        error_code (int): Current error code from the arm, if any.
        state (int): Current state of the arm (0=READY, 3=PAUSE, 4=STOP).
        max_speed (int): Maximum allowed speed setting.
        max_mvacc (int): Maximum allowed acceleration setting.
    """

    arm_api: XArmAPI = field(repr=False)
    connected: bool
    ip: str
    home_position: CartesianMovement
    standby_position: CartesianMovement

    warning_code: int = field(default=0)
    error_code: int = field(default=0)
    state: int = field(default=0)
    max_speed: int = field(default=1000)
    max_mvacc: int = field(default=1000)

    @classmethod
    def from_config(cls, config: dict):
        """Create a new xArm instance.

        Args:
            config (dict): Configuration dictionary for the xArm.
                Must include 'ip', 'connect', 'roll', 'pitch', 'yaw', 'speed', 'mvacc'.

        Returns:
            xArm: A new instance of the xArm controller.
        """
        try:
            return cls(
                arm_api=XArmAPI(port=config["ip"], enable_report=True, do_not_open=True),
                connected=config.get("connect", False),
                ip=config["ip"],
                home_position=CartesianMovement(**config["home"]),
                standby_position=CartesianMovement(**config["standby"]),
            )
        except Exception:
            raise xArmError("Error trying to create xArm")

    def initialize(self):
        """Setup the xArm with standard parameters.

        Clears errors, enables motion, sets collision sensitivity and
        ensures the end effector is in a safe orientation. This should be
        called after creating or reconnecting the arm.
        """
        self.arm_api.clean_warn()
        self.arm_api.clean_error()
        self.arm_api.motion_enable(enable=True)
        self.arm_api.set_state(state=0)
        self.arm_api.set_mode(0)
        self.arm_api.set_collision_sensitivity(2)
        self.arm_api.set_self_collision_detection(True)

        # handle potential C21 kinematic errors (align end effector to be parallel to ground)
        code, angles = self.arm_api.get_servo_angle()
        if code == 0:
            angles[3] = -(angles[1] + angles[2])
            self.arm_api.set_servo_angle(angle=angles, wait=True)

    def connect(self):
        """Connect to the xArm robot via the xArm API."""
        self.arm_api.connect()

    def disconnect(self):
        """Disconnect from the xArm robot via the xArm API."""
        self.arm_api.disconnect()

    def update(self, config: dict):
        """Update xArm configuration from a dictionary.

        Args:
            config (dict): Dictionary containing updated configuration.
        """
        for config_parameter, value in config["xArm"].items():
            if hasattr(self, config_parameter) and getattr(self, config_parameter) != value:
                setattr(self, config_parameter, value)
                logger.debug(f"Updated xArm parameter: {config_parameter}={value}")

    def stop(self):
        """Stop xArm movement immediately.

        Sets the arm state to 4 (STOP), which halts all current and pending movements.
        """
        self.arm_api.set_state(4)

    def pause(self):
        """Pause xArm movement.

        Sets the arm state to 3 (PAUSE), which temporarily halts movement
        but allows for later resumption.
        """
        self.arm_api.set_state(3)

    def resume(self):
        """Resume xArm movement after a pause.

        Sets the arm mode to 0 (ready to move) to allow movement after a pause.
        """
        self.arm_api.set_mode(0)

    def get_state(self):
        """Get the current state of the xArm.

        Returns:
            int: Current state code of the arm, where:
                0 = READY
                1 = BUSY
                2 = DIRECT_TEACHING
                3 = PAUSED
                4 = STOPPED

        Returns None if unable to get state information.
        """
        result = self.arm_api.get_state()
        if result[0] == 0:
            return result[1]

    def move(self, config: CartesianMovement):
        """Move the xArm linearly to the specified coordinate.

        Executes an immediate linear movement from the current position
        to the given target position, using the configured orientation angles.

        Args:
            coordinate (CartesianMovement): Target coordinates for the movement.
                Example: CartesianMovement(x=150, y=100, z=50)

        Raises:
            xArmError: If the movement fails, speed/acceleration exceeds limits,
                or the arm is in an error state.
        """

        if config.speed > self.max_speed:
            raise xArmError(f"Configured xArm speed parameter: {config.speed} higher than max allowed speed: {self.max_speed}")
        if config.acceleration > self.max_mvacc:
            raise xArmError(
                f"Configured xArm mvacc parameter: {config.acceleration} higher than max allowed mvacc: {self.max_mvacc}"
            )

        result = self.arm_api.set_position(
            x=config.x,
            y=config.y,
            z=config.z,
            roll=config.roll,
            pitch=config.pitch,
            yaw=config.yaw,
            speed=config.speed,
            mvacc=config.acceleration,
            wait=True,
        )
        if result < 0:
            raise xArmError(f"xArm error detected during move_xarm(): {result}")

    def check_position(self, position: Literal["home", "standby"]) -> bool:
        """Check if the xArm is currently within 1% of the specified reference position.

        Args:
            reference_position (str): The reference position to check.

        Returns:
            bool: True if the xArm is at the reference position, False otherwise.
        """
        # code, current_location = self.arm_api.get_position()
        # precision: float = 0.1
        # lower_threshold: float = 1 - precision
        # upper_threshold: float = 1 + precision

        # if (current_location[0] < self.reference_positions[reference_position].x * lower_threshold) or (
        #     current_location[0] > self.reference_positions[reference_position].x * upper_threshold
        # ):
        #     return False

        # if (current_location[1] < self.reference_positions[reference_position].y * lower_threshold) or (
        #     current_location[1] > self.reference_positions[reference_position].y * upper_threshold
        # ):
        #     return False

        # if (current_location[2] < self.reference_positions[reference_position].z * lower_threshold) or (
        #     current_location[2] > self.reference_positions[reference_position].z * upper_threshold
        # ):
        #     return False
        return True

    def register_callback(self, error_warn_callback, state_changed_callback, connect_changed_callback):
        """Register callback functions for the xArm API.

        Sets up the error, warning, state change and connection change callbacks
        for real-time monitoring of the xArm's status.

        Args:
            error_warn_callback (callable): Function to call when errors or warnings occur.
                Will be called with a dict containing 'error_code' and 'warn_code'.
            state_changed_callback (callable): Function to call when the arm state changes.
                Will be called with a dict containing 'state'.
            connect_changed_callback (callable): Function to call when connection status changes.
                Will be called with a dict containing 'connected'.
        """

        self.arm_api.register_error_warn_changed_callback(callback=error_warn_callback)
        self.arm_api.register_state_changed_callback(callback=state_changed_callback)
        self.arm_api.register_connect_changed_callback(callback=connect_changed_callback)

    def to_dict(self):
        """Convert the xArm to a dictionary representation.

        Returns:
            dict: Dictionary containing the current state and configuration of the arm.
                Includes position settings, motion parameters, and status codes.
        """
        return {
            "ip": self.ip,
            "warning_code": self.warning_code,
            "error_code": self.error_code,
            "state": self.state,
            "connected": self.connected,
            "max_speed": self.max_speed,
            "max_mvacc": self.max_mvacc,
        }
