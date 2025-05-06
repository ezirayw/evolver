import logging
from dataclasses import dataclass, field

from xarm.wrapper import XArmAPI

from htevolver.exceptions import xArmError

logger = logging.getLogger(__name__)


@dataclass
class xArmCoordinate:
    """Represents an xArm coordinate in 3D space.

    Attributes:
        x: The x-coordinate value.
        y: The y-coordinate value.
        z: The z-coordinate value.
    """

    x: float
    y: float
    z: float


@dataclass
class xArm:
    arm_api: XArmAPI
    ip: str
    roll: int
    pitch: int
    yaw: int
    speed: int
    mvacc: int
    warning_code: int = field(default=0)
    error_code: int = field(default=0)
    state: int = field(default=0)
    connected: bool = field(default=False)
    max_speed: int = field(default=1000)
    max_mvacc: int = field(default=1000)

    @classmethod
    def create(cls, config: dict):
        return cls(
            arm_api=XArmAPI(port=config["ip"], enable_report=True, do_not_open=config["connect"]),
            ip=config["ip"],
            roll=config["roll"],
            pitch=config["pitch"],
            yaw=config["yaw"],
            speed=config["speed"],
            mvacc=config["mvacc"],
        )

    def setup(self):
        """Setup the xArm with standard parameters.

        Clears errors, enables motion, sets collision sensitivity and
        ensures the end effector is in a safe orientation.
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
        self.arm_api.connect()

    def disconnect(self):
        self.arm_api.disconnect()

    def reset(self):
        if not self.connected:
            self.arm_api.connect()
        self.arm_api.clean_warn()
        self.arm_api.clean_error()
        self.arm_api.motion_enable(True)
        self.arm_api.set_state(0)
        code, angles = self.arm_api.get_servo_angle()
        if code == 0:
            angles[3] = -(angles[1] + angles[2])
            self.arm_api.set_servo_angle(angle=angles, wait=True)

    def update(self, xarm_config: dict):
        for config_parameter, value in xarm_config["xArm"].items():
            if hasattr(self, config_parameter) and getattr(self, config_parameter) != value:
                setattr(self, config_parameter, value)
                logger.debug(f"Updated xArm parameter: {config_parameter}={value}")

    def stop(self):
        self.arm_api.set_state(4)

    def pause(self):
        self.arm_api.set_state(3)

    def resume(self):
        self.arm_api.set_mode(0)

    def get_state(self):
        result = self.arm_api.get_state()
        if result[0] == 0:
            return result[1]

    async def move(self, coordinate: xArmCoordinate):
        """Moves the xArm linearly to the specified coordinate.

        Executes an immediate linear movement from the current position
        to the given target position.

        Args:
            coordinate (xArmCoordinate): Target coordinates for the movement.
                Example: xArmCoordinate(x=150, y=100, z=50)

        Raises:
            xArmError: If the movement fails or the arm is in an error state.
        """

        if self.speed > self.max_speed:
            raise xArmError(f"Configured xArm speed parameter: {self.speed} higher than max allowed speed: {self.max_speed}")
        if self.mvacc > self.max_mvacc:
            raise xArmError(f"Configured xArm mvacc parameter: {self.mvacc} higher than max allowed mvacc: {self.max_mvacc}")

        result = self.arm_api.set_position(
            x=coordinate.x,
            y=coordinate.y,
            z=coordinate.z,
            roll=self.roll,
            pitch=self.pitch,
            yaw=self.yaw,
            speed=self.speed,
            mvacc=self.mvacc,
            wait=True,
        )
        if result < 0:
            raise xArmError(f"xArm error detected during move_xarm(): {result}")

    def register_callback(self, error_warn_callback, state_changed_callback, connect_changed_callback):
        """Registers callback functions for the xArm API.

        Sets up the error, warning, state change and connection change callbacks
        for real-time monitoring of the xArm's status.
        """

        self.arm_api.register_error_warn_changed_callback(callback=error_warn_callback)
        self.arm_api.register_state_changed_callback(callback=state_changed_callback)
        self.arm_api.register_connect_changed_callback(callback=connect_changed_callback)

    def to_dict(self):
        return {
            "ip": self.ip,
            "roll": self.roll,
            "pitch": self.pitch,
            "yaw": self.yaw,
            "speed": self.speed,
            "mvacc": self.mvacc,
            "warning_code": self.warning_code,
            "error_code": self.error_code,
            "state": self.state,
            "connected": self.connected,
            "max_speed": self.max_speed,
            "max_mvacc": self.max_mvacc,
        }
