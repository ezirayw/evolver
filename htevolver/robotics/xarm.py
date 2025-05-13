import logging
from dataclasses import dataclass, field

from xarm.wrapper import XArmAPI

from htevolver.exceptions import xArmError

logger = logging.getLogger(__name__)


@dataclass
class xArmCoordinate:
    """Represents an xArm coordinate in 3D space.

    Stores 3D coordinates (x, y, z) for identifying positions in the xArm's
    coordinate system.

    Attributes:
        x (float): The x-coordinate value in the xArm system.
        y (float): The y-coordinate value in the xArm system.
        z (float): The z-coordinate value (height) in the xArm system.
    """

    x: float
    y: float
    z: float


@dataclass
class xArm:
    """Controls the xArm robotic arm for HT-eVOLVER.

    Manages communication with the physical xArm, including movement operations,
    orientation control, and error handling.

    Attributes:
        arm_api (XArmAPI): API interface to the physical xArm.
        ip (str): IP address of the xArm controller.
        connected (bool): Connection status of the arm.
        roll (int): Roll angle for the end effector in degrees.
        pitch (int): Pitch angle for the end effector in degrees.
        yaw (int): Yaw angle for the end effector in degrees.
        speed (int): Movement speed (1-1000).
        mvacc (int): Movement acceleration (1-1000).
        warning_code (int): Current warning code from the arm, if any.
        error_code (int): Current error code from the arm, if any.
        state (int): Current state of the arm (0=READY, 3=PAUSE, 4=STOP).
        max_speed (int): Maximum allowed speed setting.
        max_mvacc (int): Maximum allowed acceleration setting.
    """

    arm_api: XArmAPI
    ip: str
    connected: bool

    roll: int
    pitch: int
    yaw: int
    speed: int
    mvacc: int
    warning_code: int = field(default=0)
    error_code: int = field(default=0)
    state: int = field(default=0)
    max_speed: int = field(default=1000)
    max_mvacc: int = field(default=1000)

    @classmethod
    def create(cls, config: dict):
        """Create a new xArm instance.

        Args:
            config (dict): Configuration dictionary for the xArm.
                Must include 'ip', 'connect', 'roll', 'pitch', 'yaw', 'speed', 'mvacc'.

        Returns:
            xArm: A new instance of the xArm controller.

        Examples:
            ```
            config = {
                "ip": "192.168.1.10",
                "connect": True,
                "roll": 180,
                "pitch": 0,
                "yaw": 0,
                "speed": 500,
                "mvacc": 500
            }
            arm = xArm.create(config)
            ```
        """
        return cls(
            arm_api=XArmAPI(port=config["ip"], enable_report=True, do_not_open=config["connect"]),
            ip=config["ip"],
            connected=config.get("connect", False),
            roll=config["roll"],
            pitch=config["pitch"],
            yaw=config["yaw"],
            speed=config["speed"],
            mvacc=config["mvacc"],
        )

    def setup(self):
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
        """Connect to the physical xArm hardware.

        Establishes a connection to the xArm controller at the configured IP address.
        """
        self.arm_api.connect()

    def disconnect(self):
        """Disconnect from the physical xArm hardware.

        Closes the connection to the xArm controller.
        """
        self.arm_api.disconnect()

    def reset(self):
        """Reset the xArm to clear errors.

        Reconnects if disconnected, clears warnings and errors, enables motion,
        and aligns the end effector to be parallel to the ground to avoid kinematic errors.
        """
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
        """Update arm configuration from a dictionary.

        Args:
            xarm_config (dict): Dictionary containing updated configuration parameters.
                Should include an 'xArm' section with parameters like 'roll', 'pitch',
                'yaw', 'speed', 'mvacc'.

        Examples:
            ```
            config_update = {
                "xArm": {
                "speed": 700,
                "mvacc": 600
                }
            }
            arm.update(config_update)
            ```
        """
        for config_parameter, value in xarm_config["xArm"].items():
            if hasattr(self, config_parameter) and getattr(self, config_parameter) != value:
                setattr(self, config_parameter, value)
                logger.debug(f"Updated xArm parameter: {config_parameter}={value}")

    def stop(self):
        """Stop all arm movement immediately.

        Sets the arm state to 4 (STOP), which halts all current and pending movements.
        """
        self.arm_api.set_state(4)

    def pause(self):
        """Pause arm movement.

        Sets the arm state to 3 (PAUSE), which temporarily halts movement
        but allows for later resumption.
        """
        self.arm_api.set_state(3)

    def resume(self):
        """Resume arm movement after a pause.

        Sets the arm mode to 0 (ready to move) to allow movement after a pause.
        """
        self.arm_api.set_mode(0)

    def get_state(self):
        """Get the current state of the arm.

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

    async def move(self, coordinate: xArmCoordinate):
        """Move the xArm linearly to the specified coordinate.

        Executes an immediate linear movement from the current position
        to the given target position, using the configured orientation angles.

        Args:
            coordinate (xArmCoordinate): Target coordinates for the movement.
                Example: xArmCoordinate(x=150, y=100, z=50)

        Raises:
            xArmError: If the movement fails, speed/acceleration exceeds limits,
                or the arm is in an error state.

        Examples:
            ```
            target = xArmCoordinate(x=150, y=100, z=50)
            await arm.move(target)
            ```
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

        Examples:
            ```
            def on_error(data):
                print(f"Error: {data['error_code']}, Warning: {data['warn_code']}")

            def on_state(data):
                print(f"State changed to {data['state']}")

            def on_connect(data):
                print(f"Connected: {data['connected']}")

            arm.register_callback(on_error, on_state, on_connect)
            ```
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
