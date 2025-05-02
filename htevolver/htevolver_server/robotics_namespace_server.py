import asyncio
import logging
import time
from dataclasses import asdict, dataclass, field

import numpy as np
import skimage as ski
import socketio
import yaml
from skimage.transform import EuclideanTransform
from tecancavro.models import XCaliburD
from tecancavro.syringe import SyringeError, SyringeTimeout
from tecancavro.transport import TecanAPISerial
from xarm.wrapper import XArmAPI

from htevolver.exceptions import ExitRobotics, OperationEventError, RoboticsError, xArmError
from htevolver.shared import FluidTypes, RoboticsRoutines, RoboticsState, RoboticsStatus, ServerResult, xArmStatus

logger = logging.getLogger(__name__)


#### DECORATORS ####
def operation_decorator(func):
    """Decorator for robotic operations functions to manage server state and error handling.

    Handles updating the server status, interrupt checking,
    and error handling for called operation functions.

    Args:
        func (callable): The helper function to decorate.

    Returns:
        callable: The wrapped function.
    """

    async def wrapper(self, *args, **kwargs):
        try:
            self.status.state = RoboticsState.BUSY
            await self.check_for_interrupt()

            # run the target helper function
            await func(self, *args, **kwargs)

            self.status.state = RoboticsState.IDLE
        except ExitRobotics as e:
            self.status.state = RoboticsState.IDLE
            logger.info(f"STOP detected prior to running {func.__name__}")
            raise e
        except (xArmError, SyringeError, SyringeTimeout) as e:
            self.status.state = RoboticsState.EMERGENCY_STOP
            raise OperationEventError(f"error running {func.__name__}: {e}")

    return wrapper


def routine_decorator(routine_type: RoboticsRoutines):
    """Decorator for robotics routines that manages server status and routine results.

    Handles updating the robotics configuration and manages the server status state, timing, error handling, and
    constructs the standardized return data package sent to the client.

    Args:
        routine_type (RoboticsRoutines): The type of routine being executed.
            Example: RoboticsRoutines.PIPETTE

    Returns:
        callable: A decorator function.
    """

    def decorator(func):
        async def wrapper(self, *args, **kwargs):
            if self.status.state == RoboticsState.READY:
                self.load_conf()
                self.pipette_head.update(self.robotics_conf)
                self.arm.update(self.robotics_conf)
                start_time = time.time()
                try:
                    self.status.routine = routine_type
                    logger.info(f"Running {func.__name__} routine")

                    result, message = await func(self, *args, **kwargs)

                    logger.info(f"Done running the {func.__name__} routine")
                    end_time = time.time()
                    self.status.routine = RoboticsRoutines.NO_ROUTINE
                    self.status.state = RoboticsState.READY

                    return ServerResult(
                        done=result,
                        namespace="/robotics",
                        routine=routine_type.name,
                        status=self.status.to_dict(),
                        elapsed_time=end_time - start_time,
                        message=f"{func.__name__}: {message}",
                    )
                except ExitRobotics:
                    end_time = time.time()
                    self.status.routine = RoboticsRoutines.NO_ROUTINE
                    self.status.state = RoboticsState.READY
                    return ServerResult(
                        done=False,
                        namespace="/robotics",
                        routine=routine_type.name,
                        status=self.status.to_dict(),
                        elapsed_time=end_time - start_time,
                        message=f"STOP detected while running {func.__name__}, exiting ",
                    )
                except OperationEventError as e:
                    end_time = time.time()
                    return ServerResult(
                        done=False,
                        namespace="/robotics",
                        routine=routine_type.name,
                        status=self.status.to_dict(),
                        elapsed_time=end_time - start_time,
                        message=f"Error encountered trying to run {func.__name__}: {e}",
                    )
            else:
                logger.warning(f"Tried running the {func.__name__} routine but the robotics namespace is not in a READY state")
                return ServerResult(
                    done=False,
                    namespace="/robotics",
                    routine=routine_type.name,
                    status=self.status.to_dict(),
                    elapsed_time=0.00,
                    message=f"Tried running the {func.__name__} routine but the robotics namespace is not in a READY state",
                )

        return wrapper

    return decorator


@dataclass
class Pump:
    position_id: int
    hardware: XCaliburD | None
    ports: dict[int, FluidTypes] = field(default_factory=lambda: {1: FluidTypes.EMPTY, 2: FluidTypes.EMPTY})
    empty: bool = field(default=True)
    active_port: int = field(default=1)

    def connect(self): ...
    def disconnect(self):
        if self.hardware:
            del self.hardware.com_link

    def prime(self): ...

    def check_empty(self):
        """Checks if all ports for a pump have EMPTY fluid type.

        Updates the pump's empty state based on whether all of its ports
        are of type EMPTY, indicating it shouldn't be used.
        """
        for port_index, fluid_type in self.ports.items():
            if fluid_type != FluidTypes.EMPTY:
                self.empty = False
                return
        self.empty = True

    def set_port(self, desired_fluid_type: FluidTypes):
        """Sets the active port based on the desired fluid type.

        Automatically selects the lowest port number if multiple ports
        contain the same fluid type.

        Args:
            desired_fluid_type (FluidTypes): The fluid type to set as active.
                Example: FluidTypes.MEDIA

        Raises:
            RoboticsError: If the pump doesn't have the requested fluid type.
        """
        available_ports = []
        for port_index, fluid_type in self.ports.items():
            if desired_fluid_type == fluid_type:
                available_ports.append(port_index)

        if not available_ports:
            raise RoboticsError(f"Pump_{self.position_id} does not have the following fluid_type: {desired_fluid_type}")

        # Use the lowest port number if multiple ports have the same fluid type
        self.active_port = min(available_ports)


@dataclass
class PipetteHead:
    pumps: list[Pump]
    pump_num: int = field(default=0)
    universal_fluids: dict[FluidTypes, bool] = field(default_factory=lambda: {FluidTypes.EMPTY: True})
    num_windows: int = field(default=0)
    active_window: list[int | None] = field(default_factory=lambda: [])
    active_pumps: list[Pump] = field(default_factory=lambda: [])
    primed: bool = field(default=False)

    def update(self, robotics_conf: dict):
        """Update the PipetteHead attributes based on the input configuration."""

        serial_port = robotics_conf["pump_serial_port"]
        for position_index in range(4):
            port_config: dict[int, FluidTypes] = {}
            for port, fluid_type in robotics_conf["pipette_head_pumps"][position_index].items():
                if fluid_type in FluidTypes.__members__:
                    port_config[port] = FluidTypes[fluid_type]
                else:
                    port_config[port] = FluidTypes.EMPTY
                    logger.warning(
                        f"Invalid fluid type found in config: {fluid_type}, defaulting to EMPTY for position_{position_index}"
                    )

            if self.pumps[position_index].ports != port_config:
                self.pumps[position_index] = Pump(
                    position_id=position_index,
                    hardware=XCaliburD(
                        com_link=TecanAPISerial(position_index, ser_port=serial_port, ser_baud=9600),
                    ),
                    ports=port_config,
                )

        for pump in self.pumps:
            pump.check_empty()
            if not pump.empty:
                self.pump_num += 1

        for fluid_type in FluidTypes:
            self.universal_fluids[fluid_type] = (
                sum(1 for pump in self.pumps if any(ft == fluid_type for _, ft in pump.ports.items())) == self.pump_num
            )

        if self.universal_fluids:
            self.num_windows = int(6 / self.pump_num)
        if not self.universal_fluids:
            self.num_windows = 6 + (self.pump_num - 1)
        self.active_pumps = []
        self.active_window = []
        self.primed = False


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
class VialCoordinate:
    """Represents a coordinate in the vial grid system.

    Attributes:
        x: The x-coordinate value.
        y: The y-coordinate value.
    """

    x: float
    y: float


@dataclass(kw_only=True)
class xArmPlane:
    vial0_x: float
    vial0_y: float
    vial17_x: float
    vial17_y: float
    z: float
    transform_matrix: EuclideanTransform = field(init=False)

    def rigid_transform(self):
        """Calculates the rigid transformation matrix between coordinate systems.

        Creates a transformation matrix for converting vial coordinates into
        xArm coordinates using the calibration points.
        """

        vial_coordinates = np.array([[0, 36], [90, 0]])
        vial_0 = np.array([self.vial0_x, self.vial0_y])
        vial_17 = np.array([self.vial17_x, self.vial17_y])
        np.array([vial_0, vial_17])
        tform = ski.transform.EuclideanTransform()
        tform.estimate(vial_coordinates, np.array([vial_0, vial_17]))
        self.transform_matrix = tform

    def vial_to_xarm(self, evolver_coordinates: VialCoordinate) -> xArmCoordinate:
        """Transforms vial coordinates to xArm coordinates.

        Applies the rigid transformation matrix to convert from the evolver
        coordinate system to the xArm coordinate system.

        Args:
            evolver_coordinates (VialCoordinate): Coordinates in the evolver system.
                Example: VialCoordinate(x=18, y=36)

        Returns:
            xArmCoordinate: The transformed coordinates in the xArm system.
        """
        np_coordinates = np.array([[evolver_coordinates.x], [evolver_coordinates.y], [1]])
        transformed = np.dot(self.transform_matrix, np_coordinates)
        return xArmCoordinate(x=transformed[0][0], y=transformed[1][0], z=self.z)


@dataclass
class SmartStationRobotics:
    id: int
    xArmPlane_in: xArmPlane
    xArmPlane_out: xArmPlane
    wash_location: VialCoordinate = field(default_factory=lambda: VialCoordinate(x=72, y=-29))
    wash_depth: float = field(init=False)
    vial_map: list[list[int]] = field(
        default_factory=lambda: [[0, 1, 2, 3, 4, 5], [11, 10, 9, 8, 7, 6], [12, 13, 14, 15, 16, 17]]
    )

    def update(self, robotics_conf: dict):
        """Update the SmartStation xArmPlane calibration points based on the input configuration."""
        for calibration_point, positon in robotics_conf["plane_calibration"][self.id]["plane_out"].items():
            if hasattr(self, calibration_point) and getattr(self.xArmPlane_out, calibration_point) != positon:
                setattr(
                    self.xArmPlane_out,
                    calibration_point,
                    positon,
                )

        for calibration_point, positon in robotics_conf["plane_calibration"][self.id]["plane_in"].items():
            if hasattr(self, calibration_point) and getattr(self.xArmPlane_in, calibration_point) != positon:
                setattr(
                    self.xArmPlane_in,
                    calibration_point,
                    positon,
                )


@dataclass
class StationPumpCommands:
    vial_0: dict[str, int] = field(default_factory=lambda: {})
    vial_1: dict[str, int] = field(default_factory=lambda: {})
    vial_2: dict[str, int] = field(default_factory=lambda: {})
    vial_3: dict[str, int] = field(default_factory=lambda: {})
    vial_4: dict[str, int] = field(default_factory=lambda: {})
    vial_5: dict[str, int] = field(default_factory=lambda: {})
    vial_6: dict[str, int] = field(default_factory=lambda: {})
    vial_7: dict[str, int] = field(default_factory=lambda: {})
    vial_8: dict[str, int] = field(default_factory=lambda: {})
    vial_9: dict[str, int] = field(default_factory=lambda: {})
    vial_10: dict[str, int] = field(default_factory=lambda: {})
    vial_11: dict[str, int] = field(default_factory=lambda: {})
    vial_12: dict[str, int] = field(default_factory=lambda: {})
    vial_13: dict[str, int] = field(default_factory=lambda: {})
    vial_14: dict[str, int] = field(default_factory=lambda: {})
    vial_15: dict[str, int] = field(default_factory=lambda: {})
    vial_16: dict[str, int] = field(default_factory=lambda: {})
    vial_17: dict[str, int] = field(default_factory=lambda: {})


@dataclass
class xArm:
    arm_api: XArmAPI
    status: xArmStatus
    ip: str
    roll: int
    pitch: int
    yaw: int
    speed: int
    mvacc: int
    max_speed: int = field(default=1000)
    max_mvacc: int = field(default=1000)

    @classmethod
    def from_dict(cls, config: dict):
        return cls(
            arm_api=XArmAPI(config["xArm"]["ip"], enable_report=True, do_not_open=config["xArm"]["connect"]),
            status=xArmStatus(),
            ip=config["xArm"]["ip"],
            roll=config["xArm"]["ip"],
            pitch=config["xArm"]["ip"],
            yaw=config["xArm"]["ip"],
            speed=config["xArm"]["ip"],
            mvacc=config["xArm"]["ip"],
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
        if not self.status.connected:
            self.arm_api.connect()
        self.arm_api.clean_warn()
        self.arm_api.clean_error()
        self.arm_api.motion_enable(True)
        self.arm_api.set_state(0)
        code, angles = self.arm_api.get_servo_angle()
        if code == 0:
            angles[3] = -(angles[1] + angles[2])
            self.arm_api.set_servo_angle(angle=angles, wait=True)

    def update(self, robotics_conf: dict):
        for config_parameter, value in robotics_conf["xArm"].items():
            if hasattr(self, config_parameter) and getattr(self, config_parameter) != value:
                setattr(self, config_parameter, value)
                logger.debug(f"Updated xArm parameter: {config_parameter}={value}")

    def set_state(self, state: int):
        self.arm_api.set_state(state)

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


class RoboticsServerNamespace(socketio.AsyncNamespace):
    def __init__(
        self,
        robotics_conf: dict,
        robotics_conf_path: str,
        namespace: str = "/robotics",
    ):
        super().__init__(namespace)
        self.robotics_conf: dict = robotics_conf
        self.robotics_conf_path: str = robotics_conf_path
        self.arm_command_queue: list[xArmCoordinate] = []
        pumps: list[Pump] = []
        for position_index in range(4):
            port_config: dict[int, FluidTypes] = {}
            for port, fluid_type in self.robotics_conf["pipette_head_pumps"][position_index]["ports"].items():
                if fluid_type in FluidTypes.__members__:
                    port_config[port] = FluidTypes[fluid_type]
                else:
                    port_config[port] = FluidTypes.EMPTY
                    logger.warning(
                        f"Invalid fluid type found in config: {fluid_type}, defaulting to EMPTY for position_{position_index}"
                    )
            if self.robotics_conf["pipette_head_pumps"][position_index]["connect"]:
                pumps.append(
                    Pump(
                        position_id=position_index,
                        hardware=XCaliburD(
                            com_link=TecanAPISerial(position_index, ser_port=self.robotics_conf["serial_port"], ser_baud=9600),
                        ),
                        ports=port_config,
                    )
                )
            else:
                pumps.append(
                    Pump(
                        position_id=position_index,
                        hardware=None,
                        ports=port_config,
                    )
                )

        self.pipette_head: PipetteHead = PipetteHead(pumps)

        # initialize SmartStations
        self.stations: list[SmartStationRobotics] = []
        plane_calibration = self.robotics_conf["plane_calibration"]
        for station_id in range(4):
            plane_out = xArmPlane(**plane_calibration[station_id]["plane_out"])
            plane_in = xArmPlane(**plane_calibration[station_id]["plane_in"])
            self.stations.append(SmartStationRobotics(station_id, plane_out, plane_in))

        # initialize XArm instance
        self.arm = xArm.from_dict(self.robotics_conf)
        self.arm.register_callback(self.error_warn_change_callback, self.state_changed_callback, self.connect_changed_callback)
        self.arm.setup()

        # initialize RoboticsStatus instance
        self.status: RoboticsStatus = RoboticsStatus(
            state=RoboticsState.IDLE,
            routine=RoboticsRoutines.NO_ROUTINE,
            active_station=-1,
            active_pumps=[],
            vial_window=[],
            xArm=self.arm.status,
        )
        logger.info("Robotics namespace initialized")

    async def on_connect(self, sid):
        """Handles client connection to the robotics server.

        Args:
            sid (str): Session ID of the connecting client.
        """
        logger.info("Client connected to robotics_eVOLVER server")

    async def on_disconnect(self, sid):
        """Handles client disconnection from the robotics server.

        Args:
            sid (str): Session ID of the disconnecting client.
        """
        logger.info("Client disconnected to robotics_eVOLVER server")

    async def on_pause(self, sid):
        """Pauses robotics operations if currently busy.

        Sets the robotics state to PAUSE if the system is currently BUSY.

        Args:
            sid (str): Session ID of the client.
        """
        self.pause_robotics()

    async def on_resume(self, sid):
        """Resumes robotics operations if previously paused.

        Sets the robotics state back to BUSY if previously in PAUSE state.

        Args:
            sid (str): Session ID of the client.
        """
        self.resume_robotics()

    async def on_stop(self, sid):
        """Stops all robotics operations immediately.

        Sets the state to STOP and exits all active processes.

        Args:
            sid (str): Session ID of the client.
        """
        self.stop_robotics()

    async def on_request_status(self, sid):
        """Responds with the current robotics status.

        Emits the current status to the requesting client.

        Args:
            sid (str): Session ID of the requesting client.
        """
        logger.info("Request for current robotics status received.")
        await self.emit("get_status", self.status.to_dict(), to=sid)

    async def on_request_conf(self, sid):
        """Responds with the current robotics configuration.

        Emits the current configuration to the requesting client.

        Args:
            sid (str): Session ID of the requesting client.
        """
        await self.emit("get_conf", self.robotics_conf, to=sid)
        logger.info("Current robotics configuration sent to requesting client.")

    async def on_request_types(self, sid):
        """Responds with the FluidTypes, RoboticsState, RoboticsRoutines definitions.

        Emits the current configuration to the requesting client.

        Args:
            sid (str): Session ID of the requesting client.
        """
        states_dict = {member.name: member.value for member in RoboticsState}
        routines_dict = {member.name: member.value for member in RoboticsRoutines}
        fluids_dict = {member.name: member.value for member in FluidTypes}
        await self.emit("get_types", {"states": states_dict, "routines": routines_dict, "fluids": fluids_dict}, to=sid)
        logger.info("Current RoboticsState and RoboticsRoutines sent to requesting client.")

    async def on_override_status(self, sid, override_data: dict):
        """Overrides the robotics status for manual intervention.

        Allows manual overriding of status.state and status.primed_syringe_pumps
        for recovery from problem situations.

        Args:
            sid (str): Session ID of the client.
            data (dict): Status values to override.
                Example: {"state": 0, "primed_syringe_pumps": True}
        """

        for override_key, value in override_data.items():
            if hasattr(self.status, override_key):
                # Check the type of the existing attribute and ensure new value matches
                attribute_value = getattr(self.status, override_key)
                attr_type = type(attribute_value)
                try:
                    # Try to cast the new value to the correct type
                    typed_value = attr_type(value)
                    setattr(self.status, override_key, typed_value)
                except (ValueError, TypeError):
                    logger.warning(f"Invalid type for {override_key}: expected {attr_type.__name__}, got {type(value).__name__}")

        logger.info(f"Robotics namespace state overriden with {override_data}.")

    async def on_connect_xArm(self, sid):
        """Reconnects to the xArm robot.

        Attempts to establish a connection with the xArm hardware.

        Args:
            sid (str): Session ID of the client.
        """
        self.arm.connect()
        logger.info("Robotics namespace reconnected to xArm.")

    async def on_reset_xArm(self, sid):
        """Reconnects and resets the xArm robot.

        Reconnects if disconnected and resets the arm to clear errors.

        Args:
            sid (str): Session ID of the client.
        """

        self.arm.reset()
        logger.info("Robotics namespace resetting xArm.")

    async def on_connect_pumps(self, sid): ...
    async def on_disconnect_pumps(self, sid): ...

    async def on_initialize_pumps(self, sid):
        """Initializes the XCaliburD/Tecan syringe pumps.

        Updates the pipette head state and initializes all non-empty pumps.

        Args:
            sid (str): Session ID of the client.
        """

        for index, pump in enumerate(self.pipette_head.pumps):
            if not self.pipette_head.pumps[index].empty:
                try:
                    if pump.hardware:
                        pump.hardware.init()
                except (SyringeError, SyringeTimeout) as e:
                    logger.warning(
                        f"Error trying to initialize {self.pipette_head.pumps[index].position_id} in position {index}: {e}"
                    )

        logger.info("Robotics namespace initialized non-empty XCaliburD pumps on the PipetteHead")

    @routine_decorator(RoboticsRoutines.PIPETTE)
    async def on_pipette_routine(self, sid, pipette_commands: dict[int, tuple[str, int]]):
        """Executes a multi-pump pipette command.

        Runs the PipetteHead to execute fluid transfer operations with the specified volumes.

        Args:
            sid (str): Session ID of the client.
            pipette_commands (dict[int, tuple[str, int]]): Maps pump indices to fluid type and volume.
                Example: {0: ("MEDIA", 100), 2: ("DRUG", 50)}

        Returns:
            ServerResult: Result of the routine execution.
        """
        pump_commands = [0] * 4
        for pump_index, pump_command in pipette_commands.items():
            if pump_command[0] not in FluidTypes.__members__:
                logger.warning(f"Invalid fluid type: {pump_command[0]}")
                return [False, f"Invalid fluid type: {pump_command[0]}"]
            if pump_command[1] < 0:
                logger.warning(f"Invalid volume: {pump_command[1]}")
                return [False, f"Invalid volume: {pump_command[1]}"]

            self.pipette_head.pumps[pump_index].set_port(FluidTypes[pump_command[0]])
            self.pipette_head.active_pumps.append(self.pipette_head.pumps[pump_index])
            self.status.active_pumps.append(asdict(self.pipette_head.pumps[pump_index]))

        await self.pipette_event(pump_commands)
        self.pipette_head.active_pumps = []
        self.status.active_pumps = []
        return [True, "executed successfully"]

    @routine_decorator(RoboticsRoutines.FILLING_VIALS_PUMPS)
    async def on_fill_vials_routine(self, sid, fill_commands: dict[int, tuple[str, int]]):
        """Fills vials in a station with specified fluid types and volumes.

        Args:
            sid (str): Session ID of the client.
            fill_commands (dict[int, tuple[str, int]]): Maps station IDs to fluid type and volume.
                Example: {0: ("MEDIA", 1000), 1: ("DRUG", 500)}

        Returns:
            ServerResult: Result of the routine execution.
        """

        for station_id, fill_command in fill_commands.items():
            if fill_command[0] not in FluidTypes.__members__:
                logger.warning(f"Invalid fluid type entered: {fill_command[0]}")
                return [False, f"Invalid fluid type entered: {fill_command[0]}"]
            if fill_command[1] > 7000:
                logger.warning(f"Invalid volume entered: {fill_command[1]}")
                return [False, f"Invalid volume entered: {fill_command[1]}"]

        station_pump_commands = {}
        for station_id, fill_command in fill_commands.items():
            station_pump_commands[station_id] = StationPumpCommands(
                vial_0={fill_command[0]: fill_command[1]},
                vial_1={fill_command[0]: fill_command[1]},
                vial_2={fill_command[0]: fill_command[1]},
                vial_3={fill_command[0]: fill_command[1]},
                vial_4={fill_command[0]: fill_command[1]},
                vial_5={fill_command[0]: fill_command[1]},
                vial_6={fill_command[0]: fill_command[1]},
                vial_7={fill_command[0]: fill_command[1]},
                vial_8={fill_command[0]: fill_command[1]},
                vial_9={fill_command[0]: fill_command[1]},
                vial_10={fill_command[0]: fill_command[1]},
                vial_11={fill_command[0]: fill_command[1]},
                vial_12={fill_command[0]: fill_command[1]},
                vial_13={fill_command[0]: fill_command[1]},
                vial_14={fill_command[0]: fill_command[1]},
                vial_15={fill_command[0]: fill_command[1]},
                vial_16={fill_command[0]: fill_command[1]},
                vial_17={fill_command[0]: fill_command[1]},
            )
        await self.influx_snake_helper(station_pump_commands)
        return [True, "executed successfully"]

    @routine_decorator(RoboticsRoutines.PRIMING_INFLUX)
    async def on_prime_pumps(self, sid, prime_commands: dict[int, tuple[str, int]]):
        """Primes syringe pumps on the PipetteHead.

        Prepares the syringe pumps for use by filling them with specified fluid types.

        Args:
            sid (str): Session ID of the client.
            prime_commands (dict[int, tuple[str, int]]): Maps pump indices to fluid type and volume.
                Example: {0: ("MEDIA", 500), 1: ("DRUG", 500)}

        Returns:
            ServerResult: Result of the routine execution.
        """

        pump_commands = [0] * 4
        for pump_index, pump_command in prime_commands.items():
            if pump_command[0] not in FluidTypes.__members__:
                logger.warning(f"Invalid fluid type: {pump_command[0]}")
                return [False, f"Invalid fluid type: {pump_command[0]}"]

            if pump_command[1] < 0:
                logger.warning(f"Invalid volume: {pump_command[1]}")
                return [False, f"Invalid volume: {pump_command[1]}"]

            self.pipette_head.pumps[pump_index].set_port(FluidTypes[pump_command[0]])
            self.pipette_head.active_pumps.append(self.pipette_head.pumps[pump_index])

        await self.run_pumps("prime", pump_commands)
        self.pipette_head.active_pumps = []
        self.pipette_head.primed = True
        return [True, "executed successfully"]

    @routine_decorator(RoboticsRoutines.DILUTION)
    async def on_dilution_routine(self, sid, dilution_commands: dict[int, dict[int, dict[str, int]]]):
        """Executes a dilution routine across SmartStations.

        Performs dilution operations based on user-supplied commands mapping fluid types
        and volumes to specific vials and stations.

        Args:
            sid (str): Session ID of the client.
            dilution_commands (dict[int, dict[int, dict[str, int]]]): Nested mapping of station, vial,
                fluid type, and volume.
                Example: {0: {3: {"MEDIA": 100, "DRUG": 50}, 4: {"MEDIA": 150}}}

        Returns:
            ServerResult: Result of the routine execution.
        """

        influx_commands: dict[int, StationPumpCommands] = {}
        for station_id, dilution_command in dilution_commands.items():
            influx_commands[station_id] = StationPumpCommands()
            for vial_id, pump_commands in dilution_command.items():
                influx_commands[station_id].__setattr__(f"vial_{vial_id}", pump_commands)
        await self.influx_snake_helper(influx_commands)
        return [True, "executed successfully"]

    async def influx_snake_helper(self, station_pump_commands: dict[int, StationPumpCommands]):
        """Executes sequential pipette events in a snake-like pattern across stations.

        Moves the pipette head in a snake pattern across vials, executing wash steps
        and fluid transfer operations according to the provided commands.

        Args:
            station_pump_commands (dict[int, StationPumpCommands]): Maps station IDs to
                pump commands for each vial.
                Example: {0: StationPumpCommands(vial_0={"MEDIA": 100}, vial_1={"DRUG": 50})}

        Raises:
            OperationEventError: If an error occurs during pipetting operations.
        """

        coordinate = VialCoordinate(x=-18, y=36)

        for station_id, pump_commands in station_pump_commands.items():
            self.status.active_station = station_id
            station = next(s for s in self.stations if s.id == station_id)
            vial_map = station.vial_map
            change_row = False

            for row_num in range(3):
                overhangs = [None] * (self.pipette_head.num_windows - 6)
                virtual_vial_row: list[int | None] = vial_map[row_num] + overhangs

                for window_index in range(self.pipette_head.num_windows):
                    ################# WASH STEP #################
                    # check pipette_head fluid types to build the vial window
                    if self.pipette_head.universal_fluids:
                        self.pipette_head.active_window = virtual_vial_row[
                            window_index * self.pipette_head.pump_num : window_index * self.pipette_head.pump_num
                            + self.pipette_head.pump_num
                        ]

                    if not self.pipette_head.universal_fluids:
                        self.pipette_head.active_window.append(virtual_vial_row[window_index])
                        if window_index >= self.pipette_head.pump_num:
                            self.pipette_head.active_window.pop(0)

                    logger.info(f"current vial window below pump head is: {self.pipette_head.active_window}")

                    for movement_step in range(3):
                        move_0 = station.xArmPlane_out.vial_to_xarm(coordinate)
                        move_1 = station.xArmPlane_out.vial_to_xarm(station.wash_location)
                        move_2 = station.xArmPlane_in.vial_to_xarm(station.wash_location)

                        self.arm_command_queue.extend([move_0, move_1, move_2])

                    try:
                        await self.pipette_event([0, 0, 0, 0])
                    except RoboticsError as e:
                        logger.error(e)
                        raise OperationEventError(f"Error running wash pipette event in influx_snake_helper(): {e}")

                    ################# INFLUX STEP #################
                    # if at end of row, next active_window will be the next row at same x location
                    if change_row:
                        coordinate.y -= 18

                    # otherwise keep y the same and modify x to move left or right
                    else:
                        row_logic = row_num % 2
                        coordinate.x = coordinate.x + (row_logic * 18)

                    for movement_step in range(3):
                        move_0 = station.xArmPlane_out.vial_to_xarm(station.wash_location)
                        move_1 = station.xArmPlane_out.vial_to_xarm(coordinate)
                        move_2 = station.xArmPlane_in.vial_to_xarm(coordinate)

                        self.arm_command_queue.extend([move_0, move_1, move_2])

                    # get pump volume commands for current vial window
                    pump_volumes = [0] * 4
                    for index, vial in enumerate(self.pipette_head.active_window):
                        # no pump detected above this vial (station overhang)
                        if vial is None:
                            continue

                        # pump detected above vial
                        # if the pump has the desired fluid_type, set the port and get the volume data
                        if vial:
                            target_pump = self.pipette_head.pumps[index]
                            for fluid_type_str, volume in getattr(pump_commands, f"vial_{vial}").items():
                                fluid_type = FluidTypes[fluid_type_str]
                                if any(port_fluid_type == fluid_type for port_num, port_fluid_type in target_pump.ports.items()):
                                    target_pump.set_port(fluid_type)
                                    pump_volumes[index] = volume
                                    # Remove the key after use to avoid duplicates
                                    getattr(pump_commands, f"vial_{vial}").pop(fluid_type_str)
                                    break
                            self.pipette_head.active_pumps.append(target_pump)

                    try:
                        await self.pipette_event(pump_volumes)
                        self.pipette_head.active_pumps = []
                    except RoboticsError as e:
                        logger.error(e)
                        raise OperationEventError(f"Error running influx pipette event in influx_snake_helper(): {e}")

                    # finished dilutions for current active_window, moving to next set of vials
                    change_row = False

                # change row
                change_row = True
            try:
                reset_location = station.xArmPlane_out.vial_to_xarm(coordinate)
                await self.arm.move(reset_location)
            except xArmError as e:
                logger.error(e)
                raise OperationEventError(f"Error moving arm above station at the end of influx_snake_helper(): {e}")

        # update status
        self.pipette_head.active_window = []
        self.status.active_station = -1

    @operation_decorator
    async def pipette_event(self, pump_commands: list[int]) -> None:
        """Performs coordinated aspiration and dispense actions with syringe pumps.

        Executes a complete pipetting cycle including aspiration and dispensing,
        coordinated with xArm movements if commands are queued.

        Args:
            pump_commands (list[int]): List of volumes for each pump position.
                Example: [100, 0, 50, 0]

        Raises:
            RoboticsError: If an error occurs during aspiration or dispense.
        """

        ################# ASPIRATION STEP #################
        logger.info(f"running aspiration during: {self.status.routine}")
        try:
            async with asyncio.TaskGroup() as aspiration_tasks:
                aspiration_tasks.create_task(self.run_pumps("extract", pump_commands))

                if self.arm_command_queue:
                    aspiration_tasks.create_task(self.execute_xArm_commands())
        except* (xArmError, SyringeError, SyringeTimeout) as e:
            logger.error(f"error trying to execute aspiration tasks during pipette_event(): {e}")
            raise e

        await self.check_for_interrupt()

        ################# DISPENSE STEP #################
        logger.info(f"running dispense during: {self.status.routine}")
        try:
            await self.run_pumps("dispense", pump_commands)
        except (SyringeError, SyringeTimeout) as e:
            logger.error(f"error trying to execute dispense during pipette_event(): {e}")
            raise e
        # verify that syringe pumps are ready to receive future commands
        logger.info(f"finished pipette event for: {self.status.routine}")

    @operation_decorator
    async def run_pumps(self, method_name: str, pump_commands: list[int]):
        """Executes a specified pump method across multiple syringe pumps.

        Sequentially runs the specified XCaliburD method on each pump with
        the corresponding volume command.

        Args:
            method_name (str): A valid XCaliburD method to execute.
                Example: "extract" or "dispense"
            pump_commands (list[int]): List of volumes for each pump position.
                Example: [100, 0, 50, 0]

        Raises:
            SyringeError: If a syringe operation fails.
            SyringeTimeout: If a syringe operation times out.
        """
        for index, command in enumerate(pump_commands):
            if not self.pipette_head.pumps[index].hardware:
                continue
            try:
                method = getattr(self.pipette_head.pumps[index], method_name)

                # wrapper function that adds the command to the pump's command chain and calls executeChain() & waitReady()
                def execute_pump_method(pump_method, method_args):
                    pump_method(*method_args)

                    delay = self.pipette_head.pumps[index].hardware.executeChain()
                    self.pipette_head.pumps[index].hardware.waitReady(delay)

                # run the blocking method in the default executor (thread pool)
                await asyncio.get_event_loop().run_in_executor(None, execute_pump_method, method, pump_commands)

            except (SyringeError, SyringeTimeout) as e:
                logger.error(f"Error with pump_position: {index} during {method_name}: {e}")
                raise e

    @operation_decorator
    async def execute_xArm_commands(self):
        """Sequentially executes all commands in the xArm command queue.

        Creates multi-step paths by executing each command in the queue
        one after the other, removing commands as they complete.

        Raises:
            xArmError: If a movement fails or if the queue is empty.
        """

        if not self.arm_command_queue:
            logger.warning("tried running execute_xArm_commands but no xArm commands found in queue")
            raise xArmError("tried running execute_xArm_commands but no xArm commands found in queue")

        while self.arm_command_queue:
            for index, command in enumerate(self.arm_command_queue):
                try:
                    await self.arm.move(command)
                    self.arm_command_queue.pop(index)
                except xArmError as e:
                    logger.error(f"tried running command {command} but following error ecnountered: {e}")
                    raise xArmError(f"error trying to run execute_xArm_commands: {e}")

    async def broadcast(self):
        """Broadcasts the current robotics status to all connected clients.

        Emits the current status information to all clients, includes
        checking for potential error conditions like overflows.
        """
        # check for potential overflow based on sensitivity threshold
        # overflow_trigger_map = {'left': [], 'right': []}
        # overflow_trigger_map['left'] =  [x * self.robotics_conf['overflow_voltage_step'] for x in self.robotics_conf['overflow_trigger_map']['left']]
        # overflow_trigger_map['right'] =  [x * self.robotics_conf['overflow_voltage_step'] for x in self.robotics_conf['overflow_trigger_map']['right']]
        # for quad_index in range(4):
        #    if (self.htevolver_client.overflow_data['left'][quad_index] > self.robotics_conf['overflow_voltage_threshold']) or (self.htevolver_client.overflow_data['right'][quad_index] > self.robotics_conf['overflow_voltage_threshold']):
        #        self.status.overflow_status['quads'][quad_index] = True
        #        self.stop_robotics()

        # identify the vial(s) that overflowed
        #        for index in range(18):
        #            if self.htevolver_client.overflow_data['right'][quad_index] >= overflow_trigger_map['right'][index] - self.robotics_conf['overflow_voltage_threshold'] or self.htevolver_client.overflow_data['right'][quad_index] <= overflow_trigger_map['right'][index] + self.robotics_conf['overflow_voltage_threshold']:
        #                for vial_index in range(index, index + 5):
        #                    if self.htevolver_client.overflow_data['left'][quad_index] >= overflow_trigger_map['left'][index] - self.robotics_conf['overflow_voltage_threshold'] or self.htevolver_client.overflow_data['left'][quad_index] <= overflow_trigger_map['left'][index] + self.robotics_conf['overflow_voltage_threshold']:
        #                        self.status.overflow_status['vial'] = vial_index

        # emit robotics status to all connected clients
        logging.info(f"robotics status broadcast: {self.status}")
        await self.emit("broadcast", self.status.to_dict())

    def error_warn_change_callback(self, xarm_api_data: dict):
        """Updates error and warning codes based on xArm feedback.

        Args:
            data (dict): A dictionary containing error and warning codes.
                Example: {"error_code": 0, "warn_code": 0}
        """

        self.status.xArm.error_code = xarm_api_data["error_code"]
        self.status.xArm.warning_code = xarm_api_data["warn_code"]
        logger.debug(f"xArm error/warn change callback input: {xarm_api_data}")
        if xarm_api_data["error_code"] != 0:
            self.emergency_stop_robotics()
            logger.error(f"xArm error_code encountered: {xarm_api_data['error_code']}")
        if xarm_api_data["warn_code"] != 0:
            logger.warning(f"xArm warning_code encountered: {xarm_api_data['warn_code']}")

    def state_changed_callback(self, xarm_api_data: dict):
        """Updates xArm state based on controller feedback.

        Args:
            data (dict): Contains the xArm state information.
                Example: {"state": 0}
        """

        self.status.xArm.state = xarm_api_data["state"]
        logger.debug(f"xArm state change callback input: {xarm_api_data}")
        if xarm_api_data["state"] == 4:
            self.emergency_stop_robotics()
            logger.error(f"xArm entered stop state: {xarm_api_data['state']}")

    def connect_changed_callback(self, xarm_api_data: dict):
        """Updates xArm connection status based on controller feedback.

        Args:
            data (dict): Contains the connection status.
                Example: {"connected": True}
        """

        self.status.xArm.connected = xarm_api_data["connected"]
        logger.debug(f"xArm connect change callback input: {xarm_api_data}")

    def load_conf(self):
        """Loads in the robotics configuration from memory.

        Loads the latest settings from the robotics_conf file and updates the PipetteHead and SmartStations
        to ensure current operations use up-to-date configuration values.
        """

        with open(self.robotics_conf_path, "r") as conf:
            self.robotics_conf = yaml.safe_load(conf)

        for station in self.stations:
            station.update(self.robotics_conf)
        self.pipette_head.update(self.robotics_conf)

    def stop_robotics(self):
        """Stops all robotics and pump operations due to user intervention.

        Terminates all syringe pump commands and puts xArm into stop state.
        """

        self.status.state = RoboticsState.STOP
        try:
            for pump in self.pipette_head.pumps:
                if pump.hardware:
                    pump.hardware.terminateCmd()
                    pump.hardware.resetChain()
            self.arm.set_state(4)
        except (SyringeError, SyringeTimeout) as e:
            logger.error(f"error encountered trying to call stop_robotics(): {e}")
        logger.info("Robotics namespace put into STOP state, active processes have been exited.")

    def pause_robotics(self):
        """Pauses robotics operations if currently busy.

        Sets the robotics state to PAUSE, pauses xArm operations by setting its
        state to 3 (pause), and terminates pending syringe pump commands.
        """
        if self.status.state == RoboticsState.BUSY:
            self.status.state = RoboticsState.PAUSE
            self.arm.set_state(3)
            for pump in self.pipette_head.pumps:
                if pump.hardware:
                    pump.hardware.terminateCmd()
        logger.info("Robotics namespace put into PAUSE state")

    def resume_robotics(self):
        if self.status.state == RoboticsState.PAUSE:
            self.status.state = RoboticsState.BUSY
            self.arm.set_state(0)
            for pump in self.pipette_head.pumps:
                if pump.hardware:
                    pump.hardware.sendRcv("", execute=True)
        logger.info("Robotics namespace put back into BUSY state, resuming previously paused activity.")

    def emergency_stop_robotics(self):
        """Stops all robotics and pump operations in emergency situations.

        Terminates all syringe pump commands, triggers emergency stop on the xArm,
        and disconnects from the hardware. Requires manual intervention to restart.
        """

        try:
            self.stop_robotics()
            for pump in self.pipette_head.pumps:
                pump.disconnect()
            self.arm.disconnect()
        except (SyringeError, SyringeTimeout) as e:
            logger.error(f"error encountered trying to call stop_robotics(): {e}")
        logger.info("Robotics namespace put into EMERGENCY_STOP state")

    async def check_for_interrupt(self):
        """Checks for pause or stop signals during routine execution.

        Pauses execution if a PAUSE state is detected, resuming when state
        changes or raising an exception if a STOP is received.
        """

        while self.status.state == RoboticsState.PAUSE:
            if self.status.state == RoboticsState.STOP:
                raise ExitRobotics
            else:
                await asyncio.sleep(0.1)
