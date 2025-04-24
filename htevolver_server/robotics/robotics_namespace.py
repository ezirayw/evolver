import asyncio
import logging
import os
import time
from dataclasses import asdict, dataclass, field
from enum import Enum

import numpy as np
import skimage as ski
import socketio
import yaml
from htevolver_client.interfaces.htevolver_interface import HTEvolverNamespace
from skimage.transform import EuclideanTransform
from tecancavro.models import SyringeError, SyringeTimeout, XCaliburD
from tecancavro.transport import TecanAPISerial
from xarm.wrapper import XArmAPI

logger = logging.getLogger(__name__)


#### CUSTOM EXCEPTIONS ####
class xArmError(Exception): ...


class PipetteEventError(Exception): ...


class HelperEventError(Exception): ...


class ExitRobotics(Exception): ...


#### ENUMS ####
class FluidType(Enum):
    EMPTY = 0
    MEDIA = 1
    DRUG = 2
    STERILIZE = 3
    WASH = 4


class RoboticsState(Enum):
    IDLE = 0
    BUSY = 1
    PAUSE = 2
    RESUME = 3
    STOP = 5
    EMERGENCY_STOP = 4


class RoboticsRoutines(Enum):
    NO_ROUTINE = 0
    DILUTION = 1
    PIPETTE = 2
    FILLING_VIALS_PUMPS = 3
    FILLING_VIALS_IPP = 4
    PRIMING_INFLUX = 5
    PRIMING_EFFLUX = 6


class IPPpolarity(Enum):
    INFLUX = 0
    EFFLUX = 1


#### DECORATORS ####
def helper_decorator(func):
    """Decorator for helper functions that will update robotics conf, update status, manage error handling, and execute other accessory background tasks"""

    async def wrapper(self, *args, **kwargs):
        try:
            self.status.state = RoboticsState.BUSY
            self.update_conf()
            await self.check_for_interrupt()

            # run the target helper function
            await func(self, *args, **kwargs)

            self.status.state = RoboticsState.IDLE
        except (xArmError, SyringeError, SyringeTimeout) as e:
            self.status.state = RoboticsState.EMERGENCY_STOP
            raise HelperEventError(f"error running {func.__name__}: {e}")

    return wrapper


def routine_decorator(routine_type: RoboticsRoutines):
    """Decorator for routines that will run routine functions and construct the return data package that is sent to the client"""

    def decorator(func):
        async def wrapper(self, *args, **kwargs):
            if self.status.state == RoboticsState.IDLE:
                start_time = time.time()
                try:
                    self.status.routine = routine_type
                    logger.info(f"Running {func.__name__} routine")

                    result, message = await func(self, *args, **kwargs)

                    logger.info(f"Done running the {func.__name__} routine")
                    end_time = time.time()
                    self.status.routine = RoboticsRoutines.NO_ROUTINE

                    return RoutineResult(
                        done=result,
                        routine=routine_type,
                        status=asdict(self.status),
                        elapsed_time=end_time - start_time,
                        message=f"{func.__name__}: {message}",
                    )

                except HelperEventError as e:
                    end_time = time.time()
                    self.status.state = RoboticsState.EMERGENCY_STOP
                    return RoutineResult(
                        done=False,
                        routine=routine_type,
                        status=asdict(self.status),
                        elapsed_time=end_time - start_time,
                        message=f"Error encountered trying to run {func.__name__}: {e}",
                    )
            else:
                logger.warning(f"Tried running the {func.__name__} routine but the robotics namespace is not idle")
                return RoutineResult(
                    done=False,
                    routine=routine_type,
                    status=asdict(self.status),
                    elapsed_time=0.00,
                    message=f"Tried running the {func.__name__} routine but the robotics namespace is not idle",
                )

        return wrapper

    return decorator


@dataclass
class PumpConfig:
    name: str = field(default="empty_position")
    type: FluidType = field(default=FluidType.EMPTY)


@dataclass
class PipetteHead:
    pump_config: list[PumpConfig]
    pumps: list[XCaliburD]
    pump_num: int = field(default=0)
    uniform_type: bool = field(default=True)
    window_num: int = field(default=0)
    vial_window: list[int | None] = field(default_factory=list)

    def update(self):
        """
        Updates the properties of the pump head based on the current configuration.

        Checks if all non-empty pump positions have the same fluid type and calculates window numbers for vial operations.
        """
        pump_types = [pump.type for pump in self.pump_config]
        non_empty_pumps = [type for type in pump_types if type != FluidType.EMPTY]
        self.uniform_type = len(set(non_empty_pumps)) == 1
        self.pump_num = len(non_empty_pumps)
        if self.uniform_type:
            self.window_num = int(6 / self.pump_num)
        else:
            self.window_num = 6 + (self.pump_num - 1)


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
    z: float  # defines where in space the xArmPlane exists
    transform_matrix: EuclideanTransform = field(default_factory=EuclideanTransform)

    def __post_init__(self):
        self.rigid_transform()

    def rigid_transform(self):
        """Calculate the rigid transformation matrix between two sets of coordinates. Used to convert vial coordinates into xArm coordinates."""
        vial_coordinates = np.array([[0, 36], [90, 0]])
        vial_0 = np.array([self.vial0_x, self.vial0_y])
        vial_17 = np.array([self.vial17_x, self.vial17_y])
        np.array([vial_0, vial_17])
        tform = ski.transform.EuclideanTransform()
        tform.estimate(vial_coordinates, np.array([vial_0, vial_17]))
        self.transform_matrix = tform

    def vial_to_xarm(self, evolver_coordinates: VialCoordinate) -> xArmCoordinate:
        """Transform coordinates from evolver coordinate system to xArm coordinate system.

        This method applies the rigid transformation matrix to convert coordinates
        from the evolver reference frame to the xarm reference frame.

        Args:
            evolver_coordinates (list[int]): List containing [x, y] coordinates in the evolver system.

        Returns:
            xArmCoordinate: The transformed coordinates in the xarm coordinate system.
        """
        np_coordinates = np.array([[VialCoordinate.x], [VialCoordinate.y], [1]])
        transformed = np.dot(self.transform_matrix, np_coordinates)
        return xArmCoordinate(x=transformed[0][0], y=transformed[1][0], z=self.z)


@dataclass
class EffluxBoard:
    ipp_frequency: int = field(default=5)  # Hz
    ipp_duration: int = field(default=30)  # seconds
    ipp_polarity: IPPpolarity = field(default=IPPpolarity.EFFLUX)


@dataclass
class SmartStation:
    xArmPlane_in: xArmPlane
    xArmPlane_out: xArmPlane
    wash_location: VialCoordinate = field(default_factory=lambda: VialCoordinate(x=72, y=-29))
    wash_depth: float = field(init=False)
    efflux_board: EffluxBoard = field(default_factory=EffluxBoard)
    vial_map: list[list[int]] = field(default_factory=lambda: [[0, 1, 2, 3, 4, 5], [11, 10, 9, 8, 7, 6], [12, 13, 14, 15, 16, 17]])


@dataclass
class StationPumpCommands:
    vial_0: dict[str, int]
    vial_1: dict[str, int]
    vial_2: dict[str, int]
    vial_3: dict[str, int]
    vial_4: dict[str, int]
    vial_5: dict[str, int]
    vial_6: dict[str, int]
    vial_7: dict[str, int]
    vial_8: dict[str, int]
    vial_9: dict[str, int]
    vial_10: dict[str, int]
    vial_11: dict[str, int]
    vial_12: dict[str, int]
    vial_13: dict[str, int]
    vial_14: dict[str, int]
    vial_15: dict[str, int]
    vial_16: dict[str, int]
    vial_17: dict[str, int]


@dataclass
class xArmStatus:
    warning_code: int = 0
    error_code: int = 0
    arm_state: int = 0
    connected: bool = False


@dataclass
class RoboticsStatus:
    state: RoboticsState = field(default=RoboticsState.IDLE)
    routine: RoboticsRoutines = field(default=RoboticsRoutines.NO_ROUTINE)
    active_station: int = field(default=-1)
    active_syringe_pumps: list = field(default_factory=list)
    vial_window: list = field(default_factory=list)
    xArm: xArmStatus = field(default_factory=lambda: xArmStatus())
    primed_ipp: bool = field(default=False)
    primed_syringe_pumps: bool = field(default=False)
    # overflow_status: dict = field(default_factory=lambda: {"quads": [False, False, False, False], "vial": None})


@dataclass
class RoutineResult:
    done: bool
    routine: RoboticsRoutines
    status: dict
    elapsed_time: float
    message: str


class RoboticsNamespace(socketio.AsyncNamespace):
    def __init__(
        self,
        robotics_conf: dict,
        htevolver_client: HTEvolverNamespace,
        namespace: str = "/robotics",
        robotics_conf_path: str = os.path.join(os.path.expanduser("~"), "robotics_conf.yml"),
    ):
        super().__init__(namespace)
        self.robotics_conf: dict = robotics_conf
        self.htevolver_client: HTEvolverNamespace = htevolver_client
        self.robotics_conf_path: str = robotics_conf_path
        self.status: RoboticsStatus = RoboticsStatus()
        self.xArm_command_queue: list[xArmCoordinate] = []

        # initialize SmartStations
        self.stations: dict[int, SmartStation] = {}
        plane_calibration = self.robotics_conf["xArmPlane_calibration"]
        for index in range(4):
            plane_out = xArmPlane(**plane_calibration[index]["xArmPlane_out"])
            plane_in = xArmPlane(**plane_calibration[index]["xArmPlane_in"])
            self.stations[index] = SmartStation(plane_out, plane_in)

        # initialize PipetteHead
        pump_configs: list[PumpConfig] = []
        pumps: list[XCaliburD] = []
        serial_port = self.robotics_conf["pump_serial_port"]
        for pump_index in range(4):
            pump_configs[pump_index] = PumpConfig(**robotics_conf["pumps"][pump_index])
            pumps[pump_index] = XCaliburD(
                com_link=TecanAPISerial(pump_index, ser_port=serial_port, ser_baud=9600),
            )
        self.pipette_head: PipetteHead = PipetteHead(pump_configs, pumps)
        self.pipette_head.update()

        # initialize XArm instance
        self.arm = XArmAPI(self.robotics_conf["xarm_ip"], enable_report=True)
        self.setup_xArm()
        self.status.xArm.connected = self.arm.connected
        self.register_callback()
        logger.info("robotics_evolver server initialized")

    async def on_connect(self, sid, environ, auth):
        """Called when client connects to server."""
        logger.info("Client connected to robotics_eVOLVER server")

    async def on_disconnect(self, sid):
        """Called when client disconnects from server."""
        logger.info("Client disconnected to robotics_eVOLVER server")

    async def on_pause(self, sid):
        """Switch status state to PAUSE."""

        # if self.status.mode != 'exit' and self.status.mode != 'idle' and self.status.mode != 'emergency_stop':
        if self.status.state == RoboticsState.BUSY:
            self.status.state = RoboticsState.PAUSE
        logger.info("Robotics namespace put into PAUSE state")

    async def on_resume(self, sid):
        """Switch status state to RESUME if previously in PAUSE.

        Args:
            `sid`: session ID of the client
        """

        if self.status.state == RoboticsState.PAUSE:
            self.status.state = RoboticsState.BUSY
            self.arm.set_state(0)
        logger.info("Robotics namespace put into BUSY state, resuming previously paused activity.")

    async def on_stop(self, sid):
        """Switch state to STOP.

        Args:
            `sid`: session ID of the client
        """

        self.status.state = RoboticsState.STOP
        self.stop_robotics()
        logger.info("Robotics namespace put into STOP state, exiting active processes.")

    async def on_request_status(self, sid):
        """Request the current robotics status."""

        logger.info("Request for current robotics status received.")
        await self.emit("get_status", asdict(self.status), to=sid)

    async def on_override_status(self, sid, data: dict):
        """Override `status.state`, `status.primed_syringe_pumps`, and `status.primed_ipp`.

        Args:
            `sid`: session ID of the client.
            `data`: contains new robotics status information within respective keys."""

        if "state" in data:
            try:
                new_state = RoboticsState(data["state"])
                self.status.state = new_state
            except (KeyError, ValueError):
                logger.warning(f"Invalid state value provided: {data['state']} when overriding status state")

        if "primed_syringe_pumps" in data:
            self.status.primed_syringe_pumps = data["primed_syringe_pumps"]
        if "primed_ipp" in data:
            self.status.primed_ipp = data["prime_ipp"]

        logger.info(f"Robotics namespace state overriden with {data}.")

    async def on_reconnect_xArm(self, sid):
        """Reconnect xArm.

        Args:
            `sid`: session ID of the client.
        """
        self.arm.connect()
        logger.info("Robotics namespace reconnected to xArm.")

    async def on_reset_xArm(self, sid):
        """Reconnect and reset xArm

        Args:
            `sid`: session ID of the client.
        """

        if not self.status.xArm.connected:
            self.arm.connect()
        self.reset_xArm()
        logger.info("Robotics namespace resetting xArm.")

    async def on_initialize_pumps(self, sid):
        """Initialize XCaliburD/Tecan syringe pumps

        Args:
            `sid`: session ID of the client
        """

        self.pipette_head.update()
        for index, tecan_pump in enumerate(self.pipette_head.pumps):
            if self.pipette_head.pump_config[index].type != FluidType.EMPTY:
                try:
                    tecan_pump.init()
                except (SyringeError, SyringeTimeout) as e:
                    logger.warning(f"Error trying to initialize {self.pipette_head.pump_config[index].name} in position {index}: {e}")

        logger.info("Robotics namespace initialized non-empty XCaliburD pumps on the PipetteHead")

    @routine_decorator(RoboticsRoutines.PIPETTE)
    async def on_pipette_routine(self, sid, pipette_commands: dict[str, int]):
        """Run the PipetteHead to execute a multi-pump, pipette command.

        Args:
            `sid`: session ID of the client.
            `pipette_commands`: dictionary mapping a pipette volume value to a key representing the name of the syringe pump in the PipetteHead."""

        pump_commands = [0] * 4
        for pump_index, tecan_pump in enumerate(self.pipette_head.pumps):
            target_pump = self.pipette_head.pump_config[pump_index].name
            self.status.active_syringe_pumps.append(target_pump)
            if target_pump in pipette_commands:
                pump_commands[pump_index] = pipette_commands[target_pump]

        await self.pipette_event(pump_commands)
        self.status.active_syringe_pumps = []
        return [True, "executed successfully"]

    @routine_decorator(RoboticsRoutines.FILLING_VIALS_PUMPS)
    async def on_fill_pump_routine(self, sid, fluid_types: list[str], fill_volumes: list[int] = [5000, 5000, 5000, 5000]):
        """Fill vials of a station with a FluidType using the PipetteHead. Fill volume defaults to max_volume config

        Args:
            `sid`: The session ID of the client.
            `types`: list of valid FluidType names to use for filling a station's vials, with the index corresponding to a station
            `fill_volumes`: list of vial volumes to fill to, with the index corresponding to a station."""

        if len(fluid_types) != len(fill_volumes):
            logger.warning("fluid_types and fill_volumes arguments are not same size")
            return [False, "fluid_types and fill_volumes arguments are not same size"]

        for fluid_type in fluid_types:
            if fluid_type not in FluidType.__members__:
                logger.warning(f"Invalid fluid type entered: {fluid_type}")
                return [False, f"Invalid fluid type entered: {fluid_type}"]

        station_pump_commands = {}
        for station_index in range(len(fluid_types)):
            station_pump_commands[station_index] = StationPumpCommands(
                vial_0={fluid_types[station_index]: fill_volumes[station_index]},
                vial_1={fluid_types[station_index]: fill_volumes[station_index]},
                vial_2={fluid_types[station_index]: fill_volumes[station_index]},
                vial_3={fluid_types[station_index]: fill_volumes[station_index]},
                vial_4={fluid_types[station_index]: fill_volumes[station_index]},
                vial_5={fluid_types[station_index]: fill_volumes[station_index]},
                vial_6={fluid_types[station_index]: fill_volumes[station_index]},
                vial_7={fluid_types[station_index]: fill_volumes[station_index]},
                vial_8={fluid_types[station_index]: fill_volumes[station_index]},
                vial_9={fluid_types[station_index]: fill_volumes[station_index]},
                vial_10={fluid_types[station_index]: fill_volumes[station_index]},
                vial_11={fluid_types[station_index]: fill_volumes[station_index]},
                vial_12={fluid_types[station_index]: fill_volumes[station_index]},
                vial_13={fluid_types[station_index]: fill_volumes[station_index]},
                vial_14={fluid_types[station_index]: fill_volumes[station_index]},
                vial_15={fluid_types[station_index]: fill_volumes[station_index]},
                vial_16={fluid_types[station_index]: fill_volumes[station_index]},
                vial_17={fluid_types[station_index]: fill_volumes[station_index]},
            )
        await self.influx_snake_helper(station_pump_commands)
        return [True, "executed successfully"]

    @routine_decorator(RoboticsRoutines.FILLING_VIALS_IPP)
    async def on_fill_ipp_routine(self, sid, data: dict):
        ipp_commands = {}
        for quad in data["target_quads"]:
            ipp_commands[quad] = {"polarity": 1}
        await self.efflux_ipp_helper(ipp_commands)

    @routine_decorator(RoboticsRoutines.PRIMING_INFLUX)
    async def on_prime_influx_routine(self, sid, prime_commands: dict[str, int]):
        """Prime syringe pumps on the PipetteHead based.

        Args:
            `sid`: session ID of the client.
            `prime_commands`: dictionary mapping a priming volume value to a key representing the name of the syringe pump in the PipetteHead."""

        pump_commands = [0] * 4
        for pump_index, tecan_pump in enumerate(self.pipette_head.pumps):
            target_pump = self.pipette_head.pump_config[pump_index].name
            self.status.active_syringe_pumps.append(target_pump)
            if target_pump in prime_commands:
                pump_commands[pump_index] = prime_commands[target_pump]

        await self.run_pumps("prime", pump_commands)
        self.status.active_syringe_pumps = []
        self.status.primed_syringe_pumps = True
        return [True, "executed successfully"]

    @routine_decorator(RoboticsRoutines.PRIMING_EFFLUX)
    async def on_prime_efflux_routine(self, sid, data):
        """Prime EffluxBoard by actuating IPPs in forward and reverse polarities.

        Args:
            `sid`: session ID.
            `data`: contains the parameters to run the helper function.

        Returns:
            dict: contains the result of the prime_efflux routine."""
        await self.efflux_ipp_helper(ipp_commands)

    async def on_dilution_routine(self, sid, data: dict):
        """Perform cooridinated syringe pump, xArm, and IPP hardware to dilute cultures for target vials.

        Args:
            sid (str): session ID.
            self: The reference to the current instance of the class.
            data (dict): The dilution commands to be executed.
                'commands' (dict): contains list of commands under 'syringe_pump_commands' and 'ipp_efflux_command' keys.
                'target_quads': (list) list of target smart quads for influx commands
                'mode': (str) mode of operation. Can be 'dilution' or 'fill_vials',
                'wash': (bool) flag indicating whether to perform a wash step.

        Returns:
            dict: contains the result of the dilution process.

        Raises:
            HelperEventError: If an error occurs during the influx routine process."""

        # get start time of influx routine to later calculate total elapsed time, useful for clients to gauge routine duration
        start_time = time.time()

        # execute influx routine using influx_snake_helper function
        if self.status.mode == "idle":
            try:
                logger.info("Executing the following influx routine command: %s" % data)
                self.status.mode = "dilution"
                data["mode"] = "dilution"
                await self.influx_snake_helper(data)
                await asyncio.sleep(3)  # add delay to give time for culture mixing prior to efflux
                await self.efflux_ipp_helper(data["commands"]["ipp_efflux_command"])
                self.status.mode = "idle"
                end_time = time.time()
                return {
                    "done": True,
                    "routine": "influx",
                    "status": asdict(self.status),
                    "elapsed_time": end_time - start_time,
                    "message": "influx_snake_helper called successfully",
                }

            except HelperEventError as e:
                end_time = time.time()
                logger.exception(e)
                return {
                    "done": False,
                    "routine": "influx",
                    "status": asdict(self.status),
                    "elapsed_time": end_time - start_time,
                    "message": "HelperEventError encountered, check HT_eVOLVER logs for traceback",
                }
            except ExitRobotics:
                end_time = time.time()
                logger.info("Exiting influx routine")
                return {
                    "done": False,
                    "routine": "influx",
                    "status": asdict(self.status),
                    "elapsed_time": end_time - start_time,
                    "message": "Exiting influx routine",
                }
        else:
            return {
                "done": False,
                "routine": "influx",
                "status": asdict(self.status),
                "message": "status mode not idle",
            }

    def update_conf(self):
        """Updates namespace config by loading the contents of the robotics_conf file."""

        with open(self.robotics_conf_path, "r") as conf:
            self.robotics_conf = yaml.safe_load(conf)

    def register_callback(self):
        """Register the error_warn_changed_callback and state_changed_callback for the xArm 5."""

        self.arm.register_error_warn_changed_callback(callback=self.error_warn_change_callback)
        self.arm.register_state_changed_callback(callback=self.state_changed_callback)
        self.arm.register_connect_changed_callback(callback=self.connect_changed_callback)

    def error_warn_change_callback(self, data: dict):
        """Update the error and warning codes in the status class attribute.

        Args:
            data (dict): A dictionary containing the error and warning codes."""

        self.status.xArm.error_code = data["error_code"]
        self.status.xArm.warning_code = data["warn_code"]
        if data["error_code"] != 0:
            self.stop_robotics()
            logger.error(f"xArm error_code encountered: {data['error_code']}")
        if data["warn_code"] != 0:
            logger.warning(f"xArm warning_code encountered: {data['warn_code']}")

    def state_changed_callback(self, data: dict):
        """Update the arm state in the status class attribute based on the received data.

        Args:
            data (dict): contains the xArm state."""

        logger.debug(data)
        self.status.xArm.arm_state = data["state"]
        if data["state"] == 4:
            self.stop_robotics()
            logger.error(f"xArm entering error state: {data['state']}")

    def connect_changed_callback(self, data: dict):
        """Update the arm connection status in the status class attribute based on the received data.

        Args:
            data (dict): A dictionary containing the connection status."""

        self.status.xArm.connected = data["connected"]
        logger.info(f"xArm connection status changed to: {data['connected']}")

    def stop_robotics(self):
        """Stop all robotics and tecan syringe pump operations. Used in cases of emergencies or unforseen errors. Requires manual intervention to restart."""

        logger.warning("stop_robotics() called - check logs to identify cause of error")
        try:
            for tecan_pump in self.pipette_head.pumps:
                tecan_pump.terminateCmd()
            self.arm.emergency_stop()
            self.arm.disconnect()
        except (SyringeError, SyringeTimeout) as e:
            logger.error(f"error encountered trying to call stop_robotics(): {e}")

    async def check_for_interrupt(self):
        """Called during routines to catch pause or exit signals from client. Hangs if pause is detected and/or raises ExitRobotics exception if exit is detected"""

        # pause detected, halt influx routine until pause is lifted or influx routine is stopped
        while self.status.state == RoboticsState.PAUSE:
            await asyncio.sleep(0.1)

    def setup_xArm(self):
        self.arm.clean_warn()
        self.arm.clean_error()
        self.arm.motion_enable(enable=True)
        self.arm.set_state(state=0)
        self.arm.set_mode(0)
        self.arm.set_collision_sensitivity(2)
        self.arm.set_self_collision_detection(True)
        # handle potential C21 kinematic errors (align end effector to be parallel to ground)
        code, angles = self.arm.get_servo_angle()
        if code == 0:
            angles[3] = -(angles[1] + angles[2])
            self.arm.set_servo_angle(angle=angles, wait=True)
        self.status.xArm.connected = self.arm.connected

    def reset_xArm(self):
        """Clear potential warnings/errors and align end effector."""
        self.arm.clean_warn()
        self.arm.clean_error()
        self.arm.motion_enable(True)
        self.arm.set_state(0)
        code, angles = self.arm.get_servo_angle()
        if code == 0:
            angles[3] = -(angles[1] + angles[2])
            self.arm.set_servo_angle(angle=angles, wait=True)

    async def pipette_event(self, pump_commands: list[int]) -> None:
        """Perform aspiration and dispense actions with syringe pumps that can be coordinated with the xArm."""

        await self.check_for_interrupt()

        ################# ASPIRATION STEP #################
        logger.info(f"running aspiration during: {self.status.routine.name}")
        try:
            async with asyncio.TaskGroup() as aspiration_tasks:
                aspiration_tasks.create_task(self.run_pumps("extract", pump_commands))

                if self.xArm_command_queue:
                    aspiration_tasks.create_task(self.execute_xArm_commands())
        except* (xArmError, SyringeError, SyringeTimeout) as e:
            logger.error(e)
            raise PipetteEventError(f"error trying to execute aspiration tasks during pipette_event(): {e}")

        await self.check_for_interrupt()

        ################# DISPENSE STEP #################
        logger.info(f"running dispense during: {self.status.routine.name}")
        try:
            await self.run_pumps("dispense", pump_commands)
        except (SyringeError, SyringeTimeout) as e:
            logger.error(e)
            raise PipetteEventError(f"error trying to execute dispense during pipette_event(): {e}")

        await self.check_for_interrupt()

        # verify that syringe pumps are ready to receive future commands
        logger.info(f"finished pipette event for: {self.status.routine.name}")

    @helper_decorator
    async def efflux_ipp_helper(self, data: dict):
        """Helper method to use efflux IPPs in polarity for efflux during influx routines or to add media/fluids into vials.

        Args:
            data (dict):
                polarity (int): The polarity of the efflux IPP actuation. Can be 0 (towards waste) or 1 (into vials).
                duration (int): The duration of the efflux IPP actuation, in seconds.
                frequency (int): Number of actuation events per second, Hz. Dictates flow rate.
        Returns:
            result (dict): A dictionary with the result of the priming operation."""

        # use parameters found in data, otherwise pull from defaults found in robotics_conf
        ipp_commands = []
        for quad in data:
            ipp_address_key = quad + "_efflux"
            if "duration" in data[quad]:
                duration = data[quad]["duration"]
            else:
                duration = self.robotics_conf["ipp_efflux_settings"]["duration"]

            if "frequency" in data[quad]:
                frequency = data[quad]["frequency"]
            else:
                frequency = self.robotics_conf["ipp_efflux_settings"]["frequency"]

            if "polarity" in data[quad]:
                polarity = data[quad]["polarity"]
            else:
                polarity = self.robotics_conf["ipp_efflux_settings"]["polarity"]
            ipp_commands.append(self.make_ipp_command(duration, frequency, ipp_address_key, polarity))

        ipp_efflux_command = ["--"] * 48  # empty command
        # collapse generated commands to a single command
        for index in range(len(ipp_commands[0])):
            for ipp_command in ipp_commands:
                if ipp_command[index] != "--":
                    ipp_efflux_command[index] = ipp_command[index]

        # send efflux command
        self.htevolver_client.fluid_command(ipp_efflux_command)

    @helper_decorator
    async def influx_snake_helper(self, station_pump_commands: dict[int, StationPumpCommands]):
        """Helper function for executing sequential pipette_event(s) in a snake-like pattern across Smart Quads. Use for dilution and fill_vial routines events if xArm is desired.

        Args:

        Raises:
            HelperEventError: Raised in the event a pipetteEvent Exception is caught to coordinate experiment management."""

        coordinate = VialCoordinate(x=-18, y=36)
        self.pipette_head.update()

        for station_index, pump_commands in station_pump_commands.items():
            self.status.active_station = station_index
            station = self.stations[station_index]
            vial_map = station.vial_map
            change_row = False

            for row_num in range(3):
                overhangs = [None] * (self.pipette_head.window_num - 6)
                virtual_vial_row: list[int | None] = vial_map[row_num] + overhangs

                for window_index in range(self.pipette_head.window_num):
                    ################# WASH STEP #################
                    # check pipette_head fluid types to build the vial window
                    if self.pipette_head.uniform_type:
                        self.pipette_head.vial_window = virtual_vial_row[
                            window_index * self.pipette_head.pump_num : window_index * self.pipette_head.pump_num
                            + self.pipette_head.pump_num
                        ]

                    if not self.pipette_head.uniform_type:
                        self.pipette_head.vial_window.append(virtual_vial_row[window_index])
                        if window_index >= self.pipette_head.pump_num:
                            self.pipette_head.vial_window.pop(0)

                    logger.info(f"current vial window below pump head is: {self.pipette_head.vial_window}")

                    for movement_step in range(3):
                        move_0 = station.xArmPlane_out.vial_to_xarm(coordinate)
                        move_1 = station.xArmPlane_out.vial_to_xarm(station.wash_location)
                        move_2 = station.xArmPlane_in.vial_to_xarm(station.wash_location)

                        self.xArm_command_queue.extend([move_0, move_1, move_2])

                    try:
                        await self.pipette_event([0, 0, 0, 0])
                    except PipetteEventError as e:
                        logger.error(e)
                        raise HelperEventError(f"Error running wash pipette event in influx_snake_helper(): {e}")

                    ################# INFLUX STEP #################
                    # if at end of row, next vial_window will be the next row at same x location
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

                        self.xArm_command_queue.extend([move_0, move_1, move_2])

                    # get pump volume commands for current vial window
                    pump_volumes = [0] * 4
                    for index, vial in enumerate(self.pipette_head.vial_window):
                        # no pump detected above this vial (station overhang)
                        if vial is None:
                            continue

                        # pump detected above vial, find the cognate pump command if it exists
                        else:
                            target_pump = self.pipette_head.pump_config[index].name
                            self.status.active_syringe_pumps.append(target_pump)
                            if target_pump in getattr(pump_commands, f"vial_{vial}").keys():
                                pump_volumes[index] = getattr(pump_commands, f"vial_{vial}")[target_pump]

                    try:
                        await self.pipette_event(pump_volumes)
                        self.status.active_syringe_pumps = []
                    except PipetteEventError as e:
                        logger.error(e)
                        raise HelperEventError(f"Error running influx pipette event in influx_snake_helper(): {e}")

                    # finished dilutions for current vial_window, moving to next set of vials
                    change_row = False

                # change row
                change_row = True
            try:
                reset_location = station.xArmPlane_out.vial_to_xarm(coordinate)
                await self.move_xarm(reset_location)
            except xArmError as e:
                logger.error(e)
                raise HelperEventError(f"Error moving arm above station at the end of influx_snake_helper(): {e}")

        # update status
        self.status.vial_window = []
        self.status.active_station = -1

    async def run_pumps(self, method_name: str, pump_commands: list[int]):
        """
        Executes the same XCaliburD method across multiple syringe pumps sequentially

        Args:
            `method_name`: a valid XCaliburD method
            `pump_commands`: list of volumes whose indices correspond a cognate pump slot in the pipette_head

        """
        for index, command in enumerate(pump_commands):
            try:
                method = getattr(self.pipette_head.pumps[index], method_name)

                # wrapper function that adds the command to the pump's command chain and calls executeChain() & waitReady()
                def execute_pump_method(pump_method, method_args):
                    pump_method(*method_args)
                    delay = self.pipette_head.pumps[index].executeChain()
                    self.pipette_head.pumps[index].waitReady(delay)

                # run the blocking method in the default executor (thread pool)
                await asyncio.get_event_loop().run_in_executor(None, execute_pump_method, method, pump_commands)

            except (SyringeError, SyringeTimeout) as e:
                logger.error(f"Error with pump_position: {index} during {method_name}: {e}")
                raise e

    async def move_xarm(self, coordinate: xArmCoordinate):
        """Linear movement from current position to given xArm coordinate that is run immediately when called

        Args:
            `coordinate`: x,y,z coordinates pointing to target location
            `execute`: dictates whether the passed command is run immediately
        Raises:
            xArmError
        """

        self.update_conf()
        xarm_params = self.robotics_conf["xarm_params"]

        if self.status.xArm.arm_state == 4:
            raise xArmError("xArm in stop state, requires reset")
        else:
            result = self.arm.set_position(
                x=coordinate.x,
                y=coordinate.y,
                z=coordinate.z,
                roll=xarm_params["roll"],
                pitch=xarm_params["pitch"],
                yaw=xarm_params["yaw"],
                speed=xarm_params["speed"],
                mvacc=xarm_params["mvacc"],
                wait=True,
            )
            if result < 0:
                raise xArmError(f"xArm error detected during move_xarm(): {result}")

    async def execute_xArm_commands(self):
        """Sequentially execute all commands in the xArm_command_queue to create multi-step paths"""

        if not self.xArm_command_queue:
            logger.warning("tried running execute_xArm_commands but no xArm commands found in queue")
            raise xArmError("tried running execute_xArm_commands but no xArm commands found in queue")
        else:
            for index, command in enumerate(self.xArm_command_queue):
                try:
                    await self.move_xarm(command)
                    self.xArm_command_queue.pop(index)
                except xArmError as e:
                    logger.error(f"tried running command {command} but following error ecnountered: {e}")
                    raise xArmError(f"error trying to run execute_xArm_commands: {e}")

    # Server event handlers. Must be registered using self.register_event_handlers() before usage
    async def broadcast(self):
        """Broadcasts the current robotics status to all connected clients. Also check if robotics server is connected to OctoPrint servers."""
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
        await self.emit("broadcast", self.status)
