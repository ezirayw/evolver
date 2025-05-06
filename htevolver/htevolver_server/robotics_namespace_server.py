import asyncio
import logging
import time
from dataclasses import dataclass, field

import socketio
import yaml
from tecancavro.syringe import SyringeError, SyringeTimeout

from htevolver.exceptions import ExitRobotics, PipetteHeadError, RoboticsError, xArmError
from htevolver.robotics.pipette_head import FluidTypes, PipetteHead
from htevolver.robotics.smart_station import SmartStationRobotics, VialCoordinate
from htevolver.robotics.xarm import xArm, xArmCoordinate
from htevolver.shared import (
    RoboticsRoutines,
    RoboticsState,
)

logger = logging.getLogger(__name__)


@dataclass
class ServerResult:
    done: bool
    namespace: str
    routine: str
    status: dict
    elapsed_time: float
    message: str


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
            if self.state == RoboticsState.READY:
                self.state = RoboticsState.BUSY
                self.load_config()
                self.update_robotics()
                start_time = time.time()

                try:
                    await self.check_for_interrupt()
                    self.routine = routine_type

                    logger.info(f"Running {func.__name__} routine")
                    result, message = await func(self, *args, **kwargs)

                    logger.info(f"Done running the {func.__name__} routine")
                    end_time = time.time()
                    self.routine = RoboticsRoutines.NO_ROUTINE
                    self.state = RoboticsState.READY
                    return ServerResult(
                        done=result,
                        namespace="/robotics",
                        routine=routine_type.name,
                        status=self.to_dict(),
                        elapsed_time=end_time - start_time,
                        message=f"{func.__name__}: {message}",
                    )
                except ExitRobotics:
                    end_time = time.time()
                    self.routine = RoboticsRoutines.NO_ROUTINE
                    self.state = RoboticsState.READY
                    logger.info(f"STOP detected while running {func.__name__}, exiting")
                    return ServerResult(
                        done=False,
                        namespace="/robotics",
                        routine=routine_type.name,
                        status=self.to_dict(),
                        elapsed_time=end_time - start_time,
                        message=f"STOP detected while running {func.__name__}, exiting",
                    )
                except RoboticsError as e:
                    end_time = time.time()
                    self.state = RoboticsState.EMERGENCY_STOP
                    logger.error(f"Error encountered trying to run {func.__name__}: {e}")
                    return ServerResult(
                        done=False,
                        namespace="/robotics",
                        routine=routine_type.name,
                        status=self.to_dict(),
                        elapsed_time=end_time - start_time,
                        message=f"Error encountered trying to run {func.__name__}: {e}",
                    )
            else:
                logger.warning(f"Tried running the {func.__name__} routine but the robotics namespace is not in a READY state")
                return ServerResult(
                    done=False,
                    namespace="/robotics",
                    routine=routine_type.name,
                    status=self.to_dict(),
                    elapsed_time=0.00,
                    message=f"Tried running the {func.__name__} routine but the robotics namespace is not in a READY state",
                )

        return wrapper

    return decorator


@dataclass
class StationPumpCommands:
    vial_0: dict[str, int] = field(default_factory=dict)
    vial_1: dict[str, int] = field(default_factory=dict)
    vial_2: dict[str, int] = field(default_factory=dict)
    vial_3: dict[str, int] = field(default_factory=dict)
    vial_4: dict[str, int] = field(default_factory=dict)
    vial_5: dict[str, int] = field(default_factory=dict)
    vial_6: dict[str, int] = field(default_factory=dict)
    vial_7: dict[str, int] = field(default_factory=dict)
    vial_8: dict[str, int] = field(default_factory=dict)
    vial_9: dict[str, int] = field(default_factory=dict)
    vial_10: dict[str, int] = field(default_factory=dict)
    vial_11: dict[str, int] = field(default_factory=dict)
    vial_12: dict[str, int] = field(default_factory=dict)
    vial_13: dict[str, int] = field(default_factory=dict)
    vial_14: dict[str, int] = field(default_factory=dict)
    vial_15: dict[str, int] = field(default_factory=dict)
    vial_16: dict[str, int] = field(default_factory=dict)
    vial_17: dict[str, int] = field(default_factory=dict)


class RoboticsServerNamespace(socketio.AsyncNamespace):
    def __init__(
        self,
        robotics_config: dict,
        robotics_config_path: str,
        namespace: str = "/robotics",
    ):
        super().__init__(namespace)
        self.robotics_config: dict = robotics_config
        self.robotics_config_path: str = robotics_config_path
        self.arm_command_queue: list[xArmCoordinate] = []

        self.state = RoboticsState.READY
        self.routine = RoboticsRoutines.NO_ROUTINE
        self.active_stations = -1
        self.active_vials: list[int | None] = []

        # instantiate robotics modules
        self.pipette_head: PipetteHead = PipetteHead.create(self.robotics_config["pipette_head"])
        self.stations: dict[int, SmartStationRobotics] = {}
        for station_id, station_config in self.robotics_config["smart_stations"].items():
            if station_config["connect"]:
                self.stations[station_id] = SmartStationRobotics.create(station_config)
        self.arm = xArm.create(self.robotics_config["xArm"])

        self.arm.register_callback(self.error_warn_change_callback, self.state_changed_callback, self.connect_changed_callback)
        self.arm.setup()

        logger.info("Robotics namespace initialized")

    async def on_connect(self, sid) -> None:
        """Handles client connection to the robotics namespace.

        Args:
            sid (str): Session ID of the connecting client.
        """
        logger.info("Client connected to robotics namespace=")

    async def on_disconnect(self, sid) -> None:
        """Handles client disconnection from the robotics namespace.

        Args:
            sid (str): Session ID of the disconnecting client.
        """
        logger.info("Client disconnected to robotics namespace")

    async def on_request_status(self, sid) -> None:
        """Sends the current state of the robotics namespace to the client.

        Args:
            sid (str): Session ID of the requesting client.
        """
        await self.emit("get_status", self.to_dict(), to=sid)
        logger.info("Finished processing REQUEST_STATUS on robotics namespace")

    async def on_request_config(self, sid) -> None:
        """Sends the current robotics configuration to the client

        Args:
            sid (str): Session ID of the requesting client.
        """
        await self.emit("get_conf", self.robotics_config, to=sid)
        logger.info("Finished processing REQUEST_CONFIG on robotics namespace")

    async def on_request_types(self, sid) -> None:
        """Send the FluidTypes, RoboticsState, RoboticsRoutines definitions to the client.

        Args:
            sid (str): Session ID of the requesting client.
        """
        states = tuple((member.name, member.value) for member in RoboticsState)
        routines = tuple((member.name, member.value) for member in RoboticsRoutines)
        fluids = tuple((member.name, member.value) for member in FluidTypes)
        await self.emit("get_types", {"states": states, "routines": routines, "fluids": fluids}, to=sid)
        logger.info("Finished processing REQUEST_TYPES on robotics namespace")

    async def on_override_status(self, sid, override_data: dict) -> None:
        """Overrides the robotics status for manual intervention.

        Allows manual overriding of status.state and status.primed_syringe_pumps
        for recovery from problem situations.

        Args:
            sid (str): Session ID of the client.
            data (dict): Status values to override.
                Example: {"state": 0, "primed_syringe_pumps": True}
        """

        for override_key, value in override_data.items():
            if hasattr(self, override_key):
                # Check the type of the existing attribute and ensure new value matches
                attribute_value = getattr(self, override_key)
                attr_type = type(attribute_value)
                try:
                    # Try to cast the new value to the correct type
                    typed_value = attr_type(value)
                    setattr(self, override_key, typed_value)
                except (ValueError, TypeError):
                    logger.warning(f"Invalid type for {override_key}: expected {attr_type.__name__}, got {type(value).__name__}")
        logger.info(f"Finished processing OVERRIDE_STATUS on robotics namespace: {override_data}")

    async def on_pause(self, sid) -> None:
        """Pauses robotics operations if currently busy.

        Sets the robotics state to PAUSE if the system is currently BUSY.

        Args:
            sid (str): Session ID of the client.
        """
        self.pause_robotics()
        logger.info("Finished processing PAUSE on robotics namespace")

    async def on_resume(self, sid) -> None:
        """Resumes robotics operations if previously paused.

        Sets the robotics state back to BUSY if previously in PAUSE state.

        Args:
            sid (str): Session ID of the client.
        """
        self.resume_robotics()
        logger.info("Finished processing RESUME on robotics namespace")

    async def on_stop(self, sid) -> None:
        """Stops all robotics operations immediately.

        Sets the state to STOP and exits all active processes.

        Args:
            sid (str): Session ID of the client.
        """
        self.stop_robotics()
        logger.info("Finished processing STOP on robotics namespace")

    async def on_connect_xArm(self, sid) -> None:
        """Attempts to establish a connection with the xArm hardware.

        Args:
            sid (str): Session ID of the client.
        """
        self.arm.connect()
        logger.info("Finished processing CONNECT_xARM on robotics namespace")

    async def on_reset_xArm(self, sid) -> None:
        """Reconnects if disconnected and resets the arm to clear errors.

        Args:
            sid (str): Session ID of the client.
        """

        self.arm.reset()
        logger.info("Finished processing RESET_xARM on robotics namespace")

    async def on_connect_pumps(self, sid, pump_list: list[int] = [0, 1, 2, 3]) -> None:
        """Connects to PipetteHead syringe pumps.

        Args:
            sid (str): Session ID of the client.
            pump_list (list[int]): List of PipetteHead pump IDs to connect.
                Example: [0, 1, 3]
        """

        self.pipette_head.connect(pump_list)
        logger.info(f"Finished processing CONNECT_PUMPS on robotics namespace: {pump_list}")

    async def on_disconnect_pumps(self, sid, pump_list: list[int] = []) -> None:
        """Disconnects from PipetteHead syringe pumps.

        Args:
            sid (str): Session ID of the client.
            pump_list (list[int]): List of PipetteHead pump IDs to disconnect.
                Example: [0, 2, 3]
        """
        for pump_id in pump_list:
            self.pipette_head.disconnect(pump_list)

    @routine_decorator(RoboticsRoutines.PRIMING_INFLUX)
    async def on_prime_pumps(self, sid, pump_list: list[int] = []) -> tuple[bool, str]:
        """Primes desired syringe pumps on the PipetteHead.

        Prepares the syringe pump ports for eventual use by filling the input tubing
        connecting the port to the reservoir. Should be called repeatedly until fully primed

        Args:
            sid (str): Session ID of the client.
            pump_list (list[int]): List of PipetteHead pump IDs to prime.
                Example: [0, 2, 3]

        Returns:
            ServerResult: Result of the routine execution.
        """
        await self.pipette_head.prime(pump_list)
        return (True, "executed successfully")

    @routine_decorator(RoboticsRoutines.PUMP_INITIALIZE)
    async def on_initialize_pumps(self, sid, pump_list: list[int] = []) -> tuple[bool, str]:
        """Initializes the XCaliburD/Tecan syringe pumps.

        Updates the pipette head state and initializes all non-empty pumps.

        Args:
            sid (str): Session ID of the client.
            pump_list (list[int]): List of PipetteHead pump IDs to initialize.
                Example: [0, 2, 3]
        """

        await self.pipette_head.prime(pump_list)
        return (True, "executed successfully")

    @routine_decorator(RoboticsRoutines.PIPETTE)
    async def on_pipette_routine(self, sid, pipette_commands: list[int]) -> tuple[bool, str]:
        """Executes a multi-pump pipette command.

        Runs the PipetteHead to execute fluid transfer operations with the specified volumes.

        Args:
            sid (str): Session ID of the client.
            pipette_commands (dict[int, tuple[str, int]]): Maps pump indices to fluid type and volume.
                Example: {0: ("MEDIA", 100), 2: ("DRUG", 50)}

        Returns:
            ServerResult: Result of the routine execution.
        """

        await self.pipette_event(pipette_commands)
        return (True, "executed successfully")

    @routine_decorator(RoboticsRoutines.FILLING_VIALS_PUMPS)
    async def on_fill_vials_routine(self, sid, fill_commands: dict[int, tuple[str, int]]) -> tuple[bool, str]:
        """Fills vials in a SmartStation with a specified fluid and volume.

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
                return (False, f"Invalid fluid type entered: {fill_command[0]}")
            if fill_command[1] > 7000:
                logger.warning(f"Invalid volume entered: {fill_command[1]}")
                return (False, f"Invalid volume entered: {fill_command[1]}")

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
        return (True, "executed successfully")

    @routine_decorator(RoboticsRoutines.DILUTION)
    async def on_dilution_routine(self, sid, dilution_commands: dict[int, dict[int, dict[str, int]]]) -> tuple[bool, str]:
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
        return (True, "executed successfully")

    async def influx_snake_helper(self, station_pump_commands: dict[int, StationPumpCommands]) -> None:
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
            self.active_station = station_id
            station = self.stations[station_id]
            vial_map = station.vial_map
            change_row = False

            for row_num in range(3):
                overhangs = [None] * (self.pipette_head.num_windows - 6)
                virtual_vial_row: list[int | None] = vial_map[row_num] + overhangs

                for window_index in range(self.pipette_head.num_windows):
                    ################# WASH STEP #################
                    # check pipette_head fluid types to build the vial window
                    if self.pipette_head.universal:
                        self.active_vials = virtual_vial_row[
                            window_index * self.pipette_head.pump_num : window_index * self.pipette_head.pump_num
                            + self.pipette_head.pump_num
                        ]

                    if not self.pipette_head.universal:
                        self.active_vials.append(virtual_vial_row[window_index])
                        if window_index >= self.pipette_head.pump_num:
                            self.active_vials.pop(0)

                    logger.info(f"current vial window below pump head is: {self.active_vials}")

                    for movement_step in range(3):
                        move_0 = station.xArmPlane_out.vial_to_xarm(coordinate)
                        move_1 = station.xArmPlane_out.vial_to_xarm(station.wash_location)
                        move_2 = station.xArmPlane_in.vial_to_xarm(station.wash_location)

                        self.arm_command_queue.extend([move_0, move_1, move_2])

                    try:
                        await self.pipette_event([0, 0, 0, 0])
                    except RoboticsError as e:
                        logger.error(f"Error running wash step of influx_snake_helper: {e}")
                        raise RoboticsError(f"Error running wash step of influx_snake_helper: {e}")

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

                        self.arm_command_queue.extend([move_0, move_1, move_2])

                    # get pump volume commands for current vial window
                    pipette_volumes: list[int] = [0] * 4
                    for index, vial in enumerate(self.active_vials):
                        # no pump detected above this vial (station overhang)
                        if vial is None:
                            continue

                        # pump detected above vial
                        # if the pump has the desired fluid_type, set the port and get the volume data
                        if vial:
                            target_pump = self.pipette_head.pumps[index]
                            for fluid_type, volume in getattr(pump_commands, f"vial_{vial}").items():
                                if target_pump.primary == FluidTypes[fluid_type]:
                                    pipette_volumes[index] = volume
                                    # Remove the command after use to avoid duplicates
                                    getattr(pump_commands, f"vial_{vial}").pop(fluid_type)
                                    break

                    try:
                        await self.pipette_event(pipette_volumes)
                    except RoboticsError as e:
                        logger.error(f"Error running dilution step of influx_snake_helper: {e}")
                        raise RoboticsError(f"Error running dilution step of influx_snake_helper: {e}")

                    # finished dilutions for current vial_window, moving to next set of vials
                    change_row = False

                # change row
                change_row = True
            try:
                reset_location = station.xArmPlane_out.vial_to_xarm(coordinate)
                await self.arm.move(reset_location)
            except xArmError as e:
                logger.error(e)
                raise RoboticsError(f"Error moving arm above station at the end of influx_snake_helper(): {e}")

        # update status
        self.active_vials = []
        self.active_station = -1

    async def pipette_event(self, pipette_volumes: list[int], move_arm: bool = False) -> None:
        """Coordinates xArm and PipetteHead for dynamic pipetting.

        Executes a complete pipetting cycle (aspirate and dispense), coordinated with xArm movements if commands are queued.
        xArm command queue is executed during aspirate stage

        Args:
            pump_commands (list[int]): List of volumes for each pump position.
                Example: [100, 0, 50, 0]
            move_arm (bool, optional): Whether to execute queued arm movements during aspiration.
                Defaults to False.

        Raises:
            RoboticsError: If an error occurs during aspiration or dispense.
        """

        ################# ASPIRATION STEP #################
        await self.check_for_interrupt()
        logger.info(f"Running aspiration during: {self.routine}")
        try:
            async with asyncio.TaskGroup() as aspiration_tasks:
                aspiration_tasks.create_task(self.pipette_head.aspirate(pipette_volumes))

                if self.arm_command_queue and move_arm:
                    aspiration_tasks.create_task(self.execute_xArm_commands())
        except* (PipetteHeadError, xArmError) as e:
            logger.error(f"Error trying to execute aspiration tasks during pipette_event(): {e}")
            raise RoboticsError(f"Error trying to execute aspiration tasks during pipette_event(): {e}")

        await self.check_for_interrupt()

        ################# DISPENSE STEP #################
        await self.check_for_interrupt()
        logger.info(f"Running dispense during: {self.routine}")
        try:
            await self.pipette_head.dispense(pipette_volumes)
        except (PipetteHeadError, xArmError) as e:
            logger.error(f"Error trying to execute dispense during pipette_event(): {e}")
            raise RoboticsError(f"Error trying to execute dispense tasks during pipette_event(): {e}")

    async def execute_xArm_commands(self) -> None:
        """Sequentially executes all commands in the xArm command queue.

        Creates multi-step paths by executing each command in the queue
        one after the other, removing commands as they complete.

        Raises:
            xArmError: If a movement fails or if the queue is empty.
        """

        if not self.arm_command_queue:
            logger.warning("Tried running executing xArm commands but queue is empty")
            raise xArmError("Tried running executing xArm commands but queue is empty")

        while self.arm_command_queue:
            for index, command in enumerate(self.arm_command_queue):
                try:
                    await self.check_for_interrupt()
                    await self.arm.move(command)
                    self.arm_command_queue.pop(index)
                except xArmError as e:
                    logger.error(f"Error trying to run execute_xArm_commands: {e}")
                    raise xArmError(f"Error trying to run execute_xArm_commands: {e}")

    async def broadcast(self):
        """Broadcasts the current robotics status to all connected clients.

        Emits the current status information to all clients, includes
        checking for potential error conditions like overflows.
        """
        # check for potential overflow based on sensitivity threshold
        # overflow_trigger_map = {'left': [], 'right': []}
        # overflow_trigger_map['left'] =  [x * self.robotics_config['overflow_voltage_step'] for x in self.robotics_config['overflow_trigger_map']['left']]
        # overflow_trigger_map['right'] =  [x * self.robotics_config['overflow_voltage_step'] for x in self.robotics_config['overflow_trigger_map']['right']]
        # for quad_index in range(4):
        #    if (self.htevolver_client.overflow_data['left'][quad_index] > self.robotics_config['overflow_voltage_threshold']) or (self.htevolver_client.overflow_data['right'][quad_index] > self.robotics_config['overflow_voltage_threshold']):
        #        self.status.overflow_status['quads'][quad_index] = True
        #        self.stop_robotics()

        # identify the vial(s) that overflowed
        #        for index in range(18):
        #            if self.htevolver_client.overflow_data['right'][quad_index] >= overflow_trigger_map['right'][index] - self.robotics_config['overflow_voltage_threshold'] or self.htevolver_client.overflow_data['right'][quad_index] <= overflow_trigger_map['right'][index] + self.robotics_config['overflow_voltage_threshold']:
        #                for vial_index in range(index, index + 5):
        #                    if self.htevolver_client.overflow_data['left'][quad_index] >= overflow_trigger_map['left'][index] - self.robotics_config['overflow_voltage_threshold'] or self.htevolver_client.overflow_data['left'][quad_index] <= overflow_trigger_map['left'][index] + self.robotics_config['overflow_voltage_threshold']:
        #                        self.status.overflow_status['vial'] = vial_index

        # emit robotics status to all connected clients
        await self.emit("broadcast", self.to_dict())
        logging.info(f"Robotics broadcast: {self.pipette_head}\n{self.arm}\n{self.stations}")

    def error_warn_change_callback(self, xarm_api_data: dict):
        """Updates error and warning codes based on xArm feedback.

        Args:
            data (dict): A dictionary containing error and warning codes.
                Example: {"error_code": 0, "warn_code": 0}
        """

        self.arm.error_code = xarm_api_data["error_code"]
        self.arm.warning_code = xarm_api_data["warn_code"]
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

        self.arm.state = xarm_api_data["state"]
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

        self.arm.connected = xarm_api_data["connected"]
        logger.debug(f"xArm connect change callback input: {xarm_api_data}")

    def load_config(self):
        """Loads in the robotics configuration from memory.

        Loads the latest settings from the robotics_config file and updates the PipetteHead and SmartStations
        to ensure current operations use up-to-date configuration values.
        """

        with open(self.robotics_config_path, "r") as conf:
            self.robotics_config = yaml.safe_load(conf)

    def update_robotics(self):
        """Updates robotics components with the latest configuration.

        Updates SmartStations, PipetteHead, and xArm with the current configuration
        values to ensure that operations use up-to-date settings.
        """
        for station_id, station in self.stations.items():
            station.update(self.robotics_config["smart_stations"][station_id])
        self.pipette_head.update(self.robotics_config["pipette_head"])
        self.arm.update(self.robotics_config["xArm"])

    def stop_robotics(self):
        """Stops all robotics and pump operations due to user intervention.

        Terminates all syringe pump commands and puts xArm into stop state.
        """

        self.state = RoboticsState.STOP
        self.pipette_head.stop()
        self.arm.stop()
        logger.info("Robotics namespace put into STOP state, active processes have been exited.")

    def pause_robotics(self):
        """Pauses robotics operations if currently busy.

        Sets the robotics state to PAUSE, pauses xArm operations by setting its
        state to 3 (pause), and terminates pending syringe pump commands.
        """
        if self.state == RoboticsState.BUSY:
            self.state = RoboticsState.PAUSE
            self.pipette_head.pause()
            self.arm.pause()
        logger.info("Robotics namespace put into PAUSE state")

    def resume_robotics(self):
        """Resumes robotics operations if previously paused.

        Sets the robotics state back to BUSY, resumes xArm operations by setting its
        state to 0 (running), and resumes pending syringe pump commands.
        """
        if self.state == RoboticsState.PAUSE:
            self.state = RoboticsState.BUSY
            self.pipette_head.resume()
            self.arm.resume()
        logger.info("Robotics namespace put back into BUSY state, resuming previously paused activity.")

    def emergency_stop_robotics(self):
        """Stops all robotics and pump operations in emergency situations.

        Terminates all syringe pump commands, triggers emergency stop on the xArm,
        and disconnects from the hardware. Requires manual intervention to restart.
        """

        try:
            self.stop_robotics()
            for pump in self.pipette_head.pumps:
                pump.disconnect(delete=True)
            self.arm.disconnect()
        except (SyringeError, SyringeTimeout) as e:
            logger.error(f"error encountered trying to call stop_robotics(): {e}")
        logger.info("Robotics namespace put into EMERGENCY_STOP state")

    async def check_for_interrupt(self):
        """Checks for pause or stop signals during routine execution.

        Pauses execution if a PAUSE state is detected, resuming when state
        changes or raising an exception if a STOP is received.
        """

        while self.state == RoboticsState.PAUSE:
            if self.state == RoboticsState.STOP:
                raise ExitRobotics
            else:
                await asyncio.sleep(0.1)

    def to_dict(self):
        status = {
            "status": (self.state.name, self.state.value),
            "routine": (self.routine.name, self.routine.value),
            "active_stations": self.active_stations,
            "xarm": self.arm.to_dict(),
            "pipette_head": self.pipette_head.to_dict(),
        }
        return status
