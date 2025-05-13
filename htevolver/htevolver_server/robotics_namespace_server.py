import asyncio
import logging
import time
from collections import deque
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
    """Container for results returned by server-side robotics routines.

    Stores the result of a robotics operation including success status,
    timing information, and current system state.

    Attributes:
        done (bool): Whether the operation completed successfully.
        namespace (str): The namespace that processed the operation.
        routine (str): Name of the routine that was executed.
        status (dict): Current status of the robotics system.
        elapsed_time (float): Time taken to execute the operation in seconds.
        message (str): Descriptive message about the operation result.
    """

    done: bool
    namespace: str
    routine: str
    status: dict
    elapsed_time: float
    message: str


def routine_decorator(routine_type: RoboticsRoutines):
    """Decorator for robotics routines that manages server status and routine results.

    Handles updating the robotics configuration and manages the server status state,
    timing, error handling, and constructs the standardized return data package
    sent to the client.

    Args:
        routine_type (RoboticsRoutines): The type of routine being executed.

    Returns:
        callable: A decorator function that wraps robotics routines.

    Examples:
        ```
        @routine_decorator(RoboticsRoutines.PIPETTE)
        async def on_pipette_routine(self, sid, pipette_commands):
            # Function implementation
            return (True, "executed successfully")
        ```
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
    """Container for pump commands for all vials in a Smart Station.

    Stores fluid dispensing commands for each of the 18 vials in a Smart Station.
    Each vial can have multiple fluid types and volumes specified.

    Attributes:
        vial_0 to vial_17 (dict[str, int]): Dictionary mapping fluid types to volumes
            for each vial position. Keys are fluid type names and values are volumes in μL.
    """

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
    """Server namespace for handling robotics hardware control.

    Manages the robotics hardware components including the xArm robot and PipetteHead.
    Handles client requests for robotics operations, configuration, and status updates.
    Coordinates complex robotics routines such as pipetting, dilutions, and vial filling.

    Attributes:
        robotics_config (dict): Configuration for robotics components.
        robotics_config_path (str): Path to the robotics configuration file.
        arm_command_queue (deque[xArmCoordinate]): Queue of arm movement commands.
        state (RoboticsState): Current state of the robotics system.
        routine (RoboticsRoutines): Currently executing routine if any.
        active_stations (int): Currently active station ID or -1 if none.
        active_vials (list[int | None]): Currently active vial IDs.
        pipette_head (PipetteHead): The PipetteHead instance for fluid handling.
        stations (dict[int, SmartStationRobotics]): Dictionary mapping station IDs to SmartStationRobotics instances.
        arm (xArm): The xArm robot instance.
    """

    def __init__(
        self,
        robotics_config: dict,
        robotics_config_path: str,
        namespace: str = "/robotics",
    ):
        super().__init__(namespace)
        self.robotics_config: dict = robotics_config
        self.robotics_config_path: str = robotics_config_path
        self.arm_command_queue: deque[xArmCoordinate] = deque()

        self.state = RoboticsState.READY
        self.routine = RoboticsRoutines.NO_ROUTINE
        self.active_stations = -1
        self.active_vials: list[int | None] = []

        # instantiate robotics modules
        self.pipette_head: PipetteHead = PipetteHead.create(self.robotics_config["pipette_head"])
        logger.info(f"PipetteHead successfully created: {self.pipette_head.to_dict()}")
        self.stations: dict[int, SmartStationRobotics] = {}
        for station_id, station_config in self.robotics_config["smart_stations"].items():
            if station_config["connect"]:
                self.stations[station_id] = SmartStationRobotics.create(station_config)
        logger.info(f"SmartStations successfully created: {[station for station in self.stations.values()]}")
        self.arm = xArm.create(self.robotics_config["xArm"])
        self.arm.register_callback(self.error_warn_change_callback, self.state_changed_callback, self.connect_changed_callback)
        self.arm.setup()
        logger.info(f"xArm successfully crated: {self.arm}")

        logger.info("Robotics namespace initialized")

    async def on_connect(self, sid) -> None:
        """Handle client connection to the robotics namespace.

        Called when a client connects to the robotics namespace.

        Args:
            sid (str): Session ID of the connecting client.
        """
        logger.info("Client connected to robotics namespace=")

    async def on_disconnect(self, sid) -> None:
        """Handle client disconnection from the robotics namespace.

        Called when a client disconnects from the robotics namespace.

        Args:
            sid (str): Session ID of the disconnecting client.
        """
        logger.info("Client disconnected to robotics namespace")

    async def on_request_status(self, sid) -> None:
        """Send the current robotics system status to the client.

        Responds to a client request for the current status of the robotics system.

        Args:
            sid (str): Session ID of the requesting client.
        """
        await self.emit("get_status", self.to_dict(), to=sid)
        logger.info("Finished processing REQUEST_STATUS on robotics namespace")

    async def on_request_config(self, sid) -> None:
        """Send the current robotics configuration to the client.

        Responds to a client request for the current robotics configuration.

        Args:
            sid (str): Session ID of the requesting client.
        """
        await self.emit("get_conf", self.robotics_config, to=sid)
        logger.info("Finished processing REQUEST_CONFIG on robotics namespace")

    async def on_override_status(self, sid, override_data: dict) -> None:
        """Override system status values for manual intervention.

        Allows manual overriding of system status attributes for recovery
        from error states.

        Args:
            sid (str): Session ID of the client.
            override_data (dict): Dictionary mapping attribute names to their new values.
                Example: {"state": RoboticsState.READY.value}
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
        """Pause robotics operations.

        Sets the robotics system to PAUSE state if currently BUSY.
        Pauses the PipetteHead and xArm operations.

        Args:
            sid (str): Session ID of the client.
        """
        self.pause_robotics()
        logger.info("Finished processing PAUSE on robotics namespace")

    async def on_resume(self, sid) -> None:
        """Resume paused robotics operations.

        Resumes the robotics system from PAUSE state back to BUSY.
        Resumes the PipetteHead and xArm operations.

        Args:
            sid (str): Session ID of the client.
        """
        self.resume_robotics()
        logger.info("Finished processing RESUME on robotics namespace")

    async def on_stop(self, sid) -> None:
        """Stop all robotics operations immediately.

        Sets the system to STOP state and terminates all active processes.
        Stops the PipetteHead and xArm operations.

        Args:
            sid (str): Session ID of the client.
        """
        self.stop_robotics()
        logger.info("Finished processing STOP on robotics namespace")

    async def on_connect_xArm(self, sid) -> None:
        """Connect to the xArm hardware.

        Attempts to establish a connection with the xArm robot.

        Args:
            sid (str): Session ID of the client.
        """
        self.arm.connect()
        logger.info("Finished processing CONNECT_xARM on robotics namespace")

    async def on_reset_xArm(self, sid) -> None:
        """Reset the xArm to clear errors.

        Reconnects if disconnected and resets the arm to clear error states.

        Args:
            sid (str): Session ID of the client.
        """

        self.arm.reset()
        logger.info("Finished processing RESET_xARM on robotics namespace")

    async def on_enable_pumps(self, sid, pump_list: list[int] = [0, 1, 2, 3]) -> None:
        """Enable PipetteHead syringe pumps.

        Enables specified syringe pumps to execute commands.

        Args:
            sid (str): Session ID of the client.
            pump_list (list[int], optional): List of pump IDs to connect.
                Defaults to [0, 1, 2, 3].
        """

        self.pipette_head.enable(pump_list)
        logger.info(f"Finished processing ENABLE_PUMPS on robotics namespace: {pump_list}")

    async def on_disable_pumps(self, sid, pump_list: list[int] = []) -> None:
        """Disables PipetteHead syringe pumps.

        Disable specified syringe pumps.

        Args:
            sid (str): Session ID of the client.
            pump_list (list[int], optional): List of pump IDs to disable.
                Defaults to an empty list.
        """
        for pump_id in pump_list:
            self.pipette_head.disable(pump_list)

    @routine_decorator(RoboticsRoutines.PRIMING_INFLUX)
    async def on_prime_pumps(self, sid, pump_list: list[int] = []) -> tuple[bool, str]:
        """Prime the specified syringe pumps.

        Prepares the syringe pumps for use by filling the input tubing with fluid.

        Args:
            sid (str): Session ID of the client.
            pump_list (list[int], optional): List of pump IDs to prime.
                Defaults to an empty list.

        Returns:
            tuple[bool, str]: Success status and descriptive message.

        Examples:
            Called by the client via:
            ```
            client.prime_syringe_pumps([0, 1])
            ```
        """
        await self.pipette_head.prime(pump_list)
        return (True, "executed successfully")

    @routine_decorator(RoboticsRoutines.PUMP_INITIALIZE)
    async def on_initialize_pumps(self, sid, pump_list: list[int] = []) -> tuple[bool, str]:
        """Initialize the syringe pumps.

        Initializes the XCaliburD/Tecan syringe pumps for operation.

        Args:
            sid (str): Session ID of the client.
            pump_list (list[int], optional): List of pump IDs to initialize.
                Defaults to an empty list.

        Returns:
            tuple[bool, str]: Success status and descriptive message.

        Examples:
            Called by the client via:
            ```
            client.initialize_pumps([0, 1])
            ```
        """

        await self.pipette_head.prime(pump_list)
        return (True, "executed successfully")

    @routine_decorator(RoboticsRoutines.PIPETTE)
    async def on_pipette_routine(self, sid, pipette_commands: list[int]) -> tuple[bool, str]:
        """Execute a pipetting operation.

        Runs the PipetteHead to transfer fluids with the specified volumes.

        Args:
            sid (str): Session ID of the client.
            pipette_commands (list[int]): List of volumes for each pump.
                Example: [100, 0, 50, 0] for 100μL from pump 0 and 50μL from pump 2.

        Returns:
            tuple[bool, str]: Success status and descriptive message.

        Examples:
            Called by the client via:
            ```
            client.pipette({0: ("MEDIA", 100), 2: ("DRUG", 50)})
            ```
        """

        await self.pipette_event(pipette_commands)
        return (True, "executed successfully")

    @routine_decorator(RoboticsRoutines.FILLING_VIALS_PUMPS)
    async def on_fill_vials_routine(self, sid, fill_commands: dict[int, tuple[str, int]]) -> tuple[bool, str]:
        """Fill vials with specified fluids.

        Fills vials in specified SmartStations with the given fluid types and volumes.

        Args:
            sid (str): Session ID of the client.
            fill_commands (dict[int, tuple[str, int]]): Dictionary mapping station IDs
                to tuples of (fluid_type, volume_μL).
                Example: {0: ("MEDIA", 1000), 1: ("DRUG", 500)}

        Returns:
            tuple[bool, str]: Success status and descriptive message.

        Examples:
            Called by the client via:
            ```
            client.fill_vials({0: ("MEDIA", 1000), 1: ("DRUG", 500)})
            ```
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
        """Execute dilutions across SmartStations.

        Performs dilution operations by adding specified fluids and volumes
        to specific vials across different stations.

        Args:
            sid (str): Session ID of the client.
            dilution_commands (dict[int, dict[int, dict[str, int]]]): Nested dictionary mapping:
                - station_id -> vial_id -> fluid_type -> volume
                Example: {0: {3: {"MEDIA": 100, "DRUG": 50}}}

        Returns:
            tuple[bool, str]: Success status and descriptive message.

        Examples:
            Called by the client via:
            ```
            client.dilutions({
                0: {
                    3: {"MEDIA": 100, "DRUG": 50},
                    4: {"MEDIA": 150}
                }
            })
            ```
        """

        influx_commands: dict[int, StationPumpCommands] = {}
        for station_id, dilution_command in dilution_commands.items():
            influx_commands[station_id] = StationPumpCommands()
            for vial_id, pump_commands in dilution_command.items():
                for fluid_type, volume in pump_commands.items():
                    if volume < 0:
                        return (
                            False,
                            f"Negative volume detected for vial_{vial_id} in Smart Station_{station_id}: {(fluid_type, volume)}",
                        )
                influx_commands[station_id].__setattr__(f"vial_{vial_id}", pump_commands)
        await self.influx_snake_helper(influx_commands)
        return (True, "executed successfully")

    async def influx_snake_helper(self, station_pump_commands: dict[int, StationPumpCommands]) -> None:
        """Execute sequential pipetting in a snake-like pattern.

        Moves the pipette head in a snake pattern across vials, executing wash steps
        and fluid transfer operations according to the provided commands.

        Args:
            station_pump_commands (dict[int, StationPumpCommands]): Dictionary mapping
                station IDs to pump commands for each vial.

        Raises:
            RoboticsError: If an error occurs during pipetting operations.
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
                        self.arm_command_queue.append(station.xArmPlane_out.vial_to_xarm(coordinate))
                        self.arm_command_queue.append(station.xArmPlane_out.vial_to_xarm(station.wash_location))
                        self.arm_command_queue.append(station.xArmPlane_in.vial_to_xarm(station.wash_location))

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
                        self.arm_command_queue.append(station.xArmPlane_out.vial_to_xarm(station.wash_location))
                        self.arm_command_queue.append(station.xArmPlane_out.vial_to_xarm(coordinate))
                        self.arm_command_queue.append(station.xArmPlane_in.vial_to_xarm(coordinate))

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
                                if target_pump.primary_fluid == FluidTypes[fluid_type]:
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
        """Coordinate xArm and PipetteHead for pipetting.

        Executes a complete pipetting cycle (aspirate and dispense), optionally
        coordinated with xArm movements if commands are queued.

        Args:
            pipette_volumes (list[int]): List of volumes for each pump position.
                Example: [100, 0, 50, 0] for 100μL from pump 0 and 50μL from pump 2.
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
        """Execute commands in the xArm command queue.

        Sequentially executes all commands in the xArm command queue,
        removing commands as they complete.

        Raises:
            xArmError: If a movement fails or if the queue is empty.
        """

        if not self.arm_command_queue:
            logger.warning("Tried running executing xArm commands but queue is empty")
            raise xArmError("Tried running executing xArm commands but queue is empty")

        while self.arm_command_queue:
            command = self.arm_command_queue[0]
            try:
                await self.check_for_interrupt()
                await self.arm.move(command)
                self.arm_command_queue.pop()
            except xArmError as e:
                logger.error(f"Error trying to run execute_xArm_commands: {e}")
                raise xArmError(f"Error trying to run execute_xArm_commands: {e}")

    async def broadcast(self):
        """Broadcast the current robotics status to all connected clients.

        Emits the current status information to all clients, including
        state, active routine, xArm and PipetteHead status.
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
        """Update error and warning codes based on xArm feedback.

        Called by the xArm API when error or warning states change.

        Args:
            xarm_api_data (dict): Dictionary containing error and warning codes.
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
        """Update xArm state based on controller feedback.

        Called by the xArm API when the arm's state changes.

        Args:
            xarm_api_data (dict): Dictionary containing the xArm state.
                Example: {"state": 0}
        """

        self.arm.state = xarm_api_data["state"]
        logger.debug(f"xArm state change callback input: {xarm_api_data}")
        if xarm_api_data["state"] == 4:
            self.emergency_stop_robotics()
            logger.error(f"xArm entered stop state: {xarm_api_data['state']}")

    def connect_changed_callback(self, xarm_api_data: dict):
        """Update xArm connection status based on controller feedback.

        Called by the xArm API when the connection status changes.

        Args:
            xarm_api_data (dict): Dictionary containing the connection status.
                Example: {"connected": True}
        """

        self.arm.connected = xarm_api_data["connected"]
        logger.debug(f"xArm connect change callback input: {xarm_api_data}")

    def load_config(self):
        """Load the robotics configuration from disk.

        Reads the latest settings from the robotics_config file to ensure
        current operations use up-to-date configuration values.
        """

        with open(self.robotics_config_path, "r") as conf:
            self.robotics_config = yaml.safe_load(conf)

    def update_robotics(self):
        """Update robotics components with the latest configuration.

        Updates SmartStations, PipetteHead, and xArm with the current configuration
        values to ensure that operations use up-to-date settings.
        """
        for station_id, station in self.stations.items():
            station.update(self.robotics_config["smart_stations"][station_id])
        self.pipette_head.update(self.robotics_config["pipette_head"])
        self.arm.update(self.robotics_config["xArm"])

    def stop_robotics(self):
        """Stop all robotics and pump operations.

        Terminates all syringe pump commands and puts xArm into stop state
        due to user intervention.
        """

        self.state = RoboticsState.STOP
        self.pipette_head.stop()
        self.arm.stop()
        logger.info("Robotics namespace put into STOP state, active processes have been exited.")

    def pause_robotics(self):
        """Pause robotics operations.

        Sets the robotics state to PAUSE if currently BUSY, pauses xArm operations
        by setting its state to 3 (pause), and terminates pending syringe pump commands.
        """
        if self.state == RoboticsState.BUSY:
            self.state = RoboticsState.PAUSE
            self.pipette_head.pause()
            self.arm.pause()
        logger.info("Robotics namespace put into PAUSE state")

    def resume_robotics(self):
        """Resume paused robotics operations.

        Sets the robotics state back to BUSY if previously PAUSE,
        resumes xArm operations by setting its state to 0 (running),
        and resumes pending syringe pump commands.
        """
        if self.state == RoboticsState.PAUSE:
            self.state = RoboticsState.BUSY
            self.pipette_head.resume()
            self.arm.resume()
        logger.info("Robotics namespace put back into BUSY state, resuming previously paused activity.")

    def emergency_stop_robotics(self):
        """Emergency stop all robotics operations.

        Terminates all syringe pump commands, triggers emergency stop on the xArm,
        and disconnects from the hardware. Requires manual intervention to restart.
        """

        try:
            self.stop_robotics()
            for pump in self.pipette_head.pumps:
                pump.disable(delete=True)
            self.arm.disconnect()
        except (SyringeError, SyringeTimeout) as e:
            logger.error(f"error encountered trying to call stop_robotics(): {e}")
        logger.info("Robotics namespace put into EMERGENCY_STOP state")

    async def check_for_interrupt(self):
        """Check for pause or stop signals during routine execution.

        Pauses execution if a PAUSE state is detected, resuming when state
        changes or raising an exception if a STOP is received.

        Args:
            timeout (int, optional): Maximum time in seconds to wait in PAUSE state.
                Defaults to 60. Set to 0 to disable timeout.

        Raises:
            ExitRobotics: If a STOP state is detected.
        """

        while self.state == RoboticsState.PAUSE:
            if self.state == RoboticsState.STOP:
                raise ExitRobotics
            else:
                await asyncio.sleep(0.1)

    def to_dict(self):
        """Convert the robotics namespace state to a dictionary.

        Returns:
            dict: Dictionary representation of the robotics namespace state,
                including status, routine, active stations, and component details.
        """
        status = {
            "state": (self.state.name, self.state.value),
            "routine": (self.routine.name, self.routine.value),
            "active_stations": self.active_stations,
            "xarm": self.arm.to_dict(),
            "pipette_head": self.pipette_head.to_dict(),
        }
        return status
