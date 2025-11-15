import asyncio
import logging
import time
from collections import deque
from dataclasses import asdict, dataclass

import socketio
import yaml
from tecancavro.syringe import SyringeError, SyringeTimeout

from htevolver.exceptions import (
    DispenseHeadError,
    RoboticsRoutineError,
    RoboticsServerError,
    SmartStationError,
    StationInfluxCommandError,
    StopRobotics,
    xArmError,
)
from htevolver.robotics.dispense_head import DispenseHeadXCaliburD
from htevolver.robotics.interfaces import DispenseHeadProtocol
from htevolver.robotics.smart_station import SmartStationRobotics, StationCoordinate
from htevolver.robotics.xarm import xArm, xArmCoordinate
from htevolver.shared import RoboticsRoutines, RoboticsState, ServerResultCodes, StationInfluxCommand

logger = logging.getLogger(__name__)


@dataclass
class ServerResult:
    """Container for return data following a robotics routine request.

    Stores the result of a robotics operation, including success status,
    timing information, and current system state.

    Attributes:
        namespace (str): The namespace that processed the operation.
        event (str): The event or routine that was executed.
        status (dict): Current status of the robotics system.
        elapsed_time (float): Time taken to execute the operation in seconds.
        message (str): Descriptive message about the operation result.
        code (int): Error code for the operation, useful for error handling and programmatic responses.
            See ServerResultCodes for details.
    """

    namespace: str
    event: str
    status: dict
    elapsed_time: float
    message: str
    code: int


def robotics_routine(routine_type: RoboticsRoutines):
    """Decorator for robotics routines that manages server status and routine results.

    This decorator handles updating the robotics configuration, manages the server status state,
    tracks timing, handles errors, and constructs the standardized return data package
    sent to the client.

    Args:
        routine_type (RoboticsRoutines): The type of routine being executed.

    Returns:
        callable: A decorator function that wraps robotics routines.

    Example:
        @robotics_routine(RoboticsRoutines.PIPETTE)
        async def on_dispense_routine(self, sid, dispense_commands):
            # Function implementation
            return (True, "executed successfully")
    """

    def decorator(func):
        async def wrapper(self: RoboticsServerNamespace, *args, **kwargs):
            if self.state == RoboticsState.READY:
                self.state = RoboticsState.BUSY
                self.load_config()
                self.update_robotics()
                start_time = time.time()

                try:
                    self.check_for_interrupt()
                    self.routine = routine_type

                    logger.info(f"Running routine: {routine_type.name}")
                    await func(self, *args, **kwargs)

                    logger.info(f"Done running routine: {routine_type.name}")
                    end_time = time.time()
                    self.routine = RoboticsRoutines.NO_ROUTINE
                    self.state = RoboticsState.READY
                    return asdict(
                        ServerResult(
                            namespace="/robotics",
                            event=func.__name__,
                            status=self.to_dict(),
                            elapsed_time=end_time - start_time,
                            message=ServerResultCodes.SUCCESS.name,
                            code=ServerResultCodes.SUCCESS.value,
                        )
                    )

                except StopRobotics:
                    end_time = time.time()
                    self.routine = RoboticsRoutines.NO_ROUTINE
                    self.state = RoboticsState.READY
                    logger.info(f"Stop state detected while running {routine_type.name}, exiting")
                    return asdict(
                        ServerResult(
                            namespace="/robotics",
                            event=func.__name__,
                            status=self.to_dict(),
                            elapsed_time=end_time - start_time,
                            message=ServerResultCodes.EXIT_ROUTINE.name,
                            code=ServerResultCodes.EXIT_ROUTINE.value,
                        )
                    )

                except DispenseHeadError:
                    end_time = time.time()
                    self.state = RoboticsState.EMERGENCY_STOP
                    logger.exception(f"DispenseHead error encountered trying to run {routine_type.name}", stack_info=True)
                    return asdict(
                        ServerResult(
                            namespace="/robotics",
                            event=func.__name__,
                            status=self.to_dict(),
                            elapsed_time=end_time - start_time,
                            message=ServerResultCodes.ROBOTICS_ERROR.name,
                            code=ServerResultCodes.ROBOTICS_ERROR.value,
                        )
                    )

                except xArmError:
                    end_time = time.time()
                    self.state = RoboticsState.EMERGENCY_STOP
                    logger.exception(f"xArm error encountered trying to run {routine_type.name}", stack_info=True)
                    return asdict(
                        ServerResult(
                            namespace="/robotics",
                            event=func.__name__,
                            status=self.to_dict(),
                            elapsed_time=end_time - start_time,
                            message=ServerResultCodes.ROBOTICS_ERROR.name,
                            code=ServerResultCodes.ROBOTICS_ERROR.value,
                        )
                    )

                except RoboticsRoutineError:
                    end_time = time.time()
                    self.state = RoboticsState.EMERGENCY_STOP
                    logger.exception(f"DispenseHead error encountered trying to run {routine_type.name}", stack_info=True)
                    return asdict(
                        ServerResult(
                            namespace="/robotics",
                            event=func.__name__,
                            status=self.to_dict(),
                            elapsed_time=end_time - start_time,
                            message=ServerResultCodes.ROBOTICS_ERROR.name,
                            code=ServerResultCodes.ROBOTICS_ERROR.value,
                        )
                    )
            else:
                logger.warning(f"Tried running the {routine_type.name} routine but not in ready state.")
                return asdict(
                    ServerResult(
                        namespace="/robotics",
                        event=func.__name__,
                        status=self.to_dict(),
                        elapsed_time=0.00,
                        message=ServerResultCodes.NOT_READY.name,
                        code=ServerResultCodes.NOT_READY.value,
                    )
                )

        return wrapper

    return decorator


class RoboticsServerNamespace(socketio.AsyncNamespace):
    """Server namespace for handling robotics hardware control.

    Manages robotics hardware components including the xArm robot and DispenseHeads.
    Handles client requests for robotics operations, configuration, and status updates.
    Coordinates complex robotics routines such as pipetting, influx, and vial filling.

    Attributes:
        robotics_config (dict): Configuration for robotics components.
        robotics_config_path (str): Path to the robotics configuration file.
        arm_command_queue (deque[xArmCoordinate]): Queue of arm movement commands.
        state (RoboticsState): Current state of the robotics system.
        routine (RoboticsRoutines): Currently executing routine if any.
        active_station_id (int): Currently active station ID or -1 if none.
        active_vials (list[int]): Currently active vial IDs.
        dispense_heads (dict[str, DispenseHeadProtocol]): Fluid type to DispenseHead mapping.
        stations (dict[int, SmartStationRobotics]): Station ID to SmartStationRobotics mapping.
        arm (xArm): The xArm robot instance.
    """

    dispense_head_factory: dict[str, type[DispenseHeadProtocol]] = {"xcaliburd": DispenseHeadXCaliburD}
    namespace: str = "/robotics"

    def __init__(
        self,
        robotics_config: dict,
        robotics_config_path: str,
    ):
        """Initialize the RoboticsServerNamespace and all hardware components.

        Args:
            robotics_config (dict): Configuration for robotics components.
            robotics_config_path (str): Path to the robotics configuration file.

        Raises:
            RoboticsServerError: If hardware initialization fails.
        """
        super().__init__(RoboticsServerNamespace.namespace)
        self.robotics_config: dict = robotics_config
        self.robotics_config_path: str = robotics_config_path
        self.arm_command_queue: deque[xArmCoordinate] = deque()

        self.state: RoboticsState = RoboticsState.READY
        self.routine: RoboticsRoutines = RoboticsRoutines.NO_ROUTINE
        self.active_station_id: int = -1
        self.active_vials: list[int] = []

        # instantiate robotics modules
        self.dispense_heads: dict[str, DispenseHeadProtocol] = {}
        for fluid_type, dispense_head_config in self.robotics_config["dispense_heads"].items():
            dispense_head_type: type[DispenseHeadProtocol] = dispense_head_config["type"]
            self.dispense_heads[fluid_type] = dispense_head_type.from_config(dispense_head_config)
        logger.info(
            f"DispenseHeads successfully created: {[dispense_head.to_dict() for dispense_head in self.dispense_heads.values()]}"
        )

        try:
            self.stations: dict[int, SmartStationRobotics] = {}
            for station_id, station_config in self.robotics_config["smart_stations"].items():
                self.stations[station_id] = SmartStationRobotics.from_config(station_config)
            logger.info(f"SmartStations successfully created: {[station for station in self.stations.values()]}")
        except SmartStationError:
            logger.exception("Error trying to create SmartStation instances, aborting server initialization...", stack_info=True)
            raise RoboticsServerError("Error trying to create SmartStation instances, aborting server initialization...")

        try:
            self.arm = xArm.from_config(self.robotics_config["xArm"])
            self.arm.register_callback(
                self.error_warn_change_callback, self.state_changed_callback, self.connect_changed_callback
            )
            self.arm.initialize()
            logger.info(f"xArm successfully created: {self.arm}")
        except xArmError:
            logger.exception("Error trying to create xArm instance, aborting server initialization...", stack_info=True)
            raise RoboticsServerError("Error trying to create xArm instance, aborting server initialization...")

        logger.info("Robotics namespace initialized")

    def get_active_head(self) -> str:
        """Return the fluid type of the currently active (in-use) DispenseHead.

        Returns:
            found_fluid_type (str): The fluid type name if a DispenseHead is active, otherwise an empty string.
        """
        found_fluid_type: str = ""
        for fluid_type, dispense_head in self.dispense_heads.items():
            if dispense_head.in_use:
                found_fluid_type = fluid_type
        return found_fluid_type

    def check_fluid_type(self, fluid_type_input: str) -> bool:
        """Check if the provided fluid type has a corresponding DispenseHead configured.

        Args:
            fluid_type_input (str): The fluid type to check.

        Returns:
            bool: True if the fluid type is configured, False otherwise.
        """
        fluid_type_found: bool = False
        if fluid_type_input in self.dispense_heads.keys():
            fluid_type_found = True
        return fluid_type_found

    async def on_connect(self, sid, environ) -> None:
        """Handle a client connection to the robotics namespace.

        Args:
            sid (str): Session ID of the connecting client.
            environ (dict): Connection environment.
        """
        logger.info("Client connected to robotics namespace=")

    async def on_disconnect(self, sid, reason) -> None:
        """Handle a client disconnection from the robotics namespace.

        Args:
            sid (str): Session ID of the disconnecting client.
            reason (str): Reason for disconnection.
        """
        logger.info("Client disconnected to robotics namespace")

    async def on_request_status(self, sid) -> dict:
        """Send the current robotics system status to the client.

        Args:
            sid (str): Session ID of the requesting client.

        Returns:
            dict: Dictionary representation of ServerResult object.
        """
        logger.info("Received request for the current robotics namespace status.")
        return asdict(
            ServerResult(
                namespace="/robotics",
                event="on_request_status",
                status=self.to_dict(),
                elapsed_time=0.00,
                message=ServerResultCodes.SUCCESS.name,
                code=ServerResultCodes.SUCCESS.value,
            )
        )

    async def on_request_config(self, sid) -> dict:
        """Send the current robotics configuration to the client.

        Args:
            sid (str): Session ID of the requesting client.

        Returns:
            dict: Dictionary representation of ServerResult object.
        """
        logger.info("Received robotics namespace configuration request.")
        return asdict(
            ServerResult(
                namespace="/robotics",
                event="on_request_config",
                status=self.robotics_config,
                elapsed_time=0.00,
                message=ServerResultCodes.SUCCESS.name,
                code=ServerResultCodes.SUCCESS.value,
            )
        )

    async def on_override(self, sid, override_data: dict) -> dict:
        """Override system state, routine, and/or configuration for manual intervention.

        Allows manual overriding of system state, routine, and/or configuration attributes to update
        configuration values or enable recovery from error states.

        Args:
            sid (str): Session ID of the client.
            override_data (dict): Dictionary mapping state, routine, and/or config attribute names to new values.

        Returns:
            dict: Dictionary representation of ServerResult object.
        """
        logger.info(f"Received robotics namespace override request: {override_data}")
        code: ServerResultCodes = ServerResultCodes.SUCCESS
        message: str = ServerResultCodes.SUCCESS.name

        if "state" in override_data:
            try:
                self.state = RoboticsState(override_data["state"])
            except ValueError:
                message = "Aborting on_override, invalid state input. Must be a valid member of RoboticsState Enum"
                code = ServerResultCodes.REQUEST_ERROR
                logger.exception(message, stack_info=True)

        if "routine" in override_data:
            try:
                self.routine = RoboticsRoutines(override_data["routine"])
            except ValueError:
                message = "Aborting on_override, invalid routine input. Must be a valid member of RoboticsRoutines Enum"
                code = ServerResultCodes.REQUEST_ERROR
                logger.exception(message, stack_info=True)

        if "config" in override_data:
            for key, value in override_data["config"].items():
                if key in self.robotics_config:
                    attribute_value = self.robotics_config[key]
                    attr_type = type(attribute_value)
                    if isinstance(value, attr_type):
                        self.robotics_config[key] = value
                    else:
                        message = f"Aborting on_override, type mismatch for {key}: expected {attr_type.__name__}, got {type(value).__name__}"
                        code = ServerResultCodes.REQUEST_ERROR
                        logger.exception(message, stack_info=True)

            self.update_robotics()

        return asdict(
            ServerResult(
                namespace="/robotics",
                event="on_override",
                status=self.to_dict(),
                elapsed_time=0.00,
                message=message,
                code=code.value,
            )
        )

    async def on_pause(self, sid) -> dict:
        """Pause robotics operations.

        Sets the robotics system to PAUSE state if currently BUSY.
        Pauses DispenseHead and xArm operations.

        Args:
            sid (str): Session ID of the client.

        Returns:
            dict: Dictionary representation of ServerResult object
        """
        logger.info("Received robotics namespace pause request.")
        code: ServerResultCodes = ServerResultCodes.SUCCESS
        message: str = ServerResultCodes.SUCCESS.name

        try:
            self.pause_robotics()
        except (DispenseHeadError, xArmError) as e:
            message = f"Aborting on_pause, {type(e).__name__} detected"
            code = ServerResultCodes.ROBOTICS_ERROR
            logger.exception(message, stack_info=True)

        return asdict(
            ServerResult(
                namespace="/robotics",
                event="on_pause",
                status=self.to_dict(),
                elapsed_time=0.00,
                message=message,
                code=code.value,
            )
        )

    async def on_resume(self, sid) -> dict:
        """Resume paused robotics operations.

        Resumes the robotics system from PAUSE state back to BUSY.
        Resumes DispenseHead and xArm operations.

        Args:
            sid (str): Session ID of the client.

        Returns:
            dict: Dictionary representation of ServerResult object
        """
        logger.info("Received robotics namespace resume request.")
        code: ServerResultCodes = ServerResultCodes.SUCCESS
        message: str = ServerResultCodes.SUCCESS.name

        try:
            self.resume_robotics()
        except (DispenseHeadError, xArmError) as e:
            message = f"Aborting on_resume, {type(e).__name__} detected"
            code = ServerResultCodes.ROBOTICS_ERROR
            logger.exception(message, stack_info=True)

        return asdict(
            ServerResult(
                namespace="/robotics",
                event="on_resume",
                status=self.to_dict(),
                elapsed_time=0.00,
                message=message,
                code=code.value,
            )
        )

    async def on_stop(self, sid) -> dict:
        """Stop all robotics operations immediately.

        Sets the system to STOP state and terminates all active processes.
        Stops DispenseHead and xArm operations.

        Args:
            sid (str): Session ID of the client.

        Returns:
            dict: Dictionary representation of ServerResult object
        """
        logger.info("Received robotics namespace stop request.")
        code: ServerResultCodes = ServerResultCodes.SUCCESS
        message: str = ServerResultCodes.SUCCESS.name

        try:
            self.stop_robotics()
        except (DispenseHeadError, xArmError) as e:
            message = f"Aborting on_stop, {type(e).__name__} detected"
            code = ServerResultCodes.ROBOTICS_ERROR
            logger.exception(message, stack_info=True)

        return asdict(
            ServerResult(
                namespace="/robotics",
                event="on_stop",
                status=self.to_dict(),
                elapsed_time=0.00,
                message=message,
                code=code.value,
            )
        )

    async def on_connect_xArm(self, sid) -> dict:
        """Connect to the xArm robot.

        Args:
            sid (str): Session ID of the client.

        Returns:
            dict: Dictionary representation of ServerResult object
        """
        logger.info("Received robotics namespace request to connect to the xArm.")
        code: ServerResultCodes = ServerResultCodes.SUCCESS
        message: str = ServerResultCodes.SUCCESS.name

        try:
            self.arm.connect()
            self.arm.initialize()
        except xArmError as e:
            message = f"Aborting on_connect_xArm, {type(e).__name__} detected"
            code = ServerResultCodes.ROBOTICS_ERROR
            logger.exception(message, stack_info=True)

        return asdict(
            ServerResult(
                namespace="/robotics",
                event="on_connect_xArm",
                status=self.to_dict(),
                elapsed_time=0.00,
                message=message,
                code=code.value,
            )
        )

    async def on_reset_xArm(self, sid) -> dict:
        """Reset the xArm to clear errors.

        Args:
            sid (str): Session ID of the client.

        Returns:
            dict: Dictionary representation of ServerResult object
        """
        logger.info("Received robotics namespace request to reset the xArm.")
        code: ServerResultCodes = ServerResultCodes.SUCCESS
        message: str = ServerResultCodes.SUCCESS.name

        try:
            self.arm.initialize()
        except xArmError as e:
            message = f"Aborting on_reset_xArm, {type(e).__name__} detected"
            logger.exception(message, stack_info=True)
            code = ServerResultCodes.ROBOTICS_ERROR

        return asdict(
            ServerResult(
                namespace="/robotics",
                event="on_reset_xArm",
                status=self.to_dict(),
                elapsed_time=0.00,
                message=message,
                code=code.value,
            )
        )

    async def on_enable_heads(self, sid, dispense_head_list: list[str] = []) -> dict:
        """Enable specific DispenseHeads using their fluid_type name.

        Args:
            sid (str): Session ID of the client.
            dispense_head_list (list[str], optional): List of fluid type DispenseHeads to enable. Defaults to an empty list.

        Returns:
            dict: Dictionary representation of ServerResult object.
        """
        logger.info("Received robotics namespace request to disable DispenseHeads.")
        code: ServerResultCodes = ServerResultCodes.SUCCESS
        message: str = ServerResultCodes.SUCCESS.name

        if not dispense_head_list:
            dispense_head_list = [fluid_type for fluid_type in self.dispense_heads.keys()]
        for fluid_type in dispense_head_list:
            if not self.check_fluid_type(fluid_type):
                message = f"Aborting on_enable_heads, invalid DispenseHead input: {fluid_type}"
                code = ServerResultCodes.REQUEST_ERROR
                logger.exception(message, stack_info=True)
                return asdict(
                    ServerResult(
                        namespace="/robotics",
                        event="on_enable_pumps",
                        status=self.to_dict(),
                        elapsed_time=0.00,
                        message=message,
                        code=code.value,
                    )
                )

            try:
                self.dispense_heads[fluid_type].enable_head()
            except DispenseHeadError as e:
                message = f"Aborting on_enable_heads, {type(e).__name__} detected for {fluid_type}"
                code = ServerResultCodes.ROBOTICS_ERROR
                logger.exception(message, stack_info=True)

        return asdict(
            ServerResult(
                namespace="/robotics",
                event="on_enable_heads",
                status=self.to_dict(),
                elapsed_time=0.00,
                message=message,
                code=code.value,
            )
        )

    async def on_disable_heads(self, sid, dispense_head_list: list[str] = []) -> dict:
        """Disable specific DispenseHeads using their fluid_type name.

        Args:
            sid (str): Session ID of the client.
            dispense_head_list (list[str], optional): List of fluid type DispenseHeads to disable. Defaults to an empty list.

        Returns:
            dict: Dictionary representation of ServerResult object.
        """
        logger.info("Received robotics namespace request to disable DispenseHeads.")
        code: ServerResultCodes = ServerResultCodes.SUCCESS
        message: str = ServerResultCodes.SUCCESS.name

        if not dispense_head_list:
            fluid_type = [fluid_type for fluid_type in self.dispense_heads.keys()]
        for fluid_type in dispense_head_list:
            if not self.check_fluid_type(fluid_type):
                message = f"Aborting on_disable_heads, invalid DispenseHead input: {fluid_type}"
                code = ServerResultCodes.REQUEST_ERROR
                logger.exception(message, stack_info=True)
                return asdict(
                    ServerResult(
                        namespace="/robotics",
                        event="on_enable_pumps",
                        status=self.to_dict(),
                        elapsed_time=0.00,
                        message=message,
                        code=code.value,
                    )
                )

            try:
                self.dispense_heads[fluid_type].disable_head()
            except DispenseHeadError as e:
                message = f"Aborting on_disable_heads, {type(e).__name__} detected for {fluid_type}"
                code = ServerResultCodes.ROBOTICS_ERROR
                logger.exception(message, stack_info=True)

        return asdict(
            ServerResult(
                namespace="/robotics",
                event="on_disable_heads",
                status=self.to_dict(),
                elapsed_time=0.00,
                message=message,
                code=code.value,
            )
        )

    @robotics_routine(RoboticsRoutines.TOOL_CHANGE)
    async def on_change_head(self, sid, target_fluid_type: str):
        """Change the currently active DispenseHead. Must be in standby reference position first.

        Args:
            sid (str): Session ID of the client.
            target_fluid_type (str): Target DispenseHead to switch to.

        Returns:
            dict: Dictionary representation of ServerResult object.
        """
        logger.info(f"Received robotics namespace request to change DispenseHead to {target_fluid_type}.")

        if not self.check_fluid_type(target_fluid_type):
            raise RoboticsRoutineError(f"Invalid fluid type entered: {target_fluid_type}")

        if not self.arm.check_position("standby"):
            raise xArmError("xArm not at standby reference position")

        self.change_tool(target_fluid_type)

    @robotics_routine(RoboticsRoutines.HOME)
    async def on_home(self):
        """Bring xArm to its configured home reference position. Checks to see if the xArm is in its standby reference position prior to moving.

        Raises:
            xArmError: If xArm current position not within 5% of configured standby reference position
        """
        if self.arm.check_position("standby"):
            await self.to_home()
        else:
            raise xArmError("xArm not at standby reference position")

    @robotics_routine(RoboticsRoutines.STANDBY)
    async def on_standby(self):
        """Bring xArm to standby reference position. Checks to see if the xArm is in its home reference position prior to moving.

        Raises:
            xArmError: If xArm current position not within 5% of configured standby reference position
        """
        if self.arm.check_position("home"):
            await self.to_standby()
        else:
            raise xArmError("xArm not at home reference position")

    @robotics_routine(RoboticsRoutines.PRIMING_INFLUX)
    async def on_prime_dispenseheads(self, sid, dispense_head_list: list[str] = []):
        """Prime specified DispenseHeads using their fluid_type name. If left empty, all DispenseHeads are primed.

        Args:
            sid (str): Session ID of the client.
            dispense_head_list (list[str], optional): List of fluid types to prime. Defaults to an empty list.
        """
        if not dispense_head_list:
            dispense_head_list = list(self.dispense_heads.keys())
        for fluid_type in dispense_head_list:
            if not self.check_fluid_type(fluid_type):
                raise RoboticsRoutineError(f"Invalid DispenseHead input: {fluid_type}")

            self.dispense_heads[fluid_type].prime()

    @robotics_routine(RoboticsRoutines.INITIALIZE)
    async def on_initialize(self, sid, dispense_head_list: list[str] = []):
        """Initialize the specified DispenseHeads using their fluid_type name. If left empty, all DispenseHeads are initialized.

        Args:
            sid (str): Session ID of the client.
            dispense_head_list (list[str], optional): List of fluid types to initialize. Defaults to an empty list.
        """
        if not dispense_head_list:
            dispense_head_list = list(self.dispense_heads.keys())
        for fluid_type in dispense_head_list:
            if not self.check_fluid_type(fluid_type):
                raise RoboticsRoutineError(f"Invalid DispenseHead input: {fluid_type}")

            self.dispense_heads[fluid_type].initialize_head()

    @robotics_routine(RoboticsRoutines.PIPETTE)
    async def on_pipette_routine(self, sid, pipette_commands: list[int]):
        """Execute a basic pipette operation with the currently in use DispenseHead. DispenseHead will aspirate and dispense fluid according to input volume command

        The DispenseHead will aspirate and dispense fluid according to the input volume command.

        Args:
            sid (str): Session ID of the client.
            pipette_commands (list[int]): List of pipette volume commands. Volume index corresponds to syringe pump on the DispenseHead.
        """
        active_head: str = self.get_active_head()
        if active_head != "":
            await self.dispense_heads[active_head].aspirate(pipette_commands)
            self.dispense_heads[active_head].dispense(pipette_commands)
        else:
            raise RoboticsRoutineError("No in_use DispenseHead detected")

    @robotics_routine(RoboticsRoutines.INFLUX)
    async def on_influx_routine(self, sid, influx_commands: dict[int, list[dict]]):
        """Execute multi-fluid influx into specific vials across SmartStations. Converts commands into StationInfluxCommand objects.

        Args:
            sid (str): Session ID of the client.
            influx_commands (dict[int, list[dict]]): Dictionary maspping SmartStation IDs to lists of dictionaries containing station influx commands.
        """
        if not self.arm.check_position("standby"):
            raise RoboticsRoutineError("xArm not in standby position")

        processed_commands: dict[int, list[StationInfluxCommand]] = {}
        for station_id, station_commands in influx_commands.items():
            try:
                processed_commands[station_id] = [StationInfluxCommand(**station_command) for station_command in station_commands]
            except StationInfluxCommandError:
                raise RoboticsRoutineError("Error processing influx command input")

        for station_id, station_influx_commands in processed_commands.items():
            for station_influx_command in station_influx_commands:
                if not self.check_fluid_type(station_influx_command.fluid_type):
                    logger.error(f"Invalid fluid type entered: {station_influx_command.fluid_type}")
                    raise RoboticsRoutineError(f"Invalid fluid type entered: {station_influx_command.fluid_type}")

                vial_number: int = len(SmartStationRobotics.vial_map)
                volume_list: list[int] = [station_influx_command.get_vial_volume(i) for i in range(vial_number)]
                if not any(
                    self.dispense_heads[station_influx_command.fluid_type].validate_volume(volume) for volume in volume_list
                ):
                    logger.error(f"Invalid volume entered for DispenseHead_{station_influx_command.fluid_type}")
                    raise RoboticsRoutineError(f"Invalid volume entered for DispenseHead_{station_influx_command.fluid_type}")

                self.change_tool(station_influx_command.fluid_type)
                await self.to_standby()
                await self.station_influx(station_influx_command)
                await self.to_standby()

    async def station_influx(self, station_influx_command: StationInfluxCommand) -> None:
        """Execute an influx routine based on the provided StationInfluxCommand.

        Args:
            station_influx_command (StationInfluxCommand): Command containing influx data.

        Raises:
            DispenseHeadError: If a dispense head operation fails.
            xArmError: If an xArm operation fails.
        """
        fluid_type: str = station_influx_command.fluid_type
        if not self.dispense_heads[fluid_type].in_use:
            logger.error(f"{station_influx_command.fluid_type}_DispeneHead not set to in_use")
            raise RoboticsRoutineError(f"{station_influx_command.fluid_type}_DispeneHead not set to in_use")

        max_vial_number: int = len(SmartStationRobotics.vial_map)
        vial_list: list[int] = list(range(max_vial_number))
        vial_window_size: int = self.dispense_heads[fluid_type].pump_number
        influx_event_number: int = int(max_vial_number / vial_window_size)

        station: SmartStationRobotics = self.stations[station_influx_command.station_id]
        self.active_station_id = station_influx_command.station_id
        self.active_vials = vial_list[0:vial_window_size]

        target_influx_coordinate: StationCoordinate = StationCoordinate(x=0, y=36)
        end_of_row_vials: list[int] = [vial_row[-1] for vial_row in SmartStationRobotics.vial_map]
        for influx_event_step in range(influx_event_number):
            # queue xArm movements to move DispenseHead to space above active_vials and into active_vials
            self.arm_command_queue.append(station.xArmPlane_out.vial_to_xarm(target_influx_coordinate))
            self.arm_command_queue.append(station.xArmPlane_in.vial_to_xarm(target_influx_coordinate))
            # extract influx volumes for active_vial set from influx command
            influx_command: list[int] = [station_influx_command.get_vial_volume(vial) for vial in self.active_vials]

            # run influx_event
            try:
                await self.influx_event(fluid_type, influx_command)
            except (DispenseHeadError, xArmError):
                logger.exception(f"Error running step_number {influx_event_step} during station_influx()")
                raise

            # bring DispenseHead out of active vial set
            self.arm_command_queue.append(station.xArmPlane_out.vial_to_xarm(target_influx_coordinate))
            await self.execute_xArm_commands()

            # update target coordinate for next set of vials
            if any(vial in end_of_row_vials for vial in self.active_vials):
                # Move to next row of vials if any active vial is at the end of a row
                target_influx_coordinate.x = 0
                target_influx_coordinate.y = target_influx_coordinate.y - 18
            else:
                target_influx_coordinate.x += 18 * vial_window_size

    async def influx_event(self, fluid_type: str, influx_volumes: list[int]) -> None:
        """Coordinate xArm and DispenseHead for an individual influx event.

        Brings DispenseHead into vials and executes aspirate/dispense cycle.

        Args:
            fluid_type (str): Name of DispenseHead in use for the influx event.
            influx_volumes (list[int]): List of influx volumes for aspirate/dispense.

        Raises:
            RoboticsRoutineError: If specified DispenseHead is not in use.
            DispenseHeadError: If a dispense head operation fails.
            xArmError: If an xArm operation fails.
        """

        ################# ASPIRATION STEP #################
        self.check_for_interrupt()
        logger.info(f"Running aspiration during: {self.routine}")
        try:
            async with asyncio.TaskGroup() as aspiration_tasks:
                aspiration_tasks.create_task(self.dispense_heads[fluid_type].aspirate(influx_volumes))
                if self.arm_command_queue:
                    aspiration_tasks.create_task(self.execute_xArm_commands())
        except DispenseHeadError:
            logger.exception("DispenseHead failed during influx_event aspiration tasks", stack_info=True)
            raise
        except xArmError:
            logger.exception("xArm failed during influx_event aspiration tasks", stack_info=True)
            raise

        ################# DISPENSE STEP #################
        self.check_for_interrupt()
        logger.info(f"Running dispense during: {self.routine}")
        try:
            self.dispense_heads[fluid_type].dispense(influx_volumes)
        except DispenseHeadError:
            logger.exception("DispenseHead failed during influx_event dispense", stack_info=True)
            raise
        except xArmError:
            logger.exception("xArm failed during influx_event dispense", stack_info=True)
            raise

    async def execute_xArm_commands(self) -> None:
        """Execute all commands in the xArm command queue.

        Sequentially executes all commands in the xArm command queue, removing commands as they complete.

        Raises:
            xArmError: If a movement fails or if the queue is empty.
        """

        if not self.arm_command_queue:
            logger.warning("Tried running executing xArm commands but queue is empty")
            raise xArmError("Tried running executing xArm commands but queue is empty")

        while self.arm_command_queue:
            command = self.arm_command_queue[0]
            try:
                self.check_for_interrupt()
                self.arm.move(command)
                self.arm_command_queue.pop()
            except xArmError:
                logger.exception("Error trying to run execute_xArm_commands", stack_info=True)
                raise xArmError("Error trying to run execute_xArm_commands")

    def change_tool(self, target_fluid_type: str) -> None:
        """Switch the active DispenseHead to the specified fluid type.

        Unloads the current DispenseHead and loads the new one, updating internal state accordingly.

        Args:
            target_fluid_type (str): The fluid type to activate.
        """

        self.to_unload_head()
        self.to_load_head(target_fluid_type)
        for fluid_type, dispense_head in self.dispense_heads.items():
            if fluid_type == target_fluid_type:
                dispense_head.in_use = True
            else:
                dispense_head.in_use = False

    def to_unload_head(self):
        """Move the xArm to unload the currently in-use DispenseHead and return it to the tool holder.

        This method should be called before loading a new DispenseHead.
        """
        try:
            # TODO: add logic for xarm movements
            logger.info("Unloading DispenseHead")
        except xArmError:
            logger.exception("Error trying to run to_unload_head", stack_info=True)
            raise xArmError("Error trying to run to_unload_head")

    def to_load_head(self, new_fluid_type: str):
        """Move the xArm to load a new DispenseHead from its tool holder.

        Args:
            new_fluid_type (str): The fluid type of the DispenseHead to load.

        The xArm end effector must be empty before calling this method.
        """
        try:
            # TODO: add logic for xarm movements
            logger.info("Loading DispenseHead")
        except xArmError:
            logger.exception("Error trying to run to_load_head", stack_info=True)
            raise xArmError("Error trying to run to_load_head")

    async def to_home(self) -> None:
        """Move the xArm linearly from its standby to home reference position.

        Useful for tasks requiring manual intervention.
        """
        self.arm_command_queue.append(self.arm.reference_positions["home"])
        await self.execute_xArm_commands()

    async def to_standby(self) -> None:
        """Move the xArm linearly from its standby to home reference position.

        The xArm must be in STANDBY position prior to running routines that interface with SmartStations.
        """
        self.arm_command_queue.append(self.arm.reference_positions["standby"])
        await self.execute_xArm_commands()

    async def broadcast(self):
        """Broadcast the current robotics system status to all connected clients.

        Emits the current status information, including state, active routine, xArm, and DispenseHead status.
        """

        # emit robotics status to all connected clients
        await self.emit("broadcast", self.to_dict())
        logger.info(f"DispenseHead Broadcast: {self.dispense_heads}")
        logger.info(f"xArm Broadcast: {self.arm}")
        logger.info(f"SmartStation Broadcast: {self.stations}")

    def error_warn_change_callback(self, xarm_api_data: dict):
        """Update error and warning codes based on xArm feedback.

        Called by the xArm API when error or warning states change.

        Args:
            xarm_api_data (dict): Dictionary containing error and warning codes.
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
        """

        self.arm.connected = xarm_api_data["connected"]
        logger.debug(f"xArm connect change callback input: {xarm_api_data}")

    def save_config(self):
        """Save the robotics configuration to disk.

        Writes the current settings to the robotics_config file to persist any changes made during operation.
        """

        with open(self.robotics_config_path, "w") as conf:
            yaml.dump(self.robotics_config, conf, default_flow_style=False)
        logger.info("Robotics configuration saved to disk")

    def load_config(self):
        """Load the robotics configuration from disk.

        Reads the latest settings from the robotics_config file to ensure current operations use up-to-date configuration values.
        """

        with open(self.robotics_config_path, "r") as conf:
            self.robotics_config = yaml.safe_load(conf)

    def update_robotics(self):
        """Update robotics components with the latest configuration.

        Updates SmartStations, DispenseHead, and xArm with the current configuration values.
        """
        for station_id, station in self.stations.items():
            station.update(self.robotics_config["smart_stations"][station_id])
        for fluid_type, dispense_head in self.dispense_heads.items():
            dispense_head.update_head(self.robotics_config["dispense_head"][fluid_type])
        self.arm.update(self.robotics_config["xArm"])

    def stop_robotics(self):
        """Stop all robotics and pump operations.

        Terminates all syringe pump commands and puts xArm into stop state due to user intervention.
        """

        self.state = RoboticsState.STOP
        try:
            for dispense_head in self.dispense_heads.values():
                if dispense_head.in_use:
                    dispense_head.stop()
            self.arm.stop()
        except (DispenseHeadError, xArmError):
            raise
        logger.info("Robotics namespace put into STOP state, active processes have been exited.")

    def pause_robotics(self):
        """Pause robotics operations.

        Sets the robotics state to PAUSE if currently BUSY, pauses xArm operations, and terminates pending syringe pump commands.
        """
        if self.state == RoboticsState.BUSY:
            self.state = RoboticsState.PAUSE
            try:
                for dispense_head in self.dispense_heads.values():
                    if dispense_head.in_use:
                        dispense_head.pause()
                self.arm.pause()
            except (DispenseHeadError, xArmError):
                raise
        logger.info("Robotics namespace put into PAUSE state")

    def resume_robotics(self):
        """Resume paused robotics operations.

        Sets the robotics state back to BUSY if previously PAUSE, resumes xArm operations, and resumes pending syringe pump commands.
        """
        if self.state == RoboticsState.PAUSE:
            self.state = RoboticsState.BUSY
            try:
                for dispense_head in self.dispense_heads.values():
                    if dispense_head.in_use:
                        dispense_head.resume()
                self.arm.resume()
            except (DispenseHeadError, xArmError):
                raise
        logger.info("Robotics namespace put back into BUSY state, resuming previously paused activity.")

    def emergency_stop_robotics(self):
        """Emergency stop all robotics operations.

        Terminates all syringe pump commands, triggers emergency stop on the xArm,
        and disconnects from the hardware. Requires manual intervention to restart.
        """

        try:
            self.stop_robotics()
            for dispense_head in self.dispense_heads.values():
                dispense_head.disable_head()
            self.arm.disconnect()
        except (SyringeError, SyringeTimeout):
            logger.exception("error encountered trying to call stop_robotics()", stack_info=True)
        logger.info("Robotics namespace put into EMERGENCY_STOP state")

    def check_for_interrupt(self):
        """Check for pause or stop signals during routine execution.

        Pauses robotics modules if a PAUSE state is detected and resumes when state changes out of PAUSE.
        Raises an exception if a STOP is received.

        Raises:
            StopRobotics: If stop state is detected.
        """

        while self.state == RoboticsState.PAUSE:
            if self.state == RoboticsState.STOP:
                raise StopRobotics("Robotics namespace state set to stop")
            else:
                time.sleep(0.1)

    def to_dict(self):
        """Convert the robotics namespace state to a dictionary.

        Returns:
            dict: Dictionary representation of the robotics namespace state,
                including status, routine, active stations, and component details.
        """
        status = {
            "state": (self.state.name, self.state.value),
            "routine": (self.routine.name, self.routine.value),
            "active_stations": self.active_station_id,
            "xarm": self.arm.to_dict(),
            "dispense_head": {fluid_type: dispense_head.to_dict() for fluid_type, dispense_head in self.dispense_heads.items()},
        }
        return status
