import asyncio
import logging
import pathlib
import time
from collections import deque
from dataclasses import asdict
from typing import Annotated, Any, Literal

import yaml
from fastapi import APIRouter, BackgroundTasks, Depends, HTTPException, Path, Request, status
from fastapi.concurrency import run_in_threadpool
from pydantic import BaseModel, Field, NonNegativeInt
from tecancavro.models import SyringeError
from tecancavro.syringe import SyringeTimeout

from htevolver.dependencies import ErrorModel, RoboticsRoutine, RoboticsState, RoboticsStatus, StationInfluxCommandBody
from htevolver.exceptions import DispenseHeadError, RoboticsError, RoboticsStop, xArmError
from htevolver.robotics.dispensehead import DispenseHeadXCaliburD, DummyDispenseHead
from htevolver.robotics.interfaces import DispenseHeadProtocol
from htevolver.robotics.smart_station import SmartStationRobotics, StationCoordinate
from htevolver.robotics.xarm import CartesianMovement, xArm

router = APIRouter(prefix="/robotics")
logger = logging.getLogger(__name__)


class Robotics:
    """Server namespace for handling robotics hardware control.

    Manages robotics hardware components and coordinates complex robotics routines
    such as pipetting, influx, and vial filling.

    Attributes:
        config (dict): Configuration for robotics components.
        config_path (pathlib.Path): pathlib.Path to the robotics configuration file.
        arm_command_queue (deque[CartesianMovement]): Queue of xArm movement commands.
        state (RoboticsState): Current state of the robotics system.
        routine (RoboticsRoutine): Currently executing routine if any.
        active_station (int): Currently active station ID or -1 if none.
        active_vials (list[int]): Currently active vial IDs.
        dispenseheads (dict[str, DispenseHeadProtocol]): Fluid type to DispenseHead mapping.
        stations (dict[int, SmartStationRobotics]): Station ID to SmartStationRobotics mapping.
        xarm (UFactoryArm): The UFactory robot instance.
    """

    dispensehead_factory: dict[str, type[DispenseHeadProtocol]] = {
        "XCaliburD": DispenseHeadXCaliburD,
        "Dummy": DummyDispenseHead,
    }

    def __init__(self, config: dict, config_path: pathlib.Path) -> None:
        self.config: dict = config
        self.config_path: pathlib.Path = config_path
        self.xarm_command_queue: deque[CartesianMovement] = deque()

        self.status: RoboticsStatus = RoboticsStatus(
            state=RoboticsState.READY,
            routine=RoboticsRoutine.NO_ROUTINE,
            start_time=0,
            end_time=0,
            elapsed_time=0,
        )
        self.active_station: str | None = None
        self.active_vials: list[int] = []

        # instantiate robotics modules
        self.dispenseheads: dict[str, DispenseHeadProtocol] = {}
        for fluid_type, dispensehead_config in self.config["dispenseheads"].items():
            dispensehead_type: type[DispenseHeadProtocol] = dispensehead_config["pump_type"]
            self.dispenseheads[fluid_type] = Robotics.dispensehead_factory[dispensehead_type].from_config(dispensehead_config)
        logger.info(
            f"DispenseHeads successfully created: {[dispensehead.to_dict() for dispensehead in self.dispenseheads.values()]}"
        )

        self.stations: dict[str, SmartStationRobotics] = {}
        for station_id, station_config in self.config["smart_stations"].items():
            if station_config is not None:
                self.stations[f"station_{station_id}"] = SmartStationRobotics.from_config(station_config)
        logger.info(f"SmartStations successfully created: {[station for station in self.stations.values()]}")

        self.xarm = xArm.from_config(self.config["xarm"])
        self.xarm.register_callback(
            self._error_warn_change_callback, self._state_changed_callback, self._connect_changed_callback
        )
        logger.info(f"xArm successfully created: {self.xarm}")

        logger.info("Robotics state initialized")

    def _get_active_dispensehead(self) -> str:
        """Return the fluid type of the currently active (in-use) DispenseHead.

        Returns:
            found_fluid_type (str): The fluid type name if a DispenseHead is active, otherwise an empty string.
        """
        found_fluid_type: str = ""
        for fluid_type, dispensehead in self.dispenseheads.items():
            if dispensehead.active:
                found_fluid_type = fluid_type
        return found_fluid_type

    def _is_valid_dispenseheads(self, fluid_types: list[str]) -> bool:
        """Check if the provided fluid types have corresponding DispenseHeads.

        Args:
            fluid_type_input (str): The fluid type to check.

        Returns:
            bool: True if the fluid type is configured, False otherwise.
        """
        valid: bool = True
        for fluid_type in fluid_types:
            if fluid_type not in self.dispenseheads.keys():
                valid = False
        return valid

    async def _station_influx(self, station_influx_command: StationInfluxCommandBody) -> None:
        """Execute an influx routine based on the provided StationInfluxCommand.

        Args:
            station_influx_command (StationInfluxCommandBody): Command containing influx data.

        Raises:
            RoboticsRoutineError: If specified DispenseHead is not in use.
            DispenseHeadError: If a dispense head operation fails.
            xArmError: If an xArm operation fails.
        """
        if not self.dispenseheads[station_influx_command.dispensehead].active:
            logger.exception(f"DispeneHead {station_influx_command.dispensehead} not set to active", stack_info=True)
            raise RoboticsError(f"{station_influx_command.dispensehead}_DispeneHead not set to active")

        vial_window_size: int = self.dispenseheads[station_influx_command.dispensehead].pump_number
        end_of_row_vials: list[int] = [vial_row[-1] for vial_row in SmartStationRobotics.vial_map]
        windows = []
        for row in SmartStationRobotics.vial_map:
            for start in range(0, len(row), vial_window_size):
                windows.append(row[start : start + vial_window_size])

        station: SmartStationRobotics = self.stations[f"station_{station_influx_command.station_id}"]
        self.active_station = f"station_{station_influx_command.station_id}"
        target_influx_coordinate: StationCoordinate = StationCoordinate(x=0, y=36)

        for step_number, window in enumerate(windows):
            self.active_vials = window

            # queue xArm movements to move DispenseHead to space above active_vials and into active_vials
            self.xarm_command_queue.append(
                station.xArmPlane_out.vial_to_xarm(target_influx_coordinate, speed=50, acceleration=1000)
            )
            self.xarm_command_queue.append(
                station.xArmPlane_in.vial_to_xarm(target_influx_coordinate, speed=50, acceleration=1000)
            )
            influx_command: dict[str, int] = {
                f"pump_{i}": getattr(station_influx_command, f"vial_{vial_id}") for i, vial_id in enumerate(self.active_vials)
            }
            try:
                await self._influx_event(station_influx_command.dispensehead, influx_command)
            except (DispenseHeadError, xArmError):
                logger.exception(f"Error running step_number {step_number} during _station_influx()")
                raise

            # bring DispenseHead out of active vial set
            self.xarm_command_queue.append(
                station.xArmPlane_out.vial_to_xarm(target_influx_coordinate, speed=50, acceleration=1000)
            )
            await self._execute_arm_queue()

            # update target_influx_coordinate for new row if needed
            if window[-1] in end_of_row_vials:
                # Moved to next row
                target_influx_coordinate.x = 0
                target_influx_coordinate.y -= 18
            else:
                target_influx_coordinate.x += 18 * vial_window_size

    async def _influx_event(self, dispensehead: str, influx_command: dict[str, int]) -> None:
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
        await self._check_for_interrupt()
        logger.info(f"Running aspiration during: {self.status.routine}")
        try:
            async with asyncio.TaskGroup() as aspiration_tasks:
                aspiration_tasks.create_task(asyncio.to_thread(self.dispenseheads[dispensehead].aspirate, influx_command))
                if self.xarm_command_queue:
                    aspiration_tasks.create_task(self._execute_arm_queue())
        except DispenseHeadError:
            logger.exception("DispenseHead failed during aspiration tasks", stack_info=True)
            raise
        except xArmError:
            logger.exception("xArm failed during aspiration tasks", stack_info=True)
            raise

        ################# DISPENSE STEP #################
        await self._check_for_interrupt()
        logger.info(f"Running dispense during: {self.status.routine}")
        try:
            await asyncio.to_thread(self.dispenseheads[dispensehead].dispense, influx_command)
        except DispenseHeadError:
            logger.exception("DispenseHead failed during _influx_event dispense", stack_info=True)
            raise
        except xArmError:
            logger.exception("xArm failed during _influx_event dispense", stack_info=True)
            raise

    async def _execute_arm_queue(self) -> None:
        """Execute all commands in the xArm command queue.

        Sequentially executes all commands in the xArm command queue, removing commands as they complete.

        Raises:
            xArmError: If a movement fails or if the queue is empty.
        """

        if not self.xarm_command_queue:
            logger.warning("Tried running executing xArm commands but queue is empty")
            raise xArmError("Tried running executing xArm commands but queue is empty")

        while self.xarm_command_queue:
            command = self.xarm_command_queue[0]
            try:
                await self._check_for_interrupt()
                self.xarm.move(command)
                self.xarm_command_queue.pop()
            except xArmError:
                logger.exception("Error trying to run _execute_arm_queue", stack_info=True)
                raise xArmError("Error trying to run _execute_arm_queue")

    def _change_tool(self, dispensehead_target: str) -> None:
        """Unload current DispenseHead and load target DispenseHead provided

        Args:
            dispensehead_target (str): The DispenseHead to switch to.
        """

        self._unload_head()
        self._load_head(dispensehead_target)
        for dispensehead in self.dispenseheads:
            if dispensehead_target == dispensehead:
                self.dispenseheads[dispensehead].active = True
            else:
                self.dispenseheads[dispensehead].active = False

    def _unload_head(self):
        """Move the xArm to unload the currently active DispenseHead into its tool holder.

        This method must be called before loading a new DispenseHead.
        """
        try:
            # TODO: add logic for xarm movements
            logger.info("Unloading DispenseHead")
        except xArmError:
            logger.exception("Error trying to run _unload_head", stack_info=True)
            raise xArmError("Error trying to run _unload_head")

    def _load_head(self, dispensehead_target: str) -> None:
        """Move the xArm to load a new DispenseHead from its tool holder. The xArm end effector must be empty before calling this method.

        Args:
            dispensehead_target (str): The DispenseHead to load.
        """
        try:
            # TODO: add logic for xarm movements
            logger.info("Loading DispenseHead")
        except xArmError:
            logger.exception("Error trying to run _load_head", stack_info=True)
            raise xArmError("Error trying to run _load_head")

    async def _home(self) -> None:
        """Move the xArm linearly from its standby to home reference position."""
        self.xarm_command_queue.append(self.xarm.home_position)
        await self._execute_arm_queue()

    async def _standby(self) -> None:
        """Move the xArm linearly from its standby to home reference position.

        The xArm must be in STANDBY position prior to running routines that interface with SmartStations.
        """
        self.xarm_command_queue.append(self.xarm.standby_position)
        await self._execute_arm_queue()

    def _error_warn_change_callback(self, xarm_api_data: dict):
        """_update error and warning codes based on xArm feedback.

        Called by the xArm API when error or warning states change.

        Args:
            xarm_api_data (dict): Dictionary containing error and warning codes.
        """

        self.xarm.error_code = xarm_api_data["error_code"]
        self.xarm.warning_code = xarm_api_data["warn_code"]
        logger.debug(f"xArm error/warn change callback input: {xarm_api_data}")
        if xarm_api_data["error_code"] != 0:
            self._emergency_stop()
            logger.error(f"xArm error_code encountered: {xarm_api_data['error_code']}")
        if xarm_api_data["warn_code"] != 0:
            logger.warning(f"xArm warning_code encountered: {xarm_api_data['warn_code']}")

    def _state_changed_callback(self, xarm_api_data: dict):
        """_update xArm state based on controller feedback.

        Called by the xArm API when the internal state changes.

        Args:
            xarm_api_data (dict): Dictionary containing the xArm state.
        """

        self.xarm.state = xarm_api_data["state"]
        logger.debug(f"xArm state change callback input: {xarm_api_data}")
        if xarm_api_data["state"] == 4:
            self._emergency_stop()
            logger.error(f"xArm entered _stop state: {xarm_api_data['state']}")

    def _connect_changed_callback(self, xarm_api_data: dict):
        """_update xArm connection status based on controller feedback.

        Called by the xArm API when the connection status changes.

        Args:
            xarm_api_data (dict): Dictionary containing the connection status.
        """

        self.xarm.connected = xarm_api_data["connected"]
        logger.debug(f"xArm connect change callback input: {xarm_api_data}")

    def _save_config(self):
        """Saves internal robotics configuration to YAML file"""
        with self.config_path.open("w") as conf_file:
            yaml.dump(self.config, conf_file)
        logger.info("eVOLVER state config saved to YAML")

    def _load_config(self):
        """Load the robotics configuration from disk.

        Reads the latest settings from the robotics_config file to ensure current operations use up-to-date configuration values.
        """

        with open(self.config_path, "r") as conf:
            self.robotics_config = yaml.safe_load(conf)

    def _update(self):
        """_update robotics components with the latest configuration.

        Updates SmartStations, DispenseHead, and xArm with the current configuration values.
        """
        for station_id, station in self.stations.items():
            station.update(self.robotics_config["smart_stations"][station_id])
        for fluid_type, dispensehead in self.dispenseheads.items():
            dispensehead.update_head(self.robotics_config["dispensehead"][fluid_type])
        self.xarm.update(self.robotics_config["xArm"])

    def _stop(self):
        """Stop all robotics and pump operations. Puts robotics system into STOP state."""

        self.status.state = RoboticsState.STOP
        try:
            for dispensehead in self.dispenseheads.values():
                if dispensehead.active:
                    dispensehead.stop()
            self.xarm.stop()
        except (DispenseHeadError, xArmError):
            raise
        logger.info("Robotics put into STOP state, active processes have been exited.")

    def _pause(self):
        """Pause robotics operations and puts robotics system into PAUSE state"""
        if self.status.state == RoboticsState.BUSY:
            self.status.state = RoboticsState.PAUSE
            try:
                for dispensehead in self.dispenseheads.values():
                    if dispensehead.active:
                        dispensehead.pause()
                self.xarm.pause()
            except (DispenseHeadError, xArmError):
                raise
        logger.info("Robotics put into PAUSE state")

    def _resume(self):
        """Resume paused robotics operations. Sets robotics state back to BUSY if previously PAUSE"""
        if self.status.state == RoboticsState.PAUSE:
            self.status.state = RoboticsState.BUSY
            try:
                for dispensehead in self.dispenseheads.values():
                    if dispensehead.active:
                        dispensehead.resume()
                self.xarm.resume()
            except (DispenseHeadError, xArmError):
                raise
        logger.info("Robotics put back into BUSY state, resuming previously paused activity.")

    def _emergency_stop(self):
        """Emergency stop all robotics operations. Puts robotics into EMERGENCY_STOP state and disconnects/disables all connected hardware.
        Requires manual intervention to recover.
        """

        try:
            self._stop()
            for dispensehead in self.dispenseheads.values():
                dispensehead.disable()
            self.xarm.disconnect()
        except (SyringeError, SyringeTimeout):
            logger.exception("error encountered trying to call _stop()", stack_info=True)
        logger.info("Robotics put into EMERGENCY_STOP state")

    async def _check_for_interrupt(self):
        """Check for pause or stop signals during execution of routines.

        Pauses robotics modules if a PAUSE state is detected and resumes when state changes out of PAUSE.
        Raises an exception if a STOP is received.

        Raises:
            RoboticsStop: If _stop state is detected.
        """

        while self.status.state == RoboticsState.PAUSE:
            if self.status.state == RoboticsState.STOP:
                raise RoboticsStop
            else:
                await asyncio.sleep(0.1)


@router.get("/status", status_code=status.HTTP_200_OK)
def robotics_status(request: Request) -> dict:
    """Get the current status of the robotics system. Returns dictionary represention of Robotics instance."""
    robotics: Robotics = request.state.robotics
    return asdict(robotics.status)


@router.get("/config", status_code=status.HTTP_200_OK)
def config(request: Request) -> dict:
    """Get the current configuration of the robotics system."""
    robotics: Robotics = request.state.robotics
    return robotics.config


@router.post(
    "/override/{mode}/{value}",
    status_code=status.HTTP_204_NO_CONTENT,
    responses={
        status.HTTP_400_BAD_REQUEST: {"model": ErrorModel, "description": "Invalid config value"},
    },
)
def override_status(mode: Literal["state, routine"], value: str, request: Request):
    """Manually override robotics system state or routine"""
    logger.info("Received robotics override request")
    robotics: Robotics = request.state.robotics
    override_dict: dict[str, type] = {"state": RoboticsState, "routine": RoboticsRoutine}
    try:
        setattr(robotics, mode, override_dict[mode](value))
    except ValueError:
        msg: str = f"{value} is not a valid value for overriding {mode}"
        logger.exception(msg, stack_info=True)
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=msg)


class OverrideConfigBody(BaseModel):
    key: str
    value: Any


@router.post(
    "/override_config",
    status_code=status.HTTP_204_NO_CONTENT,
    responses={
        status.HTTP_400_BAD_REQUEST: {"model": ErrorModel, "description": "Invalid config value"},
    },
)
def override_config(new_config: OverrideConfigBody, request: Request):
    """Manually override robotics configuration"""
    logger.info("Received robotics override config request")
    robotics: Robotics = request.state.robotics
    if new_config.key in robotics.config:
        attribute_value = robotics.config[new_config.key]
        attr_type = type(attribute_value)
        if isinstance(new_config.value, attr_type):
            robotics.config[new_config.key] = new_config.value
        else:
            msg: str = f"Mismatch override value type, got {type(new_config.value)} expected {attr_type}"
            logger.exception(msg, stack_info=True)
            raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=msg)
    else:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=f"Override key submitted not found: {new_config.key}")


@router.post(
    "/pause",
    status_code=status.HTTP_204_NO_CONTENT,
    responses={status.HTTP_500_INTERNAL_SERVER_ERROR: {"model": ErrorModel, "description": "Hardware problem"}},
)
async def pause(request: Request):
    """Put the robotics system into PAUSE state if currently BUSY"""
    logger.info("Recevied robotics _pause request")
    robotics: Robotics = request.state.robotics
    try:
        await run_in_threadpool(robotics._pause)
    except (DispenseHeadError, xArmError):
        msg: str = "Robotics module error trying to pause robotics, check server logs"
        logger.exception(msg, stack_info=True)
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=msg)


@router.post(
    "/resume",
    status_code=status.HTTP_204_NO_CONTENT,
    responses={status.HTTP_500_INTERNAL_SERVER_ERROR: {"model": ErrorModel, "description": "Hardware problem"}},
)
async def resume(request: Request):
    """Resume robotics system if in PAUSE state"""
    logger.info("Received robotics _resume request")
    robotics: Robotics = request.state.robotics
    try:
        await run_in_threadpool(robotics._resume)
    except (DispenseHeadError, xArmError):
        msg: str = "Robotics module error trying to resume robotics, check server logs"
        logger.exception(msg, stack_info=True)
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=msg)


@router.post(
    "/stop",
    status_code=status.HTTP_204_NO_CONTENT,
    responses={status.HTTP_500_INTERNAL_SERVER_ERROR: {"model": ErrorModel, "description": "Hardware problem"}},
)
async def stop(request: Request):
    """Kill active robotics routines if in BUSY or PAUSE state"""
    logger.info("Received robotics _stop request")
    robotics: Robotics = request.state.robotics
    try:
        await run_in_threadpool(robotics._stop)
    except (DispenseHeadError, xArmError):
        msg: str = "Robotics module error trying to stop robotics, check server logs"
        logger.exception(msg, stack_info=True)
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=msg)


@router.post(
    "/connect_xarm",
    status_code=status.HTTP_204_NO_CONTENT,
    responses={status.HTTP_500_INTERNAL_SERVER_ERROR: {"model": ErrorModel, "description": "Hardware problem"}},
)
def connect_xarm(request: Request):
    """Connect to the xArm"""
    logger.info("Received robotics request to connect to the xArm")
    robotics: Robotics = request.state.robotics
    try:
        robotics.xarm.connect()
        robotics.xarm.initialize()
    except xArmError:
        msg: str = "Error trying to connect to xArm, check server logs"
        logger.exception(msg, stack_info=True)
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=msg)


@router.post(
    "/reset_xarm",
    status_code=status.HTTP_204_NO_CONTENT,
    responses={status.HTTP_500_INTERNAL_SERVER_ERROR: {"model": ErrorModel, "description": "Hardware problem"}},
)
def reset_xarm(request: Request):
    """Reset the xArm to clear errors"""
    logger.info("Received robotics request to reset xArm")
    robotics: Robotics = request.state.robotics
    try:
        robotics.xarm.initialize()
    except xArmError:
        msg: str = "Error trying to connect to xArm, check server logs"
        logger.exception(msg, stack_info=True)
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=msg)


@router.post(
    "/enable_head/{dispensehead}",
    status_code=status.HTTP_204_NO_CONTENT,
    responses={
        status.HTTP_404_NOT_FOUND: {"model": ErrorModel, "description": "Query parameter not found"},
        status.HTTP_500_INTERNAL_SERVER_ERROR: {"model": ErrorModel, "description": "Hardware problem"},
    },
)
def enable_dispensehead(
    dispensehead_target: Annotated[str, Path(title="DispenseHead to enable.", example="media")],
    request: Request,
):
    """Enable target DipsenseHead to facilitate future usage"""
    logger.info(f"Received robotics request to enable DispenseHead {dispensehead_target}")
    robotics: Robotics = request.state.robotics
    if dispensehead_target not in robotics.dispenseheads:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=f"DispenseHead {dispensehead_target} not found")
    try:
        robotics.dispenseheads[dispensehead_target].enable()
    except DispenseHeadError:
        msg: str = f"Error trying to enable DispenseHead {dispensehead_target}, check server logs"
        logger.exception(msg, stack_info=True)
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=msg)


@router.post(
    "/enable_heads",
    status_code=status.HTTP_204_NO_CONTENT,
    responses={status.HTTP_500_INTERNAL_SERVER_ERROR: {"model": ErrorModel, "description": "Hardware problem"}},
)
def enable_dispenseheads(request: Request):
    """Enable all DipsenseHeads to facilitate future usage"""
    logger.info("Received robotics request to enable all DispenseHeads")
    robotics: Robotics = request.state.robotics
    try:
        for dispensehead in robotics.dispenseheads.values():
            dispensehead.enable()
    except xArmError:
        msg: str = "Error trying to enable DispenseHeads, check server logs"
        logger.exception(msg, stack_info=True)
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=msg)


@router.post(
    "/disable_head/{dispensehead_target}",
    status_code=status.HTTP_204_NO_CONTENT,
    responses={
        status.HTTP_404_NOT_FOUND: {"model": ErrorModel, "description": "Query parameter not found"},
        status.HTTP_500_INTERNAL_SERVER_ERROR: {"model": ErrorModel, "description": "Hardware problem"},
    },
)
def disable_dispensehead(
    dispensehead_target: Annotated[str, Path(title="DispsenseHead to disable.", example="media")],
    request: Request,
):
    """Disable target DipsenseHead to block future usage"""
    logger.info(f"Received robotics request to enable DispenseHead {dispensehead_target}")
    robotics: Robotics = request.state.robotics
    if dispensehead_target not in robotics.dispenseheads:
        msg: str = f"DispenseHead {dispensehead_target} not found"
        logger.exception(msg, stack_info=True)
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=msg)
    try:
        robotics.dispenseheads[dispensehead_target].disable()
    except DispenseHeadError:
        msg: str = f"Error trying to disable DispenseHead {dispensehead_target}, check server logs"
        logger.exception(msg, stack_info=True)
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=msg)


@router.post(
    "/disable_heads",
    status_code=status.HTTP_204_NO_CONTENT,
    responses={status.HTTP_500_INTERNAL_SERVER_ERROR: {"model": ErrorModel, "description": "Hardware problem"}},
)
def disable_dispenseheads(request: Request):
    """Disable all DipsenseHeads to block future usage"""
    logger.info("Received robotics request to enable all DispenseHeads")
    robotics: Robotics = request.state.robotics
    try:
        for dispensehead in robotics.dispenseheads.values():
            dispensehead.disable()
    except xArmError:
        msg: str = "error trying to disable DispenseHeads, check server logs"
        logger.exception(msg, stack_info=True)
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=msg)


### ROUTINES ###
def routine_start(request: Request) -> float:
    robotics: Robotics = request.state.robotics
    if robotics.status.state == RoboticsState.READY:
        robotics.status.state = RoboticsState.BUSY
        robotics._load_config()
        robotics._update()
        start_time = time.time()
    else:
        msg: str = "robotics not in READY state to run routine"
        logger.exception(msg, stack_info=True)
        raise HTTPException(status_code=status.HTTP_503_SERVICE_UNAVAILABLE, detail=msg)
    return start_time


def routine_end(robotics: Robotics, start_time: float, exit_status: RoboticsState):
    end_time: float = time.time()
    robotics.status.elapsed_time = end_time - start_time
    robotics.status.state = exit_status
    if exit_status == RoboticsState.READY:
        robotics.status.routine = RoboticsRoutine.NO_ROUTINE
    if exit_status == RoboticsState.EMERGENCY_STOP:
        robotics._emergency_stop()


async def _change_head_background(dispensehead_target: str, robotics: Robotics, start_time: float):
    try:
        await run_in_threadpool(robotics._change_tool, dispensehead_target)
        routine_end(robotics, start_time, RoboticsState.READY)
    except xArmError:
        msg: str = f"Error trying to change DispenseHead to {dispensehead_target}, check server logs"
        logger.exception(msg, stack_info=True)
        routine_end(robotics, start_time, RoboticsState.EMERGENCY_STOP)


@router.post(
    "/routine/change_head/{dispensehead_target}",
    status_code=status.HTTP_204_NO_CONTENT,
    responses={
        status.HTTP_404_NOT_FOUND: {"model": ErrorModel, "description": "Query parameter not found"},
        status.HTTP_500_INTERNAL_SERVER_ERROR: {"model": ErrorModel, "description": "Hardware problem"},
    },
)
async def change_head(
    dispensehead_target: Annotated[str, Path(title="DispenseHead to change to.", example="media")],
    request: Request,
    start_time: Annotated[float, Depends(routine_start)],
    background_tasks: BackgroundTasks,
):
    """Exchange the DipsenseHead for the one given in the endpoint"""
    logger.info(f"Received robotics request to change DipsenseHead to {dispensehead_target}")
    robotics: Robotics = request.state.robotics
    robotics.status.routine = RoboticsRoutine.TOOL_CHANGE
    if not robotics._is_valid_dispenseheads([dispensehead_target]):
        msg: str = f"DispenseHead {dispensehead_target} not found"
        logger.exception(msg, stack_info=True)
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=msg)
    if not robotics.xarm.check_position("standby"):
        msg: str = "xArm not in standby position"
        logger.exception(msg)
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=msg)
    background_tasks.add_task(
        _change_head_background,
        dispensehead_target,
        robotics,
        start_time,
    )


class DispenseHeadBody(BaseModel):
    targets: list[str] = Field(title="List of DispenseHeads to use for request.", examples=["media", "inducer", "buffer"])


async def _initialize_dispenseheads_background(dispenseheads: DispenseHeadBody, robotics: Robotics, start_time: float):
    for dispensehead in dispenseheads.targets:
        try:
            await run_in_threadpool(robotics.dispenseheads[dispensehead].initialize)
            routine_end(robotics, start_time, RoboticsState.READY)
        except DispenseHeadError:
            msg: str = f"Error trying to prime DispenseHead {dispensehead}"
            logger.exception(msg, stack_info=True)
            routine_end(robotics, start_time, RoboticsState.EMERGENCY_STOP)


@router.post(
    "/initialize_dispenseheads",
    status_code=status.HTTP_204_NO_CONTENT,
    responses={status.HTTP_404_NOT_FOUND: {"model": ErrorModel, "description": "Query parameter not found"}},
)
async def initialize_dispenseheads(
    dispenseheads: DispenseHeadBody,
    request: Request,
    start_time: Annotated[float, Depends(routine_start)],
    background_tasks: BackgroundTasks,
):
    """Initialize specific DispenseHead"""
    logger.info(f"Received request to initialize DipsenseHeads {dispenseheads.targets}")
    robotics: Robotics = request.state.robotics
    robotics.status.routine = RoboticsRoutine.INITIALIZE
    if not robotics._is_valid_dispenseheads(dispenseheads.targets):
        msg: str = f"Invalid DispenseHead in {dispenseheads.targets}, not found"
        logger.exception(msg, stack_info=True)
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=msg)
    background_tasks.add_task(_initialize_dispenseheads_background, dispenseheads, robotics, start_time)


async def _prime_dispenseheads_background(dispenseheads: DispenseHeadBody, robotics: Robotics, start_time: float):
    for dispensehead in dispenseheads.targets:
        try:
            await run_in_threadpool(robotics.dispenseheads[dispensehead].prime)
            routine_end(robotics, start_time, RoboticsState.READY)
        except DispenseHeadError:
            msg: str = f"Error trying to prime DispenseHead {dispensehead}"
            logger.exception(msg, stack_info=True)
            routine_end(robotics, start_time, RoboticsState.EMERGENCY_STOP)


@router.post(
    "/prime_dispenseheads",
    status_code=status.HTTP_204_NO_CONTENT,
    responses={status.HTTP_404_NOT_FOUND: {"model": ErrorModel, "description": "Query parameter not found"}},
)
async def prime_dispenseheads(
    dispenseheads: DispenseHeadBody,
    request: Request,
    start_time: Annotated[float, Depends(routine_start)],
    background_tasks: BackgroundTasks,
):
    """Prime target DispenseHeads with fluid to prepare for dispensing"""
    logger.info(f"Received request to prime DispenseHeads {dispenseheads.targets}")
    robotics: Robotics = request.state.robotics
    robotics.status.routine = RoboticsRoutine.PRIMING_INFLUX
    if not robotics._is_valid_dispenseheads(dispenseheads.targets):
        msg: str = f"Invalid DispenseHead in {dispenseheads.targets}, not found"
        logger.exception(msg, stack_info=True)
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=msg)
    background_tasks.add_task(_prime_dispenseheads_background, dispenseheads, robotics, start_time)


async def _standby_background(robotics: Robotics, start_time: float):
    try:
        await robotics._standby()
        routine_end(robotics, start_time, RoboticsState.READY)
    except xArmError:
        msg: str = "Error trying to move xArm to standby, check server logs"
        logger.exception(msg, stack_info=True)
        routine_end(robotics, start_time, RoboticsState.EMERGENCY_STOP)


@router.post(
    "/standby",
    status_code=status.HTTP_204_NO_CONTENT,
    responses={status.HTTP_500_INTERNAL_SERVER_ERROR: {"model": ErrorModel, "description": "Hardware problem"}},
)
async def standby(request: Request, start_time: Annotated[float, Depends(routine_start)], background_tasks: BackgroundTasks):
    """Move the xArm to the standby position"""
    logger.info("Received request to move xArm to standby position")
    robotics: Robotics = request.state.robotics
    robotics.status.routine = RoboticsRoutine.STANDBY
    if not robotics.xarm.check_position("home"):
        msg: str = "xArm is not in home position, cannot move to standby"
        logger.exception(msg, stack_info=True)
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=msg)
    background_tasks.add_task(_standby_background, robotics, start_time)


async def _home_background(robotics: Robotics, start_time: float):
    try:
        await robotics._home()
        routine_end(robotics, start_time, RoboticsState.READY)
    except xArmError:
        msg: str = "Error trying to move xArm to standby, check server logs"
        logger.exception(msg, stack_info=True)
        routine_end(robotics, start_time, RoboticsState.EMERGENCY_STOP)


@router.post(
    "/home",
    status_code=status.HTTP_204_NO_CONTENT,
    responses={status.HTTP_500_INTERNAL_SERVER_ERROR: {"model": ErrorModel, "description": "Hardware problem"}},
)
async def home(request: Request, start_time: Annotated[float, Depends(routine_start)], background_tasks: BackgroundTasks):
    """Home the xArm to the preset home position"""
    logger.info("Received request to move xArm to home position")
    robotics: Robotics = request.state.robotics
    robotics.status.routine = RoboticsRoutine.HOME
    if not robotics.xarm.check_position("standby"):
        msg: str = "xArm is not in standby position, cannot home"
        logger.exception(msg, stack_info=True)
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=msg)
    background_tasks.add_task(_home_background, robotics, start_time)


class PipetteCommandBody(BaseModel):
    pump_0: NonNegativeInt = Field(0, le=1000, title="Volume (uL) to pipette for DispenseHead pump_0", examples=[500])
    pump_1: NonNegativeInt = Field(0, le=1000, title="Volume (uL) to pipette for DispenseHead pump_1", examples=[500])
    pump_2: NonNegativeInt = Field(0, le=1000, title="Volume (uL) to pipette for DispenseHead pump_2", examples=[500])
    pump_3: NonNegativeInt = Field(0, le=1000, title="Volume (uL) to pipette for DispenseHead pump_3", examples=[500])
    pump_4: NonNegativeInt = Field(0, le=1000, title="Volume (uL) to pipette for DispenseHead pump_4", examples=[500])
    pump_5: NonNegativeInt = Field(0, le=1000, title="Volume (uL) to pipette for DispenseHead pump_5", examples=[500])


async def _pipette_background(command: PipetteCommandBody, robotics: Robotics, start_time: float):
    try:
        active_head: str = robotics._get_active_dispensehead()
        await run_in_threadpool(robotics.dispenseheads[active_head].aspirate, command.model_dump())
        await run_in_threadpool(robotics.dispenseheads[active_head].dispense, command.model_dump())
        routine_end(robotics, start_time, RoboticsState.READY)
    except DispenseHeadError:
        msg: str = f"Error trying to pipette with DispenseHead {robotics._get_active_dispensehead()}"
        logger.exception(msg, stack_info=True)
        routine_end(robotics, start_time, RoboticsState.EMERGENCY_STOP)


@router.post(
    "/pipette",
    status_code=status.HTTP_204_NO_CONTENT,
    responses={status.HTTP_500_INTERNAL_SERVER_ERROR: {"model": ErrorModel, "description": "Hardware problem"}},
)
async def pipette(
    command: PipetteCommandBody,
    request: Request,
    start_time: Annotated[float, Depends(routine_start)],
    background_tasks: BackgroundTasks,
):
    """Execute a basic pipette operation with the currently in use DispenseHead. DispenseHead will aspirate and dispense fluid according to input volume command"""
    logger.info("Received pipette request")
    robotics: Robotics = request.state.robotics
    if robotics._get_active_dispensehead() == "":
        msg: str = "No active DispenseHead is set for pipetting."
        logger.exception(msg, stack_info=True)
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=msg)
    background_tasks.add_task(_pipette_background, command, robotics, start_time)


async def _influx_background(command: dict[int, list[StationInfluxCommandBody]], robotics: Robotics, start_time: float):
    for station_id, station_influx_commands in command.items():
        for station_influx_command in station_influx_commands:
            try:
                await run_in_threadpool(robotics._change_tool, station_influx_command.dispensehead)
                await run_in_threadpool(robotics._standby)
                await robotics._station_influx(station_influx_command)
                await run_in_threadpool(robotics._standby)
            except (DispenseHeadError, xArmError):
                msg: str = f"Robotics module error during influx routine for SmartStation {station_id}, check logs"
                logger.exception(msg, stack_info=True)
                routine_end(robotics, start_time, RoboticsState.EMERGENCY_STOP)
    routine_end(robotics, start_time, RoboticsState.READY)


@router.post(
    "/influx",
    status_code=status.HTTP_204_NO_CONTENT,
    responses={
        status.HTTP_404_NOT_FOUND: {"model": ErrorModel, "description": "Query parameter not found"},
        status.HTTP_500_INTERNAL_SERVER_ERROR: {"model": ErrorModel, "description": "Hardware problem"},
    },
)
async def influx(
    command: dict[int, list[StationInfluxCommandBody]],
    request: Request,
    start_time: Annotated[float, Depends(routine_start)],
    background_tasks: BackgroundTasks,
):
    """Execute multi-fluid influx into specific vials across SmartStations. Converts commands into StationInfluxCommand objects."""
    logger.info("Received robotics influx request")
    robotics: Robotics = request.state.robotics
    robotics.status.routine = RoboticsRoutine.INFLUX

    if not robotics.xarm.check_position("standby"):
        msg: str = "xArm not in standby position"
        logger.warning(msg, stack_info=True)
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=msg)

    for station_influx_commands in command.values():
        for station_influx_command in station_influx_commands:
            if not robotics._is_valid_dispenseheads([station_influx_command.dispensehead]):
                msg: str = f"DispenseHead {station_influx_command.dispensehead} not found"
                logger.warning(msg)
                raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=msg)
    background_tasks.add_task(_influx_background, command, robotics, start_time)
