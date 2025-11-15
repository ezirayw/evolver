import logging
import time
from dataclasses import asdict
from typing import Callable

import socketio

from htevolver.exceptions import ClientError
from htevolver.shared import HTEvolverStatus, RoboticsRoutines, RoboticsState, StationInfluxCommand

logger = logging.getLogger(__name__)


def routine_decorator(func: Callable):
    """Decorator for sending routine jobs to HTeVOLVER.

    Handles checking ready state and logging for robotics routine commands.

    Args:
        func (callable): The helper function to decorate.

    Returns:
        callable: The wrapped function.

    Examples:
        >>> @routine_decorator
        ... def some_routine(self, commands):
        ...     self.emit("some_routine", commands)
    """

    def wrapper(self, *args, **kwargs):
        if self._check_ready():
            func(self, *args, **kwargs)
            logger.info(f"Initiating the {func.__name__.upper()} with the command: {args[0]}")
        else:
            logger.warning(f"Robotics namespace not ready to handle the routine {func.__name__.upper()}")

    return wrapper


class RoboticsClientNamespace(socketio.ClientNamespace):
    """Client namespace for communicating with the robotics server.

    Handles communication with the robotics namespace of the server, including
    sending routine commands and handling status updates.

    Attributes:
        save (bool): Whether to save data locally.
        directory (str): Directory for saving data.
        status (HTEvolverStatus): Status of the HT Evolver system.
        robotics_ns (dict): Robotics namespace data.
        server_config (dict): Server configuration data.
        server_types (dict[str, dict[str, int]]): Server data type definitions.
        ack (bool): Acknowledgment flag for communication.
    """

    namespace: str = "/robotics"

    def __init__(self, save: bool, directory: str, status: HTEvolverStatus):
        super().__init__(RoboticsClientNamespace.namespace)
        self.save: bool = save
        self.directory: str = directory
        self.status: HTEvolverStatus = status
        self.server_config: dict = {}
        self.server_types: dict[str, dict[str, int]] = {}
        self.ack: bool = False

    def on_connect(self):
        """Handle connection to the server."""
        logger.info("Client connected to HTeVOVLER server via robotics namespace")

    def on_disconnect(self):
        """Handle disconnection from the server."""
        logger.info("Client disconnected from HTeVOLVER server via robotics namespace")

    def on_reconnect(self):
        """Handle reconnection to the server."""
        logger.info("Client reconnected to HTeVOLVER server via robotics namespace")

    def on_broadcast(self, status: dict):
        """Handle broadcast data from the server.

        Processes broadcast data received from the server, which includes
        the current state of the robotics system, active operations, and
        the status of the xArm and DispenseHead.

        Args:
            broadcast_data (dict): Broadcast data received from the server.
        """
        self.status.robotics = status
        logger.info(f"Robotics namespace broadcast: {self.status.robotics}")

    def _request_robotics_status(self):
        """Request status information from the server.

        Requests the status information from robotics namespace server. Server response
        is processed by supplied callback lambda function.
        """
        logger.info("Requesting robotics namespace status from HTeVOLVER server.")
        self.emit("request_config", callback=lambda status_data: setattr(self, "status", status_data))
        logger.info("Finished processing robotics namespace status request.")

    def _request_robotics_config(self):
        """Request configuration from the server.

        Requests the current configuration of the robotics namespace server. Server response
        is processed by supplied callback lambda function.
        """
        logger.info("Requesting robotics namespace configuration from HTeVOLVER server.")
        self.emit("request_config", callback=lambda config_data: setattr(self, "server_config", config_data))
        logger.info("Finished processing robotics namespace configuration request.")

    def _connect_xArm(self):
        """Request the server to connect to the xArm."""
        self.emit("connect_xArm")
        logger.info("Connecting xArm to HTeVOLVER")

    def _disconnect_xArm(self):
        """Request the server to disconnect from the xArm."""
        self.emit("disconnect_xArm")
        logger.info("Disconnecting xArm from HTeVOLVER")

    def _reset_xArm(self):
        """Request the server to reset connection to the xArm."""
        self.emit("reset_xArm")
        logger.info("Resetting xArm")

    def _enable_dispenseheads(self, dispense_head_list: list[str] = []):
        """Request the server to enable DispenseHeads using their fluid_type name. Empty list enables all DispenseHeads

        Args:
            dispense_head_list (list[str], optional): List of fluid type DispenseHeads to enable. Defaults to an empty list.

        """
        self.emit("on_enable_heads", dispense_head_list)
        logger.info(f"Enabling DispenseHeads {dispense_head_list} on HTeVOLVER")

    def _disable_dispenseheads(self, dispense_head_list: list[str] = []):
        """Request the server to disable DispenseHeads using their fluid_type name. Empty list disables all DispenseHeads

        Args:
            dispense_head_list (list[str], optional): List of fluid type DispenseHeads to disable. Defaults to an empty list.

        """
        self.emit("on_disable_heads", dispense_head_list)
        logger.info(f"Disabling DispenseHeads {dispense_head_list} on HTeVOLVER")

    def _override(self, override_parameter: str, override_data: dict | int):
        """Requests server to override robotics namespace state, routine, and configuration.

        Args:
            override_parameter (str): The robotics namespace parameter to override.
            override_data (dict | int): Desired data to override target robotics namespace parameter.
        """
        valid_parameters: list[str] = ["state", "routine", "config"]
        logger.info("Received request to override robotics namespace status/config.")
        if override_parameter not in valid_parameters:
            logger.error(f"Aborting override, invalid parameter entered: {override_parameter}")
            raise ClientError(f"Aborting override, invalid parameter entered: {override_parameter}")

        if override_parameter == "state":
            try:
                RoboticsState(override_data)
            except ValueError:
                logger.error(f"Aborting override, invalid state entered: {override_data}")

        if override_parameter == "routine":
            try:
                RoboticsRoutines(override_data)
            except ValueError:
                logger.error(f"Aborting override, invalid routine entered: {override_data}")

        self.emit("override", {override_parameter: override_data})
        logger.info(f"Overriding robotics namespace state on HTeVOLVER server with: {override_parameter, override_data}")

    def _pause(self):
        """Pause active robotics routines by suspending active DispenseHead and xArm operations."""
        self.emit("pause_robotics")
        logger.info("Paused experiment")

    def _resume(self):
        """Resumes recently paused robotics routines."""
        self.emit("resume_robotics")
        logger.info("Resumed experiment")

    def _stop(self):
        """Kills active or paused robotics routines by ending DispenseHead and xArm operations and gracefully exiting robotic routines."""
        self.emit("stop_robotics")
        logger.info("Stopped experiment")

    def _check_ready(self):
        """Check if the robotics system is ready for a new routine.

        Verifies that the robotics system is not currently executing a routine.
        Blocks until a status acknowledgment is received from the server.

        Returns:
            bool: True if the system is ready (no active routine), False otherwise.

        """
        self.ack = False
        self._request_robotics_status()
        while not self.ack:
            time.sleep(0.1)

        routine = RoboticsRoutines[self.status.robotics["routine"][1]]
        state = RoboticsState[self.status.robotics["state"][1]]
        if routine == RoboticsRoutines.NO_ROUTINE and state == RoboticsState.READY:
            return True
        else:
            return False

    @routine_decorator
    def _pipette(self, pipette_commands: list[int]):
        """Execute a basic pipette operation with currently in use DispenseHead.

        Puts the robotics namespace into a BUSY state. Checks for negative volumes prior to sending request.

        Args:
            pipette_commands (list): Dictionary containing dipense commands.
        """
        for volume in pipette_commands:
            if volume < 0:
                raise ClientError(f"Blocking pipette request, negative volume found: {volume}")
        self.emit("dipense_routine", pipette_commands)

    @routine_decorator
    def _prime_dispenseheads(self, dispense_head_list: list[str]):
        """Execute a DispenseHead priming cycle

        Puts the robotics namespace into a BUSY state.

         Args:
             prime_commands (list): List containing DispenseHeads to prime.
             volume (int): Volume to dipense during priming. Defaults to 10mL
        """
        self.emit("prime_dispenseheads", dispense_head_list)

    @routine_decorator
    def _influx(self, influx_commands: dict[int, list[StationInfluxCommand]]):
        """Process multi-fluid influx command prior to sending to server.

        Puts robotics namespace into a BUSY state. Serializes StationInfluxCommand objects into dictionaries prior to sending request.

        Args:
            influx_commands (dict): Dictionary mapping SmartStation IDs to list of StationInfluxCommand instances
        """
        influx_commands_dict: dict[int, list[dict]] = {}
        for station_id, station_commands in influx_commands.items():
            influx_commands_dict[station_id] = [asdict(station_command) for station_command in station_commands]

        self.emit("influx_routine", influx_commands_dict)
