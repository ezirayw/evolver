import logging
import time
from typing import Callable

import socketio

from htevolver.shared import HTEvolverStatus, RoboticsRoutines, RoboticsState

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

    def __init__(self, save: bool, directory: str, status: HTEvolverStatus, namespace: str = "/robotics"):
        super().__init__(namespace)
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
        the status of the xArm and PipetteHead.

        Args:
            broadcast_data (dict): Broadcast data received from the server, containing
                status information about the robotics system, including state, routine,
                active stations, xArm status and pipette head status.
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
        """Request the server to connect to the xArm.

        Requests server to establish a connection with the xArm via its Python SDK API interface.
        """
        self.emit("connect_xArm")
        logger.info("Connecting xArm to HTeVOLVER server")

    def _disconnect_xArm(self):
        """Request the server to disconnect to the xArm.

        Requests server to disconnect from the xArm via its Python SDK API interface.
        """
        self.emit("disconnect_xArm")
        logger.info("Disconnecting xArm from HTeVOLVER server")

    def _enable_influx(self):
        """Request the server to enable PipetteHead syringe pumps.

        PipetteHead must be enabled prior to running any influx operations.
        """

        self.emit("enable_influx")
        logger.info("Enabling PipetteHead syringe pumps on HTeVOLVER server")

    def _disable_influx(self):
        """Request the server to disable PipetteHead syringe pumps.

        Useful for preventing unwanted influx operations.
        """
        self.emit("disable_influx")
        logger.info("Disabling PipetteHead syringe pumps on HTeVOLVER server")

    def _override(self, override_commands: dict):
        """Override the robotics status on the server.

        Requests server to override robotics namespace state, routine, and configuration.

        Args:
            override_commands (dict): Dictionary of status values to override.
                Keys should match attributes in the RoboticsServerNamespace class.
        """
        self.emit("override", override_commands)
        logger.info(f"Overriding robotics namespace state on HTeVOLVER server with: {override_commands}")

    def _pause(self):
        """Pause active routines in the robotics namespace backend.

        Requests the server to put the robotics namespace into a pause state. Suspends any active
        PipetteHead and xArm operations.
        """
        self.emit("pause_robotics")
        logger.info("Paused experiment")

    def _resume(self):
        """Resumes recently paused routines in the robotics namespace backend.

        Requests the server to put the robotics namespace into a resume state. Resumes paused
        PipetteHead and xArm operations.
        """
        self.emit("resume_robotics")
        logger.info("Resumed experiment")

    def _stop(self):
        """Kills active robotics routines in the robotics namespace backend.

        Requests the server to put the robotics namespace into a stop state. Kills PipetteHead and xArm
        operations and gracefully exits active robotic routines.
        """
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
    def _pipette(self, pipette_commands: dict[int, int]):
        """Execute a basic pipette operation with the PipetteHead

        Requests the server to perform a pipetting operation with the PipetteHead.
        Puts the robotics namespace into a busy state.

        Args:
            pipette_commands (dict): Dictionary containing pipette commands.
                Key value pairs map to PipetteHead Pump ID and pipette volume.
        """
        self.emit("pipette_routine", pipette_commands)

    @routine_decorator
    def _prime_pipettehead(self, prime_commands: list[int]):
        """Execute a PipetteHead priming cycle

        Requests the server to prime the specified syringe pumps on the PipetteHead. Function is expected to be called
        repeatedly with experimenter input to ensure that lines are completely filled prior to running experiments.
        Puts the robotics namespace into a busy state.

         Args:
             prime_commands (list): List containing PipetteHead Pump IDs to prime
             volume (int): Volume to pipette during priming. Defaults to 10mL
        """
        self.emit("prime_pipettehead", prime_commands)

    @routine_decorator
    def _influx(self, influx_commands: dict):
        """Execute a influx routine across SmartStation vials with the PipetteHead.

        Requests the server to perform an influx cycle across HT-eVOLVER based on the specified target vials and influx volume
        inputs. Coordinates xArm to move PipetteHead in a snake pattern across target SmartStations. Puts robotics namespace into
        a busy state

        Args:
            influx_commands (dict): Nested dictionary mapping:
                - station_id -> vial_id -> fluid_type -> volume
                Example structure: {0: {3: {"MEDIA": 100, "DRUG": 50}}}
                This would add 100μL of MEDIA and 50μL of DRUG to vial 3 in station 0.
        """
        self.emit("influx_routine", influx_commands)

    @routine_decorator
    def _fill_vials(self, fill_commands: dict):
        """Fill vials with specified fluids.

        Requests the server to fill all vials within a target SmartStation with influx volume inputs. Only 1 fluid type allowed
        per SmartStation. Similar to _influx() in how modules operate. Puts robotics namespace into
        a busy state.

        Args:
            fill_commands (dict): Dictionary mapping station IDs to tuples of (fluid_type, volume_μL).
                This applies the same fluid and volume to ALL vials in the specified stations.
        """
        self.emit("fill_vials_routine", fill_commands)
