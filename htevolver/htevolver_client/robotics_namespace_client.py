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

    def wrapper(self: RoboticsClientNamespace, *args, **kwargs):
        if self.check_ready():
            # run the target function
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
        server_conf (dict): Server configuration data.
        server_types (dict[str, dict[str, int]]): Server data type definitions.
        ack (bool): Acknowledgment flag for communication.
    """

    def __init__(self, save: bool, directory: str, status: HTEvolverStatus, namespace: str = "/robotics"):
        super().__init__(namespace)
        self.save: bool = save
        self.directory: str = directory
        self.status: HTEvolverStatus = status
        self.server_conf: dict = {}
        self.server_types: dict[str, dict[str, int]] = {}
        self.ack: bool = False

    def on_connect(self):
        """Handle connection to the server.

        Called when a connection is established to the server.
        Automatically requests status, configuration, and type information.
        """
        self.request_robotics_status()
        self.request_robotics_conf()
        logger.info("Client connected to HTeVOVLER server via robotics namespace")

    def on_disconnect(self):
        """Handle disconnection from the server."""
        logger.info("Client disconnected from HTeVOLVER server via robotics namespace")

    def on_reconnect(self):
        """Handle reconnection to the server.

        Called when a connection is re-established after a disconnection.
        """
        logger.info("Client reconnected to HTeVOLVER server via robotics namespace")

    def on_broadcast(self, status: dict):
        """Handle status broadcast data from the server.

        Processes status broadcast data received from the server, which includes
        the current state of the robotics system, active operations, and
        the status of the xArm and PipetteHead.

        Args:
            broadcast_data (dict): Broadcast data received from the server, containing
                status information about the robotics system, including state, routine,
                active stations, xArm status and pipette head status.
        """
        self.status.robotics = status
        logger.info(f"Robotics namespace broadcast: {self.status.robotics}")

    def on_get_status(self, status: dict):
        """Handle status data received from the server.

        Updates the local status with data received from the server.
        This is the response to a request_robotics_status() call.

        Args:
            status (dict): Status data received from the server, containing
                robotics state, current routine, active stations, and status
                of the xArm and pipette head components.
        """
        self.status.robotics = status
        logger.info(f"Robotics namespace broadcast processed: {self.status}")

    def on_get_conf(self, data: dict):
        """Handle configuration data received from the server.

        Updates the local configuration with data received from the server.
        This configuration includes settings for the xArm, SmartStations,
        and PipetteHead components.

        Args:
            data (dict): Configuration data received from the server, containing
                xArm settings, SmartStation vial mappings and coordinates, and
                PipetteHead configurations.
        """
        self.server_conf = data
        logger.info("Received robotics namespace configuration from HTeVOLVER server.")

    def acknowledge(self):
        """Acknowledge receipt of status request.

        Called by the server as a callback to confirm that a status request was received.
        Sets the ack flag to True which unblocks the check_ready() method.
        """
        self.ack: bool = True

    def request_robotics_status(self):
        """Request status information from the server.

        Asks the server to send current robotics status information.
        The server will respond by calling the on_get_status() callback
        with the current state of the robotics system.

        Examples:
            >>> robotics_ns.request_robotics_status()
        """
        self.emit("request_status", callback=self.acknowledge)
        logger.info("Requesting robotics namespace status from HTeVOLVER server.")

    def request_robotics_conf(self):
        """Request configuration from the server.

        Asks the server to send current robotics configuration information.
        The server will respond by calling the on_get_conf() callback with
        the current robotics configuration data from robotics_conf.yml.

        Examples:
            >>> robotics_ns.request_robotics_conf()
        """
        self.emit("request_conf")
        logger.info("Requesting robotics namespace configuration from HTeVOLVER server.")

    def connect_xArm(self):
        """Request the server to connect to the xArm.

        Asks the server to establish a connection with the robotic arm.
        Triggers arm.connect() on the server, which connects to the xArm
        hardware via the XArmAPI interface.

        Examples:
            >>> robotics_ns.connect_xArm()
        """
        self.emit("connect_xArm")
        logger.info("Connecting xArm to HTeVOLVER server")

    def disconnect_xArm(self):
        """Request the server to disconnect to the xArm.

        Asks the server to disconnect with the robotic arm.
        Triggers arm.disconnect() on the server, which disconnects the xArm
        hardware via the XArmAPI interface.

        Examples:
            >>> robotics_ns.disconnect_xArm()
        """
        self.emit("disconnect_xArm")
        logger.info("Disconnecting xArm from HTeVOLVER server")

    def enable_pumps(self):
        """Request the server to connect to syringe pumps.

        Asks the server to enable PipetteHead syringe pumps. Triggers
        pipette_head.enable() on the server, which enables execution of
        syringe pump commands.

        Examples:
            >>> robotics_ns.enable_pumps()
        """

        self.emit("enable_pumps")
        logger.info("Enabling PipetteHead syringe pumps on HTeVOLVER server")

    def disable_pumps(self):
        """Request the server to connect to syringe pumps.

        Asks the server to disable PipetteHead syringe pumps. Triggers
        pipette_head.disable() on the server, which disables execution of
        syringe pump commands.

        Examples:
            >>> robotics_ns.disable_pumps()
        """

        self.emit("disable_pumps")
        logger.info("Disabling PipetteHead syringe pumps on HTeVOLVER server")

    def override_status(self, override_commands: dict):
        """Override the robotics status on the server.

        Allows manual intervention to change the robotics state.
        Dynamically sets attributes based on the provided
        dictionary, allowing recovery from error states.

        Args:
            override_commands (dict): Dictionary of status values to override.
                Keys should match attributes in the RoboticsServerNamespace class.

        Examples:
            >>> robotics_ns.override_status({"state": RoboticsState.READY.value})
        """
        self.emit("on_override_status", override_commands)
        logger.info(f"Overriding robotics namespace state on HTeVOLVER server with: {override_commands}")

    def pause_experiment(self):
        """Pause the current experiment.

        Requests the server to pause the current robotics operation.
        Triggers pause_robotics() on the server, which:
        1. Sets the robotics state to PAUSE if currently BUSY
        2. Pauses the PipetteHead by terminating current commands
        3. Pauses the xArm by setting its state to PAUSE (3)

        Examples:
            >>> robotics_ns.pause_experiment()
        """
        self.emit("pause_robotics")
        logger.info("Paused experiment")

    def resume_experiment(self):
        """Resume the paused experiment.

        Requests the server to resume a previously paused robotics operation.
        Triggers resume_robotics() on the server, which:
        1. Sets the robotics state back to BUSY if previously PAUSE
        2. Resumes the PipetteHead operations
        3. Resumes the xArm by setting its state to RUNNING (0)

        Examples:
            >>> robotics_ns.resume_experiment()
        """
        self.emit("resume_robotics")
        logger.info("Resumed experiment")

    def stop_experiment(self):
        """Stop the current experiment.

        Requests the server to completely stop the current robotics operation.
        Triggers stop_robotics() on the server, which:
        1. Sets the robotics state to STOP
        2. Terminates all syringe pump commands
        3. Puts the xArm into stop state
        4. Any active routines will detect this state and safely exit

        Examples:
            >>> robotics_ns.stop_experiment()
        """
        self.emit("stop_robotics")
        logger.info("Stopped experiment")

    def check_ready(self):
        """Check if the robotics system is ready for a new routine.

        Verifies that the robotics system is not currently executing a routine.
        Blocks until a status acknowledgment is received from the server.

        Returns:
            bool: True if the system is ready (no active routine), False otherwise.

        Examples:
            >>> if robotics_ns.check_ready():
            ...     robotics_ns.pipette(pipette_commands)
        """
        self.ack = False
        self.request_robotics_status()
        while not self.ack:
            time.sleep(0.1)

        routine = RoboticsRoutines[self.status.robotics["routine"][1]]
        state = RoboticsState[self.status.robotics["state"][1]]
        if routine == RoboticsRoutines.NO_ROUTINE and state == RoboticsState.READY:
            return True
        else:
            return False

    @routine_decorator
    def pipette(self, pipette_commands: dict):
        """Execute a pipetting routine.

        Requests the server to perform a pipetting operation with the PipetteHead.
        Triggers the @routine_decorator(RoboticsRoutines.PIPETTE) decorated function
        on the server, which:
        1. Sets the state to BUSY
        2. Executes pipette_event() with the provided volumes
        3. Coordinates the xArm movement and PipetteHead aspirate/dispense operations

        Args:
            pipette_commands (dict): Dictionary mapping pump indices to fluid type and volume tuples.
                The keys are pump indices (0-3) and values are tuples of (fluid_type, volume_μl).

        Examples:
            >>> robotics_ns.pipette({0: ("MEDIA", 100), 2: ("DRUG", 50)})
        """
        self.emit("pipette_routine", pipette_commands)

    @routine_decorator
    def prime_syringe_pumps(self, prime_commands: list[int]):
        """Prime the syringe pumps.

        Requests the server to prime the specified syringe pumps, filling the
        tubing with the appropriate fluid. Triggers the @routine_decorator(RoboticsRoutines.PRIMING_INFLUX)
        decorated function on the server, which:
        1. Sets the state to BUSY
        2. Calls pipette_head.prime() for each specified pump
        3. Primes the pumps by drawing fluid from reservoirs and dispensing

        Args:
            prime_commands (list): List containing pump IDs to prime

        Examples:
            >>> robotics_ns.prime_syringe_pumps([0, 2])  # Prime pumps 0 and 2
        """
        self.emit("prime_pumps", prime_commands)

    @routine_decorator
    def dilutions(self, dilution_commands: dict):
        """Execute a dilution routine.

        Requests the server to perform a series of dilutions in specific vials.
        Triggers the @routine_decorator(RoboticsRoutines.DILUTION)
        decorated function on the server, which:
        1. Sets the state to BUSY
        2. Converts commands to the StationPumpCommands format
        3. Executes influx_snake_helper() to perform the dilutions
        4. Moves the xArm and PipetteHead to dispense fluids in a snake pattern

        Args:
            dilution_commands (dict): Nested dictionary mapping:
                - station_id -> vial_id -> fluid_type -> volume
                Example structure: {0: {3: {"MEDIA": 100, "DRUG": 50}}}
                This would add 100μL of MEDIA and 50μL of DRUG to vial 3 in station 0.

        Examples:
            >>> robotics_ns.dilutions({
            ...     0: {  # Station 0
            ...         3: {"MEDIA": 100, "DRUG": 50},  # Vial 3 gets MEDIA and DRUG
            ...         4: {"MEDIA": 150}  # Vial 4 gets only MEDIA
            ...     }
            ... })
        """
        self.emit("dilution_routine", dilution_commands)

    @routine_decorator
    def fill_vials(self, fill_commands: dict):
        """Fill vials with specified fluids.

        Requests the server to fill all vials in specified stations with fluid.
        Triggers the @routine_decorator(RoboticsRoutines.FILLING_VIALS_PUMPS)
        decorated function on the server, which:
        1. Sets the state to BUSY
        2. Validates fluid types and volumes
        3. Creates StationPumpCommands for all vials in the station
        4. Executes influx_snake_helper() to fill all vials

        Args:
            fill_commands (dict): Dictionary mapping station IDs to tuples of (fluid_type, volume_μL).
                This applies the same fluid and volume to ALL vials in the specified stations.

        Examples:
            >>> robotics_ns.fill_vials({
            ...     0: ("MEDIA", 1000),  # All vials in station 0 get 1000μL of MEDIA
            ...     1: ("DRUG", 500)     # All vials in station 1 get 500μL of DRUG
            ... })
        """
        self.emit("fill_vials_routine", fill_commands)


if __name__ == "__main__":
    print("Please run eVOLVER.py instead")
