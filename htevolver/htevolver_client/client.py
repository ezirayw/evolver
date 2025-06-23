import logging
import os
import time

import socketio

from htevolver.exceptions import ClientError
from htevolver.htevolver_client.evolver_namespace_client import EvolverClientNamespace
from htevolver.htevolver_client.robotics_namespace_client import RoboticsClientNamespace
from htevolver.shared import HTEvolverStatus, RoboticsRoutines, RoboticsState

logger = logging.getLogger(__name__)


class HTEvolverClient:
    """Master client for interacting with the HT Evolver system.

    This class manages connections to both the eVOLVER and robotics namespaces,
    allowing access to both hardware components from a single interface.
    Users interact directly with the namespace objects for specific functionality.

    Attributes:
        ip (str): IP address of the eVOLVER server
        port (int): Port number of the eVOLVER server
        save (bool): Whether to save data locally
        directory (str): directory for saving experiment data
        sio (socketio.Client): SocketIO client for communication
        evolver (EvolverClientNamespace): Namespace for eVOLVER control
        robotics (RoboticsClientNamespace): Namespace for robotics control
        connected (bool): Whether the client is connected to the server
    """

    def __init__(
        self,
        ip: str,
        port: int,
        save: bool,
        exp_directory: str,
        station_ids: list[int] = [],
        data_buffer_size: int = 10,
    ):
        self.ip = ip
        self.exp_directory = exp_directory
        self.save = save
        self.station_ids = station_ids or [0, 1, 2, 3]
        self.port = port
        self.data_buffer_size = data_buffer_size

        # Create data directory if it doesn't exist
        if self.save and not os.path.exists(self.exp_directory):
            os.makedirs(self.exp_directory)
            for station_id in self.station_ids:
                station_dir = os.path.join(self.exp_directory, f"station_{station_id}")
                if not os.path.exists(station_dir):
                    os.makedirs(station_dir)

        self.status = HTEvolverStatus(connected=False, start_time=time.time(), elapsed_time=0.0)

        self.evolver = EvolverClientNamespace(
            save=self.save,
            directory=self.exp_directory,
            status=self.status,
            station_ids=self.station_ids,
            data_buffer_size=self.data_buffer_size,
        )
        self.robotics = RoboticsClientNamespace(save=self.save, directory=self.exp_directory, status=self.status)
        self.sio = socketio.Client()
        self.sio.register_namespace(self.evolver)
        self.sio.register_namespace(self.robotics)
        self.sio.connect(f"http://{self.ip}:{self.port}")
        logger.info("Successfully established HT-eVOLVER client instance")

    def connect(self) -> None:
        """Connect to HTeVOLVER.

        Initiates connection to both the eVOLVER and robotics namespaces.
        After connection, it requests robotics status and configuration data.

        Raises:
            ConnectionError: If the connection to the server fails.

        Examples:
            >>> client = HTEvolverClient(ip="192.168.1.10", save=True)
            >>> client.connect()
            >>> # Now the client is connected to the server
        """
        self.robotics._request_robotics_status()
        self.robotics._request_robotics_config()
        self.robotics._connect_xArm()
        self.robotics._enable_influx()

    def disconnect(self) -> None:
        """Disconnect from HTeVOLVER.

        Closes the connection to the server if currently connected.

        Examples:
            >>> client = HTEvolverClient(ip="192.168.1.10", save=True)
            >>> client.connect()
            >>> # Do some operations
            >>> client.disconnect()
            >>> # Connection is now closed
        """
        self.robotics._disconnect_xArm()
        self.robotics._disable_influx()
        self.sio.disconnect()
        logger.info("Disconnected from the HTeVOLVER server")

    def override(self, override_parameter: str, override_data: dict | int):
        """Override parameters in the robotics namespace.

        Sends an override request to the robotics namespace on the server. Allows changing specific states & properties
        of the robotics namespace during runtime. Useful for manual intervention or updating status during culture routines.

        Args:
            override_parameter (str): The robotics namespace parameter to override.
                Must be a valid override parameter (e.g., "state", "config").
            override_data (dict | int): Desired data to override target robotics namespace parameter.

        Raises:
            ClientError: Invalid parameter used for override.

        Examples:
            >>> # Override the state of the robotics system
            >>> client.override("state", 0)
            >>> # Override configuration parameters
            >>> client.override("config", {"pipette_speed": 10})
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

        self.robotics._override({override_parameter: override_data})
        logger.info(f"Sent override command for parameter: {override_parameter} with data: {override_data}")

    def set_calibration(self, station_id: int, calibration_parameter: str, filename: str = ""):
        """Set calibration for a specific station.

        Requests the server to load the specified calibration file for
        the given station.

        Args:
            station_id (int): ID of the station to set calibration for.
            filename (str): Name of the calibration file to load.

        Raises:
            ClientError: Invalid calibration parameter
        Examples:
            >>> client.set_calibration(0, "calibration_data_temp_2025-04-13_04-07-12.json")
        """
        logger.info(f"Setting SmartStation {station_id} with most recent temp calibration.")
        if calibration_parameter not in ["od", "temp"]:
            logger.error(f"Aborting set_calibration, invalid calibration parameter: {calibration_parameter}")
            raise ClientError(f"Aborting, invalid calibration parameter: {calibration_parameter}")
        self.evolver.request_calibration(calibration_parameter, station_id)

    def update_temp(self, new_temp_setpoints: dict[int, int]):
        """Update temperature settings for specified stations.

        Sets the temperature for multiple stations and sends the command to the server.

        Args:
            new_station_temps (dict[int, int]): Dictionary mapping SmartStation IDs to new temperature setpoints.

        Raises:
            ClientError: Invalid temperature setpoint.

        Examples:
            >>> client.update_temp({0: 30, 2: 25})  # Set station 0 to 30°C and station 2 to 25°C
        """
        # TODO convert celsius back into raw value
        for station_id, new_temp in new_temp_setpoints.items():
            if new_temp > 50 or new_temp < 10:
                logger.error(
                    f"Aborting update_temp, invalid temperature value detected for SmartStation {station_id}: {new_temp}"
                )
                raise ClientError(
                    f"Aborting update_temp, invalid temperature value detected for SmartStation {station_id}: {new_temp}"
                )

        temp_command: list[int] = [0] * 4
        for station_id in range(4):
            if station_id in new_temp_setpoints:
                self.evolver.stations[station_id].temp_setting = new_temp_setpoints[station_id]
            temp_command[station_id] = self.evolver.stations[station_id].temp_setting
        self.evolver.send_command("temp", temp_command, immediate=True, recurring=True)

    def update_stir(self, new_stir_speeds: dict[int, float]):
        """Update stirring settings for specified stations.

        Sets the stirring speed for multiple stations and sends the command to the server.
        The input RPM values are converted to raw PWM values by multiplying by 500, the max PWM.

        Args:
            new_station_rpms (dict[int, float]): Dictionary mapping SmartStation IDs to new stir speeds.

        Raises:
            ClientError:
        Examples:
            >>> client.update_stir({0: 0.8, 1: 0.5})  # Set station 0 to 80% and station 1 to 50% of max speed
        """
        for station_id, new_stir in new_stir_speeds.items():
            if new_stir < 0:
                logger.error(f"Aborting update_stir, negative stir speed detected for SmartStation {station_id}: {new_stir}")
                raise ClientError(f"Aborting update_stir, negative stir speed detected for SmartStation {station_id}: {new_stir}")

        stir_command: list[int] = [0] * 4
        for station_id in range(4):
            if station_id in new_stir_speeds:
                self.evolver.stations[station_id].stir_setting = stir_command[station_id]
            stir_speed = int(self.evolver.stations[station_id].stir_setting * 500)
            stir_command[station_id] = stir_speed

        self.evolver.send_command("stir", stir_command, immediate=True, recurring=True)

    def get_temp_data(
        self, station_list: list[int], num_data_points: int = 1
    ) -> dict[int, tuple[tuple[float | int, float], ...]]:
        """Retrieve temperature data for specified stations.

        Gets the most recent temperature data (voltage and transformed values)
        for the specified stations.

        Args:
            station_list (list[int]): List of station IDs to get data for.
            num_data_points (int): Number of recent data points to retrieve. Defaults to 1.

        Returns:
            dict[int, tuple[tuple[float | int, float], ...]]: Dictionary mapping station IDs to
                tuples of (voltage, transformed_value) pairs.

        Examples:
            >>> temp_data = client.get_temp_data([0, 1], num_data_points=3)
            >>> temp_data[0]  # Get data for station 0
            ((1500.0, 30.5), (1502.0, 30.4), (1498.0, 30.6))
        """
        return_data: dict[int, tuple[tuple[float | int, float], ...]] = {}
        for station_id in station_list:
            data_entry: list[tuple[float | int, float]] = []
            for index in range(num_data_points):
                temp_data = self.evolver.stations[station_id].temp_data[-1 - index]
                data_entry.append((temp_data.voltage, temp_data.transformed))
            return_data[station_id] = tuple(data_entry)
        return return_data

    def get_od_data(
        self, station_list: list[int], num_data_points: int = 1
    ) -> dict[int, dict[int, tuple[tuple[float | int, float], ...]]]:
        """Retrieve optical density data for specified stations.

        Gets the most recent OD data (voltage and transformed values)
        for all vials in the specified stations.

        Args:
            station_list (list[int]): List of station IDs to get data for.
            num_data_points (int): Number of recent data points to retrieve. Defaults to 1.

        Returns:
            dict[int, dict[int, tuple[tuple[float | int, float], ...]]]: Nested dictionary mapping
                station IDs to dictionaries of vial IDs to tuples of (voltage, transformed_value) pairs.

        Examples:
            >>> od_data = client.get_od_data([0], num_data_points=2)
            >>> od_data[0][3]  # Get data for station 0, vial 3
            ((2550.0, 0.342), (2555.0, 0.345))
        """
        return_data: dict[int, dict[int, tuple[tuple[float | int, float], ...]]] = {}
        for station_id in station_list:
            return_data[station_id] = {}
            for vial_id in range(18):
                data_entry: list[tuple[float | int, float],] = []
                for index in range(num_data_points):
                    od_data = self.evolver.stations[station_id].od_data[vial_id][-1 - index]
                    data_entry.append((od_data.voltage, od_data.transformed))
                return_data[station_id][vial_id] = tuple(data_entry)
        return return_data

    def get_new_temp(self, station_list: list[int], num_readings: int = 3) -> dict[int, tuple[int | float, ...]]:
        """Collect incoming temperature voltage readings.

        Blocks until the specified number of new temperature readings are collected.

        Args:
            station_list (list[int]): List of station IDs to collect readings from.
            num_readings (int): Number of readings to take for each station. Defaults to 3.

        Returns:
            dict[int, tuple[int | float, ...]]: Dictionary mapping station IDs to tuples of voltage readings.

        Examples:
            >>> temp_readings = client.get_new_temp([0, 1], num_readings=5)
            >>> temp_readings[0]  # Raw voltage readings for station 0
            (1500.0, 1502.0, 1498.0, 1501.0, 1499.0)
        """

        voltage_readings: dict[int, list[int | float]] = {station_id: [] for station_id in station_list}
        read_num: int = 0
        current_counter = self.evolver.broadcast_counter

        while read_num < num_readings:
            time.sleep(0.1)
            if current_counter != self.evolver.broadcast_counter:
                logger.debug(f"New broadcast detected, storing voltage values for read {read_num}")
                for station_id in station_list:
                    new_temp_data = self.evolver.stations[station_id].temp_data[-1].voltage
                    voltage_readings[station_id].append(new_temp_data)
                current_counter = self.evolver.broadcast_counter
                read_num += 1

        return_data = {station_id: tuple(temp_data) for station_id, temp_data in voltage_readings.items()}
        return return_data

    def get_new_od(self, station_list: list[int], num_readings: int = 3) -> dict[int, dict[int, tuple[int | float, ...]]]:
        """Collect incoming optical density voltage readings.

        Blocks until the specified number of new OD readings are collected.

        Args:
            station_list (list[int]): List of station IDs to collect readings from.
            num_readings (int): Number of readings to take for each container. Defaults to 3.

        Returns:
            dict[int, dict[int, tuple[int | float, ...]]]: Nested dictionary mapping station IDs
                to dictionaries of vial IDs to tuples of voltage readings.

        Examples:
            >>> od_readings = client.get_new_od([0], num_readings=3)
            >>> od_readings[0][5]  # Raw voltage readings for station 0, vial 5
            (2550.0, 2552.0, 2548.0)
        """

        voltage_readings: dict[int, dict[int, list[int | float]]] = {}
        read_num: int = 0
        current_counter = self.evolver.broadcast_counter
        for station_id in station_list:
            voltage_readings[station_id] = {vial_id: [] for vial_id in range(18)}

        while read_num < num_readings:
            time.sleep(0.1)
            if current_counter != self.evolver.broadcast_counter:
                logger.debug(f"New broadcast detected, storing voltage values for read {read_num}")
                for station_id in station_list:
                    for vial_id in range(18):
                        new_od_data = self.evolver.stations[station_id].od_data[vial_id][-1].voltage
                        voltage_readings[station_id][vial_id].append(new_od_data)
                current_counter = self.evolver.broadcast_counter
                read_num += 1

        return_data: dict[int, dict[int, tuple[int | float, ...]]] = {}
        for station_id in station_list:
            return_data[station_id] = {vial_id: tuple(od_data) for vial_id, od_data in voltage_readings[station_id].items()}
        return return_data

    def request_robotics_config(self, config_parameter: str = "") -> dict:
        """Request robotics configuration from the server.

        Fetches the current robotics configuration from the server. If a specific
        parameter is requested, returns only that parameter's value.

        Args:
            config_parameter (str): Specific configuration parameter to retrieve.
                Defaults to empty.

        Returns:
            dict: Either the complete robotics configuration dictionary or
                the value of the specified parameter if found, empty dictionary otherwise.

        Examples:
            >>> # Get the entire robotics configuration
            >>> config = client.request_robotics_config()
            >>> # Get a specific configuration parameter
            >>> pipette_config = client.request_robotics_config("pipette_head")
        """
        logger.info("HT-eVOLVER client requesting robotics configuration.")
        self.robotics._request_robotics_config()
        if config_parameter:
            return self.robotics.server_config.get(config_parameter, {})
        else:
            return self.robotics.server_config

    def pipette(self, pipette_commands: dict[int, int]):
        """Execute a pipetting operation.

        Sends a pipette command to the robotics system to aspirate and dispense
        fluids with the specified volumes.

        Args:
            pipette_commands (dict): Map pipette volumes (in μL) to PipetteHead Pump ID keys.

        Examples:
            >>> # Pipette 100μL from pump 0 and 200μL from pump 2
            >>> client.pipette({0:100, 2:200})
        """
        logger.info(f"HT-eVOLVER client sending the following PipetteHead pipettte command: {pipette_commands}")
        self.robotics._pipette(pipette_commands)

    def prime_pipettehead(self, prime_commands: list[int]):
        """Prime the specified syringe pumps.

        Sends a command to prime the specified syringe pumps. Priming fills the tubing for influx usage. For each syringe pump
        specified, priming cycle will pipette set volume for all configured ports to fill tubing lines. Required prior to running
        influx operations.

        Args:
            prime_commands (list): List of PipetteHead Pump IDs to prime.
                Example: [0, 1] to prime pumps 0 and 1.

        Examples:
            >>> # Prime pumps 0 and 1
            >>> client.prime_pipettehead([0, 1])
        """
        logger.info(f"HT-eVOLVER client sending the following PipetteHead prime command: {prime_commands}")
        self.robotics._prime_pipettehead(prime_commands)

    def influx(self, influx_commands: dict):
        """Execute influx in specific vials across SmartStations.

        Sends a influx command to the robotics system to pipette target fluids into specified SmartStation vials.
        Vials can receive influx inputs from any configured PipetteHead syringe pump. Influx volume inputs cannot
        exceed the physical capacity of the syringe pump.

        Args:
            influx_commands (dict): Nested dictionary mapping influx volumes to SmartStation IDs & vial IDs
                to fluid types to volumes. Structure: {station_id: {vial_id: {"FLUID_TYPE": volume}}}.
                Example: {0: {3: {"MEDIA": 100, "DRUG": 50}}} adds 100μL of MEDIA and 50μL of DRUG
                to vial 3 in station 0.

        Examples:
            >>> # Add fluids to multiple vials across stations
            >>> client.influx({
            ...     0: {  # Station 0
            ...         3: {"MEDIA": 100, "DRUG": 50},  # Vial 3 gets MEDIA and DRUG
            ...         4: {"MEDIA": 150}  # Vial 4 gets only MEDIA
            ...     }
            ... })
        """
        logger.info(f"HT-eVOLVER client sending the following influxs command: {influx_commands}")
        self.robotics._influx(influx_commands)

    def influx_ipp(self, influx_commands: dict[int, int]):
        """Execute influx using millifluidic boards for target SmartStations.

        Sends a influx-oriented IPP command to target SmartStations by operating IPPs in reverse. Enables rapid
        filling of SmartStation vials for experiment setup. Requires desired fluid source to be connected to millifluidic
        waste port(s), so not as flexible as using influx() in terms of fluid source multiplexing. Volume cannot exceed
        maximum vial culture capacity of 6mL.

        Args:
            influx_commands (dict): Dictionary mapping SmartStation IDs to desired influx volume (uL).

        Examples:
        >>> # Add 1000uL into all vials in SmartStation:0 and 500uL into all vials in SmartStation:3
            >>> client.efflux({0: 1000, 3: 500})
        """
        for station_id, volume in influx_commands.items():
            if volume > 6000:
                logger.error(
                    f"Aborting influx_ipp, volume greater than vial capacity detected for SmartStation {station_id}: {volume}"
                )
                raise ClientError(
                    f"Aborting influx_ipp, volume greater than vial capacity detected for SmartStation {station_id}: {volume}"
                )
        logger.info(f"HT-eVOLVER client sending the following influxs command: {influx_commands}")
        self.evolver._run_ipps(influx_commands)

    def efflux(self, efflux_commands: dict[int, int]):
        """Execute efflux for target SmartStations

        Sends an IPP command to target SmartStations to run efflux. Efflux volume is the same across all vials for the specified
        SmartStation.

        Args:
            efflux_commands (dict): Dictionary mapping SmartStation IDs to desired efflux volume (uL).

        Examples:
            >>> # Remove 1000uL from all vials in SmartStation:0 and 500uL from all vials in SmartStation:3
            >>> client.efflux({0: 1000, 3: 500})
        """
        for station_id, volume in efflux_commands.items():
            if volume < 0:
                logger.error(f"Aborting efflux, negative volume detected for SmartStation {station_id}: {volume}")
                raise ClientError(f"Aborting efflux, negative volume detected for SmartStation {station_id}: {volume}")
        self.evolver._run_ipps(efflux_commands)

    def pause(self):
        """Pause active robotic routines on HT-eVOLVER.

        Sends a pause request to suspend routines. Useful for facilitating manual interventions during experiments,
        like exchanging fluid reservoirs, culture sampling, and/or troubleshooting.

        Examples:
            >>> client.pause()
        """
        self.robotics._pause()

    def resume(self):
        """Resumes recently paused robotic routines on HT-eVOLVER.

        Sends a resume request to resume paused routines. Useful for facilitating manual interventions during experiments,
        like exchanging fluid reservoirs, culture sampling, and/or troubleshooting.

        Examples:
            >>> client.resume()
        """
        self.robotics._resume()

    def stop(self):
        """Kills active robotics routines on HT-eVOLVER.

        Sends a stop request to gracefully exit active robotics routines. Useful for conditionally ending experiments.

        Examples:
            >>> client.stop()
        """
        self.robotics._stop()
