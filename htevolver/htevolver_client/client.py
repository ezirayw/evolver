import logging
import os
import time

import socketio

from htevolver.htevolver_client.evolver_namespace_client import EvolverClientNamespace
from htevolver.htevolver_client.robotics_namespace_client import RoboticsClientNamespace
from htevolver.shared import HTEvolverStatus

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
        directory (str): directory for saving data
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
        directory: str = "/home/pi/htevolver/experiments",
        calibration_directory: str = "/home/pi/htevolver/calibration",
        station_ids: list[int] = [],
        data_buffer_size: int = 10,
    ):
        self.ip = ip
        self.directory = directory
        self.calibration_directory = "/home/pi/htevolver/calibration"
        self.save = save
        self.station_ids = station_ids or [0, 1, 2, 3]
        self.port = port
        self.data_buffer_size = data_buffer_size
        self.logger = logging.getLogger(__name__)

        # Create data directory if it doesn't exist
        if self.save and not os.path.exists(self.directory):
            os.makedirs(self.directory)
            for station_id in self.station_ids:
                station_dir = os.path.join(self.directory, f"station_{station_id}")
                if not os.path.exists(station_dir):
                    os.makedirs(station_dir)

        self.status = HTEvolverStatus(connected=False, start_time=time.time(), elapsed_time=0.0)

        self.evolver = EvolverClientNamespace(
            save=self.save,
            directory=self.directory,
            status=self.status,
            station_ids=self.station_ids,
            data_buffer_size=self.data_buffer_size,
        )
        self.robotics = RoboticsClientNamespace(save=self.save, directory=self.directory, status=self.status)
        self.sio = socketio.Client()
        self.sio.register_namespace(self.evolver)
        self.sio.register_namespace(self.robotics)
        self.sio.connect(f"http://{self.ip}:{self.port}")
        self.logger.info("Successfully established HT-eVOLVER client instance")

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
        self.robotics.request_robotics_status()
        self.robotics.request_robotics_conf()
        self.robotics.connect_xArm()
        self.robotics.enable_pumps()

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
        self.robotics.disconnect_xArm()
        self.robotics.disable_pumps()
        self.sio.disconnect()
        self.logger.info("Disconnected from the HTeVOLVER server")
        self.sio.disconnect()

    def set_temp_calibration(self, station_id: int, filename: str):
        """Set temperature calibration for a specific station.

        Requests the server to load the specified calibration file for
        the given station.

        Args:
            station_id (int): ID of the station to set calibration for.
            filename (str): Name of the calibration file to load.

        Examples:
            >>> client.set_temp_calibration(0, "calibration_data_temp_2025-04-13_04-07-12.json")
        """
        self.logger.info(f"Setting the temperature calibration for Smart Station {station_id} with filename: {filename}")
        self.evolver.request_calibration("temp", station_id, filename)

    def update_temp(self, new_station_temps: dict[int, int]):
        """Update temperature settings for specified stations.

        Sets the temperature for multiple stations and sends the command to the server.

        Args:
            new_station_temps (dict[int, int]): Dictionary mapping station IDs to temperature settings.

        Examples:
            >>> client.update_temp({0: 30, 2: 25})  # Set station 0 to 30°C and station 2 to 25°C
        """
        # TODO convert celsius back into raw value
        station_temps: list[int] = [0] * 4
        for station_index in range(4):
            if station_index in new_station_temps:
                self.evolver.stations[station_index].temp_setting = new_station_temps[station_index]
            station_temps[station_index] = self.evolver.stations[station_index].temp_setting
        self.evolver.send_command("temp", station_temps, immediate=True, recurring=True)

    def update_stir(self, new_station_rpms: dict[int, float]):
        """Update stirring settings for specified stations.

        Sets the stirring speed for multiple stations and sends the command to the server.
        The input RPM values are converted to raw PWM values by multiplying by 500, the max PWM.

        Args:
            new_station_rpms (dict[int, float]): Dictionary mapping station IDs to stirring speeds
                (in RPM or fractional units).

        Examples:
            >>> client.update_stir({0: 0.8, 1: 0.5})  # Set station 0 to 80% and station 1 to 50% of max speed
        """
        station_rpms: list[int] = [0] * 4
        for station_index in range(4):
            if station_index in new_station_rpms:
                self.evolver.stations[station_index].stir_setting = new_station_rpms[station_index]
            stir_PWM_value = int(self.evolver.stations[station_index].stir_setting * 500)
            station_rpms[station_index] = stir_PWM_value

        self.evolver.send_command("stir", station_rpms, immediate=True, recurring=True)

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
            for vial_id in range(18):
                data_entry: list[tuple[float | int, float],] = []
                for index in range(num_data_points):
                    od_data = self.evolver.stations[station_id].od_data[vial_id][-1 - index]
                    data_entry.append((od_data.voltage, od_data.transformed))
                return_data[station_id][vial_id] = tuple(data_entry)
        return return_data

    def get_new_temp(
        self, station_list: list[int], num_readings: int = 3, timeout: int = 60
    ) -> dict[int, tuple[int | float, ...]]:
        """Collect incoming temperature voltage readings.

        Blocks until the specified number of new temperature readings are collected
        or until the timeout is reached.

        Args:
            station_list (list[int]): List of station IDs to collect readings from.
            num_readings (int): Number of readings to take for each station. Defaults to 3.
            timeout (int): Maximum time in seconds to wait for readings. Defaults to 60.

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

        self.logger.info("TEMPERATURE readings starting, do not move vials or exit...")
        start_time = time.time()
        while read_num < num_readings:
            current_time = time.time()
            if current_counter != self.evolver.broadcast_counter:
                self.logger.info(f"New broadcast detected, storing voltage values for read {read_num}")
                for station_id in station_list:
                    new_temp_data = self.evolver.stations[station_id].temp_data[-1].voltage
                    voltage_readings[station_id].append(new_temp_data)
                current_counter = self.evolver.broadcast_counter
                start_time = time.time()
                read_num += 1
            if (current_time - start_time) >= timeout:
                self.logger.warning(f"Timeout exceeded on while running get_new_temp() on read number: {read_num}")
        return_data = {station_id: tuple(temp_data) for station_id, temp_data in voltage_readings.items()}
        return return_data

    def get_new_od(
        self, station_list: list[int], num_readings: int = 3, timeout: int = 60
    ) -> dict[int, dict[int, tuple[int | float, ...]]]:
        """Collect incoming optical density voltage readings.

        Blocks until the specified number of new OD readings are collected
        or until the timeout is reached.

        Args:
            station_list (list[int]): List of station IDs to collect readings from.
            num_readings (int): Number of readings to take for each container. Defaults to 3.
            timeout (int): Maximum time in seconds to wait for readings. Defaults to 60.

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

        self.logger.info("OD readings starting, do not move vials or exit...")
        start_time = time.time()
        while read_num < num_readings:
            current_time = time.time()
            if current_counter != self.evolver.broadcast_counter:
                self.logger.info(f"New broadcast detected, storing voltage values for read {read_num}")
                for station_id in station_list:
                    for vial_id in range(18):
                        new_od_data = self.evolver.stations[station_id].od_data[vial_id][-1].voltage
                        voltage_readings[station_id][vial_id].append(new_od_data)
                current_counter = self.evolver.broadcast_counter
                start_time = time.time()
                read_num += 1
            if (current_time - start_time) >= timeout:
                self.logger.warning(f"Timeout exceeded on while running get_new_od() on read number: {read_num}")

        return_data: dict[int, dict[int, tuple[int | float, ...]]] = {}
        for station_id in station_list:
            return_data[station_id] = {vial_id: tuple(od_data) for vial_id, od_data in voltage_readings[station_id].items()}
        return return_data

    def send_calibration(self, parameter: str, calibration_data: dict, timestamp: str):
        """Send calibration data to the server.

        Transmits calibration data to the server for storage and future use.

        Args:
            parameter (str): Type of calibration data ("od" or "temp").
            calibration_data (dict): Calibration data to send.
            timestamp (str): Timestamp to associate with the calibration data.

        Examples:
            >>> client.send_calibration(
            ...     "temp",
            ...     {0: {"voltage": [...], "standards": [...], "coefficients": [...], "standard_deviation": [...]}},
            ...     "2025-04-13_04-07-12"
            ... )
        """
        self.evolver.send_calibration(parameter, calibration_data, timestamp)
