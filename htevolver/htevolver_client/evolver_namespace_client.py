import logging
import os
from collections import deque
from dataclasses import dataclass, field

import numpy as np
import socketio

from htevolver.htevolver_client.data_analysis import CalibrationData
from htevolver.shared import BroadcastData, HTEvolverStatus

logger = logging.getLogger(__name__)


@dataclass
class EffluxBoard:
    """Represents the efflux board of a Smart Station.

    Manages the peristaltic pump frequency and fluid volume conversion.

    Attributes:
        ipp_frequency (int): The frequency of the integrated peristaltic pump in Hz.
            Defaults to 5.
    """

    ipp_frequency: int = field(default=5)  # Hz

    def transform(self, target_volume: int) -> int:
        """Convert the target volume to an actuation duration.

        Args:
            target_volume (int): The volume in milliliters to pump.

        Returns:
            int: Duration in milliseconds for which to actuate the pump.

        Examples:
            >>> efflux_board = EffluxBoard(ipp_frequency=5)
            >>> efflux_board.transform(10)
            30
        """
        return 30


@dataclass
class SensorData:
    """Container for raw and transformed sensor readings.

    Stores both the raw voltage/ADC reading and the calibrated/transformed value.

    Attributes:
        voltage (int|float): Raw voltage or ADC reading. Defaults to np.nan.
        transformed (float): Calibrated/transformed measurement value (e.g., temperature in °C or OD600).
            Defaults to np.nan.
    """

    voltage: int | float = field(default=np.nan)
    transformed: float = field(default=np.nan)

    def convert(self) -> tuple[int | float, float]:
        """Convert the sensor data to a tuple.

        Returns:
            tuple[int|float, float]: Tuple of (voltage, transformed).

        Examples:
            >>> sensor_data = SensorData(voltage=1500, transformed=30.5)
            >>> sensor_data.convert()
            (1500, 30.5)
        """
        return (self.voltage, self.transformed)


@dataclass
class SmartStationClient:
    """Client-side representation of a Smart Station.

    Manages and processes data for a single Smart Station, including temperature and OD readings.

    Attributes:
        id (int): ID of the Smart Station.
        data_buffer_size (int): Maximum number of readings to store.
        left_vials (list[int]): List of vial IDs on the left side of the station.
        right_vials (list[int]): List of vial IDs on the right side of the station.
        efflux_board (EffluxBoard): The efflux board for controlling fluid pumping.
        temp_data (deque[SensorData]): Queue of temperature readings.
        od_data (dict[int, deque[SensorData]]): Dictionary mapping vial IDs to queues of OD readings.
        temp_setting (int): Current temperature setting in raw units. Defaults to 25.
        stir_setting (float): Current stir setting (0.0-1.0, fraction of max speed). Defaults to 0.0.
        temp_cal (CalibrationData): Temperature calibration data.
        od_cal (dict[int, CalibrationData]): Dictionary mapping vial IDs to OD calibration data.
    """

    id: int
    data_buffer_size: int
    left_vials: list[int]
    right_vials: list[int]
    efflux_board: EffluxBoard
    temp_data: deque[SensorData]
    od_data: dict[int, deque[SensorData]]

    temp_setting: int = field(default=25)
    stir_setting: float = field(default=0.0)

    temp_cal: CalibrationData = field(default=None)
    od_cal: dict[int, CalibrationData] = field(default_factory=dict)

    @classmethod
    def create(cls, station_id: int, data_buffer_size: int):
        """Create a new SmartStationClient instance.

        Factory method to create a properly initialized SmartStationClient.

        Args:
            station_id (int): ID of the Smart Station.
            data_buffer_size (int): Maximum number of readings to store.

        Returns:
            SmartStationClient: An initialized SmartStationClient instance.

        Examples:
            >>> station = SmartStationClient.create(0, 100)
        """
        return cls(
            id=station_id,
            data_buffer_size=data_buffer_size,
            left_vials=[0, 1, 2, 6, 7, 8, 12, 13, 14],
            right_vials=[3, 4, 5, 9, 10, 11, 15, 16, 17],
            efflux_board=EffluxBoard(),
            temp_data=deque(maxlen=data_buffer_size),
            od_data={vial_id: deque(maxlen=data_buffer_size) for vial_id in range(18)},
        )

    def process_broadcast_data(self, broadcast_data: BroadcastData):
        """Process incoming broadcast data from the server.

        Updates temperature and OD data based on received broadcast data.

        Args:
            broadcast_data (BroadcastData): The broadcast data received from the server.

        Examples:
            >>> station = SmartStationClient.create(0, 100)
            >>> station.process_broadcast_data(broadcast_data)
        """
        logger = logging.getLogger(__name__)
        new_temp_entry: SensorData = SensorData()
        new_temp_entry.voltage = broadcast_data.data["temp"][self.id]
        if self.temp_cal:
            new_temp_entry.transformed = self.temp_cal.linear(
                new_temp_entry.voltage,
                *self.temp_cal.coefficients,
            )
            logger.debug(f"Transformed temp for station_{self.id}")
        else:
            logger.debug(f"Tried to transform temperature voltage for station_{self.id} but no calibration found")
        self.temp_data.append(new_temp_entry)

        for index in range(9):
            new_left_od_entry = SensorData()
            new_right_od_entry = SensorData()

            left_vial_id = self.left_vials[index]
            right_vial_id = self.right_vials[index]

            new_left_od_entry.voltage = broadcast_data.data["od_90_left"][index + 9 * self.id]
            new_right_od_entry.voltage = broadcast_data.data["od_90_right"][index + 9 * self.id]

            if self.od_cal:
                new_left_od_entry.transformed = self.od_cal[left_vial_id].sigmoid(
                    new_left_od_entry.voltage, *self.od_cal[left_vial_id].coefficients
                )
                new_right_od_entry.transformed = self.od_cal[right_vial_id].sigmoid(
                    new_right_od_entry.voltage, *self.od_cal[right_vial_id].coefficients
                )
                logger.debug(f"Transforming od_left for station_{self.id}")
            self.od_data[left_vial_id].append(new_left_od_entry)
            self.od_data[right_vial_id].append(new_right_od_entry)

        if not self.od_cal:
            logger.debug(f"Tried to transform OD voltages for station_{self.id} but no calibration found")


class EvolverClientNamespace(socketio.ClientNamespace):
    """Client namespace for communicating with the eVOLVER server.

    Handles communication with the eVOLVER namespace of the server, including
    receiving broadcast data, sending commands, and managing calibration data.

    Attributes:
        save (bool): Whether to save data locally.
        directory (str): Directory for saving data.
        status (HTEvolverStatus): Status of the HT Evolver system.
        data_buffer_size (int): Maximum number of readings to store.
        broadcast_counter (int): Counter for broadcast events.
        stations (dict[int, SmartStationClient]): Dictionary mapping station IDs to SmartStationClient instances.
    """

    def __init__(
        self,
        save: bool,
        directory: str,
        status: HTEvolverStatus,
        namespace: str = "/evolver",
        station_ids: list[int] = [0, 1, 2, 3],
        data_buffer_size: int = 100,
    ):
        super().__init__(namespace)
        self.save: bool = save
        self.directory: str = directory
        self.status: HTEvolverStatus = status
        self.data_buffer_size: int = data_buffer_size
        self.broadcast_counter = 0
        self.stations: dict[int, SmartStationClient] = {}
        for station_id in station_ids:
            self.stations[station_id] = SmartStationClient.create(station_id, data_buffer_size)

        self.evolver_conf = {}

    def on_connect(self):
        """Handle connection to the server."""
        self.request_conf()
        logger.info("Client connected to HTeVOVLER server via eVOLVER namespace")

    def on_disconnect(self, *args):
        """Handle disconnection from the server."""
        logger.info("Client disconnected from HTeVOLVER server via eVOLVER namespace")

    def on_reconnect(self, *args):
        """Handle reconnection to the server."""
        logger.info("Client reconnected to HTeVOLVER server via eVOLVER namespace")

    def on_broadcast(self, data: dict):
        """Handle broadcast data from the server.

        Processes broadcast data received from the server, updating
        station data and saving to disk if enabled.

        Args:
            data (dict): Broadcast data received from the server.
        """
        broadcast_data = BroadcastData(**data)
        logger.info(f"eVOLVER namespace broadcast: {broadcast_data}")
        if broadcast_data.phase == 1 and broadcast_data.validate():
            for station in self.stations.values():
                station.process_broadcast_data(broadcast_data)

            if self.save:
                self.save_data()

            self.broadcast_counter += 1

    def on_get_conf(self, data):
        """Handle server configuration data"""

        self.evolver_conf = data
        logger.info("Received server configuration data.")

    def on_get_calibration(self, data):
        """Handle calibration data received from the server.

        Processes calibration data received from the server, updating
        station calibration data.

        Args:
            data (dict): Calibration data received from the server.
        """
        if data["calibration_data"]:
            calibration_data = CalibrationData.from_dict(data["calibration_data"])
            if data["parameter"] == "temp":
                self.stations[data["station_id"]].temp_cal = calibration_data[data["station_id"]]
            if data["parameter"] == "od":
                self.stations[data["station_id"]].od_cal = calibration_data[data["station_id"]]

    def request_conf(self):
        """Request eVOLVER server configuration"""
        logger.info("Requesting current server configuration data")
        self.emit("request_conf")

    def request_calibration(self, parameter: str, station_id: int, filename: str):
        """Request calibration data from the server.

        Asks the server to send calibration data for a specific parameter and station.

        Args:
            parameter (str): Type of calibration data ("od" or "temp").
            station_id (int): ID of the station.
            filename (str): Name of the calibration file to request.

        Examples:
            >>> evolver_ns.request_calibration("temp", 0, "calibration_data_temp_2025-04-13_04-07-12.json")
        """
        logger.info(f"Requesting {parameter} calibration data for Smart Station {station_id}")
        self.emit("request_calibration", {"parameter": parameter, "station_id": station_id, "filename": filename})

    def send_calibration(self, serialized_calibration_data: dict, metadata: dict):
        """Send calibration data to the server.

        Transmits calibration data to the server for storage.

        Args:
            serialized_calibration_data (dict): Serialized calibration data to send to the server.
            metadata (dict): Dictionary containing information describing the calibration data,
                such as station_id, parameter type, and timestamp.

        Examples:
            >>> evolver_ns.send_calibration(
            ...     {"voltage": [...], "standards": [...], "coefficients": [...], "standard_deviation": [...]},
            ...     {"station_id": 0, "parameter": "temp", "timestamp": "2025-04-13_04-07-12"}
            ... )
        """
        self.emit("get_calibration", {"data": serialized_calibration_data, "metadata": metadata})
        logger.info(
            f"Recently generated {metadata['parameter']} calibration data for Smart Station {metadata['station_id']} sent to server."
        )

    def save_data(self):
        """Save the current sensor data to disk.

        Writes the latest temperature and OD readings to text files.
        """
        for station_id, station in self.stations.items():
            parent_dir = f"station_{station.id}"
            temp_filepath = os.path.join(self.directory, parent_dir, f"station_{station.id}_temp.txt")
            with open(temp_filepath, "a+") as text_file:
                text_file.write(
                    f"{self.status.elapsed_time}_{station.temp_data[-1].voltage}_{station.temp_data[-1].transformed}\n"
                )

            for vial_id in range(18):
                vial_filename = f"station_{station.id}_vial_{vial_id}_od.txt"
                vial_filepath = os.path.join(self.directory, parent_dir, vial_filename)
                with open(vial_filepath, "a+") as text_file:
                    text_file.write(
                        f"{self.status.elapsed_time}_{station.od_data[vial_id][-1].voltage}_{station.od_data[vial_id][-1].transformed}\n"
                    )
        logger.debug("Recent broadcast data saved to memory")

    def send_command(
        self,
        parameter: str,
        values: list[int],
        immediate: bool,
        recurring: bool,
    ):
        """Send a low level command to the server.

        Transmits a control command to the eVOLVER server. Commands must be
        untransformed for proper intrepretation. Use high-level commands to interface
        with transformed sensor/effector values.

        Args:
            parameter (str): The parameter to control (e.g., "temp", "stir").
            values (list[int]): List of values for the parameter.
            immediate (bool): Whether the command should be executed immediately.
            recurring (bool): Whether the command should be recurring.

        Examples:
            >>> evolver_ns.send_command("temp", [1800, 1900, 1850, 2000], True, True)
        """
        command = {"param": parameter, "values": values, "immediate": immediate, "recurring": recurring}

        self.emit("command", command)
        logger.info(f"Following command sent to the server via the eVOLVER namespace: {command}")

    def change_ipp_frequency(self, frequency_commands: dict[int, int]):
        """Update EffluxBoard frequency configurations.

        Args:
            frequency_commands (dict[int, int]): Dictionary mapping station IDs to frequency values (Hz).

        Examples:
            >>> evolver_ns.change_ipp_frequency({0: 10, 2: 5})  # Set station 0 to 10 Hz and station 2 to 5 Hz
        """

        for station_id, frequency in frequency_commands.items():
            self.stations[station_id].efflux_board.ipp_frequency = frequency

    def run_ipps(self, ipp_commands: dict[int, int]):
        """Actuate integrated peristaltic pumps.

        Actuates the IPPs to pump desired volumes across one or more SmartStations.
        Positive values indicate influx behavior while negative values indicate efflux behavior.

        Args:
            ipp_commands (dict[int, int]): Dictionary mapping station IDs to volumes (mL).

        Examples:
            >>> evolver_ns.run_ipps({0: 10, 2: -5})  # Pump 10mL in for station 0, remove 5mL from station 2
        """
        polarity_commands = [1] * 4
        duration_commands = [0] * 4
        for station_id, volume in ipp_commands.items():
            if volume < 0:
                polarity_commands[station_id] = -1
            duration_commands[station_id] = self.stations[station_id].efflux_board.transform(volume)

        self.send_command("ipp_polarity", polarity_commands, True, False)
        self.send_command("ipp", duration_commands, True, False)
