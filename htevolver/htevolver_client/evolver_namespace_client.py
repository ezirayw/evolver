import logging
import os
from dataclasses import asdict, dataclass, field

import numpy as np
import socketio
from data_analysis import CalibrationData

from htevolver.exceptions import EvolverError
from htevolver.shared import BroadcastData, EvolverCommand, HTEvolverStatus

logger = logging.getLogger(__name__)


@dataclass
class EffluxBoard:
    ipp_frequency: int = field(default=5)  # Hz
    primed: bool = field(default=False)

    def transform(self, target_volume: int) -> int:
        """Convert the target volume to a actuation duration based on the IPP calibration"""
        return 30


@dataclass
class SensorData:
    voltage: int | float = field(default=np.nan)
    transformed: float = field(default=np.nan)


@dataclass
class SmartStationClient:
    id: int
    data_window_length: int = field(default=10)
    left_vials: list[int] = field(default_factory=lambda: [0, 1, 2, 6, 7, 8, 12, 13, 14])
    right_vials: list[int] = field(default_factory=lambda: [3, 4, 5, 9, 10, 11, 15, 16, 17])
    efflux_board: EffluxBoard = field(default_factory=EffluxBoard)
    temp: list[SensorData] = field(default_factory=list)
    od: list[list[SensorData]] = field(default_factory=list)
    od_cal: dict[int, CalibrationData] = field(init=False)
    temp_cal: CalibrationData = field(init=False)

    def process_broadcast_data(self, broadcast_data: BroadcastData):
        new_temp_entry: SensorData = SensorData()
        new_temp_entry.voltage = broadcast_data.data["temp"][self.id]
        if not self.temp_cal:
            logger.warning(f"Tried to transform temperature voltage for station_{self.id} but no calibration found")
        else:
            new_temp_entry.transformed = self.temp_cal.linear(
                new_temp_entry.voltage,
                *self.temp_cal.coefficients,
            )
            logger.debug(f"Transformed temp for station_{self.id}")
            self.temp.append(new_temp_entry)
            if len(self.temp) > self.data_window_length:
                self.temp.pop(0)
                logger.debug("Trimmed cached temp data")

        new_od_entry: list[SensorData] = [SensorData() for vial_od in range(18)]
        for index, vial in enumerate(self.left_vials):
            new_od_entry[vial].voltage = broadcast_data.data["od_90_left"][index + 9 * self.id]
            if not self.od_cal:
                logger.warning(f"Tried to transform od_left voltages for station_{self.id} but no calibration found")
            else:
                new_od_entry[vial].transformed = self.od_cal[vial].sigmoid(
                    new_od_entry[vial].voltage, *self.od_cal[vial].coefficients
                )
                logger.debug(f"transforming od_left for station_{self.id}")

        for index, vial in enumerate(self.right_vials):
            new_od_entry[vial].voltage = broadcast_data.data["od_90_right"][index + 9 * self.id]
            if not self.od_cal:
                logger.warning(f"Tried to transform od_right voltages for station_{self.id} but no calibration found")
            else:
                new_od_entry[vial].transformed = self.od_cal[vial].sigmoid(
                    new_od_entry[vial].voltage, *self.od_cal[vial].coefficients
                )
                logger.debug(f"Transformed od_right for station_{self.id}")
        self.od.append(new_od_entry)
        if len(self.od) > self.data_window_length:
            self.od.pop(0)
            logger.debug("Trimmed cached OD data")


class EvolverClientNamespace(socketio.ClientNamespace):
    def __init__(
        self,
        save: bool,
        directory: str,
        status: HTEvolverStatus,
        namespace: str = "/evolver",
        station_ids: list[int] = [0, 1, 2, 3],
        data_window_length: int = 10,
    ):
        super().__init__(namespace)
        self.save: bool = save
        self.directory: str = directory
        self.status: HTEvolverStatus = status
        self.data_window_length: int = data_window_length
        self.broadcast_counter = 0
        self.stations: list[SmartStationClient] = []
        self.address_table: dict = {}
        for station_id in station_ids:
            self.stations.append(SmartStationClient(station_id))

    def on_connect(self, *args):
        logger.info("Client connected to HTeVOVLER server via eVOLVER namespace")

    def on_disconnect(self, *args):
        logger.info("Client disconnected from HTeVOLVER server via eVOLVER namespace")

    def on_reconnect(self, *args):
        logger.info("Client reconnected to HTeVOLVER server via eVOLVER namespace")

    def on_broadcast(self, data: dict):
        try:
            broadcast_data = BroadcastData(**data)

            if broadcast_data.phase == 1:
                for station in self.stations:
                    station.process_broadcast_data(broadcast_data)

                if self.save:
                    self.save_data()

                self.broadcast_counter += 1
            logger.info(f"eVOLVER namespace broadcast processed: {broadcast_data}")
        except TypeError as e:
            logger.warning(f"Error trying to processes eVOLVER namespace broadcast data: {e}")

    def on_get_calibration(self, data): ...

    def on_get_types(self, data): ...

    def on_get_address_table(self, data):
        self.address_table = data

    def request_address_table(self, data): ...

    def request_calibration(self, target_param: str):
        self.emit("request_calibration", target_param)
        logger.info(f"Requesting eVOLVER calibrations for: {target_param}")

    def request_status(self): ...

    def save_data(self):
        # save recent temperature data
        for station in self.stations:
            parent_dir = f"station_{station.id}"
            temp_filepath = os.path.join(self.directory, parent_dir, f"station_{station.id}_temp.txt")
            with open(temp_filepath, "a+") as text_file:
                text_file.write(f"{self.status.elapsed_time}_{station.temp[-1].voltage}_{station.temp[-1].transformed}\n")

            # save recent od data
            for vial_id in range(18):
                vial_filename = f"station_{station.id}_vial_{vial_id}_od.txt"
                vial_filepath = os.path.join(self.directory, parent_dir, vial_filename)
                with open(vial_filepath, "a+") as text_file:
                    text_file.write(
                        f"{self.status.elapsed_time}_{station.od[-1][vial_id].voltage}_{station.od[-1][vial_id].transformed}\n"
                    )
        logger.debug("Recent broadcast data saved to memory")

    def send_command(
        self,
        parameter: str,
        values: list[int],
        immediate: bool,
        recurring: bool,
    ):
        if parameter in self.address_table:
            try:
                command = EvolverCommand(
                    param=parameter, address=self.address_table[parameter], value=values, immediate=immediate, recurring=recurring
                )
                self.emit("command", asdict(command))
                logger.info(f"Following command sent to the server via the eVOLVER namespace: {command}")
            except ValueError as e:
                logger.error(f"Error trying to build valid EvolverCommand: {e}")
        else:
            logger.error(f"Passed parameter is not valid: {parameter}")
            raise EvolverError(f"Passed parameter is not valid: {parameter}")

    def change_ipp_frequency(self, frequency_commands: dict[int, int]):
        """Update EffluxBoard `frequency` configurations.

        Args:
            `frequency`: List of frequency (Hz) configs.
        """

        for station_id, frequency in frequency_commands.items():
            station = next((s for s in self.stations if s.id == station_id), None)
            if station is None:
                logger.error(f"Could not find Smart Station with id:{station_id}")
                return
            station.efflux_board.ipp_frequency = frequency

    def run_ipps(self, ipp_commands: dict[int, int]):
        """Actuate IPPs to pump desired volumes across one or more SmartStations. Positive values indicate influx behavior while negative values indicate efflux behaviour. Set `priming` to True to

        Args:
            `ipp_commands`: contains a dictionary for each Smart Station's IPP command
                `volume`: Volume to pump, mL
                `polarity`: Actuate IPPs in forward (1) or reverse (0).
        """
        polarity_commands = [1] * 4
        duration_commands = [0] * 4
        for station_id, volume in ipp_commands.items():
            station = next((s for s in self.stations if s.id == station_id), None)
            if station is None:
                logger.warning(f"Could not find Smart Station with id:{station_id}")
                return
            if volume < 0:
                polarity_commands[station_id] = -1
            if station.efflux_board.primed:
                duration_commands[station_id] = station.efflux_board.transform(volume)

        self.send_command("ipp_polarity", polarity_commands, True, False)
        self.send_command("ipp", duration_commands, True, False)
