import json
import logging
from dataclasses import dataclass, field

import matplotlib.pyplot as plt
import numpy as np
from htevolver_namespace_client import HTEvolverNamespace

logger = logging.getLogger(__name__)


def collect_voltage_readings(
    htevolver_client: HTEvolverNamespace,
    container_list: list[int],
    sensor_type: str,
    station_id=None,
    num_readings=3,
):
    """Collect voltage readings from sensors.

    Args:
        htevolver_client: The HTEvolverNamespace instance
        container_list: List of stations or vials to collect readings from
        sensor_type: Either 'temp' or 'od' to specify the sensor type
        station: Required for 'od' sensor_type to specify the station ID
        num_readings: Number of readings to take for each container

    Returns:
        Dictionary mapping container IDs to voltage reading arrays
    """
    voltage_readings = {container_id: np.zeros(num_readings) for container_id in container_list}

    print(f"{sensor_type.capitalize()} reading starting, do not move vials or exit...")
    current_counter = htevolver_client.broadcast_counter
    read_num = 0

    while read_num < num_readings:
        if current_counter != htevolver_client.broadcast_counter:
            print(f"New broadcast detected, storing voltage values for read {read_num}")

            if sensor_type == "temp":
                for station_id in container_list:
                    voltage_readings[station_id][read_num] = htevolver_client.stations[station_id].temp[-1].voltage
            elif sensor_type == "od":
                if station_id is None:
                    raise ValueError("Station ID must be provided for OD readings")
                for vial_id in container_list:
                    voltage_readings[vial_id][read_num] = htevolver_client.stations[station_id].od[-1][vial_id].voltage

            current_counter = htevolver_client.broadcast_counter
            read_num += 1

    return voltage_readings


@dataclass
class CalibrationData:
    voltage: np.ndarray
    standards: np.ndarray
    coefficients: np.ndarray
    standard_deviation: np.ndarray

    def sigmoid(self, x: int | float, a: float, b: float, c: float, d: float) -> float:
        return a + (b - a) / (1 + (10 ** ((c - x) * d)))

    def linear(self, x: int | float, a: float, b: float) -> float:
        return x * a + b


@dataclass
class GraphCalibration:
    container_type: str
    param: str
    units: str
    row: int
    column: int
    stop: int | float
    start: int = field(default=0)
    sample_num: int = field(default=500)

    def graph(self, func, calibration_data: dict[int, CalibrationData]):
        linear_space = np.linspace(self.start, self.stop, self.sample_num)
        fig, axs = plt.subplots(self.row, self.column)
        fig.suptitle(f"{self.param} Calibration Fits for HT-eVOLVER", fontsize=15)

        row = 0
        col = 0
        for object_id, data in calibration_data.items():
            axs[row, col].set_title(f"{self.container_type}:{object_id}", fontsize=13)
            axs[row, col].set_ylabel("ADC/Voltage", fontsize=12)
            axs[row, col].set_xlabel(f"Reference Units: {self.units}", fontsize=12)
            axs[row, col].scatter(data.standards, data.voltage, s=15, color="black")
            axs[row, col].errorbar(
                data.standards,
                data.voltage,
                yerr=data.standard_deviation,
                fmt="none",
            )
            axs[row, col].plot(linear_space, func(linear_space, *data.coefficients), linewidth=1, color="red")
            axs[row, col].legend(["Measured", "Fit"], fontsize=10, loc="upper right")
            # Increment column, and move to the next row if needed
            col += 1
            if col >= self.column:
                col = 0
                row += 1

        plt.show()


# Convert numpy arrays to lists for JSON serialization
def serialize_data(data):
    if isinstance(data, np.ndarray):
        return data.tolist()
    elif isinstance(data, dict):
        return {k: serialize_data(v) for k, v in data.items()}
    elif isinstance(data, list) or isinstance(data, tuple):
        return [serialize_data(item) for item in data]
    else:
        return data


def load_data(filename: str) -> dict[int, CalibrationData] | dict[int, dict[int, CalibrationData]]:
    try:
        with open(filename, "r") as f:
            deserialize_data = json.load(f)

        # Convert data back to numpy arrays
        calibration_data = {}

        # Check the first station to determine the data format
        first_station = next(iter(deserialize_data.values()))
        is_format_nested = isinstance(first_station, dict) and "voltage" not in first_station

        for key, value in deserialize_data.items():
            station = int(key)

            if is_format_nested:
                # Format: {station: {vial: CalibrationData}}
                calibration_data[station] = {}
                for vial_key, vial_data in value.items():
                    vial = int(vial_key)
                    calibration_data[station][vial] = CalibrationData(
                        voltage=np.array(vial_data["voltage"]),
                        standards=np.array(vial_data["standards"]),
                        standard_deviation=np.array(vial_data["standard_deviation"]),
                        coefficients=np.array(vial_data["coefficients"]),
                    )
            else:
                # Format: {station: CalibrationData}
                calibration_data[station] = CalibrationData(
                    voltage=np.array(value["voltage"]),
                    standards=np.array(value["standards"]),
                    standard_deviation=np.array(value["standard_deviation"]),
                    coefficients=np.array(value["coefficients"]),
                )

        return calibration_data
    except Exception as e:
        print(f"Error loading calibration data: {e}")
        return {}
