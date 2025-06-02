import datetime
import json
import logging
import os
from dataclasses import dataclass, field
from typing import Callable

import matplotlib.pyplot as plt
import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class CalibrationData:
    """Container for calibration data used in HT-eVOLVER.

    Stores and processes voltage readings, standards, calibration coefficients,
    and standard deviations for OD and temperature calibrations.

    Attributes:
        voltage (np.ndarray): Array of raw voltage/ADC readings.
        standards (np.ndarray): Array of reference measurement values.
        coefficients (np.ndarray): Curve fit coefficients for the calibration.
        standard_deviation (np.ndarray): Array of standard deviations for voltage readings.
        complete: (bool) = Whether the calibration process is complete.
        step_num: (int) = Current calibration step number or progress indicator, if calibration is incomplete.
        settings: (dict) = Additional calibration settings or configuration parameters.
    """

    voltage: np.ndarray
    standards: np.ndarray
    coefficients: np.ndarray
    standard_deviation: np.ndarray
    complete: bool = field(default=False)
    step_num: int = field(default=0)
    settings: dict = field(default_factory=dict)

    @staticmethod
    def sigmoid(x: int | float, a: float, b: float, c: float, d: float) -> float:
        """Apply a sigmoid transformation using the four-parameter logistic function.

        Used for OD calibration to fit nonlinear optical density curves.

        Args:
            x (int|float): Input value to transform.
            a (float): Lower asymptote parameter.
            b (float): Upper asymptote parameter.
            c (float): Inflection point parameter.
            d (float): Slope parameter.

        Returns:
            float: Transformed value following the sigmoid curve.

        Examples:
            >>> CalibrationData.sigmoid(0.5, 100, 1000, 0.4, 10)
            550.0
        """
        return a + (b - a) / (1 + (10 ** ((c - x) * d)))

    @staticmethod
    def linear(x: int | float, a: float, b: float) -> float:
        """Apply a linear transformation.

        Used primarily for temperature calibration to convert voltage to temperature.

        Args:
            x (int|float): Input value to transform.
            a (float): Slope parameter.
            b (float): Y-intercept parameter.

        Returns:
            float: Linearly transformed value.

        Examples:
            >>> CalibrationData.linear(1500, -0.02, 70)
            40.0
        """
        return x * a + b

    @staticmethod
    def to_json(data):
        """Convert calibration data to JSON-serializable format.

        Recursively converts numpy arrays and nested structures to standard Python types.

        Args:
            data: Data structure to convert, can contain numpy arrays, dictionaries, lists, etc.

        Returns:
            dict: JSON-serializable version of the input data.

        Examples:
            >>> data = {0: CalibrationData(np.array([1, 2]), np.array([0.1, 0.2]),
            ...                           np.array([0.01, 0.01]), np.array([1.5, 3.0]))}
            >>> CalibrationData.to_json(data)
            {0: {'voltage': [1, 2], 'standards': [0.1, 0.2],
                'coefficients': [1.5, 3.0], 'standard_deviation': [0.01, 0.01]}}
        """
        if isinstance(data, np.ndarray):
            return data.tolist()
        if isinstance(data, bool):
            return data
        elif isinstance(data, dict):
            return {k: CalibrationData.to_json(v) for k, v in data.items()}
        elif isinstance(data, list) or isinstance(data, tuple):
            return [CalibrationData.to_json(item) for item in data]
        elif isinstance(data, CalibrationData):
            return {
                "voltage": CalibrationData.to_json(data.voltage),
                "standards": CalibrationData.to_json(data.standards),
                "coefficients": CalibrationData.to_json(data.coefficients),
                "standard_deviation": CalibrationData.to_json(data.standard_deviation),
                "complete": data.complete,
                "step_num": data.step_num,
                "settings": CalibrationData.to_json(data.settings),
            }
        else:
            return data

    @classmethod
    def to_file(cls, filename: str, calibration_data: dict):
        try:
            with open(filename, "w") as f:
                json.dump(cls.to_json(calibration_data), f, indent=4)
            logger.info(f"Calibration data saved to {filename}")
        except TypeError:
            logger.exception(f"Error serializing data: {calibration_data}", stack_info=True)

            with open(filename, "w") as f:
                serializable_data_str = str(calibration_data)
                f.write(serializable_data_str)
            logger.info(f"Calibration data (as string) saved to {filename}")

    @classmethod
    def from_dict(cls, deserialize_data: dict):
        """Create calibration data objects from a dictionary structure.

        Converts JSON-formatted data back to CalibrationData objects.
        Handles both single-level (station: CalibrationData) and
        nested (station: {vial: CalibrationData}) formats.

        Args:
            deserialize_data (dict): Dictionary containing calibration data.

        Returns:
            dict: Dictionary of CalibrationData objects organized by station (and vial if applicable).

        Raises:
            StopIteration: If deserialize_data is empty.
            KeyError: If the expected fields are missing in the input data.

        Examples:
            >>> data = {
            ...     "0": {
            ...         "voltage": [1500, 1700, 1900],
            ...         "standards": [35.0, 30.0, 25.0],
            ...         "standard_deviation": [2.0, 1.5, 1.0],
            ...         "coefficients": [-0.02, 65.0]
            ...     }
            ... }
            >>> cal_data = CalibrationData.from_dict(data)
            >>> type(cal_data[0])
            <class '__main__.CalibrationData'>
        """
        calibration_data = {}

        first_station = next(iter(deserialize_data.values()))
        is_format_nested = isinstance(first_station, dict) and "voltage" not in first_station

        for key, value in deserialize_data.items():
            station = int(key)

            if is_format_nested:
                # Format: {station: {vial: CalibrationData}}
                calibration_data[station] = {}
                for vial_key, vial_data in value.items():
                    vial = int(vial_key)
                    calibration_data[station][vial] = cls(
                        voltage=np.array(vial_data["voltage"]),
                        standards=np.array(vial_data["standards"]),
                        standard_deviation=np.array(vial_data["standard_deviation"]),
                        coefficients=np.array(vial_data["coefficients"]),
                    )
            else:
                calibration_data[station] = cls(
                    voltage=np.array(value["voltage"]),
                    standards=np.array(value["standards"]),
                    standard_deviation=np.array(value["standard_deviation"]),
                    coefficients=np.array(value["coefficients"]),
                )

        return calibration_data

    @classmethod
    def from_file(cls, filename: str):
        """Load calibration data from a JSON file.

        Args:
            filename (str): Path to the calibration data JSON file.

        Returns:
            dict: Dictionary of CalibrationData objects organized by station (and vial if applicable).

        Raises:
            FileNotFoundError: If the specified file does not exist.
            json.JSONDecodeError: If the file contains invalid JSON.

        Examples:
            >>> cal_data = CalibrationData.from_file("/path/to/calibration_data_temp_2025-04-13.json")
            >>> cal_data[0].coefficients
            array([-38.55302223,  2993.94651471])
        """
        with open(filename, "r") as f:
            deserialize_data = json.load(f)
        return cls.from_dict(deserialize_data)

    @classmethod
    def save_calibration(cls, temporary_calibration_data: dict[str, "CalibrationData"], calibration_directory: str, type: str):
        logger.info("Backing up current state of calibration.")
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = os.path.join(calibration_directory, f"calibration_data_{type}_{timestamp}_INCOMPLETE.json")
        current_calibration_state = cls.to_json(temporary_calibration_data)
        cls.to_file(filename, current_calibration_state)

    @classmethod
    def from_save(cls): ...


@dataclass
class GraphCalibration:
    """Utility for visualizing calibration data and fitted curves.

    Creates calibration graphs with measured points, error bars, and fitted lines.

    Attributes:
        container_type (str): Type of container being calibrated (e.g., "Vial", "Smart Station").
        title (str): Title for the calibration plot.
        units (str): Units for the calibration (e.g., "OD600", "Celsius").
        row (int): Number of rows in the subplot grid.
        column (int): Number of columns in the subplot grid.
        stop (int|float): Maximum value for the x-axis of the plot.
        start (int): Minimum value for the x-axis of the plot (default: 0).
        sample_num (int): Number of points to sample for the fitted curve (default: 500).
    """

    container_type: str
    title: str
    units: str
    row: int
    column: int
    stop: int | float
    start: int = field(default=0)
    sample_num: int = field(default=500)

    def graph(self, func: Callable, calibration_data: dict[str, CalibrationData]):
        """Generate calibration graphs for the provided data.

        Creates a grid of subplots, each showing a calibration curve for one object
        (station or vial), with measured points, error bars, and the fitted curve.

        Args:
            func (Callable): Function to use for curve fitting (e.g., CalibrationData.sigmoid
                or CalibrationData.linear).
            calibration_data (dict[int, CalibrationData]): Dictionary mapping object IDs to
                their corresponding CalibrationData.

        Examples:
            >>> grapher = GraphCalibration(
            ...     container_type="Smart Station",
            ...     title="Temperature",
            ...     units="Celsius",
            ...     row=2,
            ...     column=2,
            ...     stop=3000,
            ...     start=1000
            ... )
            >>> grapher.graph(CalibrationData.linear, calibration_data)
        """
        linear_space = np.linspace(self.start, self.stop, self.sample_num)
        fig, axs = plt.subplots(self.row, self.column)
        fig.suptitle(f"{self.title} Calibration Fits for HT-eVOLVER", fontsize=15)

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
