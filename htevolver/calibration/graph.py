import logging

import numpy as np

from htevolver.calibration.calibration_cli import get_options
from htevolver.htevolver_client.data_analysis import CalibrationData, GraphCalibration

logger = logging.getLogger(__name__)


if __name__ == "__main__":
    options, parser = get_options()

    data_filepath = options.calibration_file
    calibration_data = CalibrationData.from_file(data_filepath)
    calibration_type = ""

    logger.info(f"Loaded calibration data: {calibration_data}")

    max_values = []
    for key, value in calibration_data.items():
        station = int(key)

        if isinstance(value, dict):
            calibration_type = "od"
            for vial_key, vial_data in value.items():
                # For nested dictionaries (in case of OD data)
                vial = int(vial_key)
                max_values.append(np.max(vial_data.standards))

        # encountered temp data
        if isinstance(value, CalibrationData):
            calibration_type = "temp"
            max_values.append(np.max(value.standards))
    max_value = np.max(max_values)

    # graph the calibration curves
    if calibration_type == "temp":
        grapher = GraphCalibration(
            container_type="Smart Station",
            title="Temperature",
            units="Celsius",
            row=2,
            column=2,
            stop=max_value,
            start=0,
            sample_num=500,
        )
        grapher.graph(CalibrationData.linear, calibration_data)

    if calibration_type == "od":
        for station_id, station_calibration_data in calibration_data.items():
            grapher = GraphCalibration(
                container_type="Vial",
                title=f"Smart Station {station_id}",
                units="OD600",
                row=3,
                column=6,
                stop=max_value,
                start=0,
                sample_num=500,
            )
            grapher.graph(CalibrationData.sigmoid, station_calibration_data)
