"""Temperature calibration for the HT-eVOLVER system.

This script performs temperature calibration for the HT-eVOLVER system. It collects
raw voltage readings from temperature sensors, matches them with temperature measurements
taken using an external probe, and fits a linear curve to the data.

The calibration procedure:
1. Guides the user through preparing vials with water for equilibration
2. Establishes a room temperature baseline
3. Gradually sweeps through a range of temperatures (both above and below room temperature)
4. At each temperature setpoint:
   - Sets the uncalibrated temperature
   - Waits for equilibration
   - Prompts the user to measure temperatures with an external probe
   - Collects voltage readings
5. Fits linear curves to the collected data
6. Saves the calibration data to a JSON file
7. Optionally graphs the calibration curves

The calibration data is also sent to the server for storage and later use by the system
to convert raw voltage readings to calibrated temperature values during experiments.
"""

import datetime
import logging
import os
import sys

import numpy as np
from scipy.optimize import curve_fit

from htevolver.calibration.calibration_cli import get_options
from htevolver.htevolver_client.client import HTEvolverClient
from htevolver.htevolver_client.data_analysis import CalibrationData, GraphCalibration

# Constants
MEASURE_VIALS = [0, 5, 8, 9, 12, 17]
MAX_TEMP = 1500
MIN_TEMP = 2500
STANDARD_NUM_MIN = 2
LOGGING_DIR: str = "/home/pi/logs"

# Configure client logger (logs to file)
client_logger = logging.getLogger("htevolver.htevolver_client")
calibration_logger = logging.getLogger("htevolver.calibration")

# Configure calibration logger (logs to console)
calibration_logger.setLevel(logging.INFO)
client_logger.setLevel(logging.INFO)

# Create handlers
file_handler = logging.FileHandler(os.path.join(LOGGING_DIR, "calibrate_temp.log"))
file_handler.setLevel(logging.INFO)
stream_handler = logging.StreamHandler(sys.stdout)
stream_handler.setLevel(logging.INFO)

client_logger.addHandler(file_handler)
calibration_logger.addHandler(stream_handler)

# Set formatter for both handlers
file_formatter = logging.Formatter(fmt="%(asctime)s - %(name)s - [%(levelname)s] - %(message)s", datefmt="%Y-%m-%d %H:%M:%S")
stream_formatter = logging.Formatter(fmt="%(message)s")
file_handler.setFormatter(file_formatter)
stream_handler.setFormatter(stream_formatter)

# Use the calibration logger for this module
logger = calibration_logger


def collect_temperature_measurements(station_list: list[int]):
    """Collect temperature measurements from specified vials using an external probe.

    Prompts the user to measure and input temperature values for each measurement vial
    in each specified station.

    Args:
        station_list (list[int]): List of station IDs to collect measurements for.

    Returns:
        dict[int, np.ndarray]: Dictionary mapping station IDs to arrays of temperature
            measurements in degrees Celsius.

    Examples:
        >>> temperature_measurements = collect_temperature_measurements([0, 1])
        >>> station_0_temps = temperature_measurements[0]  # Array of temperatures for station 0
    """
    temperature_measurements = {station_id: np.zeros(len(MEASURE_VIALS)) for station_id in station_list}

    for station_id in station_list:
        temperature_input = None
        print("\n")
        logger.info(f"Measure vial temperatures for Smart Station:{station_id} with probe to generate temperature standards")
        for position_index, vial_position in enumerate(MEASURE_VIALS):
            while True:
                try:
                    temperature_input = float(
                        input(f"Enter temperature (C) value for vial slot {vial_position} in Smart Station:{station_id}: ")
                    )
                    if temperature_input >= 0 and temperature_input <= 100:
                        validation = input(f"Entered value is {temperature_input}. Do you want to commit this value? [y/n]: ")
                        if validation == "y":
                            break

                except ValueError:
                    logger.exception("Error, must input a valid float number", stack_info=True)
            temperature_measurements[station_id][position_index] = temperature_input

    return temperature_measurements


def collect_temp_data(
    htevolver_client: HTEvolverClient, station_list: list[int], num_standards: int
) -> dict[str, CalibrationData]:
    """Collect temperature calibration data for specified stations.

    Guides the user through the temperature calibration procedure, which includes:
    1. Taking room temperature measurements
    2. Setting a series of temperature setpoints above and below room temperature
    3. Collecting voltage readings and temperature measurements at each setpoint

    Args:
        htevolver_client (HTEvolverClient): Client connected to the HT-eVOLVER system.
        station_list (list[int]): List of station IDs to calibrate.
        num_standards (int): Number of temperature points to use above and below room temperature.

    Returns:
        dict[int, CalibrationData]: Dictionary mapping station IDs to their calibration data,
            including voltage readings, standards, standard deviations, and coefficients.

    Examples:
        >>> client = HTEvolverClient("192.168.1.10", False)
        >>> calibration_data = collect_temp_data(client, [0, 1], 3)
    """
    # initialize data structures
    calibration_data: dict[str, CalibrationData] = {
        f"station_{station_id}": CalibrationData(
            voltage=np.zeros(num_standards),
            standards=np.zeros(num_standards),
            standard_deviation=np.zeros(num_standards),
            coefficients=np.zeros(2),
            complete=False,
            settings={"station_list": station_list, "num_standards": num_standards},
        )
        for station_id in station_list
    }

    # Prepare for room temperature measurements
    for station_id in station_list:
        logger.info(f"Place vials filled with 6mL of water in all vial slots in Smart Station:{station_id}.")
        while True:
            proceed = input("Ready to continue? [y/n]: ")
            if proceed == "y":
                break
    print("\n")
    logger.info("---- Starting room temperature step ----")
    logger.info("Wait for 30-60 mins to allow for room temperature equilibration...")
    while True:
        proceed = input("Ready to continue? [y/n]: ")
        if proceed == "y":
            break

    # Room temperature step
    room_temp_step_num = int(np.ceil(num_standards / 2))
    logger.info("Collecting room temperature voltage readings, do not move vials or exit. Should take about a minute...")
    voltage_triplets = htevolver_client.get_new_temp(station_list)
    logger.info("Done collecting room temperature voltage readings. Prepare to take temperature readings from vials.")
    temperature_measurements = collect_temperature_measurements(station_list)

    logger.info("Storing room temperature voltage and Celsius data.")
    for station_id in station_list:
        station_key = f"station_{station_id}"
        calibration_data[station_key].voltage[room_temp_step_num] = np.nanmedian(voltage_triplets[station_id])
        calibration_data[station_key].standard_deviation[room_temp_step_num] = np.std(voltage_triplets[station_id], dtype=float)
        calibration_data[station_key].standards[room_temp_step_num] = np.mean(temperature_measurements[station_id])

    CalibrationData.save_calibration(
        calibration_data, htevolver_client.evolver.evolver_conf["calibration_cache_directory"], "temp"
    )

    logger.info("Auto-calculating calibration setpoints based on number of temperature standards inputs.")
    for station_id in station_list:
        station_key = f"station_{station_id}"

        # Calculate step size for temperature values above room temperature
        room_temp_voltage = calibration_data[station_key].voltage[room_temp_step_num]
        num_additional_setpoints = num_standards - 3
        # Create list for setpoints above room temperature
        calibration_data[station_key].settings["setpoints"] = []
        calibration_data[station_key].settings["setpoints"].append(MIN_TEMP)

        below_rt_step = (MIN_TEMP - room_temp_voltage) / (num_additional_setpoints / 2)
        above_rt_step = (MAX_TEMP - room_temp_voltage) / (num_additional_setpoints / 2)
        for i in range(int(num_additional_setpoints / 2)):
            calibration_data[station_key].settings["setpoints"].append(MIN_TEMP - (i * below_rt_step))
        calibration_data[station_key].settings["setpoints"].append(room_temp_voltage)

        for i in range(int(num_additional_setpoints / 2)):
            calibration_data[station_key].settings["setpoints"].append(MAX_TEMP + (i * above_rt_step))
        calibration_data[station_key].settings["setpoints"].append(MAX_TEMP)

        # Combine both lists and convert to integers
        logger.info(f"Setpoints for Smart Station {station_id}: {calibration_data[station_key].settings['setpoints']} ")

    # Loop through all temperature setpoints
    for step_num in range(num_standards):
        # Skip room temperature setpoint since we already have it
        if step_num == room_temp_step_num:
            continue
        print("\n")
        logger.info(f"---- Starting temperature sweep step: {step_num}/{num_standards - 1} ----")

        # Set uncalibrated temperature for each station
        temp_commands = [0] * 4
        for station_id in station_list:
            temp_commands[station_id] = calibration_data[f"station_{station_id}"].settings["setpoints"][step_num]
        logger.info(f"Sending setpoints: {temp_commands} to HT-eVOLVER...")
        htevolver_client.evolver.send_command("temp", temp_commands, immediate=True, recurring=True)

        # Wait for equilibration
        logger.info("Wait for 30-60 mins to allow for heat equilibration...")
        while True:
            proceed = input("Ready to continue? [y/n]: ")
            if proceed == "y":
                break

        logger.info(f"Temperature readings voltage readings starting for {step_num}, do not move vials or exit...")
        voltage_triplets = htevolver_client.get_new_temp(station_list)

        logger.info("Done collecting room temperature voltage readings. Prepare to take temperature readings from vials.")
        temperature_measurements = collect_temperature_measurements(station_list)

        # Store data for this temperature point
        logger.info("Done collecting temperature data, calculating and storing median values for calibration procedure step")
        for station_id in station_list:
            station_key = f"station_{station_id}"
            calibration_data[station_key].voltage[step_num] = np.nanmedian(voltage_triplets[station_id])
            calibration_data[station_key].standard_deviation[step_num] = np.std(voltage_triplets[station_id], dtype=float)
            calibration_data[station_key].standards[step_num] = np.mean(temperature_measurements[station_id])
            calibration_data[station_key].step_num = step_num

    CalibrationData.save_calibration(
        calibration_data, htevolver_client.evolver.evolver_conf["calibration_cache_directory"], "temp"
    )

    for station_id in station_list:
        calibration_data[f"station_{station_id}"].complete = True

    return calibration_data


def fit_data(calibration_data: dict[str, CalibrationData], graph: bool = True) -> dict[str, CalibrationData]:
    """Fit linear curves to the collected temperature calibration data.

    Fits a linear function to the relationship between temperature standards and voltage readings,
    and optionally visualizes the calibration curves.

    Args:
        calibration_data (dict[int, CalibrationData]): Dictionary mapping station IDs to their
            calibration data, including voltage readings and standards.
        graph (bool, optional): Whether to create visualization graphs. Defaults to True.

    Returns:
        dict[int, CalibrationData]: Updated calibration data with fitted coefficients.

    Examples:
        >>> calibration_data = collect_temp_data(client, [0, 1], 3)
        >>> fitted_data = fit_data(calibration_data)
    """
    print("\n")
    logger.info("Generating linear fit for collected Temperature data...")
    for station_id in calibration_data:
        coefficients, cov = curve_fit(
            CalibrationData.linear, calibration_data[station_id].standards, calibration_data[station_id].voltage
        )
        calibration_data[station_id].coefficients = coefficients
    if graph:
        max_values = np.array([np.max(calibration_data[station_id].voltage) for station_id in calibration_data])
        max_value = np.max(max_values)
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

    return calibration_data


if __name__ == "__main__":
    options, parser = get_options()
    evolver_ip = options.ip_address

    if options.standard_number < STANDARD_NUM_MIN:
        logger.error(f"More standards are needed, must be at least {STANDARD_NUM_MIN}")
        sys.exit(2)

    station_list = options.stations if options.stations else [0, 1, 2, 3]

    htevolver_client = HTEvolverClient(evolver_ip, 8081, False, "/home/pi/experiments/test", station_ids=station_list)

    # Start data collection procedure
    collected_calibration_data = collect_temp_data(htevolver_client, station_list, int(options.standard_number))
    final_calibration_data = fit_data(collected_calibration_data, True)

    for station_id in final_calibration_data:
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        serialized_calibration_data = CalibrationData.to_json({station_id: final_calibration_data[station_id]})
        htevolver_client.evolver.send_calibration(
            serialized_calibration_data, metadata={"parameter": "temp", "timestamp": timestamp, "station_id": station_id}
        )

    htevolver_client.disconnect()
