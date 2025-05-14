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

import argparse
import datetime
import json
import logging
import os
import sys

import numpy as np
from scipy.optimize import curve_fit

from htevolver.htevolver_client.client import HTEvolverClient
from htevolver.htevolver_client.data_analysis import CalibrationData, GraphCalibration

# Create logging instance
logger = logging.getLogger(__name__)


# Constants
MEASURE_VIALS = [0, 5, 8, 9, 12, 17]
MAX_TEMP = 1500
MIN_TEMP = 2500
STANDARD_NUM_MIN = 2


def get_options():
    description = "Run an eVOLVER experiment from the command line"
    parser = argparse.ArgumentParser(description=description)

    parser.add_argument(
        "-i",
        "--ip_address",
        action="store",
        required=True,
        help="IP address of eVOLVER to run experiment on.",
    )

    parser.add_argument(
        "-s",
        "--standard_number",
        action="store",
        required=True,
        help="Number of standards to use, defaults to using 18",
    )

    parser.add_argument(
        "-q",
        "--stations",
        action="store",
        nargs="*",
        type=lambda s: int(s),
        required=False,
        help="List of Smart Stations to iterate calibration protocol over (space separated), defaults to all if left blank",
    )

    return parser.parse_args(), parser


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
        logger.info(f"\nMeasure vial temperatures for Smart Station:{station_id} with probe to generate temperature standards")
        for position_index, vial_position in enumerate(MEASURE_VIALS):
            while True:
                try:
                    temperature_input = float(
                        input(f"Enter temperature (C) value for vial slot {vial_position} in Smart Station:{station_id}: ")
                    )
                    if temperature_input >= 0:
                        validation = input(
                            f"Entered value is {temperature_input}. Press enter to commit this value or type 'return' to re-enter a temperature."
                        )
                        if validation == "":
                            break
                        else:
                            continue
                except ValueError:
                    logger.info("Input a valid float number")
            temperature_measurements[station_id][position_index] = temperature_input

    return temperature_measurements


def collect_temp_data(
    htevolver_client: HTEvolverClient, station_list: list[int], num_standards: int
) -> dict[int, CalibrationData]:
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
    calibration_data = {
        station_id: CalibrationData(
            voltage=np.zeros((num_standards * 2) + 3),
            standards=np.zeros((num_standards * 2) + 3),
            standard_deviation=np.zeros((num_standards * 2) + 3),
            coefficients=np.zeros(2),
        )
        for station_id in station_list
    }
    calibration_steps = (num_standards * 2) + 3

    # Prepare for room temperature measurements
    for station_id in station_list:
        while True:
            proceed = input(
                f"Place vials filled with 6mL of water in all vial slots in Smart Station:{station_id}.\nPress Enter to continue."
            )
            if proceed == "":
                break

    while True:
        proceed = input("\nWait for 30 mins to allow for room temperature equilibration...\nPress Enter to continue.")
        if proceed == "":
            break

    # Room temperature step
    room_temp_step_num = int(np.floor(calibration_steps / 2))
    # Collect voltage readings for room temperature
    voltage_triplets = htevolver_client.get_new_temp(station_list)
    # Collect temperature measurements for room temperature
    temperature_measurements = collect_temperature_measurements(station_list)

    # Store room temperature data
    for station_id in station_list:
        calibration_data[station_id].voltage[room_temp_step_num] = np.nanmedian(voltage_triplets[station_id])
        calibration_data[station_id].standard_deviation[room_temp_step_num] = np.std(voltage_triplets[station_id], dtype=float)
        calibration_data[station_id].standards[room_temp_step_num] = np.mean(temperature_measurements[station_id])

    # Calculate setpoints for all temperatures
    setpoints = {}
    for station in station_list:
        above_rt = np.delete(
            np.round(np.linspace(MAX_TEMP, calibration_data[station].voltage[room_temp_step_num], num_standards + 2)),
            -1,
        )  # get rid of room_temp
        below_rt = np.round(np.linspace(calibration_data[station].voltage[room_temp_step_num], MIN_TEMP, num_standards + 2))
        setpoints[station] = np.append(above_rt, below_rt).astype(int)
        logger.info(f"Setpoints for station {station}: {setpoints[station]} ")

    # Loop through all temperature setpoints
    for step_num in range(calibration_steps):
        # Skip room temperature setpoint since we already have it
        if step_num == room_temp_step_num:
            continue

        logger.info(f"\n---- Starting temperature sweep step: {step_num}/{calibration_steps - 1} ----")

        # Set uncalibrated temperature for each station
        temp_commands = [0] * 4
        for station in station_list:
            temp_commands[station] = int(setpoints[station][step_num])
        logger.info(f"Sending setpoints: {temp_commands} to HT-eVOLVER...")
        htevolver_client.evolver.send_command("temp", temp_commands, immediate=True, recurring=True)

        # Wait for equilibration
        while True:
            proceed = input("Wait for 30 mins to allow for heat equilibration...\nPress Enter to continue.")
            if proceed == "":
                break

        # Collect voltage readings
        voltage_triplets = htevolver_client.get_new_temp(station_list)

        # Collect temperature measurements
        temperature_measurements = collect_temperature_measurements(station_list)

        # Store data for this temperature point
        logger.info(
            f"Done collecting voltage temperature data, calculating and storing median values for calibration procedure step: {step_num}/{calibration_steps - 1}"
        )
        for station_id in station_list:
            calibration_data[station_id].voltage[step_num] = np.nanmedian(voltage_triplets[station_id])
            calibration_data[station_id].standard_deviation[step_num] = np.std(voltage_triplets[station_id], dtype=float)
            calibration_data[station_id].standards[step_num] = np.mean(temperature_measurements[station_id])

    return calibration_data


def fit_data(calibration_data: dict[int, CalibrationData], graph: bool = True) -> dict[int, CalibrationData]:
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

    logger.info("\nGenerating linear fit for collected Temperature data...")

    for station in calibration_data:
        coefficients, cov = curve_fit(
            CalibrationData.linear, calibration_data[station].standards, calibration_data[station].voltage
        )
        calibration_data[station].coefficients = coefficients
    if graph:
        max_values = np.array([np.max(calibration_data[station].voltage) for station in calibration_data])
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

    logger.setLevel(logging.DEBUG)

    # Create handlers
    file_handler = logging.FileHandler("/home/pi/logs/calibrate_temp.log")
    file_handler.setLevel(logging.INFO)
    stream_handler = logging.StreamHandler(sys.stdout)
    stream_handler.setLevel(logging.INFO)
    logger.addHandler(file_handler)
    logger.addHandler(stream_handler)

    # Set formatter for both handlers
    file_formatter = logging.Formatter(
        fmt="%(asctime)s - %(name)s - [%(levelname)s] - %(message)s\n", datefmt="%Y-%m-%d %H:%M:%S"
    )
    stream_formatter = logging.Formatter(fmt="%(name)s - [%(levelname)s] - %(message)s\n")
    file_handler.setFormatter(file_formatter)
    stream_handler.setFormatter(stream_formatter)

    if int(options.standard_number) < STANDARD_NUM_MIN:
        logger.info(f"More standards are needed, must be at least {STANDARD_NUM_MIN}")
        sys.exit(2)

    station_list = options.stations if options.stations else [0, 1, 2, 3]

    htevolver_client = HTEvolverClient(evolver_ip, 8081, False, station_ids=station_list)
    logger.info("YOOOOOOOO")
    # Start data collection procedure
    collected_calibration_data = collect_temp_data(htevolver_client, station_list, int(options.standard_number))
    final_calibration_data = fit_data(collected_calibration_data, True)

    # Generate filename with timestamp
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = os.path.join(htevolver_client.calibration_directory, f"calibration_data_temp_{timestamp}.json")

    # Convert data to serializable format
    serializable_data = CalibrationData.to_json(final_calibration_data)

    # Write to file
    try:
        with open(filename, "w") as f:
            json.dump(serializable_data, f, indent=4)
        logger.info(f"Calibration data saved to {filename}")
    except TypeError as e:
        logger.info(f"Error serializing data: {e}")
        # Handle any remaining serialization issues
        with open(filename, "w") as f:
            serializable_data_str = str(serializable_data)
            f.write(serializable_data_str)
        logger.info(f"Calibration data (as string) saved to {filename}")
