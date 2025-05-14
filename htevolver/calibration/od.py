"""Optical density (OD) calibration for the HT-eVOLVER system.

This script performs optical density (OD) calibration for the HT-eVOLVER system. It collects
raw voltage readings from OD sensors, matches them with known OD standards provided by the user,
and fits a sigmoid curve to the data.

The calibration procedure:
1. Prompts the user to enter OD values for each standard
2. Guides the user through placing standards in the correct vials
3. Collects voltage readings for each standard in each vial
4. Fits sigmoid curves to the collected data
5. Saves the calibration data to a JSON file
6. Optionally graphs the calibration curves

The calibration data is also sent to the server for storage and later use by the system
to convert raw voltage readings to calibrated OD values during experiments.
"""

import argparse
import datetime
import json
import logging
import os
import sys

import numpy as np
import socketio
from scipy.optimize import curve_fit

from htevolver.htevolver_client.client import HTEvolverClient
from htevolver.htevolver_client.data_analysis import CalibrationData, GraphCalibration

# Configure logging
logger = logging.getLogger("calibrate_od")
logger.setLevel(logging.INFO)
file_formatter = logging.Formatter(fmt="%(asctime)s - %(name)s - [%(levelname)s] - %(message)s\n", datefmt="%Y-%m-%d %H:%M:%S")
stream_formatter = logging.Formatter(fmt="%(name)s - [%(levelname)s] - %(message)s\n")

# Create handlers
file_handler = logging.FileHandler("/home/pi/logs/calibrate_od.log")
stream_handler = logging.StreamHandler()

# Set formatter for both handlers
file_handler.setFormatter(file_formatter)
stream_handler.setFormatter(stream_formatter)
logger.addHandler(file_handler)
logger.addHandler(stream_handler)


DEFAULT_VIALS_OD = list(range(18))
DEFAULT_NUM_STANDARDS: int = 18
STANDARD_NUM_MIN: int = 3


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


def collect_od_data(
    htevolver_client: HTEvolverClient, vial_list: list[int], station_id: int, num_standards: int
) -> dict[int, CalibrationData]:
    """Collect optical density calibration data for a station.

    Guides the user through placing standards in vials and collects voltage readings.
    The procedure involves a rotating pattern of standards to ensure accurate calibration
    across all vials.

    Args:
        htevolver_client (HTEvolverClient): Client connected to the HT-eVOLVER system.
        vial_list (list[int]): List of vial IDs to calibrate.
        station_id (int): ID of the station to calibrate.
        num_standards (int): Number of standard OD values to use.

    Returns:
        dict[int, CalibrationData]: Dictionary mapping vial IDs to their calibration data,
            including voltage readings, standards, standard deviations, and coefficients.

    Examples:
        >>> client = HTEvolverClient("192.168.1.10", False)
        >>> calibration_data = collect_od_data(client, [0, 1, 2, 3], 0, 5)
    """
    # sort the vial_list prior to begining any processing
    vial_list.sort()

    # create data structures to store standards
    standards = np.zeros(num_standards)
    standards_mask = list(range(num_standards))
    if num_standards != len(vial_list):
        padding = [float("nan")] * (len(vial_list) - num_standards)
        standards_mask.extend(padding)

    # get standards from user
    print("\nEnsure that standards are prepared before continuing")
    for index in range(len(standards)):
        standard_input = None
        while True:
            try:
                standard_input = float(input(f"Enter OD value for standard_{index}: "))
                if standard_input >= 0:
                    break
            except ValueError:
                print("Input a valid float number")
        standards[index] = standard_input

    # initialize data structures
    calibration_data: dict[int, CalibrationData] = {}
    for vial_id in vial_list:
        calibration_data[vial_id] = CalibrationData(
            voltage=np.zeros(num_standards),
            standards=standards,
            standard_deviation=np.zeros(num_standards),
            coefficients=np.zeros(4),
        )

    while True:
        proceed = input(
            f"\nPlace standards in Smart Station vial slots in ascending order according to vial list entered. \nExample, standard_0: {standards[0]} OD600 in vial_slot: {min(vial_list)} & standard_{len(standards) - 1}: {standards[-1]} OD600 in vial_slot: {max(standards_mask)}. \nPress Enter to start procedure: "
        )
        if proceed == "":
            break

    # enter loop which will store the median of 3 broadcast readings and instruct the user to rearrange standards
    for step_num in range(len(vial_list)):
        print(f"\n---- Starting calibration procedure step: {step_num}/{len(vial_list) - 1} ----")

        # Use the generalized collect_voltage_readings function
        voltage_triplets = htevolver_client.get_new_od(station_list=[station_id])

        # triplet data is collected, store median representative voltage value
        for vial_id in calibration_data:
            if not np.isnan(standards_mask[vial_id]):
                data_index = standards_mask[vial_id]
                calibration_data[vial_id].voltage[data_index] = np.nanmedian(voltage_triplets[station_id][vial_id])
                calibration_data[vial_id].standard_deviation[data_index] = np.std(
                    voltage_triplets[station_id][vial_id], dtype=float
                )

        # instruct user to rearrange vials and continue to next step in the procedure
        print(
            f"Done collecting voltage photodiode data, calculating and storing median values for calibration procedure step: {step_num}/{len(vial_list) - 1}"
        )
        while True:
            proceed = input(
                "Rearrange standards by moving up one position and snaking highest standard index to lowest vial position. Press Enter when done: "
            )
            if proceed == "":
                break

        print(f"\nCurrent state of standards mask: {standards_mask}")
        print("Current state of calibration_data structure")
        for vial in calibration_data:
            print(calibration_data[vial])

        # adjust standards_mask for next step
        standards_mask.insert(0, standards_mask.pop())
    return calibration_data


def fit_data(calibration_data: dict[int, CalibrationData], graph: bool = True) -> dict[int, CalibrationData]:
    """Fit sigmoid curves to the collected calibration data.

    Fits a sigmoid function to the relationship between OD standards and voltage readings,
    and optionally visualizes the calibration curves.

    Args:
        calibration_data (dict[int, CalibrationData]): Dictionary mapping vial IDs to their
            calibration data, including voltage readings and standards.
        graph (bool, optional): Whether to create visualization graphs. Defaults to True.

    Returns:
        dict[int, CalibrationData]: Updated calibration data with fitted coefficients.

    Examples:
        >>> calibration_data = collect_od_data(client, [0, 1, 2, 3], 0, 5)
        >>> fitted_data = fit_data(calibration_data)
    """
    print("\nGenerating sigmoid fit for collected OD data...")
    for vial_id, vial_calibration in calibration_data.items():
        # p0 = [62721, 62721, 0, -1]
        # maxfev=1000000000
        print(calibration_data[vial_id])
        coefficients, cov = curve_fit(
            CalibrationData.sigmoid,
            vial_calibration.standards,
            vial_calibration.voltage,
            maxfev=1000000000,
        )
        vial_calibration.coefficients = coefficients
    if graph:
        # calculate the highest value recorded during the calibration for the graph settings
        max_values = np.array([np.max(calibration_data[vial_id].voltage) for vial_id in calibration_data])
        max_value = np.max(max_values)
        grapher = GraphCalibration(
            container_type="vial",
            title="od",
            units="OD600",
            row=3,
            column=6,
            stop=max_value,
            start=0,
            sample_num=500,
        )
        grapher.graph(CalibrationData.sigmoid, calibration_data)

    return calibration_data


if __name__ == "__main__":
    options, parser = get_options()
    evolver_ip = options.ip_address

    if int(options.standard_number) < STANDARD_NUM_MIN:
        print(f"more standards are needed, must be at least {STANDARD_NUM_MIN}")
        sys.exit(2)

    station_list: list[int] = []
    if options.stations != []:
        station_list = options.stations
    else:
        station_list = [0, 1, 2, 3]

    htevolver_client = HTEvolverClient(evolver_ip, 8081, False, station_ids=station_list)
    socketIO_eVOLVER = socketio.Client()
    socketIO_eVOLVER.register_namespace(htevolver_client)
    socketIO_eVOLVER.connect("http://{0}:{1}".format(evolver_ip, 8081))

    # start data collection procedure based on target calibration protocol
    collected_calibration_data = {}
    final_calibration_data = {}

    for station_id in station_list:
        vial_list: list[int] = []
        while True:
            vials = input(
                f"\nEnter list of vials to calibrate for station: {station_id} using spaces OR leave empty to use default. Press enter to continue: "
            )
            if vials == "":
                vial_list = DEFAULT_VIALS_OD
                break
            else:
                try:
                    vial_list = [int(vial) for vial in vials.split(" ")]
                    vial_list.sort()
                    break
                except ValueError:
                    print("Invalid list, try again")
        print(vial_list)
        collected_calibration_data = collect_od_data(htevolver_client, vial_list, station_id, int(options.standard_number))
        final_calibration_data[station_id] = fit_data(collected_calibration_data, True)

    # Generate filename with timestamp
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = os.path.join(htevolver_client.calibration_directory, f"calibration_data_od_{timestamp}.json")

    # Convert data to serializable format
    serialized_calibration_data = CalibrationData.to_json(final_calibration_data)

    # Send calibration data to server for long-term storage
    htevolver_client.send_calibration("od", serialized_calibration_data, timestamp)

    # Write to file in the calibration directory backup on the client
    try:
        with open(filename, "w") as f:
            json.dump(serialized_calibration_data, f, indent=4)
    except TypeError as e:
        print(f"Error serializing data: {e}")
        with open(filename, "w") as f:
            serialized_calibration_data_str = str(serialized_calibration_data)
            f.write(serialized_calibration_data_str)
    print(f"Calibration backup saved to {filename}")

    socketIO_eVOLVER.disconnect()
