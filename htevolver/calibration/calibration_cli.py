import argparse

DEFAULT_STATION_LIST: list[int] = [0, 1, 2, 3]


def get_calibration_options():
    description = "CLI tool to run HT-eVOLVER calibration modules"
    parser = argparse.ArgumentParser(description=description, formatter_class=argparse.ArgumentDefaultsHelpFormatter)

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
        type=int,
        action="store",
        required=True,
        help="Number of standards to use (e.g. temperature steps or OD references). Must be odd for temperature",
    )

    parser.add_argument(
        "-q",
        "--station_ids",
        action="store",
        nargs="*",
        type=lambda s: int(s),
        required=False,
        help="List of SmartStations IDs to iterate calibration protocol over (space separated).",
        default=DEFAULT_STATION_LIST,
    )

    parser.add_argument(
        "-f",
        "--calibration_file",
        action="store",
        type=argparse.FileType("r"),
        required=False,
        help="Filename that contains serialized CalibrationData representing an incomplete calibration procedure.",
    )

    return parser.parse_args(), parser
