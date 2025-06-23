import argparse
import logging
import os
import sys

from htevolver.htevolver_client.client import HTEvolverClient
from htevolver.shared import RoboticsRoutines

DEFAULT_STATION_LIST: list[int] = []
DEFAULT_PUMP_LIST: list[int] = []
DEFAULT_VIALS_OD: list[int] = list(range(18))

# Configure client logger (logs to file)
logger = logging.getLogger("htevolver.htevolver_client")
logger.setLevel(logging.INFO)

# Create handlers
file_handler = logging.FileHandler(os.path.join("test_robotics.log"))
file_handler.setLevel(logging.INFO)
logger.addHandler(file_handler)

file_formatter = logging.Formatter(fmt="%(asctime)s - %(name)s - [%(levelname)s] - %(message)s", datefmt="%Y-%m-%d %H:%M:%S")
file_handler.setFormatter(file_formatter)


def get_options():
    description = "CLI tool to run HT-eVOLVER robotics tests"
    parser = argparse.ArgumentParser(description=description, formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    # General arguments
    parser.add_argument(
        "-i",
        "--ip_address",
        action="store",
        required=True,
        help="IP address of eVOLVER to run experiment on.",
    )

    parser.add_argument(
        "-f",
        "--robotic_functions",
        type=int,
        action="store",
        required=True,
        help="Robotic functions to test.",
        choices=RoboticsRoutines._member_names_,
    )

    parser.add_argument(
        "-p",
        "--pump_ids",
        action="store",
        nargs="*",
        type=lambda s: int(s),
        required=False,
        help="List of PipetteHead Pump IDs to use during pipetting and PipetteHead priming (space separated).",
        default=DEFAULT_PUMP_LIST,
    )

    parser.add_argument(
        "-l",
        "--liquid",
        action="store_true",
        required=False,
        help="Include liquid flag to run non-dry tests. Required for pipetting.",
        default=False,
    )

    # influx specific arguments
    parser.add_argument(
        "-q",
        "--station_ids",
        action="store",
        nargs="*",
        type=lambda s: int(s),
        required=False,
        help="List of SmartStation IDs to run influxs over over (space separated).",
        default=DEFAULT_STATION_LIST,
    )

    return parser.parse_args(), parser


def build_pipette_request(pump_list: list[int]) -> dict:
    pipette_commands: dict[int, int] = {}
    for pump_id in pump_list:
        while True:
            try:
                volume_input = int(input(f"Enter pipette volume for PipetteHead Pump_{pump_id} (µL): "))
                if volume_input < 0:
                    logger.error("Invalid pipette volume entered, must be positive.")
                else:
                    pipette_commands[pump_id] = volume_input
                    break
            except ValueError:
                logger.error("Invalid pipette volume input, try again.")
    return pipette_commands


def build_influx_request(use_liquid: bool, station_list: list[int], fluid_types: list[str]) -> dict:
    influx_commands: dict[int, dict[int, dict[str, int]]] = {}
    for station_id in station_list:
        influx_commands[station_id] = {}
        vial_list: list[int] = []

        while True:
            vial_list_input = input(
                f"Enter a space-separated list of vials to dilute for SmartStation:{station_id} OR leave empty to use all 18: "
            )
            if vial_list_input == "":
                vial_list = DEFAULT_VIALS_OD
                break
            try:
                vial_list = [int(vial) for vial in vial_list_input.split(" ")]
                vial_list.sort()
                break
            except ValueError:
                logger.exception("Invalid type entered, try again")

        for vial in vial_list:
            influx_commands[station_id][vial] = {}
            while True:
                fluid_input = input(f"Enter fluid input for SmartStation {station_id}, vial {vial} (space-separated): ")
                fluid_list = fluid_input.split(" ")
                error_found = any(fluid_type not in fluid_types for fluid_type in fluid_list)
                if error_found:
                    logger.warning("Invalid fluid input, try again.")
                else:
                    break

            for fluid in fluid_list:
                while True:
                    try:
                        volume_input = int(input(f"Enter influx volume to use for {fluid}: "))
                        if volume_input < 0:
                            logger.warning("Invalid influx volume entered, must be under positive.")
                        else:
                            influx_commands[station_id][vial][fluid] = volume_input
                            break
                    except ValueError:
                        logger.exception("Invalid type entered, try again")

    return influx_commands


if __name__ == "__main__":
    options, parser = get_options()

    evolver_ip = options.ip_address
    robotic_function = options.robotic_function
    station_list = options.station_list
    pump_list = options.pump_list
    use_liquid = options.liquid

    htevolver_client = HTEvolverClient(evolver_ip, 8081, False, "./test_experiments", station_ids=station_list)
    logger.info(f"Testing HT-eVOLVER {robotic_function} function")

    config = htevolver_client.request_robotics_config()
    fluid_types: list[str] = []
    if config:
        for pump_id, pump_config in config["pipette_head"]["pumps"].items():
            fluid_types.append(pump_config["fluid"])

    else:
        logger.error("Aborting, could not find valid robotics configuration from HT-eVOLVER server.")
        sys.exit()

    logger.info(f"Here is the the current xArm configuration: {config['xArm']}")
    change_config_input: str = ""
    while True:
        change_config_input = input("Do you want to change the xArm configuration? [y/n]: ")
        if change_config_input in ["y", "n"]:
            break

    if change_config_input == "y":
        new_xarm_config: dict = {}
        for config_key, config_value in config["xArm"].items():
            while True:
                try:
                    value_type = type(config_value)
                    new_value_input = input(f"Enter value for xArm parameter {config_key} (old value is {config_value}): ")
                    new_value = value_type(new_value_input)
                    if input(f"Commit {config_key}: {new_value}? [y/n]: ") == "y":
                        new_xarm_config[config_key] = new_value
                        break

                except (ValueError, TypeError):
                    logger.warning(f"Invalid value for {config_key}, try again.")

    if robotic_function == "PIPETTE":
        if use_liquid and pump_list:
            pipette_request = build_pipette_request(pump_list)
            htevolver_client.pipette(pipette_request)
        elif not use_liquid:
            logger.error("Aborting, liquid flag is required for pipetting.")
            sys.exit()
        elif not pump_list:
            logger.error("Aborting, must enter list of PipetteHead Pump IDs for pipetting.")
            sys.exit()

    if robotic_function == "PRIME":
        if use_liquid and pump_list:
            while int(input("Enter number of prime cycles to run (enter 0 to exit): ")) > 0:
                try:
                    prime_cylces_input = int(input("Enter number of prime cycles to run: "))
                    if prime_cylces_input > 0:
                        htevolver_client.prime_pipettehead(pump_list)

                except ValueError:
                    logger.warning("Invalid number of prime cycles entered, try again.")
        elif not use_liquid:
            logger.error("Aborting, liquid flag is required for priming PipetteHead.")
            sys.exit()
        elif not pump_list:
            logger.error("Aborting, must enter list of PipetteHead Pump IDs to prime.")
            sys.exit()

    if robotic_function == "INFLUX":
        pipettehead_config = htevolver_client.request_robotics_config("pipette_head")
        fluid_types: list[str] = []
        if pipettehead_config:
            for pump_id, pump_config in pipettehead_config.items():
                fluid_types.append(pump_config["fluid"])

            logger.info(f"PipetteHead configured with the fluid type: {fluid_types}")

        influx_request = build_influx_request(use_liquid, station_list, fluid_types)
        htevolver_client.influx(influx_request)

    htevolver_client.disconnect()
