import argparse
import time

import serial

from htevolver.dependencies import EvolverPacket

DEFAULT_PUMP_TIME: int = 5000  # ms
DEFAULT_OVER_PUMP_TIME: int = 0  # s
DEFAULT_DELAY: int = 2  # s
DEFAULT_VIALS: list[int] = [vial for vial in range(18)]


def get_options():
    description = "CLI tool to test directly test efflux indepedent of server"
    parser = argparse.ArgumentParser(description=description, formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument(
        "-p", "--serial_port", action="store", required=True, help="Serial port that efflux boards are connected to."
    )

    parser.add_argument(
        "-v",
        "--vials",
        action="store",
        nargs="*",
        type=lambda vial: int(vial),
        default=DEFAULT_VIALS,
        help="List of vials to run efflux for. Defaults to all vials for SmartStation 1",
    )

    parser.add_argument(
        "-t",
        "--pump_time",
        action="store",
        default=DEFAULT_PUMP_TIME,
        help="Time to run efflux per vial (milliseconds). Defaults to 20000 ms.",
    )

    parser.add_argument(
        "-e",
        "--over_pump_time",
        action="store",
        default=DEFAULT_OVER_PUMP_TIME,
        help="Time in between efflux events (milliseconds). Defaults to 10000 ms.",
    )

    return parser.parse_args(), parser


if __name__ == "__main__":
    options, parser = get_options()
    serial_connection = serial.Serial(options.serial_port, 9600, timeout=1)
    print(f"Testing efflux on vials{options.vials}")

    for vial in options.vials:
        target_payload: list[int] = [0] * len(options.vials)
        target_payload[vial] = options.pump_time
        packets = EvolverPacket.create_packet("efflux", len(options.vials), "request", target_payload)
        # efflux_start_time = 0
        try:
            print(f"Connected to {options.serial_port}")
            print(f"Sending efflux request for vial {vial} with pump time {options.pump_time}ms")
            serial_connection.write(packets[0])  # request packet
            time.sleep(0.1)
            serial_connection.write(packets[2])  # ack packet
            print(f"Running efflux for vial {vial}...")
            efflux_start_time = time.time()
            read_time = time.time()
            while True:
                if time.time() - efflux_start_time >= (options.over_pump_time + (options.pump_time / 1000) + DEFAULT_DELAY):
                    break
                data = serial_connection.readline()
                print(data)
                if data:
                    filename = f"efflux_vial_{vial}.log"
                    with open(filename, "ab") as f:
                        timestamp = f"{round(time.time() - efflux_start_time, 3)}     ".encode()
                        # f.write(timestamp + data)
                        read_time = time.time()

        except serial.SerialException as e:
            print(f"Error opening serial port {options.serial_port}: {e}")
            exit(1)
