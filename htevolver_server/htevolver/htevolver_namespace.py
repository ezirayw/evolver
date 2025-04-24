import json
import logging
import os
import struct
import time
from typing import TypedDict

import serial
import socketio
import yaml

logger = logging.getLogger(__name__)


class EvolverSerialError(Exception):
    pass


class APIError(Exception):
    pass


class SerialCommand(TypedDict):
    param: str
    address: int
    value: list[int]
    request: bool
    acknowledge: bool


class BroadcastData(TypedDict, total=False):
    data: dict[str, list[int]]
    phase: int
    config: str
    ip: str
    timestamp: float


class EvolverNamespace(socketio.AsyncNamespace):
    def __init__(
        self,
        evolver_conf: dict,
        server_ip: str,
        namespace: str = "/evolver",
        evolver_conf_path: str = os.path.join(os.path.expanduser("~"), "evolver_conf.yml"),
        calibrations_dir: str = os.path.join(os.path.expanduser("~"), "calibrations"),
    ):
        super().__init__(namespace)
        self.evolver_conf: dict = evolver_conf
        self.server_ip: str = server_ip
        self.evolver_conf_path: str = evolver_conf_path
        self.calibrations_dir: str = calibrations_dir

        self.command_queue: list[SerialCommand] = []
        self.running_immediate: bool = False
        self.running_broadcast: bool = False
        self.request_tag: int = 0
        self.acknowledge_tag: int = 1
        self.sensor_tag: int = 2
        self.echo_tag: int = 3
        self.config_tag: int = 4

        self.serial_connection: serial.Serial = serial.Serial(
            port=self.evolver_conf["serial_port"],
            baudrate=self.evolver_conf["serial_baudrate"],
            timeout=self.evolver_conf["serial_timeout"],
        )

    async def on_connect(self, sid, environ):
        logger.info("client connected to base eVOLVER server")

    async def on_disconnect(self, sid):
        logger.info("client disconnected from base eVOLVER server")

    async def on_command(self, sid, data):
        logger.info("Received COMMAND")
        param = data.get("param", None)
        value = data.get("value", [])
        immediate = data.get("immediate", False)
        recurring = data.get("recurring", False)

        # Check to see if received command matches a configured parameter, and if so, get the phase config
        command_phase = ""
        for phase in self.evolver_conf["parameters"]:
            exit = False
            for parameter in self.evolver_conf["parameters"][phase]:
                if parameter == param:
                    exit = True
                    command_phase = phase
                    break
            if exit:
                break

        if command_phase == "":
            raise APIError("Received COMMAND does not match valid parameter")

        # Initialize a new SerialCommand with the data received if its an immediate command
        if immediate:
            new_command: SerialCommand = {
                "param": param,
                "address": self.evolver_conf["parameters"][command_phase][param]["address"],
                "value": value,
                "request": True,
                "acknowledge": False,
            }

            self.command_queue.insert(0, new_command)
            logger.info("adding the following immediate command to queue: %s", new_command)
            # convert phase information to int
            phase_num = int(command_phase.split("_")[1])
            if not self.running_broadcast:
                logger.info("running the following immediate command: %s", new_command)
                self.running_immediate = True
                await self.run_commands(phase_num)
                self.running_immediate = False

        # Update the parameter information in active conf dictionary and conf file
        self.evolver_conf["parameters"][command_phase][param]["recurring"] = recurring
        if self.evolver_conf["parameters"][command_phase][param]["value"] is not None:
            self.evolver_conf["parameters"][command_phase][param]["value"] = value
        with open(
            os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__), "conf.yml")),
            "w",
        ) as ymlfile:
            yaml.dump(self.evolver_conf, ymlfile)

    async def on_getcalibration(self, sid, data):
        """Load in desired calibration file and send coefficients back to client."""

        # make sure requested parameter has a valid calibration
        if data.get("param") in self.evolver_conf["valid_calibrations"]:
            # find the calibration file for the requested parameter
            calibration_dir = self.calibrations_dir
            calibration_files = [filename for filename in os.listdir(calibration_dir) if data.get("param") in filename]

            if not calibration_files:
                logger.error(f"No calibration file found for parameter: {data.get('param')}")

            # grab the most recently created calibration file in the directory for that parameter
            calibration_filename = os.path.join(calibration_dir, sorted(calibration_files)[-1])
            calibration_data: dict = {}
            with open(calibration_filename, "r") as file:
                # Load the JSON data from the file
                calibration_data = json.load(file)

            # Send the calibration data back to the client
            await self.emit("receivecalibration", {"parameter": data.get("param"), "calibration": calibration_data}, to=sid)
            logger.info(f"Sent calibration data for {data.get('param')} to client")
        else:
            # Parameter doesn't have valid calibration
            await self.emit("receivecalibration", {"parameter": data.get("param"), "calibration": "error"}, to=sid)
            logger.warning(f"No valid calibration found for parameter: {data.get('param')}")

    def update_conf(self):
        """Updates namespace config by loading the contents of the robotics_conf file."""

        with open(self.evolver_conf_path, "r") as conf:
            self.evolver_conf = yaml.safe_load(conf)

    async def run_commands(self, phase: int):
        data: dict[str, list[int]] = {}
        while len(self.command_queue) > 0:
            command = self.command_queue.pop(0)
            try:
                returned_data = self.serial_communication(command, phase)
                if returned_data is not None:
                    data[command["param"]] = returned_data
            except (
                TypeError,
                ValueError,
                serial.serialutil.SerialException,
                EvolverSerialError,
            ) as e:
                logger.error("EvolverSerialError: %s", e)
        return data

    def cobs_encode(self, data: bytearray) -> bytearray:
        """
        Encode data using Consistent Overhead Byte Stuffing (COBS).

        Args:
            data: The data to encode

        Returns:
            COBS encoded data with a trailing zero byte
        """
        result = bytearray()
        code_index = 0
        result.append(0)  # Placeholder for the first code byte

        for byte in data:
            if byte == 0:
                # Found a zero, update the code byte
                result[code_index] = len(result) - code_index
                # Reset for next code byte
                code_index = len(result)
                result.append(0)  # Placeholder for the next code byte
            else:
                result.append(byte)
                if len(result) - code_index == 0xFF:
                    # Maximum block size reached, add a new code byte
                    result[code_index] = 0xFF
                    code_index = len(result)
                    result.append(0)  # Placeholder for the next code byte

        # Update the last code byte
        result[code_index] = len(result) - code_index

        # Add a zero byte to mark the end of the packet
        result.append(0)

        return result

    def cobs_decode(self, data: bytearray | bytes) -> bytearray:
        """
        Decode COBS-encoded data.

        Args:
            data: COBS-encoded data (without the trailing zero byte)

        Returns:
            The decoded data
        """
        result = bytearray()
        i = 0

        while i < len(data):
            code = data[i]
            if code == 0:
                break  # End of packet

            i += 1
            for j in range(1, code):
                if i < len(data):
                    result.append(data[i])
                    i += 1

            if code < 0xFF and i < len(data):
                result.append(0)

        return result

    def build_packet(self, command: SerialCommand):
        packet = bytearray()
        # Get address from command if it exists, otherwise fetch from config
        address = command["address"]
        value = command["value"]
        request = command["request"]
        acknowledge = command["acknowledge"]

        # Add the address to the header of the packet
        packet.extend(struct.pack("<1B", address))

        # Add the data length to the header of the packet
        packet.extend(struct.pack("<1B", len(value)))

        # Add the type to the packet
        # Check that parameters being sent to arduino match expected values
        if request:
            packet.extend(struct.pack("<1B", self.request_tag))
        if acknowledge:
            packet.extend(struct.pack("<1B", self.acknowledge_tag))

        # Add the data to the packet
        packet.extend(struct.pack(f"<{len(value)}I", *value))

        # Add the checksum byte to the packet
        checksum = sum(packet)
        while checksum > 0xFF:
            checksum = (checksum & 0xFF) + (checksum >> 8)
        checksum = 0xFF - checksum

        # Apply COBS encoding to the packet
        packet.extend(struct.pack("<1B", checksum))
        encoded_packet = self.cobs_encode(packet)
        return encoded_packet

    def serial_communication(self, command: SerialCommand, phase: int):
        self.serial_connection.reset_input_buffer()
        self.serial_connection.reset_output_buffer()
        logger.debug(command)
        packet_send = self.build_packet(command)
        logger.debug("serial write MESSAGE to arduino: %s", packet_send.hex(" "))
        self.serial_connection.write(packet_send)

        # Read until we get a zero byte (end of COBS packet)
        response_bytes = self.serial_connection.read_until(expected=b"\x00")
        logger.debug("serial encoded response from arduino: %s", response_bytes.hex(" "))
        if not response_bytes:
            raise EvolverSerialError("No response received from Arduino")

        # Decode the COBS-encoded response
        try:
            packed_decoded_response = self.cobs_decode(response_bytes)
            logger.debug(
                "serial decoded response from arduino: %s",
                packed_decoded_response.hex(" "),
            )
        except Exception as e:
            logger.error("Error decoding COBS response: %s", str(e))
            raise EvolverSerialError(f"Error decoding COBS response: {str(e)}")

        if len(packed_decoded_response) < 3:  # At least address, length, and type
            logger.error("Response too short: %s", packed_decoded_response.hex(" "))
            raise EvolverSerialError("Response packet too short")

        # Verify the checksum
        calculated_checksum = sum(packed_decoded_response)
        while calculated_checksum > 0xFF:
            calculated_checksum = (calculated_checksum & 0xFF) + (calculated_checksum >> 8)

        if calculated_checksum != 0xFF:
            logger.error(
                "Checksum verification failed: calculated=0x%02x, expected=0x%02x",
                calculated_checksum,
                0xFF,
            )
            raise EvolverSerialError("Checksum verification failed for response packet")

        # ACKNOWLEDGE - send acknowledgment to arduino
        command["request"] = False
        command["acknowledge"] = True
        logger.debug(command)
        packet_ack = self.build_packet(command)
        logger.debug("serial write ACK to arduino: %s", packet_ack.hex(" "))
        self.serial_connection.write(packet_ack)

        # wait for full packet transmission to arduino in a dynamic fashion since packet lengths will vary
        self.serial_connection.flush()

        # Extract response packet header information
        packet_response_data_length = packed_decoded_response[1]
        packet_response_type = packed_decoded_response[2]

        if packet_response_type == self.sensor_tag:
            # Extract data (4 bytes per value) to return
            packet_response_data = []
            for i in range(packet_response_data_length):
                if 3 + i * 4 + 3 <= len(packed_decoded_response):
                    # unpack response data payload using little-endian
                    # intrepret data integers as type int (NOT unsigned int)
                    value = struct.unpack("<i", packed_decoded_response[3 + i * 4 : 3 + i * 4 + 4])[0]
                    packet_response_data.append(value)
            return packet_response_data
        else:
            return None

    def get_evolver_status(self):
        return {
            "running_immediate": self.running_immediate,
            "running_broadcast": self.running_broadcast,
        }

    async def broadcast(self, phase: int):
        # if currently running IMMEDIATE commands exit broadcast function, otherwise continue
        if self.running_immediate:
            return False
        self.running_broadcast = True
        broadcast_data: BroadcastData = {}
        phase_string = f"phase_{phase}"

        # run any IMMEDIATE commands in command_queue
        if len(self.command_queue) > 0:
            logger.info("Running IMMEIDATE commands in command queue")
            await self.run_commands(phase)

        # send the broadcast phase state to arduinos using addresses 0x00 -> 0x03
        for arduino_address in [1, 2, 3]:
            param = f"arduino_{arduino_address}"
            new_command: SerialCommand = {
                "param": param,
                "address": arduino_address,
                "value": [phase],
                "request": True,
                "acknowledge": False,
            }
            self.command_queue.append(new_command)
        await self.run_commands(phase)

        if not self.evolver_conf["parameters"][phase_string]:
            # phase has no direct parameters to regulate skip
            logger.debug("empty phase detected")
            self.running_broadcast = False
            return True

        # after running IMMEDIATE commands, add recurring commands to the command_queue based on the phase of the control loop eVOLVER is in
        for param, config in self.evolver_conf["parameters"][phase_string].items():
            if config["recurring"]:
                new_command: SerialCommand = {
                    "param": param,
                    "address": self.evolver_conf["parameters"][phase_string][param]["address"],
                    "value": config["value"],
                    "request": True,
                    "acknowledge": False,
                }
                self.command_queue.append(new_command)
        # run RECURRING commands that were just added
        broadcast_data["phase"] = phase
        broadcast_data["data"] = await self.run_commands(phase)

        # Build broadcast packet
        broadcast_data["config"] = self.evolver_conf["parameters"][phase_string]
        broadcast_data["ip"] = self.server_ip
        broadcast_data["timestamp"] = time.time()
        logging.info("broadcasting %s", (broadcast_data))
        await self.emit("broadcast", broadcast_data)
        self.running_broadcast = False
        return True
