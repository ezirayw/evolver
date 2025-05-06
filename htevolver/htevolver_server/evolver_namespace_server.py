import json
import logging
import os
import struct
import time
from dataclasses import asdict, dataclass

import serial
import socketio
import yaml

from htevolver.exceptions import EvolverError, EvolverSerialError
from htevolver.shared import BroadcastData, CommandTags, EvolverCommand

logger = logging.getLogger(__name__)


@dataclass
class SerialCommand:
    param: str
    address: int
    value: list[int]
    tag: CommandTags


class EvolverServerNamespace(socketio.AsyncNamespace):
    def __init__(
        self,
        evolver_conf: dict,
        evolver_conf_path: str,
        namespace: str = "/evolver",
    ):
        super().__init__(namespace)
        self.evolver_conf: dict = evolver_conf
        self.evolver_conf_path: str = evolver_conf_path
        self.phase: int = 0
        self.command_queue: list[SerialCommand] = []
        self.running_immediate: bool = False
        self.running_broadcast: bool = False

        self.calibrations_directory: str = evolver_conf["calibrations_directory"]
        self.serial_connection: serial.Serial = serial.Serial(
            port=self.evolver_conf["serial_port"],
            baudrate=self.evolver_conf["serial_baudrate"],
            timeout=self.evolver_conf["serial_timeout"],
        )
        self.address_table: dict[str, int] = {}
        self.extract_parameter_addresses()

    async def on_connect(self, sid) -> None:
        """Handles client connection to the server's eVOLVER namespace.

        Args:
            sid (str): Session ID of the connecting client.
            environ (dict): Environment information about the connection.
        """

        logger.info("Client connected to the eVOLVER namespace")

    async def on_disconnect(self, sid):
        """Handles client disconnection from the eVOLVER namespace.

        Args:
            sid (str): Session ID of the disconnecting client.
        """
        logger.info("Client disconnected from the eVOLVER namespace")

    async def on_command(self, sid, command: dict):
        """Processes eVOLVER commands received from clients to control SmartStation parameters.

        Validates commands, updates configurations, and either queues them for
        later processing or executes them immediately based on the immediate flag.

        Args:
            sid (str): Session ID of the client.
            evolver_command (dict): Command data including parameter, value, and flags.
                Example: {"param": "temp", "value": [30, 30, 30, 30], "immediate": True}

        Raises:
            EvolverError: If the command doesn't match a valid parameter.
        """
        logger.info(f"Received the client command: {command}")
        try:
            evolver_command = EvolverCommand(**command)

            # Get the phase info for the parameter command
            command_phase = ""
            for phase in self.evolver_conf["parameters"]:
                exit = False
                for parameter in self.evolver_conf["parameters"][phase]:
                    if parameter == evolver_command.param:
                        exit = True
                        command_phase = phase
                        break
                if exit:
                    break

            if command_phase == "":
                raise EvolverError("Could not find valid phase data for the submitted parameter")

            # Initialize a new SerialCommand with the data received if its an immediate command
            if evolver_command.immediate:
                new_command = SerialCommand(
                    param=evolver_command.param,
                    address=self.evolver_conf["parameters"][command_phase][evolver_command.param]["address"],
                    value=evolver_command.value,
                    tag=CommandTags.REQUEST,
                )

                self.command_queue.insert(0, new_command)
                logger.info(f"Added the following immediate command to queue: {new_command}")

                if not self.running_broadcast:
                    self.running_immediate = True
                    await self.run_commands()
                    logger.info(f"Finished running the immediate command: {new_command}")
                    self.running_immediate = False

            # Update the parameter information in active conf dictionary and conf file
            self.evolver_conf["parameters"][command_phase][evolver_command.param]["recurring"] = evolver_command.recurring
            if self.evolver_conf["parameters"][command_phase][evolver_command.param]["value"] is not None:
                self.evolver_conf["parameters"][command_phase][evolver_command.param]["value"] = evolver_command.value
            self.save_conf()
        except TypeError as e:
            logger.warning(f"Error processing received EvolverCommand: {e}")

    async def on_request_calibration(self, sid, parameter: str):
        """Loads and sends calibration data for the requested parameter.

        Finds the most recent calibration file for the requested parameter and
        sends it back to the requesting client.

        Args:
            sid (str): Session ID of the client.
            data (dict): Request containing the parameter to retrieve calibration for.
                Example: {"param": "od"}
        """
        # make sure requested parameter has a valid calibration
        if parameter in self.evolver_conf["valid_calibrations"]:
            # find the calibration file for the requested parameter
            calibration_dir = self.calibrations_directory
            calibration_files = [filename for filename in os.listdir(calibration_dir) if parameter in filename]

            if not calibration_files:
                await self.emit("get_calibration", {"parameter": parameter, "calibration": ""}, to=sid)
                logger.error(f"No calibration file found for parameter: {parameter}")

            # grab the most recently created calibration file in the directory for that parameter
            calibration_filename = os.path.join(calibration_dir, sorted(calibration_files)[-1])
            calibration_data: dict = {}
            with open(calibration_filename, "r") as file:
                # Load the JSON data from the file
                calibration_data = json.load(file)

            # Send the calibration data back to the client
            await self.emit("get_calibration", {"parameter": parameter, "calibration": calibration_data}, to=sid)
            logger.info(f"Calibration data sent to requesting client : {parameter}")
        else:
            # Parameter doesn't have valid calibration
            await self.emit("get_calibration", {"parameter": parameter, "calibration": "error"}, to=sid)
            logger.warning(f"Invalid parameter received during calibration data request: {parameter}")

    async def on_request_status(self, sid):
        """Responds with the current HTeVOLVER namespace status.

        Emits the current status to the requesting client.

        Args:
            sid (str): Session ID of the requesting client.
        """
        status = {
            "phase": self.phase,
            "command_queue": [asdict(command) for command in self.command_queue],
            "running_immediate": self.running_immediate,
            "running_broadcast": self.running_broadcast,
        }
        await self.emit("get_status", status, to=sid)
        logger.info("Request for current HTeVOLVER status processed.")

    async def on_request_conf(self, sid):
        """Responds with the current HTeVOLVER namespace configuration.

        Args:
            sid (str): Session ID of the requesting client.
        """
        await self.emit("get_conf", self.evolver_conf, to=sid)
        logger.info("Request for current HTeVOLVER configuration processed")

    def extract_parameter_addresses(self):
        """Extract parameter:address mappings from evolver_conf"""

        # Extract from phase_0
        if "phase_0" in self.evolver_conf["parameters"]:
            for param, config in self.evolver_conf["parameters"]["phase_0"].items():
                if "address" in config:
                    self.address_table[param] = config["address"]

        # Extract from phase_1
        if "phase_1" in self.evolver_conf["parameters"]:
            for param, config in self.evolver_conf["parameters"]["phase_1"].items():
                if "address" in config:
                    self.address_table[param] = config["address"]

        # Extract from phase_2 if it exists
        if "phase_2" in self.evolver_conf["parameters"] and self.evolver_conf["parameters"]["phase_2"]:
            for param, config in self.evolver_conf["parameters"]["phase_2"].items():
                if "address" in config:
                    self.address_table[param] = config["address"]

    def load_conf(self):
        """Loads in the HTeVOLVER configuration from memory.

        Reads the current configuration from the file system to ensure operations
        use the most up-to-date settings.
        """
        with open(self.evolver_conf_path, "r") as conf:
            self.evolver_conf = yaml.safe_load(conf)
        logger.debug(f"Following configuration loaded from memory: {self.evolver_conf}")

    def save_conf(self):
        """Saves the internal HTeVOVLER configuration to memory.

        Updates the YAML configuration file with the latest settings
        from the namespace config, ensuring configuration persistence
        across restarts."""

        with open(
            os.path.realpath(self.evolver_conf_path),
            "w",
        ) as conf_file:
            yaml.dump(self.evolver_conf, conf_file)
        logger.debug(f"Following configuration saved to memory:{self.evolver_conf}")

    async def run_commands(self):
        """Executes all commands in the command queue.

        Processes each command in the queue by sending it to the Arduino,
        collecting returned data if available.

        Returns:
            dict: Data returned from the Arduino for each parameter.

        Raises:
            EvolverSerialError: If serial communication with the Arduino fails.
        """
        data: dict[str, list[int]] = {}
        while len(self.command_queue) > 0:
            command = self.command_queue.pop(0)
            try:
                returned_data: list[int] = self.serial_communication(command)
                data[command.param] = returned_data
                logger.debug(f"Processed the following SerialCommand: {command}")
            except (
                TypeError,
                ValueError,
                serial.serialutil.SerialException,
                EvolverSerialError,
            ) as e:
                logger.error(f"EvolverSerialError: {e}")
        return data

    def cobs_encode(self, data: bytearray) -> bytearray:
        """Encodes data using Consistent Overhead Byte Stuffing (COBS).

        Args:
            data: The data to encode

        Returns:
            bytearray: COBS encoded data with a trailing zero byte
        """

        encoded_packet = bytearray()
        code_index: int = 0
        encoded_packet.append(0)  # Placeholder for the first code byte

        for byte in data:
            if byte == 0:
                # Found a zero, update the code byte
                encoded_packet[code_index] = len(encoded_packet) - code_index
                # Reset for next code byte
                code_index = len(encoded_packet)
                encoded_packet.append(0)  # Placeholder for the next code byte
            else:
                encoded_packet.append(byte)
                if len(encoded_packet) - code_index == 0xFF:
                    # Maximum block size reached, add a new code byte
                    encoded_packet[code_index] = 0xFF
                    code_index = len(encoded_packet)
                    encoded_packet.append(0)  # Placeholder for the next code byte

        # Update the last code byte
        encoded_packet[code_index] = len(encoded_packet) - code_index

        # Add a zero byte to mark the end of the packet
        encoded_packet.append(0)

        return encoded_packet

    def cobs_decode(self, data: bytearray | bytes) -> bytearray:
        """Decodes COBS-encoded data.

        Args:
            data: COBS-encoded data (without the trailing zero byte)

        Returns:
            bytearray: The decoded data
        """
        decoded_packet = bytearray()
        i = 0

        while i < len(data):
            code = data[i]
            if code == 0:
                break  # End of packet

            i += 1
            for j in range(1, code):
                if i < len(data):
                    decoded_packet.append(data[i])
                    i += 1

            if code < 0xFF and i < len(data):
                decoded_packet.append(0)

        return decoded_packet

    def build_packet(self, command: SerialCommand) -> bytearray:
        """Builds a packet for serial communication with the Arduino.

        Constructs a properly formatted packet with address, data length,
        command type, data payload, and checksum, then applies COBS encoding.

        Args:
            command (SerialCommand): The command to convert into a packet.

        Returns:
            bytearray: The COBS-encoded packet ready for transmission.
        """
        packet = bytearray()
        # Get address from command if it exists, otherwise fetch from config
        address = command.address
        value = command.value
        tag = command.tag

        # Add the address to the header of the packet
        packet.extend(struct.pack("<1B", address))

        # Add the data length to the header of the packet
        packet.extend(struct.pack("<1B", len(value)))

        # Add the tag to the packet
        packet.extend(struct.pack("<1B", tag.value))

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

    def serial_communication(self, command: SerialCommand) -> list[int]:
        """Handles the full serial communication cycle with the Arduino.

        Sends the command packet to the Arduino, reads and validates the response,
        sends an acknowledgment, and extracts any returned sensor data.

        Args:
            command (SerialCommand): The command to send.

        Returns:
            list[int] or None: Data returned from the Arduino, if any.

        Raises:
            EvolverSerialError: If there's an error in communication or validation.
        """
        self.serial_connection.reset_input_buffer()
        self.serial_connection.reset_output_buffer()
        logger.debug(command)
        packet_send = self.build_packet(command)
        logger.debug(f"Serial write MESSAGE to arduino: {packet_send.hex(' ')}")
        self.serial_connection.write(packet_send)

        # Read until we get a zero byte (end of COBS packet)
        response_bytes = self.serial_connection.read_until(expected=b"\x00")
        logger.debug(f"Serial encoded response from arduino: {response_bytes.hex(' ')}")
        if not response_bytes:
            raise EvolverSerialError("No response received from Arduino")

        # Decode the COBS-encoded response
        try:
            packed_decoded_response = self.cobs_decode(response_bytes)
            logger.debug(f"Serial decoded response from arduino: {packed_decoded_response.hex(' ')}")
        except Exception as e:
            logger.error(f"Error decoding COBS response: {e}")
            raise EvolverSerialError(f"Error decoding COBS response: {e}")

        if len(packed_decoded_response) < 3:  # At least address, length, and type
            logger.error(f"Response too short: {packed_decoded_response.hex(' ')}")
            raise EvolverSerialError("Response packet too short")

        # Verify the checksum
        calculated_checksum = sum(packed_decoded_response)
        while calculated_checksum > 0xFF:
            calculated_checksum = (calculated_checksum & 0xFF) + (calculated_checksum >> 8)

        if calculated_checksum != 0xFF:
            logger.error(f"Checksum verification failed: calculated=0x{calculated_checksum:02x}, expected=0x{0xFF:02x}")
            raise EvolverSerialError("Checksum verification failed for response packet")

        # ACKNOWLEDGE - send acknowledgment to arduino
        # Update SerialCommand dataclass attributes
        ack_command = command
        ack_command.tag = CommandTags.ACKNOWLEDGE
        logger.debug(command)
        packet_ack = self.build_packet(ack_command)
        logger.debug(f"Serial write ACK to arduino: {packet_ack.hex(' ')}")
        self.serial_connection.write(packet_ack)

        # wait for full packet transmission to arduino in a dynamic fashion since packet lengths will vary
        self.serial_connection.flush()

        # Extract response packet header information
        packet_response_data_length = packed_decoded_response[1]
        packet_response_type = packed_decoded_response[2]
        packet_response_data: list[int] = []
        if packet_response_type == CommandTags.SENSOR.value:
            # Extract data (4 bytes per value) to return
            for i in range(packet_response_data_length):
                if 3 + i * 4 + 3 <= len(packed_decoded_response):
                    # unpack response data payload using little-endian
                    # intrepret data integers as type int (NOT unsigned int)
                    value: int = struct.unpack("<i", packed_decoded_response[3 + i * 4 : 3 + i * 4 + 4])[0]
                    packet_response_data.append(value)
        return packet_response_data

    async def broadcast(self, phase: int):
        """Broadcasts commands and data for the specified phase.

        Executes the broadcast cycle for a given phase, including sending
        phase state to arduinos, processing recurring commands, and
        broadcasting collected data to clients.

        Args:
            phase (int): The broadcast phase to execute (0, 1, or 2).

        Returns:
            bool: True if broadcast completed successfully, False otherwise.
        """
        # if currently running IMMEDIATE commands exit broadcast function, otherwise continue
        if self.running_immediate:
            return False
        self.running_broadcast = True
        self.phase = phase

        # run any immediate commands in command_queue
        if len(self.command_queue) > 0:
            await self.run_commands()
            logger.debug("Finished running immediate commands in the command queue")

        # send the broadcast phase state to arduinos using addresses 0x00 -> 0x03
        for arduino_address in [1, 2, 3]:
            param = f"arduino_{arduino_address}"
            new_command = SerialCommand(param=param, address=arduino_address, value=[self.phase], tag=CommandTags.REQUEST)
            self.command_queue.append(new_command)
        await self.run_commands()
        logger.debug("Finished sending commands updating phase states on Arduinos")

        if not self.evolver_conf["parameters"][f"phase_{phase}"]:
            logger.debug("Empty phase detected")
            self.running_broadcast = False
            return True

        # after running IMMEDIATE commands, add recurring commands to the command_queue based on the phase of the control loop eVOLVER is in
        for param, config in self.evolver_conf["parameters"][f"phase_{phase}"].items():
            if config["recurring"]:
                new_command = SerialCommand(
                    param=param,
                    address=self.evolver_conf["parameters"][f"phase_{phase}"][param]["address"],
                    value=config["value"],
                    tag=CommandTags.REQUEST,
                )
                self.command_queue.append(new_command)
        data = await self.run_commands()
        logger.debug("Finished running recurring commands")

        broadcast_data = BroadcastData(
            phase=self.phase,
            data=data,
            config=self.evolver_conf["parameters"][f"phase_{phase}"],
            timestamp=time.time(),
        )
        logging.info(f"eVOLVER Broadcast: {broadcast_data}")
        await self.emit("broadcast", asdict(broadcast_data))
        self.running_broadcast = False
        return True
