import json
import logging
import os
import struct
import time
from dataclasses import asdict, dataclass, field
from typing import ClassVar

import serial
import socketio
import yaml

from htevolver.exceptions import EvolverError, EvolverSerialError
from htevolver.htevolver_client.data_analysis import CalibrationData
from htevolver.shared import BroadcastData, CommandTags

logger = logging.getLogger(__name__)


@dataclass
class SerialCommand:
    """Container for commands sent to the Arduino via serial connection.

    Stores the command data needed for eVOLVER Arduino communication.

    Attributes:
        param (str): Parameter name being controlled (e.g., "temp", "stir").
        address (int): Arduino address for the command.
        value (list[int]): List of values to set for the parameter.
        tag (CommandTags): Command type tag (REQUEST, ACKNOWLEDGE, etc.).
    """

    param: str
    address: int
    value: list[int]
    tag: CommandTags


@dataclass
class EvolverCommand:
    """Container for eVOLVER commands received from clients.

    Validates and normalizes commands to be sent to the Arduino.

    Attributes:
        param (str): Parameter name being controlled (e.g., "temp", "stir").
        address (int): Arduino address for the command.
        value (list[int]): List of values to set for the parameter.
        immediate (bool): Whether the command should be executed immediately.
        recurring (bool): Whether the command should be recurring.
        phase (str): Phase in which the command is executed ("phase_0", etc.).
        parameter_table (ClassVar[dict]): Class-level parameter lookup table.
    """

    param: str
    address: int
    value: list[int]
    immediate: bool
    recurring: bool
    phase: str
    parameter_table: ClassVar[dict] = field(init=False)

    @classmethod
    def extract_parameter_info(cls, parameter_config: dict):
        """Extract parameter information from configuration.

        Processes the parameter configuration to build a lookup table
        for parameter addresses, phases, and data lengths.

        Args:
            parameter_config (dict): The parameters section of the eVOLVER configuration.
        """
        cls.parameter_table: dict = {}
        # Extract from phase_0
        if parameter_config["phase_0"]:
            for param, config in parameter_config["phase_0"].items():
                if "address" in config:
                    cls.parameter_table[param] = (config["address"], "phase_0", config["data_length"])

        # Extract from phase_1
        if parameter_config["phase_1"]:
            for param, config in parameter_config["phase_1"].items():
                if "address" in config:
                    cls.parameter_table[param] = (config["address"], "phase_1", config["data_length"])

        # Extract from phase_2 if it exists
        if parameter_config["phase_2"]:
            for param, config in parameter_config["phase_2"].items():
                if "address" in config:
                    cls.parameter_table[param] = (config["address"], "phase_2", config["data_length"])

    @classmethod
    def create(cls, command: dict):
        """Create an EvolverCommand from a client command dictionary.

        Validates the command data and creates an EvolverCommand instance.

        Args:
            command (dict): Command dictionary from the client.
                Example: {"param": "temp", "value": [30, 30, 30, 30], "immediate": True, "recurring": True}

        Returns:
            EvolverCommand: A validated EvolverCommand instance.

        Raises:
            EvolverError: If the command is invalid.

        Examples:
            ```
            command = {"param": "temp", "value": [30, 30, 30, 30], "immediate": True, "recurring": True}
            evolver_command = EvolverCommand.create(command)
            ```
        """
        if command["param"] not in cls.parameter_table:
            raise EvolverError(f"Not registered parameter in recently received command: {command}")

        if command["value"] != len(cls.parameter_table[command["param"]][2]):
            raise EvolverError(f"Incorrect length of values in recently received command: {command}")

        if not all(isinstance(value, int) for value in command["value"]):
            raise EvolverError(f"Non-integer found in recently received command: {command}")

        if not all(value >= 0 for value in command["value"]):
            raise EvolverError(f"Negative value detected in recently received command: {command}")

        return cls(
            param=command["param"],
            address=cls.parameter_table[command["param"]][0],
            value=command["value"],
            immediate=command["immediate"],
            recurring=command["recurring"],
            phase=cls.parameter_table[command["param"]][1],
        )


class EvolverServerNamespace(socketio.AsyncNamespace):
    """Server namespace for eVOLVER hardware control.

    Handles communication with the eVOLVER hardware via serial connection
    and processes client requests for control and data acquisition.

    Attributes:
        evolver_conf (dict): Configuration for eVOLVER.
        evolver_conf_path (str): Path to the eVOLVER configuration file.
        phase (int): Current broadcast phase (0, 1, or 2).
        command_queue (list[SerialCommand]): Queue of commands to be sent to the Arduino.
        running_immediate (bool): Whether an immediate command is being processed.
        running_broadcast (bool): Whether a broadcast is in progress.
        calibration_directory (str): Directory for calibration data.
        serial_connection (serial.Serial): Serial connection to the Arduino.
    """

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

        self.calibration_directory: str = evolver_conf["calibration_directory"]
        self.serial_connection: serial.Serial = serial.Serial(
            port=self.evolver_conf["serial_port"],
            baudrate=self.evolver_conf["serial_baudrate"],
            timeout=self.evolver_conf["serial_timeout"],
        )
        EvolverCommand.extract_parameter_info(self.evolver_conf["parameters"])

        os.makedirs(self.calibration_directory, exist_ok=True)
        temp_dir = os.path.join(self.calibration_directory, "temp")
        od_dir = os.path.join(self.calibration_directory, "od")
        os.makedirs(temp_dir, exist_ok=True)
        os.makedirs(od_dir, exist_ok=True)

        logger.info("eVOLVER namespace initialized")

    async def on_connect(self, sid) -> None:
        """Handle client connection to the server's eVOLVER namespace.

        Args:
            sid (str): Session ID of the connecting client.
        """
        logger.info("Client connected to the eVOLVER namespace")

    async def on_disconnect(self, sid):
        """Handle client disconnection from the eVOLVER namespace.

        Args:
            sid (str): Session ID of the disconnecting client.
        """
        logger.info("Client disconnected from the eVOLVER namespace")

    async def on_command(self, sid, command: dict):
        """Process eVOLVER commands received from clients.

        Validates commands, updates configurations, and either queues them for
        later processing or executes them immediately based on the immediate flag.

        Args:
            sid (str): Session ID of the client.
            command (dict): Command data including parameter, value, and flags.
                Example: {"param": "temp", "value": [30, 30, 30, 30], "immediate": True, "recurring": True}

        Raises:
            EvolverError: If the command doesn't match a valid parameter.

        Examples:
            Called by client via:
            ```
            client.evolver.send_command("temp", [30, 30, 30, 30], True, True)
            ```
        """
        logger.info(f"Received the client command: {command}")
        try:
            evolver_command = EvolverCommand.create(command)

            # Initialize a new SerialCommand with the data received if its an immediate command
            if evolver_command.immediate:
                new_command = SerialCommand(
                    param=evolver_command.param,
                    address=evolver_command.address,
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
            self.evolver_conf["parameters"][evolver_command.phase][evolver_command.param]["recurring"] = evolver_command.recurring
            if self.evolver_conf["parameters"][evolver_command.phase][evolver_command.param]["value"] is not None:
                self.evolver_conf["parameters"][evolver_command.phase][evolver_command.param]["value"] = evolver_command.value
            self.save_conf()
            logger.info(f"Finished processed received EvolverCommand: {evolver_command}")
        except EvolverError as e:
            logger.warning(f"Error processing received EvolverCommand: {e}")

    async def on_request_status(self, sid):
        """Respond with the current eVOLVER status.

        Args:
            sid (str): Session ID of the requesting client.

        Examples:
            Called by client via:
            ```
            client.evolver.request_status()
            ```
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
        """Respond with the current eVOLVER configuration.

        Args:
            sid (str): Session ID of the requesting client.

        Examples:
            Called by client via:
            ```
            client.evolver.request_conf()
            ```
        """
        await self.emit("get_conf", self.evolver_conf, to=sid)
        logger.info("Request for current HTeVOLVER configuration processed")

    async def on_request_calibration(self, sid, data):
        """Send calibration data to the client.

        Loads and sends the specified calibration file to the client.

        Args:
            sid (str): Session ID of the client.
            data (dict): Dictionary with parameter, station_id, and filename.
                Example: {"parameter": "temp", "station_id": 0, "filename": "calibration_data_temp_2023-01-01.json"}

        Examples:
            Called by client via:
            ```
            client.evolver.request_calibration("temp", 0, "calibration_data_temp_2023-01-01.json")
            ```
        """
        try:
            local_filename = os.path.join(self.calibration_directory, data["parameter"], data["filename"])
            calibration_data = CalibrationData.from_file(local_filename)
            await self.emit("get_calibration", sid, {"calibration_data": calibration_data, "station_id": data["station_id"]})
            logger.info("Finished loading calibration data")
        except FileNotFoundError:
            logger.warning(f"Error loading calibration file: {data['filename']}")
            await self.emit("get_calibration", sid, {"calibration_data": {}, "station_id": data["station_id"]})

    async def on_receive_calibration(self, sid, new_calibration_data):
        """Save calibration data received from a client.

        Stores new calibration data received from a client to the appropriate file.

        Args:
            sid (str): Session ID of the client.
            new_calibration_data (dict): Dictionary with parameter, data, and timestamp.
                Example: {"parameter": "temp", "data": {...}, "timestamp": "2023-01-01_12-34-56"}

        Examples:
            Called by client via:
            ```
            client.evolver.send_calibration("temp", calibration_data, "2023-01-01_12-34-56")
            ```
        """
        timestamp = new_calibration_data["timestamp"]
        parameter = new_calibration_data["parameter"]

        try:
            filename = os.path.join(self.calibration_directory, parameter, f"calibration_data_{parameter}_{timestamp}.json")
            with open(filename, "w") as f:
                json.dump(new_calibration_data["data"], f, indent=4)
            logger.info(f"Calibration successfully saved to {filename}")
        except Exception as e:
            logger.warning(f"Error saving calibration data: {e}. Saving a string-formatted backup")
            filename = os.path.join(
                self.calibration_directory, parameter, f"calibration_data_{parameter}_{timestamp}_STRING-BACKUP.txt"
            )
            with open(filename, "w") as f:
                new_calibration_data_str = str(new_calibration_data)
                f.write(new_calibration_data_str)

    def load_conf(self):
        """Load the eVOLVER configuration from disk.

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
        """Execute all commands in the command queue.

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
        """Encode data using Consistent Overhead Byte Stuffing (COBS).

        COBS encoding ensures that the encoded data contains no zero bytes except
        for a trailing delimiter, which is useful for framing serial communications.

        Args:
            data (bytearray): The data to encode.

        Returns:
            bytearray: COBS encoded data with a trailing zero byte.

        Examples:
            ```
            original_data = bytearray([1, 0, 2, 3, 0, 4])
            encoded_data = cobs_encode(original_data)
            # encoded_data will be bytearray([2, 1, 3, 2, 3, 2, 4, 0])
            ```
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
        """Decode COBS-encoded data.

        Reverses the COBS encoding process to recover the original data.

        Args:
            data (bytearray|bytes): COBS-encoded data (without the trailing zero byte).

        Returns:
            bytearray: The decoded data.

        Examples:
            ```
            encoded_data = bytearray([2, 1, 3, 2, 3, 2, 4, 0])
            decoded_data = cobs_decode(encoded_data)
            # decoded_data will be bytearray([1, 0, 2, 3, 0, 4])
            ```
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
        """Build a packet for serial communication with the Arduino.

        Constructs a properly formatted packet with address, data length,
        command type, data payload, and checksum, then applies COBS encoding.

        Args:
            command (SerialCommand): The command to convert into a packet.

        Returns:
            bytearray: The COBS-encoded packet ready for transmission.

        Examples:
            ```
            command = SerialCommand(
                param="temp",
                address=0x01,
                value=[30, 30, 30, 30],
                tag=CommandTags.REQUEST
            )
            packet = build_packet(command)
            ```
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
        """Handle the full serial communication cycle with the Arduino.

        Sends the command packet to the Arduino, reads and validates the response,
        sends an acknowledgment, and extracts any returned sensor data.

        Args:
            command (SerialCommand): The command to send.

        Returns:
            list[int]: Data returned from the Arduino, if any.

        Raises:
            EvolverSerialError: If there's an error in communication or validation.

        Examples:
            ```
            command = SerialCommand(param="temp", address=0x01, value=[30, 30, 30, 30], tag=CommandTags.REQUEST)
            try:
                response_data = serial_communication(command)
            except EvolverSerialError as e:
                print(f"Communication error: {e}")
            ```
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
        """Broadcast commands and data for the specified phase.

        Executes the broadcast cycle for a given phase, including sending
        phase state to arduinos, processing recurring commands, and
        broadcasting collected data to clients.

        Args:
            phase (int): The broadcast phase to execute (0, 1, or 2).

        Returns:
            bool: True if broadcast completed successfully, False otherwise.

        Examples:
            ```
            # Execute phase 0 broadcast
            result = await broadcast(0)
            if result:
                print("Broadcast completed successfully")
            else:
                print("Broadcast failed or was interrupted")
            ```
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
