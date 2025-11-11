import struct
import time
from dataclasses import dataclass, field
from enum import Enum

from htevolver.exceptions import PacketBuilderError


class ServerResultCodes(Enum):
    SUCCESS = 0
    NOT_READY = 1
    EXIT_ROUTINE = 2
    REQUEST_ERROR = 5
    ROBOTICS_ERROR = 3


class RoboticsState(Enum):
    READY = 0
    IDLE = 1
    BUSY = 2
    PAUSE = 3
    RESUME = 4
    STOP = 5
    EMERGENCY_STOP = 6


class ReferencePositions(Enum):
    STANDBY = 0
    HOME = 1
    STATION_0 = 2
    STATION_1 = 3
    STATION_2 = 4
    STATION_3 = 5
    TOOL_CHANGE_0 = 6
    TOOL_CHANGE_1 = 7
    TOOL_CHANGE_2 = 8


class RoboticsRoutines(Enum):
    NO_ROUTINE = 0
    INFLUX = 1
    PIPETTE = 2
    FILLING_VIALS = 3
    PRIMING_INFLUX = 4
    PRIMING_EFFLUX = 5
    INITIALIZE = 6
    HOME = 7
    STANDBY = 8
    TOOL_CHANGE = 9


class CommandTags(Enum):
    REQUEST = 0
    ACKNOWLEDGE = 1
    SENSOR = 2
    ECHO = 3
    CONFIG = 4


class PacketBuilder:
    command_tags: dict[str, int] = {"request": 0, "acknowledge": 1, "sensor": 2, "echo": 3, "config": 4}
    serial_addresses: dict[str, int] = {
        "arduino_0": 0,
        "arduino_1": 1,
        "arduino_2": 2,
        "arduino_3": 3,
        "od_led_left": 5,
        "od_led_right": 6,
        "od_90_left": 7,
        "od_90_right": 8,
        "temp": 9,
        "stir": 10,
        "overflow_left": 11,
        "overflow_right": 12,
        "efflux": 13,
    }

    @staticmethod
    def cobs_encode(data: bytearray):
        """
        Encodes data with Consistent Overhead Byte Stuffing (COBS).

        Args:
            data (bytes or bytearray): The data to encode.

        Returns:
            bytearray: The COBS-encoded data.
        """
        if not data:
            return bytearray(b"\x01\x00")

        # Start with an extra byte for the code
        result = bytearray()

        # Iterate through the data to find all zeros and encode
        code_index = 0
        code = 1

        # Add placeholder for first code byte
        result.append(0)

        for byte in data:
            if byte == 0:
                # Found a zero, write the code byte and reset
                result[code_index] = code
                code = 1
                code_index = len(result)
                result.append(0)  # Placeholder for next code byte
            else:
                # Non-zero byte, append it
                result.append(byte)
                code += 1
                # If the code reaches its maximum value, write it and start a new block
                if code == 0xFF:
                    result[code_index] = code
                    code = 1
                    code_index = len(result)
                    result.append(0)  # Placeholder for next code byte

        # Write the final code byte
        result[code_index] = code

        # Add the frame delimiter zero byte
        result.append(0)

        return result

    @staticmethod
    def cobs_decode(data):
        """
        Decodes data encoded with Consistent Overhead Byte Stuffing (COBS).

        This is adapted from evolver_namespace_server.py's implementation.

        Args:
            data (bytes or bytearray): The COBS-encoded data to decode.

        Returns:
            bytearray: The decoded data.
        """
        if not data or data[-1] != 0:
            raise ValueError("Invalid COBS encoded data: missing zero delimiter")

        result = bytearray()
        i = 0

        while i < len(data) - 1:  # Skip the final zero delimiter
            code = data[i]
            i += 1

            if code == 0:
                raise ValueError("Invalid COBS encoded data: unexpected zero")

            for j in range(1, code):
                if i < len(data) - 1:  # Ensure we're not at the final delimiter
                    result.append(data[i])
                    i += 1

            if code < 0xFF and i < len(data) - 1:
                result.append(0)

        return result

    @staticmethod
    def calculate_checksum(packet):
        """Calculate the checksum for a packet"""
        # Sum all bytes in the packet
        checksum = sum(packet)

        # Add carry bits back
        while checksum > 0xFF:
            checksum = (checksum & 0xFF) + (checksum >> 8)

        # Return the complement
        return 0xFF - checksum

    @classmethod
    def build_packet(
        cls, address: str, payload_length: int, command_type: str, payload: list[int], generate_ack: bool = True
    ) -> tuple[bytearray, ...]:
        if address not in cls.serial_addresses:
            raise PacketBuilderError("Invalid serial address used to build packet: {0}".format(address))
        if payload_length <= 0:
            raise PacketBuilderError("Invalid payload length used to build packet, must be greater than zero")
        if command_type not in cls.command_tags:
            raise PacketBuilderError("Invalid command type used to build packet: {0}".format(command_type))
        if len(payload) != payload_length:
            raise PacketBuilderError(
                "Invalid payload used to build packet, does not match expected payload length: {0} != {1}".format(
                    payload_length, len(payload)
                )
            )
        for index, data_member in enumerate(payload):
            if not isinstance(data_member, int):
                raise PacketBuilderError(
                    "Invalid data member type detected in payload, all values must be integers: {0}, index {1}, invalid type{2}".format(
                        data_member, index, type(data_member)
                    )
                )
        packet = bytearray()
        packet.append(cls.serial_addresses[address])
        packet.append(payload_length)
        packet.append(cls.command_tags[command_type])

        # Add data payload (4 bytes per value, little endian)
        for value in payload:
            packet.extend(struct.pack("<I", value))

        # Calculate and add checksum
        packet.append(PacketBuilder.calculate_checksum(packet))
        encoded_packet = PacketBuilder.cobs_encode(packet)

        if command_type == "request" and generate_ack:
            # Create a copy of the packet but change the type to ACKNOWLEDGE
            ack_packet = bytearray(packet)
            ack_packet[2] = cls.command_tags["acknowledge"]

            # Recalculate checksum
            ack_packet.pop()
            checksum = PacketBuilder.calculate_checksum(ack_packet)
            ack_packet.append(checksum)

            encoded_ack_packet = PacketBuilder.cobs_encode(ack_packet)

            return (encoded_packet, packet, encoded_ack_packet, ack_packet)
        return (encoded_packet, packet)


#### BROADCAST DATA STORAGE CLASSES ####
@dataclass
class BroadcastData:
    phase: int = field(default=0)
    timestamp: float = field(default=0.0)
    data: dict[str, list[int]] = field(default_factory=dict)
    config: str = field(default="")

    def validate(self, parameter_list: list[str] = ["temp", "od_90_left", "od_90_right"]):
        for parameter in parameter_list:
            if parameter not in self.data:
                return False
        return True


#### STATUS MANAGEMENT CLASSES ####
@dataclass
class HTEvolverStatus:
    connected: bool = field(default=False)
    start_time: float = field(default=0.0)
    elapsed_time: float = field(default=0.0)
    evolver: dict = field(default_factory=dict)
    robotics: dict = field(default_factory=dict)

    def get_elapsed_time(self):
        """Get the current elapsed time since experiment start (in hours)."""
        return round((time.time() - self.start_time) / 3600, 4)

    def reset_time(self):
        self.start_time = time.time()
