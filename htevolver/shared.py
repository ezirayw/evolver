import struct
import time
from dataclasses import dataclass, field
from enum import Enum

from htevolver.exceptions import PacketBuilderError, StationInfluxCommandError


@dataclass
class StationInfluxCommand:
    """Container for fluid influx commands for vials in a SmartStation.

    Attributes:
        fluid_type (str): The type of fluid to be dispensed.
        station_id (int): The ID of the SmartStation.
        vial_0 ... vial_17 (int): Influx volumes for each vial.
    """

    fluid_type: str = field(default="blank")
    station_id: int = field(default=-1)
    vial_0: int = field(default=0)
    vial_1: int = field(default=0)
    vial_2: int = field(default=0)
    vial_3: int = field(default=0)
    vial_4: int = field(default=0)
    vial_5: int = field(default=0)
    vial_6: int = field(default=0)
    vial_7: int = field(default=0)
    vial_8: int = field(default=0)
    vial_9: int = field(default=0)
    vial_10: int = field(default=0)
    vial_11: int = field(default=0)
    vial_12: int = field(default=0)
    vial_13: int = field(default=0)
    vial_14: int = field(default=0)
    vial_15: int = field(default=0)
    vial_16: int = field(default=0)
    vial_17: int = field(default=0)

    def __post_init__(self):
        for vial_id in range(18):
            volume = getattr(self, f"vial_{vial_id}")
            if volume < 0:
                raise StationInfluxCommandError(
                    f"Negative volume error when creating StationInfluxCommand: vial_{vial_id}={volume}"
                )

    @classmethod
    def uniform_influx(cls, fluid_type: str, station_id: int, volume: int) -> "StationInfluxCommand":
        """Create a StationInfluxCommand for uniform volumes across all vials"""
        vial_volumes: dict[str, int] = {f"vial_{vial_id}": volume for vial_id in range(18)}
        return cls(fluid_type=fluid_type, station_id=station_id, **vial_volumes)

    def get_vial_volume(self, vial_id: int) -> int:
        """Return the influx volume for a given vial."""
        if not (0 <= vial_id <= 17):
            raise ValueError("vial_id must be between 0 and 17")
        return getattr(self, f"vial_{vial_id}")


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


class RoboticsRoutines(Enum):
    NO_ROUTINE = 0
    INFLUX = 1
    PIPETTE = 2
    PRIMING_INFLUX = 3
    PRIMING_EFFLUX = 4
    INITIALIZE = 5
    HOME = 6
    STANDBY = 7
    TOOL_CHANGE = 8


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
