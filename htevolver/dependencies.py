import logging
import struct
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, ClassVar, Literal, TypeAlias

from pydantic import BaseModel, Field, NonNegativeInt

from htevolver.exceptions import EvolverPacketError, EvolverParameterError

logger = logging.getLogger(__name__)


class RoboticsState(Enum):
    READY = 0
    IDLE = 1
    BUSY = 2
    PAUSE = 3
    RESUME = 4
    STOP = 5
    EMERGENCY_STOP = 6


class RoboticsRoutine(Enum):
    NO_ROUTINE = 0
    INFLUX = 1
    PIPETTE = 2
    PRIMING_INFLUX = 3
    PRIMING_EFFLUX = 4
    INITIALIZE = 5
    HOME = 6
    STANDBY = 7
    TOOL_CHANGE = 8


@dataclass
class RoboticsStatus:
    state: RoboticsState
    routine: RoboticsRoutine
    start_time: float
    end_time: float
    elapsed_time: float


@dataclass
class CartesianMovement:
    """Container for static cartesian xArm coordinates or dynamic movements if speed and acceleration is provided."""

    x: float
    y: float
    z: float
    roll: int
    pitch: int
    yaw: float
    speed: int | None = None
    acceleration: int | None = None


class ErrorModel(BaseModel):
    detail: str


class ServerResultCodes(Enum):
    SUCCESS = 0
    NOT_READY = 1
    EXIT_ROUTINE = 2
    REQUEST_ERROR = 3
    ROBOTICS_ERROR = 4
    EVOLVER_EVENT_ERROR = 5
    DATA_STORE_ERROR = 6


class StationInfluxCommandBody(BaseModel):
    """Container for fluid influx commands for vials in a SmartStation.

    Attributes:
        fluid_type (str): The type of fluid to be dispensed.
        station_id (int): The ID of the SmartStation.
        vial_0 ... vial_17 (int): Influx volumes for each vial.
    """

    dispensehead: str = Field(title="Name of DispenseHead this command maps to", examples=["media"])
    station_id: int = Field(title="SmartStation ID this command maps to", examples=[0])
    vial_0: NonNegativeInt = Field(default=0, le=1000, title="Influx volume (uL)", examples=[500])
    vial_1: NonNegativeInt = Field(default=0, le=1000, title="Influx volume (uL)", examples=[500])
    vial_2: NonNegativeInt = Field(default=0, le=1000, title="Influx volume (uL)", examples=[500])
    vial_3: NonNegativeInt = Field(default=0, le=1000, title="Influx volume (uL)", examples=[500])
    vial_4: NonNegativeInt = Field(default=0, le=1000, title="Influx volume (uL)", examples=[500])
    vial_5: NonNegativeInt = Field(default=0, le=1000, title="Influx volume (uL)", examples=[500])
    vial_6: NonNegativeInt = Field(default=0, le=1000, title="Influx volume (uL)", examples=[500])
    vial_7: NonNegativeInt = Field(default=0, le=1000, title="Influx volume (uL)", examples=[500])
    vial_8: NonNegativeInt = Field(default=0, le=1000, title="Influx volume (uL)", examples=[500])
    vial_9: NonNegativeInt = Field(default=0, le=1000, title="Influx volume (uL)", examples=[500])
    vial_10: NonNegativeInt = Field(default=0, le=1000, title="Influx volume (uL)", examples=[500])
    vial_11: NonNegativeInt = Field(default=0, le=1000, title="Influx volume (uL)", examples=[500])
    vial_12: NonNegativeInt = Field(default=0, le=1000, title="Influx volume (uL)", examples=[500])
    vial_13: NonNegativeInt = Field(default=0, le=1000, title="Influx volume (uL)", examples=[500])
    vial_14: NonNegativeInt = Field(default=0, le=1000, title="Influx volume (uL)", examples=[500])
    vial_15: NonNegativeInt = Field(default=0, le=1000, title="Influx volume (uL)", examples=[500])
    vial_16: NonNegativeInt = Field(default=0, le=1000, title="Influx volume (uL)", examples=[500])
    vial_17: NonNegativeInt = Field(default=0, le=1000, title="Influx volume (uL)", examples=[500])

    @classmethod
    def uniform_influx(cls, fluid_type: str, station_id: int, volume: int) -> "StationInfluxCommandBody":
        """Create a StationInfluxCommand for uniform volumes across all vials"""
        vial_volumes: dict[str, int] = {f"vial_{vial_id}": volume for vial_id in range(18)}
        return cls(dispensehead=fluid_type, station_id=station_id, **vial_volumes)


@dataclass
class ServerResult:
    """Container for return data following a server request.

    Stores the result of a request operation, including success status,
    timing information, and current system state.

    Attributes:
        namespace (str): The namespace that processed the operation.
        event (str): The event or routine that was executed.
        status (dict): Current status of the namespace.
        elapsed_time (float): Time taken to execute the operation in seconds.
        message (str): Descriptive message about the operation result.
        code (int): Error code for the operation, useful for error handling and programmatic responses.
            See ServerResultCodes for details.
    """

    namespace: str
    event: str
    status: dict
    elapsed_time: float
    code: ServerResultCodes
    message: str = field(default="")


class EvolverPacket:
    command_tags: dict[str, int] = {
        "request": 0,
        "acknowledge": 1,
        "data": 2,
        "config": 3,
        "error": 4,
    }
    serial_addresses: dict[str, int] = {
        "arduino_0": 0,
        "arduino_1": 1,
        "arduino_2": 2,
        "arduino_3": 3,
        "od_led_left": 5,
        "od_led_right": 6,
        "od_left": 7,
        "od_right": 8,
        "temp": 9,
        "stir": 10,
        "overflow_left": 11,
        "overflow_right": 12,
        "efflux": 13,
    }

    header_length: int = 3
    byte_number: int = 4

    def __init__(self, address: str, payload_length: int, command_type: str, payload: list[int], generate_ack: bool = True):
        if address not in EvolverPacket.serial_addresses.keys():
            raise EvolverPacketError("Invalid serial address used to build packet: {0}".format(address))
        if payload_length <= 0:
            raise EvolverPacketError("Invalid payload length used to build packet, must be greater than zero")
        if command_type not in EvolverPacket.command_tags:
            raise EvolverPacketError("Invalid command type used to build packet: {0}".format(command_type))
        if len(payload) != payload_length:
            raise EvolverPacketError(
                "Invalid payload used to build packet, does not match expected payload length: {0} != {1}".format(
                    payload_length, len(payload)
                )
            )
        for index, data_member in enumerate(payload):
            if not isinstance(data_member, int):
                raise EvolverPacketError(
                    "Invalid data member type detected in payload, all values must be integers: {0}, index {1}, invalid type{2}".format(
                        data_member, index, type(data_member)
                    )
                )
        self.address: str = address
        self.payload_length: int = payload_length
        self.command_type: str = command_type
        self.payload: list[int] = payload

        decoded_packet = bytearray()
        decoded_packet.append(EvolverPacket.serial_addresses[address])
        decoded_packet.append(payload_length)
        decoded_packet.append(EvolverPacket.command_tags[command_type])

        # Add data payload (4 bytes per value, little endian)
        for value in payload:
            decoded_packet.extend(struct.pack("<I", value))

        # Calculate and add checksum
        decoded_packet.append(EvolverPacket.calculate_checksum(decoded_packet))

        self.decoded_packet: bytearray = decoded_packet
        self.encoded_packet: bytearray = EvolverPacket.cobs_encode(decoded_packet)
        self.ack_decoded_packet: bytearray | None = None
        self.ack_encoded_packet: bytearray | None = None

        if generate_ack:
            # Create a copy of the packet but change the type to ACKNOWLEDGE
            ack_decoded_packet = bytearray(decoded_packet)
            ack_decoded_packet[2] = EvolverPacket.command_tags["acknowledge"]

            # Recalculate checksum
            ack_decoded_packet.pop()
            checksum = EvolverPacket.calculate_checksum(ack_decoded_packet)

            self.ack_decoded_packet = ack_decoded_packet.append(checksum)
            self.ack_encoded_packet = EvolverPacket.cobs_encode(ack_decoded_packet)

    def __repr__(self):
        return (
            f"EvolverPacket(address={self.address}, "
            f"payload_length={self.payload_length}, "
            f"command_type={self.command_type}, "
            f"payload={self.payload}, "
            f"decoded_packet={self.get_decode_bytes()}, "
            f"encoded_packet={self.get_encode_bytes()})"
            f"ack_decoded_packet={self.get_decode_bytes(use_ack=True)}, "
            f"ack_encoded_packet={self.get_encode_bytes(use_ack=True)})"
        )

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

    @staticmethod
    def is_valid_packet(packet: bytearray) -> bool:
        """Verifies if packet is a valid COBS decoded packet using its checksum"""
        calculated_checksum: int = EvolverPacket.calculate_checksum(packet)
        if calculated_checksum != 0xFF:
            return False
        return True

    @classmethod
    def create_from_bytes(cls, input_packet: bytearray, encode: bool) -> "EvolverPacket":
        """Create an EvolverPacket from a bytearray. Useful for generating validated response packets during serial communication cycles."""
        decoded_packet: bytearray = input_packet
        if encode:
            decoded_packet = EvolverPacket.cobs_decode(input_packet)

        if len(decoded_packet) < EvolverPacket.header_length:
            raise EvolverPacketError("Error creating EvolverPacket, length of decoded packet too short")
        if not EvolverPacket.is_valid_packet(decoded_packet):
            f"Checksum verification failed: calculated=0x{EvolverPacket.calculate_checksum(decoded_packet):02x}, expected=0x{0xFF:02x}"

        address_val = decoded_packet[0]
        payload_length = decoded_packet[1]
        command_type_val = decoded_packet[2]
        payload = []
        # Each payload value is 4 bytes, little endian
        for i in range(
            EvolverPacket.header_length,
            EvolverPacket.header_length + payload_length * EvolverPacket.byte_number,
            EvolverPacket.byte_number,
        ):
            payload.append(int.from_bytes(decoded_packet[i : i + EvolverPacket.byte_number], "little"))

        # Find address and command_type keys
        address = next((k for k, v in EvolverPacket.serial_addresses.items() if v == address_val), None)
        command_type = next((k for k, v in EvolverPacket.command_tags.items() if v == command_type_val), None)
        if address is None or command_type is None:
            raise EvolverPacketError(
                "Error creating EvolverPacket, could not determine address or command_type from packet bytes"
            )

        # Build the packet object
        return cls(address, payload_length, command_type, payload)

    def get_encode_bytes(self, use_ack: bool = False, fmt: str = "") -> str:
        if use_ack and self.ack_encoded_packet:
            return fmt.join([f"{byte:02X}" for byte in self.ack_encoded_packet])
        else:
            return fmt.join([f"{byte:02X}" for byte in self.encoded_packet])

    def get_decode_bytes(self, use_ack: bool = False, fmt: str = "") -> str:
        if use_ack and self.ack_decoded_packet:
            return fmt.join([f"{byte:02X}" for byte in self.ack_decoded_packet])
        else:
            return fmt.join([f"{byte:02X}" for byte in self.decoded_packet])


SmartStationData: TypeAlias = dict[str, int | None]


@dataclass
class EvolverParameter:
    """Class that defines how parameters are managed by HT-eVOLVER. Each culture parameter managed by the EvolverNamespaceServer
    has an instance of EvolverParameter, which outlines the expected data schema for the parameter,
    functions for how data related to parameter is processed, and other relevant metadata.
    """

    parameter_type: Literal["sensor", "effector"]
    payload_length: int
    phase: Literal[0, 1, 2, 3]
    process_fn: Callable[[EvolverPacket], dict[str, SmartStationData]]
    mask: dict[str, bool] | None = field(default=None)
    recurring_payload: list[int] | None = field(default=None)
    command_dtype: type | None = field(default=None)
    voltage_dtype: type | None = field(default=None)
    calibrated_dtype: type | None = field(default=None)

    valid_process_fns: ClassVar[list[str]] = ["vial", "station"]

    @classmethod
    def create(cls, parameter_name: str, parameter_config: dict) -> "EvolverParameter":
        """Create an EvolverParameter from a configuration. Performs data validation during on the input config"""
        try:
            if not isinstance(parameter_config["payload_length"], int) and parameter_config["payload_length"] < 0:
                raise EvolverParameterError(
                    f"Invalid 'payload_length' value for EvolverParameter {parameter_name}: {parameter_config['payload_length']}"
                )

            if parameter_config["process_fn"] not in cls.valid_process_fns:
                raise EvolverParameterError(
                    f"Invalid 'process_fn' for EvolverParmater {parameter_name}: {parameter_config['process_fn']}"
                )

            match parameter_config["process_fn"]:
                case "vial":
                    parameter_config["process_fn"] = cls.process_vial
                case "station":
                    parameter_config["process_fn"] = cls.process_station

            return cls(**parameter_config)

        except KeyError:
            logger.exception("Invalid key when loading in EvolverParameter", stack_info=True)
            raise EvolverParameterError("Invalid key when loading in EvolverParameter")

    def process_station(self, response_packet: EvolverPacket) -> dict[str, SmartStationData]:
        """Extract response packet payload for EvolverParameters that regulate entire SmartStation into structured vial data.

        Args:
            response_packet (EvolverPacket): EvolverPacket object containing data from hardware.

        Returns:
            dict[str, SmartStationData]: A dictionary mapping station identifiers (e.g., "station_0")
                to another dictionary mapping vial identifiers (e.g., "vial_0") to their respective payload values
        """
        # expected payload length is 4
        # each payload value corresponds to all vials for a given SmartStation
        vials_per_station: int = 18
        station_data: dict[str, SmartStationData] = {}
        for index, value in enumerate(response_packet.payload):
            station_data[f"station_{index}"] = {f"vial_{vial_id}": value for vial_id in range(vials_per_station)}

        return station_data

    def process_vial(
        self, response_packet: EvolverPacket, smart_stations: list[int] = [0, 1, 2, 3]
    ) -> dict[str, SmartStationData]:
        """Extract response packet payload for EvolverParameters that require vial-mask filtering into structured vial data using boolean masks.

        Args:
            response_packet (EvolverPacket): EvolverPacket object containing data from hardware.
            mask (dict): Boolean mask that instructs function how to filter payload data into structured vial data

        Returns:
            dict[str, SmartStationData]: A dictionary mapping station identifiers (e.g., "station_0")
                to another dictionary mapping vial identifiers (e.g., "vial_0") to their respective payload values
        """
        if not self.mask:
            raise EvolverParameterError("Error processing data, mising required mask")
        vials_per_station: int = 18
        station_data: dict[str, SmartStationData] = {}
        data_hit: int = 0
        for station_id in smart_stations:
            station_data[f"station_{station_id}"] = {f"vial_{vial_id}": None for vial_id in range(vials_per_station)}
            for vial in station_data[f"station_{station_id}"]:
                if self.mask[vial]:
                    station_data[f"station_{station_id}"][vial] = response_packet.payload[data_hit]
                    data_hit += 1
        return station_data

    def has_valid_payload(self, evolver_command: dict):
        if "payload" not in evolver_command:
            return False
        if not isinstance(evolver_command["payload"], list):
            return False
        if len(evolver_command["payload"]) != self.payload_length:
            return False
        return True
