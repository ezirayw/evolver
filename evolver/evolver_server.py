import json
import logging
import os
import time
import struct
from dataclasses import dataclass, field
from typing import TypedDict

import serial
from serial.serialposix import Serial
import socketio
import yaml

logger = logging.getLogger(__name__)

LOCATION = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
EVOLVER_CONF_FILENAME = "conf.yml"


class EvolverSerialError(Exception):
    pass


class ConfigError(Exception):
    pass


class SerialCommand(TypedDict):
    param: str
    address: int
    value: list[int]
    immediate: bool
    recurring: bool
    acknowledge: bool


class BroadcastData(TypedDict, total=False):
    data: dict[str, list[int]]
    phase: int
    config: str
    ip: str
    timestamp: float


@dataclass
class EvolverServer:
    evolver_ip: str
    sio: socketio.AsyncServer = field(
        default_factory=lambda: socketio.AsyncServer(always_connect=True)
    )
    calibrations_filename: str = field(default="calibrations.json")
    evolver_conf: dict = field(default_factory=dict)

    serial_connection: serial.Serial = field(init=False)
    command_queue: list[SerialCommand] = field(default_factory=list)
    running_immediate: bool = field(default=False)
    running_broadcast: bool = field(default=False)
    recurring_tag: int = field(default=0)
    immediate_tag: int = field(default=1)
    acknowledge_tag: int = field(default=2)
    sensor_tag: int = field(default=3)
    echo_tag: int = field(default=4)
    serial_delay: float = field(default=0.1)

    def __post_init__(self):
        """Initialize additional attributes after instance creation."""
        with open(
            os.path.realpath(
                os.path.join(
                    os.getcwd(), os.path.dirname(__file__), EVOLVER_CONF_FILENAME
                )
            ),
            "r",
        ) as ymlfile:
            self.evolver_conf = yaml.safe_load(ymlfile)

        self.serial_delay = self.evolver_conf["serial_delay"]
        self.recurring_tag = self.evolver_conf["command_types"]["recurring"]
        self.immediate_tag = self.evolver_conf["command_types"]["immediate"]
        self.acknowledge_tag = self.evolver_conf["command_types"]["acknowledge"]
        self.sensor_tag = self.evolver_conf["command_types"]["sensor"]
        self.echo_tag = self.evolver_conf["command_types"]["echo"]
        self.serial_connection = serial.Serial(
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
        phase = data.get("param", None)
        param = data.get("param", None)
        value = data.get("value", [])
        immediate = data.get("immediate", False)
        recurring = data.get("recurring", False)

        # Update the configuration for the param
        if self.evolver_conf["parameters"][phase][param]["value"] is not None:
            self.evolver_conf["parameters"][phase][param]["value"] = value

        self.evolver_conf["experimental_params"][phase][param]["recurring"] = recurring

        # Save to config the values sent in for the parameter
        with open(
            os.path.realpath(
                os.path.join(os.getcwd(), os.path.dirname(__file__), "conf.yml")
            ),
            "w",
        ) as ymlfile:
            yaml.dump(self.evolver_conf, ymlfile)

        if immediate:
            # Initialize a new SerialCommand with the data received
            new_command: SerialCommand = {
                "param": param,
                "address": self.evolver_conf["parameters"][phase][param]["address"],
                "value": value,
                "immediate": True,
                "recurring": False,
                "acknowledge": False,
            }

            self.command_queue.insert(0, new_command)
            logger.info(
                "adding the following immediate command to queue: %s", new_command
            )
            if not self.running_broadcast:
                logger.info("running the following immediate command: %s", new_command)
                self.running_immediate = True
                await self.run_commands(phase)
                self.running_immediate = False

    async def on_getlastcommands(self, sid, data):
        await self.sio.emit("config", self.evolver_conf, namespace="/default_evolver")

    async def on_getcalibrationnames(self, sid, data):
        calibration_names = []
        logger.info("Reteiving cal names...")
        try:
            with open(os.path.join(LOCATION, self.calibrations_filename)) as f:
                calibrations = json.load(f)
                for calibration in calibrations:
                    calibration_names.append(
                        {
                            "name": calibration["name"],
                            "calibrationType": calibration["calibrationType"],
                        }
                    )
        except FileNotFoundError:
            logging.warning("Error reading calibrations file.")

        await self.sio.emit(
            "calibrationnames", calibration_names, namespace="/default_evolver"
        )

    async def on_getfitnames(self, sid, data):
        fit_names = []
        logger.info("Retrieving fit names...")
        try:
            with open(os.path.join(LOCATION, self.calibrations_filename)) as f:
                calibrations = json.load(f)
                for calibration in calibrations:
                    for fit in calibration["fits"]:
                        fit_names.append(
                            {
                                "name": fit["name"],
                                "calibrationType": calibration["calibrationType"],
                            }
                        )
        except FileNotFoundError:
            logging.warning("Error reading calibrations file.")

        await self.sio.emit("fitnames", fit_names, namespace="/default_evolver")

    async def on_getcalibration(self, sid, data):
        try:
            with open(os.path.join(LOCATION, self.calibrations_filename)) as f:
                calibrations = json.load(f)
                for calibration in calibrations:
                    if calibration["name"] == data["name"]:
                        await self.sio.emit(
                            "calibration", calibration, namespace="/default_evolver"
                        )
                        break
        except FileNotFoundError:
            logging.warning("Error reading calibrations file.")

    async def on_setrawcalibration(self, sid, data):
        try:
            calibrations = []
            with open(os.path.join(LOCATION, self.calibrations_filename)) as f:
                calibrations = json.load(f)

                # First, delete existing calibration by same name if it exists
                index_to_delete = -1
                for i, calibration in enumerate(calibrations):
                    if calibration["name"] == data["name"]:
                        index_to_delete = i
                if index_to_delete >= 0:
                    del calibrations[index_to_delete]

                """
                    Add the calibration into the list. `data` should be formatted according
                    to the cal schema, containing a name, params, and raw field.
                """
                calibrations.append(data)
            with open(os.path.join(LOCATION, self.calibrations_filename), "w") as f:
                json.dump(calibrations, f)
                await self.sio.emit(
                    "calibrationrawcallback", "success", namespace="/default_evolver"
                )
        except FileNotFoundError:
            logging.warning("Error reading calibrations file.")

    async def on_setfitcalibrations(self, sid, data):
        """
        Set a fit calibration into the calibration file. data should contain a `fit` key/value
        formatted according to the cal schema `fit` object. This function will add the fit into the
        fits list for a given calibration.
        """
        try:
            calibrations = []
            with open(os.path.join(LOCATION, self.calibrations_filename)) as f:
                calibrations = json.load(f)
                for calibration in calibrations:
                    if calibration["name"] == data["name"]:
                        if calibration.get("fits", None) is not None:
                            index_to_delete = -1
                            for i, fit in enumerate(calibration["fits"]):
                                if fit["name"] == data["fit"]["name"]:
                                    index_to_delete = i
                            if index_to_delete >= 0:
                                del calibrations["fits"][index_to_delete]
                            calibration["fits"].append(data["fit"])
                        else:
                            calibration["fits"] = [].append(data["fit"])
            with open(os.path.join(LOCATION, self.calibrations_filename), "w") as f:
                json.dump(calibrations, f)
        except FileNotFoundError:
            logging.warning("Error reading calibrations file.")

    async def on_setactiveodcal(self, sid, data):
        try:
            active_calibrations = []
            logger.info("Time to set active cals. Data received: ")
            logger.info(data)
            with open(os.path.join(LOCATION, self.calibrations_filename)) as f:
                calibrations = json.load(f)
                for calibration in calibrations:
                    active = False
                    for fit in calibration["fits"]:
                        if fit["name"] in data["calibration_names"]:
                            fit["active"] = True
                            active = True
                        else:
                            fit["active"] = False
                    if active:
                        active_calibrations.append(calibration)
                await self.sio.emit(
                    "activecalibrations",
                    active_calibrations,
                    namespace="/default_evolver",
                )
            with open(os.path.join(LOCATION, self.calibrations_filename), "w") as f:
                json.dump(calibrations, f)
        except FileNotFoundError:
            logging.warning("Error reading calibrations file.")

    async def on_getactivecal(self, sid, data):
        try:
            active_calibrations = []
            with open(os.path.join(LOCATION, self.calibrations_filename)) as f:
                calibrations = json.load(f)
                for calibration in calibrations:
                    for fit in calibration["fits"]:
                        if fit["active"]:
                            active_calibrations.append(calibration)
                            break
            await self.sio.emit(
                "activecalibrations", active_calibrations, namespace="/default_evolver"
            )
        except FileNotFoundError:
            logging.warning("Error reading calibrations file.")

    async def on_getdevicename(self, sid, data):
        with open(os.path.join(LOCATION, self.evolver_conf["device"])) as f:
            configJSON = json.load(f)
        await self.sio.emit("broadcastname", configJSON, namespace="/default_evolver")

    async def on_setdevicename(self, sid, data):
        config_path = os.path.join(LOCATION)
        logger.info("saving device name")
        if not os.path.isdir(config_path):
            os.mkdir(config_path)
        with open(os.path.join(config_path, self.evolver_conf["device"]), "w") as f:
            f.write(json.dumps(data))
        await self.sio.emit("broadcastname", data, namespace="/default_evolver")

    async def run_commands(self, current_phase: str):
        data: dict[str, list[int]] = {}
        while len(self.command_queue) > 0:
            command = self.command_queue.pop(0)
            try:
                returned_data = self.serial_communication(command, current_phase)
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

    def cobs_decode(self, data: bytearray) -> bytearray:
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
        immediate = command["immediate"]
        recurring = command["recurring"]
        acknowledge = command["acknowledge"]

        # Add the address to the header of the packet
        packet.extend(struct.pack("<1B", address))

        # Add the data length to the header of the packet
        packet.extend(struct.pack("<1B", len(value)))

        # Add the type to the packet
        # Check that parameters being sent to arduino match expected values
        if recurring:
            packet.extend(struct.pack("<1B", self.recurring_tag))
        if immediate:
            packet.extend(struct.pack("<1B", self.immediate_tag))
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

    def serial_communication(self, command: SerialCommand, current_phase: str):
        self.serial_connection.reset_input_buffer()
        self.serial_connection.reset_output_buffer()
        logger.debug(command)
        packet_send = self.build_packet(command)
        logger.debug("serial write MESSAGE to arduino: %s", packet_send.hex(" "))
        self.serial_connection.write(packet_send)
        time.sleep(self.serial_delay)

        # Read until we get a zero byte (end of COBS packet)
        response_bytes = bytearray()
        while True:
            byte = self.serial_connection.read(1)
            if not byte or byte == b"\x00":
                break
            response_bytes.extend(byte)

        if not response_bytes:
            raise EvolverSerialError("No response received from Arduino")

        # Decode the COBS-encoded response
        try:
            packed_decoded_response = self.cobs_decode(response_bytes)
            logger.debug(
                "serial response from arduino: %s", packed_decoded_response.hex(" ")
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
            calculated_checksum = (calculated_checksum & 0xFF) + (
                calculated_checksum >> 8
            )

        if calculated_checksum != 0xFF:
            logger.error(
                "Checksum verification failed: calculated=0x%02x, expected=0x%02x",
                calculated_checksum,
                0xFF,
            )
            raise EvolverSerialError("Checksum verification failed for response packet")

        # ACKNOWLEDGE - send acknowledgment to arduino
        command["immediate"] = False
        command["recurring"] = False
        command["acknowledge"] = True
        logger.debug(command)
        packet_ack = self.build_packet(command)
        logger.debug("serial write ACK to arduino: %s", packet_ack.hex(" "))
        self.serial_connection.write(packet_ack)

        # This is necessary to allow the ack to be fully written out
        time.sleep(self.serial_delay)

        # Extract response packet header information
        packet_response_data_length = packed_decoded_response[1]
        packet_response_type = packed_decoded_response[2]

        if packet_response_type == self.sensor_tag:
            # Extract data (4 bytes per value) to return
            packet_response_data = []
            for i in range(packet_response_data_length):
                if 3 + i * 4 + 3 <= len(packed_decoded_response):
                    value = struct.unpack(
                        "<I", packed_decoded_response[3 + i * 4 : 3 + i * 4 + 4]
                    )[0]
                    packet_response_data.append(value)
            return packet_response_data
        else:
            return None

    def attach(self, app):
        """
        Attach the server to the web application and setup the serial communication
        """
        self.sio.attach(app)

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
            await self.run_commands(phase_string)

        # send the broadcast phase state to arduinos using addresses 0x00 -> 0x03
        for address in [1, 2, 3]:
            param = f"arduino_{address}"
            new_command: SerialCommand = {
                "param": param,
                "address": address,
                "value": [phase],
                "recurring": True,
                "immediate": False,
                "acknowledge": False,
            }
            self.command_queue.append(new_command)
        await self.run_commands(phase_string)

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
                    "address": self.evolver_conf["parameters"][phase_string][param][
                        "address"
                    ],
                    "value": config["value"],
                    "recurring": True,
                    "immediate": False,
                    "acknowledge": False,
                }
                self.command_queue.append(new_command)
        # run RECURRING commands that were just added
        broadcast_data["phase"] = phase
        broadcast_data["data"] = await self.run_commands(phase_string)

        # Build broadcast packet
        broadcast_data["config"] = self.evolver_conf["parameters"][phase_string]
        broadcast_data["ip"] = self.evolver_ip
        broadcast_data["timestamp"] = time.time()
        logging.info("broadcasting %s", (broadcast_data))
        await self.sio.emit("broadcast", broadcast_data, namespace="/default_evolver")
        self.running_broadcast = False
        return True

    def setup_event_handlers(self):
        self.sio.on("connect", self.on_connect, namespace="/default_evolver")
        self.sio.on("disconnect", self.on_disconnect, namespace="/default_evolver")
        self.sio.on("command", self.on_command, namespace="/default_evolver")
        self.sio.on(
            "getlastcommands", self.on_getlastcommands, namespace="/default_evolver"
        )
        self.sio.on(
            "getcalibrationnames",
            self.on_getcalibrationnames,
            namespace="default_evolver",
        )
        self.sio.on("getfitnames", self.on_getfitnames, namespace="/default_evolver")
        self.sio.on(
            "getcalibration", self.on_getcalibration, namespace="/default_evolver"
        )
        self.sio.on(
            "setrawcalibration", self.on_setrawcalibration, namespace="/default_evolver"
        )
        self.sio.on(
            "setfitcalibration",
            self.on_setfitcalibrations,
            namespace="/default_evolver",
        )
        self.sio.on(
            "setactivecal", self.on_setactiveodcal, namespace="/default_evolver"
        )
        self.sio.on("getactivecal", self.on_getactivecal, namespace="/default_evolver")
        self.sio.on(
            "getdevicename", self.on_getdevicename, namespace="/default_evolver"
        )
        self.sio.on(
            "setdevicename", self.on_setdevicename, namespace="/default_evolver"
        )
