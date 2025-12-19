import logging
import pathlib
import time
from typing import Annotated, Literal

import serial
import yaml
from fastapi import APIRouter, HTTPException, Path, Query, Request, Response, status
from pydantic import BaseModel, Field

from htevolver.data.interfaces import ExperimentDataStore
from htevolver.data.storage import PolarsDataStore
from htevolver.dependencies import ErrorModel, EvolverPacket, EvolverParameter
from htevolver.exceptions import (
    EvolverPacketError,
    PolarsDataStoreError,
)

router = APIRouter(prefix="/evolver")
logger = logging.getLogger(__name__)


class Evolver:
    """State management for eVOLVER hardware and related configurations"""

    def __init__(self, config: dict, config_path: pathlib.Path) -> None:
        self.config: dict = config
        self.config_path: pathlib.Path = config_path
        self.phase: Literal[0, 1, 2, 3] = 0
        self.running_immediate: bool = False
        self.running_recurring: bool = False
        self.command_queue: list[EvolverPacket] = []
        self.experiments: dict[str, ExperimentDataStore] = {}

        self.parameter_table: dict[str, EvolverParameter] = {}
        for parameter_name, parameter_config in config["parameter_config"].items():
            self.parameter_table[parameter_name] = EvolverParameter.create(parameter_name, parameter_config)

        self.serial_connection: serial.Serial = serial.Serial(
            port=self.config.get("serial_port", None),
            baudrate=self.config.get("serial_baudrate", 9600),
            timeout=self.config.get("serial_timeout", None),
        )

        logger.info("eVOLVER state initialized")

    def save_conf(self):
        """Saves internal eVOLVER configuration to YAML file"""
        with self.config_path.open("w") as conf_file:
            yaml.dump(self.config, conf_file)
        logger.info("eVOLVER state config saved to YAML")

    def execute_commands(self, packet_queue: list[EvolverPacket]) -> list[EvolverPacket]:
        """Low level function that RS485 serial communication on eVOLVER

        1. Send outgoing packet to RS485 bus
        2. Waits
        3. Reads incoming response packet on RS485 bus
        4. Sends acknowledgment packet to RS485 bus
        5. Process recently received sensor data

        Args:
            command_packet(EvolverPacket): The command to send.

        Returns:
            response_packet(EvolverPacket): Response data converted to an EvolverPacket from RS485 node.
        """
        response_packets: list[EvolverPacket] = []
        while len(packet_queue) > 0:
            self.serial_connection.reset_input_buffer()
            self.serial_connection.reset_output_buffer()
            max_attempts: int = 3
            timeout_delay: float = 0.1
            command_packet = packet_queue.pop()
            while max_attempts > 0:
                try:
                    logger.debug(f"Sending request packet to RS485 bus: {command_packet}")
                    self.serial_connection.write(command_packet.encoded_packet)
                    max_attempts -= 1
                except serial.SerialTimeoutException:
                    logger.warning("SerialTimeout exception encountered, trying again...")
                    time.sleep(timeout_delay)
                    timeout_delay *= 2

            response_bytes: bytearray = bytearray(self.serial_connection.read_until(expected=b"\x00"))
            if not response_bytes:
                raise EvolverPacketError("No response packet received")
            logger.debug(f"Packet response from RS485 bus: {response_bytes.hex(' ')}")

            try:
                response_packet = EvolverPacket.create_from_bytes(response_bytes, encode=True)
                logger.debug(f"Decoded response packet from RS485 bus: {response_packet}")
            except EvolverPacketError:
                logger.exception(f"Error processing response packet: {response_bytes.hex(' ')}", stack_info=True)
                raise

            logger.debug(f"Sending ack packet to RS485 bus: {command_packet}")
            if command_packet.ack_encoded_packet:
                self.serial_connection.write(command_packet.ack_encoded_packet)

            # wait for full packet transmission to arduino in a dynamic fashion since packet lengths will vary
            self.serial_connection.flush()
            response_packets.append(response_packet)
        return response_packets


class CommandBody(BaseModel):
    parameter_name: str = Field(examples=["stir"], title="Name of EvolverParameter.")
    payload: list[int] = Field(examples=[[50, 50, 50, 50]], title="Command payload, must match configured payload length.")


@router.post(
    "/command",
    status_code=status.HTTP_200_OK,
    responses={
        status.HTTP_404_NOT_FOUND: {"model": ErrorModel, "description": "Invalid query value used"},
        status.HTTP_503_SERVICE_UNAVAILABLE: {"model": ErrorModel, "description": "Running eVOLVER loop"},
    },
)
async def send_command(command: CommandBody, request: Request):
    """Process immediate eVOLVER commands received from clients if not running an evolver_loop"""
    evolver: Evolver = request.state.evolver
    if evolver.running_recurring:
        raise HTTPException(status_code=status.HTTP_503_SERVICE_UNAVAILABLE, detail="eVOLVER loop is currently running")
    if command.parameter_name not in request.state.parameters:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=f"EvolverParameter {command.parameter_name} not found")
    evolver_parameter = evolver.parameter_table[command.parameter_name]
    if len(command.payload) != evolver_parameter.payload_length:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Payload length invalid: received {len(command.payload)}, expected {evolver_parameter.payload_length}",
        )

    evolver.running_immediate = True
    logger.debug(f"Sending {command.parameter_name} payload: {command.payload}")
    try:
        evolver.execute_commands(
            [
                EvolverPacket(
                    address=command.parameter_name,
                    payload_length=evolver_parameter.payload_length,
                    command_type="request",
                    payload=command.payload,
                )
            ]
        )
        logger.debug(f"Finished command for {command.parameter_name}")
    except EvolverPacketError as e:
        logger.warning("Failed to process EvolverPacket", stack_info=True)
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=str(e))


@router.post(
    "/load_experiment",
    status_code=status.HTTP_201_CREATED,
    responses={status.HTTP_500_INTERNAL_SERVER_ERROR: {"model": ErrorModel, "description": "Error loading ExperimendDataStore"}},
)
async def load_experiment(experiment: Annotated[str, Query(title="Name of ExperimentDataStore file to load.")], request: Request):
    """Load a stored ExperimentDataStore and add it to collection of app managed experiments"""
    evolver: Evolver = request.state.evolver
    logger.info(f"Loading ExperimentDataStore into memory: {experiment}")
    try:
        evolver.experiments[experiment] = PolarsDataStore.load_store(experiment)
    except PolarsDataStoreError:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=f"Could not load experiment data for {experiment}"
        )


class CreateExperimentBody(BaseModel):
    name: str = Field(title="Name for new ExperimentDataStore. Must be unique.")
    store_type: str = Field(title="Type of ExperimentDataStore to use", examples=["PolarsDataStore"])
    store_schema: dict[str, str] = Field(
        title="Schema to use for ExperimentDataStore. Dictates what type of data is stored from HT-eVOLVER"
    )
    smart_stations: list[int] = Field(title="List of SmartStations to track.", examples=[0, 1, 2, 3])


@router.post(
    "/create_experiment",
    status_code=status.HTTP_201_CREATED,
    responses={status.HTTP_400_BAD_REQUEST: {"model": ErrorModel, "description": "Invalid ExperimentDataStore name"}},
)
async def create_experiment(experiment: CreateExperimentBody, request: Request):
    """Create an ExperimentDataStore and add it to app managed experiments"""
    logger.info(f"Received request to create experiment: {experiment.name}")
    evolver: Evolver = request.state.evolver
    if experiment.name in evolver.experiments:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST, detail=f"Experiment with name {experiment.name} already in use"
        )
    temp_filepath: pathlib.Path = pathlib.Path.joinpath(
        evolver.config["experiments"]["parent_directory"], f"{evolver.config['experiment_name']}.parquet"
    )
    if temp_filepath.exists():
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=f"Experiment file already exists at: {temp_filepath}")
    evolver.experiments[experiment.name] = PolarsDataStore(**experiment.model_dump())


class DataResponse(Response):
    media_type = "application/octet-stream"

    def render(self, content: bytes) -> bytes:
        return content


@router.get(
    "/vial_data/{experiment}/{station}/{vial}/{data_num}",
    response_class=DataResponse,
    status_code=status.HTTP_200_OK,
    responses={
        status.HTTP_404_NOT_FOUND: {"model": ErrorModel, "description": "Invalid query value used"},
        status.HTTP_500_INTERNAL_SERVER_ERROR: {"model": ErrorModel, "description": "Internal error quering ExperimentDataStore"},
    },
)
async def vial_data(
    request: Request,
    experiment: Annotated[str, Path(title="Name of ExperimentDataStore to get data from.", example="station_0")],
    station: Annotated[str, Path(title="Name of SmartStation target vial resides in.", example="station_0")],
    vial: Annotated[str, Path(title="Culture vial label.", example="vial_0")],
    data_num: Annotated[int, Path(title="Number of vial data records to get", example=10)],
    parameters: Annotated[list[str], Query(title="ExperimentDataStore schema columns to get data from")] = [],
) -> bytes:
    """Get individual vial data from an active ExperimentDataStore based on desired columns and number of records. Packs data into a bytes-serialized instance of an ExperimentDataStore"""
    evolver: Evolver = request.state.evolver
    station_id: int = int(station.split("_")[-1])
    vial_id: int = int(vial.split("_")[-1])
    if experiment not in evolver.experiments:
        msg: str = f"{experiment} not found"
        logger.exception(msg, stack_info=True)
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=msg)
    if station_id not in evolver.experiments[experiment].smart_stations:
        msg: str = f"Queried SmartStation not in target ExperimentDataStore: {evolver.experiments[experiment].smart_stations}"
        logger.exception(msg, stack_info=True)
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=msg)
    for parameter in parameters:
        if parameter not in evolver.parameter_table:
            msg: str = f"EvolverParameter {parameter} not found"
            logger.exception(msg, stack_info=True)
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=msg)

    try:
        return evolver.experiments[experiment].get_vial_data(station_id, vial_id, data_num, parameters)
    except PolarsDataStoreError:
        msg: str = f"Error trying to query ExperimentDataStore {experiment}, check logs"
        logger.exception(msg, stack_info=True)
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=msg)


@router.get(
    "/station_data/{experiment}/{station}/{data_num}",
    response_class=DataResponse,
    status_code=status.HTTP_200_OK,
    responses={
        status.HTTP_404_NOT_FOUND: {"model": ErrorModel, "description": "Invalid query value used"},
        status.HTTP_500_INTERNAL_SERVER_ERROR: {"model": ErrorModel, "description": "Internal error quering ExperimentDataStore"},
    },
)
async def station_data(
    request: Request,
    experiment: Annotated[str, Path(title="Name of ExperimentDataStore to get data from.", example="station_0")],
    station: Annotated[str, Path(title="Name of SmartStation target vial resides in.", example="station_0")],
    data_num: Annotated[int, Path(title="Number of vial data records to get", example=10)],
    parameters: Annotated[
        list[str],
        Query(
            title="Desired EvolverParameter(s) to get data from. Empty list results in using the entire ExperimentDataStore schema."
        ),
    ] = [],
) -> bytes:
    """Get SmartStation-wide culture data from an active ExperimentDataStore based on desired columns and number of records. Packs data into a bytes-serialized instance of an ExperimentDataStore"""
    evolver: Evolver = request.state.evolver
    station_id: int = int(station.split("_")[-1])
    if experiment not in evolver.experiments:
        msg: str = f"{experiment} not found"
        logger.exception(msg, stack_info=True)
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=msg)
    if station_id not in evolver.experiments[experiment].smart_stations:
        msg: str = f"Queried SmartStation not in target ExperimentDataStore: {evolver.experiments[experiment].smart_stations}"
        logger.exception(msg, stack_info=True)
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=msg)
    for parameter in parameters:
        if parameter not in evolver.parameter_table:
            msg: str = f"{parameter} not found"
            logger.exception(msg, stack_info=True)
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=msg)

    try:
        return evolver.experiments[experiment].get_station_data(station_id, data_num, parameters)
    except PolarsDataStoreError:
        msg: str = f"error trying to query ExperimentDataStore {experiment}, check logs"
        logger.exception(msg, stack_info=True)
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=msg)


@router.get(
    "/parameter_data/{experiment}/{parameter}/{data_num}",
    response_class=DataResponse,
    status_code=status.HTTP_200_OK,
    responses={
        status.HTTP_404_NOT_FOUND: {"model": ErrorModel, "description": "Invalid query value used"},
        status.HTTP_500_INTERNAL_SERVER_ERROR: {"model": ErrorModel, "description": "Internal error quering ExperimentDataStore"},
    },
)
async def parameter_data(
    request: Request,
    experiment: Annotated[str, Path(title="Name of ExperimentDataStore to get data from.", example="station_0")],
    data_num: Annotated[int, Path(title="Number of vial data records to get", example=10)],
    parameter: Annotated[str, Path(title="Desired EvolverParameter to get data from.")],
) -> bytes:
    """Get experiment-wide culture data for a specific EvolverParameter from an active ExperimentDataStore. Packs data into a bytes-serialized instance of an ExperimentDataStore"""
    evolver: Evolver = request.state.evolver
    if experiment not in evolver.experiments:
        msg: str = f"{experiment} not found"
        logger.exception(msg, stack_info=True)
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=msg)
    if parameter not in evolver.parameter_table:
        msg: str = f"{parameter} not found"
        logger.exception(msg, stack_info=True)
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=msg)

    try:
        return evolver.experiments[experiment].get_parameter_data(parameter, data_num)
    except PolarsDataStoreError:
        msg: str = f"error trying to query ExperimentDataStore {experiment}, check logs"
        logger.exception(msg, stack_info=True)
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=msg)


class EvolverStatusResponse(BaseModel):
    phase: Literal[0, 1, 2, 3] = Field(title="Current phase of evolver loop.")
    command_queue: list[dict] = Field(title="Current queue of EvolverPackets to send over RS485 bus")
    running_immediate: bool = Field(title="Boolean representing state of running immediate HT-eVOLVER commands.")
    running_recurring: bool = Field(title="Boolean representing state of running evolver loop commands.")


@router.get("/status", status_code=status.HTTP_200_OK)
async def evolver_status(request: Request) -> EvolverStatusResponse:
    """Get the current status of the eVOLVER system."""
    evolver: Evolver = request.state.evolver
    return EvolverStatusResponse(
        phase=evolver.phase,
        command_queue=[vars(command) for command in evolver.command_queue],
        running_immediate=evolver.running_immediate,
        running_recurring=evolver.running_recurring,
    )


@router.get("/config", status_code=status.HTTP_200_OK)
async def config(request: Request) -> dict:
    """Get the current configuration of the eVOLVER system."""
    evolver: Evolver = request.state.evolver
    return evolver.config
