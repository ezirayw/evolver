import logging
from dataclasses import dataclass
from typing import Any, Literal

import urllib3
from fastapi import status

from htevolver.dependencies import StationInfluxCommandBody

logger = logging.getLogger(__name__)


@dataclass
class ClientResponse:
    success: bool
    data: Any | None = None


class HTEvolverClient:
    """Simple user interface for HT-eVOLVER. Formats HTTP requests and processes response data.

    Useful for building CLI and scripting applications to create, manage, and analyze culture routines
    All methods, except those querying culture data from experiments, that make a single or several HT-eVOLVER requests return a ClientResponse instance.
    Methods that query culture data return instances of the underlining ExperimentDataStore type

    Attributes:
        url (str): HT-eVOLVER server URL. Used to make all underlining requests to server.
    """

    def __init__(self, url: str):
        self.robotics_url: str = url + "/robotics"
        self.evolver_url: str = url + "/evolver"
        self.http: urllib3.PoolManager = urllib3.PoolManager()
        self.robotics_config: dict = {}
        self.robotics_status: dict = {}
        self.evolver_config: dict = {}

    def init_robotics(self, dispenseheads: list[str] = []) -> ClientResponse:
        """Initialize HT-eVOLVER robotics by making the following requests:
        1. Initializing desired DispenseHeads
        2. Connect server to xArm
        3. Enabling desired DispenseHeads."""

        logger.info("Initializing HT-eVOLVER robotics...")
        response = self.http.request(
            "POST",
            self.robotics_url + "/initialize_dispenseheads",
            json={"targets": dispenseheads},
            headers={"Content-Type": "application/json"},
        )
        if response != status.HTTP_204_NO_CONTENT:
            return ClientResponse(False, data=response.data)
        if self.http.request("POST", self.robotics_url + "/connect_xarm").status != status.HTTP_204_NO_CONTENT:
            return ClientResponse(False, data=response.data)
        if dispenseheads:
            for dispensehead in dispenseheads:
                if (
                    self.http.request("POST", self.robotics_url + f"/enable_head/{dispensehead}").status
                    != status.HTTP_204_NO_CONTENT
                ):
                    return ClientResponse(False, data=response.data)
            return ClientResponse(True)
        else:
            response = self.http.request("POST", self.robotics_url + "/enable_heads")
            return ClientResponse(success=(response.status == status.HTTP_204_NO_CONTENT), data=response.data)

    def get_status(self, mode: Literal["evolver", "robotics"]) -> ClientResponse:
        """Get hardware information from HT-eVOLVER. Setting the mode dictates either the eVOLVER or robotics module."""
        logger.info(f"Retreiving {mode.upper()} status information...")
        if mode == "evolver":
            response = self.http.request("GET", self.evolver_url + "/status")
            return ClientResponse(success=(response.status == status.HTTP_200_OK), data=response.json())

        if mode == "robotics":
            response = self.http.request("GET", self.robotics_url + "/status")
            return ClientResponse(success=(response.status == status.HTTP_200_OK), data=response.json())

    def get_config(self, mode: Literal["evolver", "robotics"]) -> ClientResponse:
        """Get hardware configuration from HT-eVOLVER. Setting the mode dictates either eVOVLER or robotics module config data."""
        logger.info(f"Retreiving {mode.upper()} config information...")
        if mode == "evolver":
            response = self.http.request("GET", self.evolver_url + "/config")
            return ClientResponse(success=response.status == status.HTTP_200_OK, data=response.data)

        if mode == "robotics":
            response = self.http.request("GET", self.robotics_url + "/config")
            return ClientResponse(success=response.status == status.HTTP_200_OK, data=response.data)

    def update_temp(self, station_ids: int | list[int], temp_setpoints: list[float]):
        """Update temperature setpoints for specified SmartStations."""
        logger.info(f"Changing SmartStation(s) {station_ids} temperature setpoints to {temp_setpoints}...")

    def update_stir(self, station_ids: int | list[int], stir_settings: list[int]):
        """Update stir setting for specified SmartStations."""
        logger.info(f"Changing SmartStation(s) {station_ids} stir settings to {stir_settings}...")

    def get_temp_data(self, station_ids: int | list[int], num_data_points: int = 1):
        """Get temperature data for specified SmartStations."""
        logger.info(f"Getting temperature data for SmartStation(s) {station_ids}...")

    def get_od_data(self, station_ids: list[int], num_data_points: int = 1):
        """Get optical density data for specified SmartStations."""
        logger.info(f"Getting OD data for SmartStation(s) {station_ids}...")

    def pipette(self, pump_0: int, pump_1: int, pump_2: int, pump_3: int, pump_4: int, pump_5: int):
        """Execute a pipette operation with currently in use DispenseHead."""
        logger.info("Pipetting with in use DispenseHead...")

    def prime_dispenseheads(self, dispenseheads: str | list[str] = []):
        """Prime the specified DispenseHead. Will prime all configured syringe pump ports."""
        logger.info(f"Priming DispenseHeads {dispenseheads}...")

    def create_influx_command(self, dispensehead: str, station_id: int, vial_volumes: dict[str, int]) -> StationInfluxCommandBody:
        """Create a valid SmartStation influx command for a given DispenseHead."""
        return StationInfluxCommandBody(dispensehead=dispensehead, station_id=station_id, **vial_volumes)

    def influx(
        self,
        station_0: StationInfluxCommandBody | list[StationInfluxCommandBody] | None = None,
        station_1: StationInfluxCommandBody | list[StationInfluxCommandBody] | None = None,
        station_2: StationInfluxCommandBody | list[StationInfluxCommandBody] | None = None,
        station_3: StationInfluxCommandBody | list[StationInfluxCommandBody] | None = None,
    ):
        """Execute multi-fluid influx into specific vials across SmartStations for vial dilutions and filling vials prior to experiments."""
        logger.info("Initiating influx routine...")

    def efflux(self, station_ids: int | list[int]):
        """Execute efflux for specificed SmartStations. Efflux volume is uniform across all vials within a SmartStation"""
        logger.info(f"Initiating efflux for SmartStation(s) {station_ids}")

    def pause(self):
        """Pause active robotic routines on HT-eVOLVER.

        Sends a pause request to suspend routines. Useful for facilitating manual interventions during experiments,
        like exchanging fluid reservoirs, culture sampling, and/or troubleshooting.

        Examples:
            >>> client.pause()
        """
        self.robotics._pause()

    def resume(self):
        """Resumes recently paused robotic routines on HT-eVOLVER.

        Sends a resume request to resume paused routines. Useful for facilitating manual interventions during experiments,
        like exchanging fluid reservoirs, culture sampling, and/or troubleshooting.

        Examples:
            >>> client.resume()
        """
        self.robotics._resume()

    def stop(self):
        """Kills active robotics routines on HT-eVOLVER.

        Sends a stop request to gracefully exit active robotics routines. Useful for conditionally ending experiments.

        Examples:
            >>> client.stop()
        """
        self.robotics._stop()
