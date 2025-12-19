from pathlib import Path
from typing import Any, ClassVar, Protocol


class ExperimentDataStore(Protocol):
    """Container for SmartStation culture data using internal polars dataframes.

    Required schema items include:
       1. smart_station_id
       2. vial_id
       3. timestamp

    Each culture vial on HT-eVOLVER is mapped to a specific smart_station_id and vial_id, enabling centraliation of all experiment data
    onto a single polars dataframe. Additional schema members are collected during ExperimentDataStore creation.
    """

    experiment_name: str
    schema: dict[str, Any]
    smart_stations: list[int]
    required_schema_members: ClassVar[dict[str, Any]] = {
        "smart_station_id": Any,
        "vial_id": Any,
        "timestamp": Any,
    }
    parent_directory: ClassVar[Path | None]

    @classmethod
    def set_parent_directory(cls, parent_directory: Path) -> None:
        """Setup the parent directory for all ExperimentDataStore instances"""
        ...

    @classmethod
    def load_store(cls, experiment_name: str) -> "ExperimentDataStore":
        """Load ExperimentDataStore into memory"""
        ...

    def save_store(self):
        """Save current state of ExperimentDataStore to memory"""
        ...

    def insert_data(self, new_data: dict[str, Any]) -> None:
        """Store new culture data into central data unit."""
        ...

    def get_vial_data(self, station_id: int, vial_id: int, data_num: int, parameters: list[str]) -> bytes:
        """Return serialized culture data for a specific vial. Filter based off of desired parameters and number of datapoints."""
        ...

    def get_station_data(self, station_id: int, data_num: int, parameters: list[str]) -> bytes:
        """Return serialized culture data for all vials within a SmartStation. Filter based off of desired parameters and number of datapoints."""
        ...

    def get_parameter_data(self, parameter: str, data_num: int) -> bytes:
        """Return serialized culture data for a specific parameter. Filter based off of number of datapoints."""
        ...
