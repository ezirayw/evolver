from datetime import datetime
from pathlib import Path
from typing import Any, ClassVar

import polars as pl
from polars.exceptions import InvalidOperationError, ShapeError

from htevolver.data.interfaces import ExperimentDataStore
from htevolver.dependencies import SmartStationData
from htevolver.exceptions import PolarsDataStoreError


def format_batch_data(culture_data: dict[str, dict[str, SmartStationData]], stations: list[int]) -> dict:
    """Format batch SmartStationData into a dictionary for insertion into a ExperimentDataStore

    Args:
        culture_data (dict): Batch SmartStation culture data collected at the end of an evolver_loop for each recurring EvolverParameter
        timestamp (float): The timestamp to associate with these measurements.

    Returns:
        dict: Dictionary with keys as column names and values as lists for insertion into PolarsDataStore.
    """
    vial_datas = []
    parameter_names = list(culture_data.keys())
    vials_per_station: int = 18
    for station_id in stations:
        for vial_id in range(vials_per_station):
            vial_data = {
                "smart_station_id": station_id,
                "vial_id": vial_id,
                "timestamp": datetime,
            }
            for parameter in parameter_names:
                # Use .get() to handle missing data gracefully
                vial_data[parameter] = (
                    culture_data.get(parameter, {}).get(f"station_{station_id}", {}).get(f"vial_{vial_id}", None)
                )
            vial_datas.append(vial_data)

    if not vial_datas:
        return {}

    column_names: list[str] = parameter_names + ["smart_station_id", "vial_id", "timestamp"]
    column_data = {column: [] for column in column_names}
    for vial_data in vial_datas:
        for column in column_data:
            column_data[column].append(vial_data[column])

    return column_data


class PolarsDataStore(ExperimentDataStore):
    """ExperimentDataStore variant using polars dataframe for data storage.

    Class instance parent_directory must be set prior to creating PolarsDataStore instances for safe saving and loading.
    """

    parent_directory: ClassVar[Path | None] = None
    required_schema_members: ClassVar[dict[str, pl.DataType]] = {
        "smart_station_id": pl.Int8(),
        "vial_id": pl.Int8(),
        "timestamp": pl.Datetime(),
    }
    non_metadata_attrs: ClassVar[list[str]] = ["experiment_name", "schema", "df"]

    def __init__(self, experiment_name: str, schema: dict[str, pl.DataType], smart_stations: list[int] = [0, 1, 2, 3]):
        if not PolarsDataStore.parent_directory:
            raise PolarsDataStoreError("Error creating PolarsDataStore instance, parent directory not set")
        self.experiment_name: str = experiment_name
        self.schema: dict[str, pl.DataType] = {**schema, **PolarsDataStore.required_schema_members}

        self.df: pl.DataFrame = pl.DataFrame(schema=schema)

        # instance metadata
        self.smart_stations: list[int] = smart_stations

    @classmethod
    def set_parent_directory(cls, parent_directory: Path):
        """Setup the parent directory for all PolarsDataStore instances. Creates the parent directory if it doesnt exist"""
        if parent_directory.is_dir():
            cls.parent_directory = parent_directory
        else:
            Path.mkdir(parent_directory)

    @classmethod
    def load_store(cls, experiment_name: str) -> "PolarsDataStore":
        """Load in existing PolarsDataStore into memory from a file."""
        if not cls.parent_directory:
            raise PolarsDataStoreError("Error loading in PolarsDataStore, parent directory not set")
        for file in cls.parent_directory.iterdir():
            if file.is_file() and file.stem == experiment_name and file.suffix == ".parquet":
                data_store = cls(experiment_name=experiment_name, schema=pl.read_parquet_schema(file))
                data_store.df = pl.read_parquet(file)
                metadata: dict[str, Any] = pl.read_parquet_metadata(file)
                for attribute, value in metadata.items():
                    setattr(data_store, attribute, value)

                return data_store
        raise PolarsDataStoreError(f"Error loading in PolarsDataStore, {experiment_name} not found")

    def save_store(self):
        """Save current state of PolarsDataStore to memory"""
        metadata: dict[str, Any] = {}
        for attribute, value in vars(self).items():
            if attribute not in PolarsDataStore.non_metadata_attrs:
                metadata[attribute] = value

        filepath: Path = Path.joinpath(PolarsDataStore.parent_directory, self.experiment_name)
        self.df.write_parquet(filepath, metadata=metadata)

    def insert_data(self, new_data: dict[str, Any]):
        """Insert new vial culture data into PolarsDataStore"""
        temp_df: pl.DataFrame = pl.DataFrame(new_data)
        try:
            temp_df.cast(self.schema)
            self.df = pl.concat([self.df, pl.DataFrame(new_data)])
        except InvalidOperationError as e:
            raise PolarsDataStoreError(f"Error processing new data: {e}")
        except ShapeError as e:
            raise PolarsDataStoreError(f"Error inserting new data {self.schema}: {e}")

    def get_vial_data(
        self,
        station_id: int,
        vial_id: int,
        data_num: int = 10,
        parameters: list[str] = [],
    ) -> bytes:
        """Return a serialized DataFrame (bytes) representing culture data for specific vial. Filter based off desired parameters and number of datapoints.

        Args:
            station_id (int): The ID of the SmartStation the desired vial resides in.
            vial_id (int): The ID of the desired vial within the specified SmartStation.
            parameters (list[str]): List of parameter names to include in the result. If empty, all parameters are included.
            data_num (int, optional): The number of most recent data points to return. Defaults to 10.

        Raises:
            PolarsDataStoreError: If there is an error retrieving the data from the PolarsDataStore.

        Returns:
            bytes: A serialized DataFrame (as bytes) containing the requested data.
        """
        try:
            data_df: pl.LazyFrame = (
                self.df.lazy()
                .filter(
                    pl.col("station_id") == station_id,
                    pl.col("vial_id") == vial_id,
                )
                .select(parameters if parameters else pl.all())
            )
            row_count_df = data_df.clone()
            row_num: int = row_count_df.select(pl.len()).collect().item()

            vial_data_bytes: bytes = b""
            if row_num < data_num:
                vial_data_bytes = data_df.collect().serialize()
            else:
                vial_data_bytes = data_df.tail(data_num).collect().serialize()
            return vial_data_bytes

        except InvalidOperationError as e:
            raise PolarsDataStoreError(f"Error retreiving vial data: {e}")

    def get_station_data(self, station_id: int, data_num: int = 10, parameters: list[str] = []) -> bytes:
        """Return a serialized DataFrame (bytes) representing culture data for all vials within a SmartStation.

        Args:
            station_id (int): The ID of the SmartStation to retrieve data for.
            parameters (list[str]): List of parameter names to include in the result. If empty, all parameters are included.
            data_num (int, optional): The number of most recent data points to return. Defaults to 10.

        Raises:
            PolarsDataStoreError: If there is an error retrieving the data from the PolarsDataStore.

        Returns:
            bytes: A serialized DataFrame (as bytes) containing the requested data.
        """
        try:
            data_df: pl.LazyFrame = (
                self.df.lazy().filter(pl.col("station_id") == station_id).select(parameters if parameters else pl.all())
            )
            row_count_df = data_df.clone()
            row_num: int = row_count_df.select(pl.len()).collect().item()

            vial_data_bytes: bytes = b""
            if row_num < data_num:
                vial_data_bytes = data_df.collect().serialize()
            else:
                vial_data_bytes = data_df.tail(data_num).collect().serialize()
            return vial_data_bytes

        except InvalidOperationError as e:
            raise PolarsDataStoreError(f"Error retreiving vial data: {e}")

    def get_parameter_data(self, parameter: str, data_num: int = 10) -> bytes:
        """Return a serialized DataFrame (bytes) representing culture data for a specific parameter.

        Args:
            paramter (str): Desired culture parameter data to extract. Covers all vials across all SmartStations.
            data_num (int, optional): The number of most recent data points to return. Defaults to 10.

        Raises:
            PolarsDataStoreError: If there is an error retrieving the data from the PolarsDataStore.

        Returns:
            bytes: A serialized DataFrame (as bytes) containing the requested data.
        """
        try:
            data_df: pl.LazyFrame = self.df.lazy().select(parameter)
            row_count_df = data_df.clone()
            row_num: int = row_count_df.select(pl.len()).collect().item()

            vial_data_bytes: bytes = b""
            if row_num < data_num:
                vial_data_bytes = data_df.collect().serialize()
            else:
                vial_data_bytes = data_df.tail(data_num).collect().serialize()
            return vial_data_bytes

        except InvalidOperationError as e:
            raise PolarsDataStoreError(f"Error retreiving vial data: {e}")
