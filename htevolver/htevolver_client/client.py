import logging
import os
import time

import socketio
from evolver_namespace_client import EvolverClientNamespace
from robotics_namespace_client import RoboticsClientNamespace
from shared import HTEvolverStatus

logger = logging.getLogger(__name__)


class HTEvolverClient:
    """
    Master client for interacting with the HT Evolver system.

    This class manages connections to both the eVOLVER and robotics namespaces,
    allowing access to both hardware components from a single interface.
    Users interact directly with the namespace objects for specific functionality.

    Attributes:
        ip (str): IP address of the eVOLVER server
        port (int): Port number of the eVOLVER server
        save (bool): Whether to save data locally
        directory (str): Directory for saving data
        sio (socketio.Client): SocketIO client for communication
        evolver (HTEvolverClientNamespace): Namespace for eVOLVER control
        robotics (RoboticsClientNamespace): Namespace for robotics control
        connected (bool): Whether the client is connected to the server
    """

    def __init__(
        self,
        ip: str,
        directory: str,
        save: bool = False,
        station_ids: list[int] = [],
        port: int = 8081,
        data_window_length: int = 10,
    ):
        """
        Initialize the HT Evolver client.

        Args:
            ip: IP address of the eVOLVER server
            port: Port number of the eVOLVER server (default: 8081)
            save: Whether to save data locally (default: True)
            directory: Directory for saving data (default: ~/htevolver_data)
            station_ids: List of station IDs to control (default: [0,1,2,3])
            data_window_length: Length of data window for storing measurements (default: 10)
        """
        self.ip = ip
        self.directory = directory
        self.save = save
        self.station_ids = station_ids or [0, 1, 2, 3]
        self.port = port
        self.data_window_length = data_window_length

        # Create socketio client
        self.sio = socketio.Client()

        # Create data directory if it doesn't exist
        if self.save and not os.path.exists(self.directory):
            os.makedirs(self.directory)
            for station_id in self.station_ids:
                station_dir = os.path.join(self.directory, f"station_{station_id}")
                if not os.path.exists(station_dir):
                    os.makedirs(station_dir)

        self.status = HTEvolverStatus(connected=False, start_time=time.time(), elapsed_time=0.0)

        self.evolver = EvolverClientNamespace(
            save=self.save,
            directory=self.directory,
            status=self.status,
            station_ids=self.station_ids,
            data_window_length=self.data_window_length,
        )
        self.robotics = RoboticsClientNamespace(save=self.save, directory=self.directory, status=self.status)

        self.sio.register_namespace(self.evolver)
        self.sio.register_namespace(self.robotics)

    def connect(self) -> None:
        """
        Connect to the eVOLVER server.

        This initiates connection to both the eVOLVER and robotics namespaces.
        After connection, it requests calibration data for temperature and OD sensors.
        """
        try:
            server_address = f"http://{self.ip}:{self.port}"
            logger.info(f"Connecting to HT Evolver server at {server_address}")
            self.sio.connect(server_address)
            self.connected = True

            # Request calibration data after connection
            self.evolver.request_calibration("temp")
            self.evolver.request_calibration("od")

            # Request robotics status and configuration
            self.robotics.request_robotics_status()
            self.robotics.request_robotics_conf()
            self.robotics.request_types()

            logger.info("Successfully connected to HT Evolver server")
        except Exception as e:
            logger.error(f"Failed to connect to HT Evolver server: {e}")
            raise ConnectionError(f"Could not connect to HT Evolver server: {e}")

    def disconnect(self) -> None:
        """Disconnect from the eVOLVER server."""
        if self.connected:
            try:
                self.sio.disconnect()
                self.connected = False
                logger.info("Disconnected from HT Evolver server")
            except Exception as e:
                logger.error(f"Error during disconnection: {e}")
