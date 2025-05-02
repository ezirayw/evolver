import logging
import time

import socketio

from htevolver.shared import HTEvolverStatus, RoboticsRoutines, RoboticsState, RoboticsStatus, xArmStatus

logger = logging.getLogger(__name__)


def routine_decorator(func):
    """Decorator for sending routine jobs to HTeVOLVER

    Handles checking ready state and logging

    Args:
        func (callable): The helper function to decorate.

    Returns:
        callable: The wrapped function.
    """

    def wrapper(self, *args, **kwargs):
        if self.check_ready():
            # run the target function
            func(self, *args, **kwargs)
            logger.info(f"Initiating the {func.__name__.upper()} with the command: {args[0]}")
        else:
            logger.warning(f"Robotics namespace not ready to handle the routine {func.__name__.upper()}")

        self.status.state = RoboticsState.IDLE

    return wrapper


class RoboticsClientNamespace(socketio.ClientNamespace):
    def __init__(self, save: bool, directory: str, status: HTEvolverStatus, namespace: str = "/robotics"):
        super().__init__(namespace)
        self.save: bool = save
        self.directory: str = directory
        self.status: HTEvolverStatus = status
        self.status.robotics_ns = RoboticsStatus(
            state=RoboticsState.IDLE,
            routine=RoboticsRoutines.NO_ROUTINE,
            active_station=-1,
            active_pumps=[],
            vial_window=[],
            xArm=xArmStatus(),
        )
        self.server_conf: dict = {}
        self.server_types: dict[str, dict[str, int]] = {}
        self.ack: bool = False

    def on_connect(self, *args):
        self.request_robotics_status()
        self.request_robotics_conf()
        self.request_types()
        logger.info("Client connected to HTeVOVLER server via robotics namespace")

    def on_disconnect(self, *args):
        logger.info("Client disconnected from HTeVOLVER server via robotics namespace")

    def on_reconnect(self, *args):
        logger.info("Client reconnected to HTeVOLVER server via robotics namespace")

    def on_broadcast(self, broadcast_data: dict):
        self.status.robotics_ns = RoboticsStatus.from_dict(broadcast_data)
        logger.info(f"Robotics namespace broadcast: {self.status}")

    def on_get_status(self, status: dict):
        self.status.robotics_ns = RoboticsStatus.from_dict(status)
        logger.info(f"Robotics namespace broadcast processed: {self.status}")

    def on_get_conf(self, data: dict):
        self.server_conf = data
        logger.info("Received robotics namespace configuration from HTeVOLVER server.")

    def on_get_types(self, types: dict):
        self.server_types = types
        logger.info("Received robotics namespace data types info from HTeVOLVER server.")

    def acknowledge(self):
        self.ack: bool = True

    def request_robotics_status(self):
        self.emit("request_status", callback=self.acknowledge)
        logger.info("Requesting robotics namespace status from HTeVOLVER server.")

    def request_robotics_conf(self):
        self.emit("request_conf")
        logger.info("Requesting robotics namespace configuration from HTeVOLVER server.")

    def request_types(self):
        self.emit("request_types")
        logger.info("Requesting robotics namespace data types from HTeVOLVER server.")

    def connect_xArm(self):
        self.emit("connect_xArm")
        logger.info("Reconnecting xArm to HTeVOLVER  server")

    def override_status(self, override_commands: dict):
        self.emit("on_override_status", override_commands)
        logger.info(f"Overriding robotics namespace state on HTeVOLVER server with: {override_commands}")

    # experiment management functions
    def pause_experiment(self):
        self.emit("pause_robotics")
        logger.info("Paused experiment")

    def resume_experiment(self):
        self.emit("resume_robotics")
        logger.info("Resumed experiment")

    def stop_experiment(self):
        self.emit("stop_robotics")
        logger.info("Stopped experiment")

    def check_ready(self):
        self.ack = False
        self.request_robotics_status()
        while not self.ack:
            time.sleep(0.1)

        if self.status.robotics_ns.routine == RoboticsRoutines.NO_ROUTINE:
            return True
        else:
            return False

    @routine_decorator
    def pipette(self, pipette_commands: dict):
        self.emit("pipette_routine", pipette_commands)

    @routine_decorator
    def prime_syringe_pumps(self, prime_commands: dict):
        self.emit("prime_pumps", prime_commands)

    @routine_decorator
    def dilutions(self, dilution_commands: dict):
        self.emit("dilution_routine", dilution_commands)

    @routine_decorator
    def fill_vials(self, fill_commands: dict):
        self.emit("fill_vials_routine", fill_commands)


if __name__ == "__main__":
    print("Please run eVOLVER.py instead")
