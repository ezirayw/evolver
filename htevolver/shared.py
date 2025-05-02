import time
from dataclasses import dataclass, field
from enum import Enum


class FluidTypes(Enum):
    EMPTY = 0
    MEDIA = 1
    DRUG = 2
    STERILIZE = 3


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
    DILUTION = 1
    PIPETTE = 2
    FILLING_VIALS_PUMPS = 3
    FILLING_VIALS_IPP = 4
    PRIMING_INFLUX = 5
    PRIMING_EFFLUX = 6


class CommandTags(Enum):
    REQUEST = 0
    ACKNOWLEDGE = 1
    SENSOR = 2
    ECHO = 3
    CONFIG = 4


#### BROADCAST DATA STORAGE CLASSES ####


@dataclass
class BroadcastData:
    ip: str = field(default="")
    phase: int = field(default=0)
    timestamp: float = field(default=0.0)
    data: dict[str, list[int]] = field(default_factory=dict)
    config: str = field(default="")


#### DATA STORAGE CLASSES ####
@dataclass
class ServerResult:
    done: bool
    namespace: str
    routine: str
    status: dict
    elapsed_time: float
    message: str


@dataclass
class SerialCommand:
    param: str
    address: int
    value: list[int]
    tag: CommandTags


@dataclass
class EvolverCommand:
    param: str
    address: int
    value: list[int]
    immediate: bool
    recurring: bool


#### STATUS MANAGEMENT CLASSES ####
@dataclass
class xArmStatus:
    warning_code: int = 0
    error_code: int = 0
    state: int = 0
    connected: bool = False


@dataclass
class RoboticsStatus:
    state: RoboticsState
    routine: RoboticsRoutines
    active_station: int
    active_pumps: list[dict]
    vial_window: list[int]
    xArm: xArmStatus

    def to_dict(self):
        return {
            "state": self.state.value,
            "routine": self.routine.value,
            "active_station": self.active_station,
            "active_pumps": self.active_pumps,
            "vial_window": self.vial_window,
            "xArm": {
                "warning_code": self.xArm.warning_code,
                "error_code": self.xArm.error_code,
                "arm_state": self.xArm.state,
                "connected": self.xArm.connected,
            },
        }

    @classmethod
    def from_dict(cls, data: dict):
        return cls(
            state=RoboticsState(data["state"]),
            routine=RoboticsRoutines(data["routine"]),
            active_station=data["active_station"],
            active_pumps=data["active_pumps"],
            vial_window=data["vial_window"],
            xArm=xArmStatus(
                warning_code=data["xArm"]["warning_code"],
                error_code=data["xArm"]["error_code"],
                state=data["xArm"]["arm_state"],
                connected=data["xArm"]["connected"],
            ),
        )


@dataclass
class EvolverStatus:
    phase: int
    command_queue: list[SerialCommand]
    running_immediate: bool
    running_broadcast: bool

    def to_dict(self):
        return {
            "phase": self.phase,
            "command_queue": [
                {"param": command.param, "address": command.address, "value": command.value, "tag": command.tag.value}
                for command in self.command_queue
            ],
            "running_immediate": self.running_immediate,
            "running_broadcast": self.running_broadcast,
        }

    @classmethod
    def from_dict(cls, data):
        command_queue = []
        for command in data["command_queue"]:
            command_queue.append(
                SerialCommand(
                    param=command["param"], address=command["address"], value=command["value"], tag=CommandTags(command["tag"])
                )
            )

        return cls(
            phase=data["phase"],
            command_queue=command_queue,
            running_immediate=data["running_immediate"],
            running_broadcast=data["running_broadcast"],
        )


@dataclass
class HTEvolverStatus:
    connected: bool = field(default=False)
    start_time: float = field(default=0.0)
    elapsed_time: float = field(default=0.0)
    evolver_ns: EvolverStatus = field(init=False)
    robotics_ns: RoboticsStatus = field(init=False)

    def get_elapsed_time(self):
        """Get the current elapsed time since experiment start (in hours)."""
        return round((time.time() - self.start_time) / 3600, 4)

    def reset_time(self):
        self.start_time = time.time()
