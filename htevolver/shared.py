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
    PUMP_INITIALIZE = 7


class CommandTags(Enum):
    REQUEST = 0
    ACKNOWLEDGE = 1
    SENSOR = 2
    ECHO = 3
    CONFIG = 4


#### BROADCAST DATA STORAGE CLASSES ####
@dataclass
class BroadcastData:
    phase: int = field(default=0)
    timestamp: float = field(default=0.0)
    data: dict[str, list[int]] = field(default_factory=dict)
    config: str = field(default="")


#### DATA STORAGE CLASSES ####
@dataclass
class EvolverCommand:
    param: str
    address: int
    value: list[int]
    immediate: bool
    recurring: bool


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
