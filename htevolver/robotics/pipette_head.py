import logging
from dataclasses import asdict, dataclass, field
from enum import Enum

from tecancavro.models import XCaliburD
from tecancavro.syringe import SyringeError, SyringeTimeout
from tecancavro.transport import TecanAPISerial

from htevolver.exceptions import PipetteHeadError, RoboticsError

logger = logging.getLogger(__name__)


class FluidTypes(Enum):
    EMPTY = 0
    MEDIA = 1
    DRUG = 2
    STERILIZE = 3


def dict_factory_pipettehead(data):
    result = {}
    for key, value in data:
        if isinstance(value, Enum):
            result[key] = (value.name, value.value)
        if isinstance(value, XCaliburD):
            result[key] = "XCaliburD"
        else:
            result[key] = value
    return result


@dataclass
class PumpPort:
    id: int
    fluid: FluidTypes = field(default=FluidTypes.EMPTY)
    starting_volume: int = field(default=0)
    current_volume: int = field(default=0)
    primed: bool = field(default=False)

    def update(self, port_config: dict):
        for config_parameter, value in port_config.items():
            if hasattr(self, config_parameter):
                attribute_value = getattr(self, config_parameter)
                attr_type = type(attribute_value)
                try:
                    new_value = attr_type(value)
                    setattr(self, config_parameter, new_value)
                except (ValueError, TypeError):
                    logger.warning(
                        f"Invalid type for {config_parameter}: expected {attr_type.__name__}, got {type(value).__name__}"
                    )
                logger.debug(f"Updated PumpPort_{self.id} parameter: {config_parameter}={value}")


#### DECORATORS ####
def pump_action(func):
    """Decorator for pump actions that require connection verification/error handling.

    Args:
        func (callable): The helper function to decorate.

    Returns:
        callable: The wrapped function.
    """

    async def wrapper(self, *args, **kwargs):
        if self.connected:
            try:
                func(self, *args, **kwargs)
            except (SyringeError, SyringeTimeout) as e:
                logger.error(f"Error trying to run {func.__name__} on PipetteHead Pump_{self.pump_id}: {e}")
                raise PipetteHeadError(f"Error trying to run {func.__name__} on PipetteHead Pump_{self.pump_id}: {e}")
        else:
            raise PipetteHeadError(f"PipetteHead Pump_{self.id} cannot run {func.__name__}, not connected")

    return wrapper


@dataclass
class Pump:
    id: int
    primary: FluidTypes = field(default=FluidTypes.EMPTY)
    ports: dict[int, PumpPort] = field(default_factory=dict)
    serial_port: str = field(default="")
    head_port: int = field(default=1)
    active_port: int = field(default=2)
    connected: bool = field(default=False)
    hardware: XCaliburD = field(init=False)

    def __post_init__(self):
        if self.connected:
            self.hardware = XCaliburD(
                com_link=TecanAPISerial(self.id, ser_port=self.serial_port, ser_baud=9600),
            )

    def connect(self):
        self.connected = True
        # TODO check if hardware com_link exists, if not, re-establish

    def disconnect(self, delete: bool = False):
        if delete:
            del self.hardware.com_link
        self.connected = False

    @pump_action
    def initialize(self):
        self.hardware.init()

    @pump_action
    def aspirate(self, volume: int):
        if self.ports[self.active_port].current_volume < self.ports[self.active_port].starting_volume * 0.1:
            max_port = max(self.ports.keys())
            if self.active_port < max_port:
                self.active_port += 1
            else:
                raise RoboticsError(f"Cannot aspirate: Need to exchange fluid reservoir(s) for PipetteHead Pump_{self.id}")

        self.hardware.extract(self.active_port, volume)
        delay = self.hardware.executeChain()
        self.hardware.waitReady(delay)

    @pump_action
    def dispense(self, volume: int):
        self.hardware.dispense(self.head_port, volume)
        delay = self.hardware.executeChain()
        self.hardware.waitReady(delay)

    @pump_action
    def prime(self):
        for port_id, port in self.ports.items():
            self.hardware.primePort(in_port=port_id, out_port=self.head_port, volume_ul=800)

    @pump_action
    def pause(self):
        self.hardware.terminateCmd()

    @pump_action
    def stop(self):
        self.hardware.terminateCmd()
        self.hardware.resetChain()

    @pump_action
    def resume(self):
        self.hardware.sendRcv("", execute=True)

    def update(self, pump_config: dict):
        if pump_config["primary"] in FluidTypes.__members__:
            self.primary = FluidTypes[pump_config["primary"]]
        for port_id, port in pump_config.items():
            port.update(pump_config[port_id])


@dataclass
class PipetteHead:
    pumps: list[Pump]
    pump_num: int
    universal: bool
    num_windows: int
    active_pumps: list[Pump] = field(default_factory=list)

    @classmethod
    def create(cls, config: dict):
        pumps: list[Pump] = [Pump(0), Pump(1), Pump(2), Pump(3)]
        logger.debug(f"PipetteHead created using the config: {config}")
        for pump_id, pump in enumerate(pumps):
            try:
                pump.primary = FluidTypes[config["pumps"][pump_id]["primary"].upper()]
            except KeyError:
                logger.warning(
                    f"Invalid primary fluid type configuration for PipetteHead Pump_{pump_id}, using default {pump.primary}: {config['pumps'][pump_id]['primary']}"
                )

            for port_id, port_config in config["pumps"][pump_id]["ports"].items():
                logger.debug(f"PipetteHead Pump_{pump_id} Port_{port_id} created using the config: {port_config}")
                try:
                    pump.ports[port_id] = PumpPort(
                        id=port_id,
                        fluid=FluidTypes[port_config["fluid"].upper()],
                        starting_volume=port_config["volume"],
                        current_volume=port_config["volume"],
                        primed=port_config["primed"],
                    )
                except KeyError:
                    pump.ports[port_id] = PumpPort(id=port_id)
                    logger.warning(
                        f"Invalid fluid type configuration for Port_{port_id} in PipetteHead Pump__{pump_id}, using default {pump.ports[port_id].fluid}: {port_config['fluid']}"
                    )
            pump.serial_port = config["serial_port"]
            pump.connected = config["pumps"][pump_id]["connect"]

        pump_num: int = sum(1 for pump in pumps if pump.primary == FluidTypes.EMPTY)
        universal: bool = all(pump.primary == pumps[0].primary for pump in pumps)
        num_windows: int = 0
        if pump_num != 0 and universal:
            num_windows = int(6 / pump_num)
        if pump_num != 0 and not universal:
            num_windows = 6 + (pump_num - 1)
        return cls(pumps=pumps, pump_num=pump_num, universal=universal, num_windows=num_windows)

    def stop(self):
        for pump in self.active_pumps:
            pump.stop()

    def pause(self):
        for pump in self.active_pumps:
            pump.pause()

    def resume(self):
        for pump in self.active_pumps:
            pump.resume()

    def connect(self, pump_list: list[int]):
        for pump_id in pump_list:
            self.pumps[pump_id].connect()

    def disconnect(self, pump_list: list[int]):
        for pump_id in pump_list:
            self.pumps[pump_id].disconnect()

    async def prime(self, pump_list: list[int]):
        for pump_id, pump in enumerate(self.pumps):
            self.active_pumps.append(pump)
            pump.prime()
            self.active_pumps.remove(pump)

    async def aspirate(self, aspirate_volumes: list[int]):
        for pump_id, pump in enumerate(self.pumps):
            if aspirate_volumes[pump_id] >= 0:
                self.active_pumps.append(pump)
                pump.aspirate(aspirate_volumes[pump_id])
                self.active_pumps.remove(pump)

    async def dispense(self, dispense_volumes: list[int]):
        for pump_id, pump in enumerate(self.pumps):
            if dispense_volumes[pump_id] >= 0:
                self.active_pumps.append(pump)
                pump.dispense(dispense_volumes[pump_id])
                self.active_pumps.remove(pump)

    async def initialize(self, pump_list: list[int]):
        for pump_id, pump in enumerate(self.pumps):
            self.active_pumps.append(pump)
            pump.initialize()
            self.active_pumps.remove(pump)

    def to_dict(self):
        """Returns the current state of the PipetteHead as a dictionary.

        Returns:
            dict: A dictionary containing the current state of the PipetteHead and its components.
        """
        return asdict(self, dict_factory=dict_factory_pipettehead)

    def update(self, pipette_head_config: dict):
        """Update the PipetteHead based on the input configuration."""

        for pump_id, pump in enumerate(self.pumps):
            pump.update(pipette_head_config["pumps"][pump_id])
