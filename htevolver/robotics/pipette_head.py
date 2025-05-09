import logging
from dataclasses import dataclass
from typing import ClassVar

from tecancavro.models import XCaliburD
from tecancavro.syringe import SyringeError, SyringeTimeout
from tecancavro.transport import TecanAPISerial

from htevolver.exceptions import PipetteHeadError, RoboticsError
from htevolver.robotics.interfaces import PumpProtocol
from htevolver.shared import FluidTypes

logger = logging.getLogger(__name__)


@dataclass
class PumpPort:
    id: int
    fluid: FluidTypes
    starting_volume: int
    current_volume: int
    primed: bool

    @classmethod
    def create(cls, port_id: int, port_config: dict):
        fluid = (
            FluidTypes[port_config.get("fluid", "empty")]
            if port_config.get("fluid", "empty") in FluidTypes.__members__
            else FluidTypes.EMPTY
        )
        return cls(
            id=port_id,
            fluid=fluid,
            starting_volume=port_config.get("volume", 0),
            current_volume=port_config.get("volume", 0),
            primed=port_config.get("primed", False),
        )

    def update(self, port_config: dict):
        for config_parameter, value in port_config.items():
            if hasattr(self, config_parameter):
                attribute_value = getattr(self, config_parameter)
                attr_type = type(attribute_value)
                try:
                    new_value = attr_type(value)
                    if new_value != attribute_value:
                        setattr(self, config_parameter, new_value)
                except (ValueError, TypeError):
                    logger.warning(
                        f"Invalid type for {config_parameter}: expected {attr_type.__name__}, got {type(value).__name__}"
                    )
                logger.debug(f"Updated PumpPort_{self.id} parameter: {config_parameter}={value}")

    def to_dict(self): ...


@dataclass
class DummyPump:
    id: int
    connected: bool

    primary_fluid: FluidTypes
    ports: dict[int, PumpPort]
    head_port: int
    active_port: int

    @classmethod
    def create(cls, pump_id: int, pump_config: dict) -> "DummyPump":
        ports: dict[int, PumpPort] = {}
        for port_id, port_config in enumerate(pump_config.get("ports", {})):
            ports[port_id] = PumpPort.create(port_id, port_config)
        return cls(
            id=pump_id,
            primary_fluid=FluidTypes.EMPTY,
            ports=ports,
            head_port=pump_config.get("head_port", 0),
            active_port=0,
            connected=pump_config.get("connect", False),
        )

    def connect(self) -> None:
        """Connect to the pump hardware."""
        logger.info(f"Connected dummy pump_{self.id}")

    def disconnect(self, delete: bool = False) -> None:
        """Disconnect from the pump hardware."""
        logger.info(f"Disconnected dummy pump_{self.id}")

    def initialize(self) -> None:
        """Initialize the pump hardware."""
        logger.info(f"Disconnected dummy pump_{self.id}")

    def aspirate(self, volume: int) -> None:
        """Aspirate the specified volume."""
        logger.info(f"Aspirating volume_{volume} on dummy pump_{self.id}")

    def dispense(self, volume: int) -> None:
        """Dispense the specified volume."""
        logger.info(f"Dispensing volume_{volume} on dummy pump_{self.id}")

    def prime(self) -> None:
        """Prime the pump for use."""
        logger.info(f"Priming dummy pump{self.id}")

    def pause(self) -> None:
        """Pause the current pump operation."""
        logger.info(f"Pausing dummy pump_{self.id}")

    def stop(self) -> None:
        """Stop the current pump operation."""
        logger.info(f"Stopping dummy pump_{self.id}")

    def resume(self) -> None:
        """Resume a paused operation."""
        logger.info(f"Resuming dummy pump_{self.id}")

    def update(self, pump_config: dict) -> None:
        """Update pump configuration from a dictionary."""
        logger.info(f"Updating dummy pump_{self.id}")

    def to_dict(self) -> dict:
        """Serialize the pump to a dictionary"""
        logger.info(f"Serializing dummy pump_{self.id} to a dictionary")
        return {"yo": "dummy"}


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
class XCaliburDPump:
    id: int
    connected: bool

    hardware_api: XCaliburD
    primary_fluid: FluidTypes
    ports: dict[int, PumpPort]
    head_port: int
    active_port: int

    @classmethod
    def create(cls, pump_id: int, pump_config: dict) -> "XCaliburDPump":
        fluid = (
            FluidTypes[pump_config.get("fluid", "empty")]
            if pump_config.get("fluid", "empty") in FluidTypes.__members__
            else FluidTypes.EMPTY
        )
        ports: dict[int, PumpPort] = {}
        for port_id, port_config in enumerate(pump_config.get("ports", {})):
            ports[port_id] = PumpPort.create(port_id, port_config)
        return cls(
            id=pump_id,
            hardware_api=XCaliburD(
                com_link=TecanAPISerial(id, ser_port=pump_config["serial_port"], ser_baud=9600),
            ),
            primary_fluid=fluid,
            ports=ports,
            head_port=pump_config.get("head_port", 0),
            active_port=0,
            connected=pump_config.get("connect", False),
        )

    def connect(self):
        self.connected = True
        # TODO check if hardware_api com_link exists, if not, re-establish

    def disconnect(self, delete: bool = False):
        if delete:
            del self.hardware_api.com_link
        self.connected = False

    def update(self, pump_config: dict):
        if pump_config["primary_fluid"] in FluidTypes.__members__:
            self.primary_fluid = FluidTypes[pump_config["primary_fluid"]]
        for port_id, port in pump_config.items():
            port.update(pump_config[port_id])

    def to_dict(self):
        return {
            "id": self.id,
            "primary_fluid": (self.primary_fluid.name, self.primary_fluid.value),
            "ports": {port_id: port.to_dict() for port_id, port in self.ports.items()},
            "head_port": self.head_port,
            "active_port": self.active_port,
            "connected": self.connected,
            "hardware_api": "XCaliburD",
        }

    @pump_action
    def initialize(self):
        self.hardware_api.init()

    @pump_action
    def aspirate(self, volume: int):
        if self.ports[self.active_port].current_volume < self.ports[self.active_port].starting_volume * 0.1:
            max_port = max(self.ports.keys())
            if self.active_port < max_port:
                self.active_port += 1
            else:
                raise RoboticsError(f"Cannot aspirate: Need to exchange fluid reservoir(s) for PipetteHead Pump_{self.id}")

        self.hardware_api.extract(self.active_port, volume)
        delay = self.hardware_api.executeChain()
        self.hardware_api.waitReady(int(delay))

    @pump_action
    def dispense(self, volume: int):
        self.hardware_api.dispense(self.head_port, volume)
        delay = self.hardware_api.executeChain()
        self.hardware_api.waitReady(int(delay))

    @pump_action
    def prime(self):
        for port_id, port in self.ports.items():
            self.hardware_api.primePort(in_port=port_id, out_port=self.head_port, volume_ul=800)

    @pump_action
    def pause(self):
        self.hardware_api.terminateCmd()

    @pump_action
    def stop(self):
        self.hardware_api.terminateCmd()
        self.hardware_api.resetChain()

    @pump_action
    def resume(self):
        self.hardware_api.sendRcv("", execute=True)


@dataclass
class PipetteHead:
    pumps: tuple[PumpProtocol, ...]
    pump_num: int
    universal: bool
    num_windows: int
    active_pumps: list[PumpProtocol]
    pump_factory: ClassVar[dict[str, type[PumpProtocol]]] = {"dummy": DummyPump, "XCaliburD": XCaliburDPump}

    @classmethod
    def create(cls, config: dict):
        pumps: list[PumpProtocol] = []
        for pump_id in range(4):
            pump_type_key = config.get(pump_id, "dummy")
            pumps.append(cls.pump_factory[pump_type_key]().create(pump_id, config[pump_id]))

        pump_num: int = sum(1 for pump in pumps if pump.primary_fluid == FluidTypes.EMPTY)
        universal: bool = all(pump.primary_fluid == pumps[0].primary_fluid for pump in pumps)
        num_windows: int = 0
        if pump_num != 0 and universal:
            num_windows = int(6 / pump_num)
        if pump_num != 0 and not universal:
            num_windows = 6 + (pump_num - 1)
        return cls(
            pumps=tuple(pumps),
            pump_num=pump_num,
            universal=universal,
            num_windows=num_windows,
            active_pumps=[],
        )

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

        return {
            "pumps": [{pump.id: pump.to_dict() for pump in self.pumps}],
            "pump_num": self.pump_num,
            "universal": self.universal,
            "num_windows": self.num_windows,
        }

    def update(self, pipette_head_config: dict):
        """Update the PipetteHead based on the input configuration."""

        for pump_id, pump in enumerate(self.pumps):
            pump.update(pipette_head_config["pumps"][pump_id])
