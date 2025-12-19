import logging
import time
from dataclasses import asdict, dataclass, field, fields
from typing import Callable

from tecancavro.models import XCaliburD
from tecancavro.syringe import SyringeError, SyringeTimeout
from tecancavro.tecanapi import TecanAPIInvalidAddress, TecanAPIInvalidFrame, TecanAPITimeout
from tecancavro.transport import UFactoryAPISerial
from xarm.wrapper import XArmAPI

from htevolver.exceptions import DispenseHeadError
from htevolver.robotics.interfaces import DispenseHeadProtocol

logger = logging.getLogger(__name__)


@dataclass
class FluidReservoir:
    """Represents a on-deck reservoir for fluid that is connected to syringe pumps. Stores information about volume changes for automated reservoir/port selection.

    Attributes:
        fluid_type (str): Name of fluid in reservoir.
        starting_volume (int): Volume of fluid in reservoir during initial setup, in milliliters (mL).
        current_volume (int): Realtime volume of fluid in reservoir, in milliliters (mL).
    """

    fluid_type: str = field(default="blank")
    starting_volume: int = field(default=0)
    current_volume: int = field(default=0)
    threshold: float = field(default=0.1)

    @classmethod
    def from_config(cls, reservoir_config: dict) -> "FluidReservoir":
        defaults = {}
        for f in fields(cls):
            defaults[f.name] = f.default

        return cls(
            fluid_type=reservoir_config.get("fluid_type", defaults["fluid_type"]),
            starting_volume=reservoir_config.get("starting_volume", defaults["starting_volume"]),
            current_volume=reservoir_config.get("current_volume", defaults["current_volume"]),
            threshold=reservoir_config.get("threshold", defaults["threshold"]),
        )

    def update_volume(self, volume_extracted: int) -> None:
        self.current_volume -= volume_extracted

    def is_fluid_available(self) -> bool:
        if self.current_volume >= self.threshold * self.starting_volume:
            return True
        else:
            return False

    def update_from_config(self, reservoir_config: dict):
        for config_parameter, value in reservoir_config.items():
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
                logger.info(f"Updated FluidReservoir parameter: {config_parameter}={value}")


@dataclass
class PumpPort:
    """Represents a fluidic port on a pump. Stores information regarding its primed status, volume consumption, and reservoir connection.

    Attributes:
        volume_consumed (int): Total volume aspirated by this port, in microliters (μL).
        primed (bool): Indicates whether the port has been primed and is ready for operation.
        reservoir_id (int): ID of reservoir that this port is connected to.
    """

    volume_consumed: int = field(default=0)
    primed: bool = field(default=False)
    reservoir_id: int = field(default=0)
    active: bool = field(default=False)

    @classmethod
    def from_config(cls, port_config: dict) -> "PumpPort":
        defaults = {}
        for f in fields(cls):
            defaults[f.name] = f.default

        return cls(
            volume_consumed=port_config.get("volume_consumed", defaults["volume_conumed"]),
            primed=port_config.get("primed", defaults["volume_conumed"]),
            reservoir_id=port_config.get("reservoir_id", defaults["volume_conumed"]),
            active=port_config.get("active", defaults["volume_conumed"]),
        )

    def update_from_config(self, port_config: dict):
        """Update port configuration from a dictionary.

        Args:
            port_config (dict): Dictionary containing updated port parameters.
                May include 'primed', 'volume_consumed', 'reservoir_id', etc.

        Examples:
            ```
            port.update({"volume_consumed": 500, "primed": True})
            ```
        """

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
                logger.info(f"Updated PumpPort parameter: {config_parameter}={value}")


#### DECORATORS ####
def pump_action(func: Callable):
    """Decorator for pump actions that require connection verification/error handling.

    Args:
        func (callable): The helper function to decorate.

    Returns:
        callable: The wrapped function which handles connection verification and errors.

    Raises:
        DispenseHeadError: If the pump is not connected or encounters an error.

    Examples:
        ```
        @pump_action
        def aspirate(self, volume):
            # Function implementation
        ```
    """

    def wrapper(self: "DispenseHeadXCaliburD", *args, **kwargs):
        if not self.active:
            raise DispenseHeadError(f"{self.head_id}_DispenseHead cannot run {func.__name__}, not in use.")
        try:
            if not DispenseHeadXCaliburD.communication_interface:
                raise (DispenseHeadError("UFactoryAPISerial interface not yet setup for XCaliburD pumps"))

            func(self, *args, **kwargs)
        except (SyringeError, SyringeTimeout):
            raise DispenseHeadError(f"Error trying to run {func.__name__} on {self.head_id}_DispenseHead")

    return wrapper


class DummyDispenseHead(DispenseHeadProtocol):
    head_id: int = 0
    active: bool = False
    enabled: bool = False
    pump_number: int = 0

    @classmethod
    def from_config(cls, head_config: dict) -> "DummyDispenseHead":
        logger.info(f"DummyDispenseHead.from_config called with: {head_config}")
        return cls()

    def enable(self) -> None:
        logger.info("DummyDispenseHead.enable called")

    def disable(self) -> None:
        logger.info("DummyDispenseHead.disable_head called")

    def initialize(self) -> None:
        logger.info("DummyDispenseHead.initialize_head called")

    def update(self, dispensehead_config: dict) -> None:
        logger.info(f"DummyDispenseHead.update_head called with: {dispensehead_config}")

    def pause(self) -> None:
        logger.info("DummyDispenseHead.pause called")

    def stop(self) -> None:
        logger.info("DummyDispenseHead.stop called")

    def resume(self) -> None:
        logger.info("DummyDispenseHead.resume called")

    def prime(self) -> None:
        logger.info("DummyDispenseHead.prime called")

    def aspirate(self, aspirate_commands: dict[str, int]) -> None:
        logger.info(f"DummyDispenseHead.aspirate called with: {aspirate_commands}")

    def dispense(self, dispense_commands: dict[str, int]) -> None:
        logger.info(f"DummyDispenseHead.dispense called with: {dispense_commands}")

    def to_dict(self) -> dict:
        logger.info("DummyDispenseHead.to_dict called")
        return {"dummy": True}


class DispenseHeadXCaliburD(DispenseHeadProtocol):
    """HT-eVOLVER DispenseHead class using XCaliburD syringe pumps. Subtype of the DispenseHeadProtocol."""

    address_all_pumps: int = 0x5F
    default_num_ports: int = 3
    default_syringe_ul: int = 1000
    default_air_gap_ul: int = 20
    default_direction: str = "CW"
    default_microstep: bool = False
    default_waste_port: int = 3
    default_head_port: int = 0
    default_slope: int = 14
    default_init_force: int = 0

    communication_interface: UFactoryAPISerial | None = None
    all_pump_interface: XCaliburD | None = None

    def __init__(self, head_id: int, pump_number: int, active=False, enabled=False):
        self.head_id = head_id
        self.pump_number = pump_number
        self.pumps: dict[str, XCaliburD] = {}
        self.ports: dict[str, dict[int, PumpPort]] = {}
        self.active = active
        self.enabled = enabled
        self.primed = False
        self.reservoirs: dict[int, FluidReservoir] = {
            0: FluidReservoir(fluid_type="blank", starting_volume=0, current_volume=0, threshold=0.1)
        }

        for pump_id in range(self.pump_number):
            pump: str = f"pump_{pump_id}"
            self.pumps[pump] = XCaliburD(
                num_ports=DispenseHeadXCaliburD.default_num_ports,
                com_link=DispenseHeadXCaliburD.communication_interface,
                address=pump_id,
                air_gap_ul=DispenseHeadXCaliburD.default_air_gap_ul,
                syringe_ul=DispenseHeadXCaliburD.default_syringe_ul,
                direction=DispenseHeadXCaliburD.default_direction,
                microstep=DispenseHeadXCaliburD.default_microstep,
                waste_port=DispenseHeadXCaliburD.default_waste_port,
                slope=DispenseHeadXCaliburD.default_slope,
                init_force=DispenseHeadXCaliburD.default_init_force,
            )
            for port_id in range(self.pumps[pump].num_ports):
                self.ports[pump][port_id] = PumpPort(volume_consumed=0, reservoir_id=0, primed=False, active=False)

    @classmethod
    def set_communication_interface(cls, xarm_instance: XArmAPI):
        DispenseHeadXCaliburD.communication_interface = UFactoryAPISerial(xarm_instance)

    @classmethod
    def create_all_pump(cls):
        if not cls.communication_interface:
            raise (DispenseHeadError("UFactoryAPISerial interface not yet setup for XCaliburD pumps"))

        cls.all_pump_interface: XCaliburD | None = XCaliburD(
            num_ports=cls.default_num_ports,
            com_link=cls.communication_interface,
            address=0x5F,
            syringe_ul=cls.default_syringe_ul,
            direction=cls.default_direction,
            microstep=cls.default_microstep,
            waste_port=cls.default_waste_port,
            slope=cls.default_slope,
            init_force=cls.default_init_force,
        )

    @classmethod
    def from_config(cls, head_config: dict) -> "DispenseHeadXCaliburD":
        """Create a new DispenseHead instance from a loaded configuration.

        Args:
            config (dict): Configuration dictionary for the dipense head.
                Must include a 'pumps' key with per-pump configuration dictionaries.

        Returns:
            DispenseHeadXCaliburD: A newly, configured DispenseHead instance.

        Examples:
            ```
            config = {
                "pumps": {
                    0: {"type": "XCaliburD", "serial_port": "/dev/ttyUSB0", "fluid": "MEDIA"},
                    1: {"type": "XCaliburD", "serial_port": "/dev/ttyUSB1", "fluid": "DRUG"}
                }
            }
            dipense_head = DispenseHead.create(config)
            ```
        """
        pump_number: int = 0
        pumps: dict[str, XCaliburD] = {}
        ports: dict[str, dict[int, PumpPort]] = {}
        reservoirs: dict[int, FluidReservoir] = {}
        for pump_id, pump_config in head_config.get("pumps", {}).items():
            pump: str = f"pump_{pump_id}"
            pumps[pump] = XCaliburD(
                com_link=cls.communication_interface,
                address=pump_id,
                syringe_ul=pump_config.get("syringe_volume", cls.default_syringe_ul),
                direction=pump_config.get("direction", cls.default_direction),
                microstep=pump_config.get("microstep", cls.default_microstep),
                waste_port=pump_config.get("waste_port_id", cls.default_waste_port),
                head_port=pump_config.get("head_port_id", cls.default_head_port),
                slope=pump_config.get("slope", cls.default_slope),
                init_force=pump_config.get("init_force", cls.default_init_force),
            )
            pump_number += 1
            for port_id, port_config in pump_config["ports"]:
                ports[pump][port_id] = PumpPort.from_config(port_config)

        for reservoir_id, reservoir_config in head_config.get("reservoirs", {}).items():
            reservoirs[reservoir_id] = FluidReservoir.from_config(reservoir_config)

        dispensehead_instance = cls(
            head_id=head_config.get("head_id", 0),
            pump_number=pump_number,
        )

        dispensehead_instance.pumps = pumps
        dispensehead_instance.ports = ports
        dispensehead_instance.reservoirs = reservoirs

        return dispensehead_instance

    def enable(self):
        """Enable the DispenseHead object. Sets enable property to True"""
        self.enabled = True

    def disable(self):
        """Enable the DispenseHead object. Sets enable property to True"""
        self.enabled = True

    def update_pump(self, pump: str, pump_config: dict):
        """Update pump configuration from a dictionary.

        Args:
            pump_config (dict): Dictionary containing updated pump config parameters.

        Examples:
            ```
            pump.update({"syringe_volume": "5000", "ports": {1: {"volume": 500}}})
            ```
        """
        for config_parameter, value in pump_config.items():
            if config_parameter == "ports":
                for port_id in pump_config["ports"]:
                    self.ports[pump][port_id].update_from_config(pump_config["ports"][port_id])

            elif hasattr(self, config_parameter):
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
                logger.info(f"Updated XCaliburDPump_{pump} parameter: {config_parameter}={value}")

    def to_dict_pump(self, pump: str) -> dict:
        pump_dict: dict = {}
        pump_dict["number_ports"] = self.pumps[pump].num_ports
        pump_dict["syringe_volume"] = self.pumps[pump].syringe_ul
        pump_dict["ports"] = {port_id: asdict(port) for port_id, port in self.ports[pump].items()}
        return pump_dict

    def update(self, dispensehead_config: dict):
        """Update a DispenseHead object. Will also update any child objects, such as pumps, ports, and reservoirs.

        Args:
            dipense_head_config (dict): Dictionary containing updated configuration.

        Examples:
            ```
            dipense_head.update(config_update)
            ```
        """
        for config_parameter, value in dispensehead_config.items():
            if config_parameter == "reservoirs":
                for reservoir_id, reservoir_config in dispensehead_config["reservoirs"].items():
                    self.reservoirs[reservoir_id].update_from_config(reservoir_config)

            if config_parameter == "pumps":
                for pump_id, pump_config in dispensehead_config["pumps"].items():
                    self.update_pump(pump_id, pump_config)

            elif hasattr(self, config_parameter):
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
                logger.info(f"Updated {self.head_id}_DispenseHead parameter: {config_parameter}={value}")

    def getset_pump_port(self, pump: str) -> tuple[bool, int]:
        """For a given syringe pump, set the port to use to for aspirating fluid by checking the reservoir volume. Returns port ID for valid port if available"""

        valid_port_found: bool = False
        valid_port_id: int = -1
        for port_id, port in self.ports[pump].items():
            if self.reservoirs[port.reservoir_id].is_fluid_available():
                port.active = True
                valid_port_found = True
                valid_port_id = port_id
            else:
                port.active = False
        return (valid_port_found, valid_port_id)

    @pump_action
    def initialize(self):
        """Initialize XCaliburD pumps on the DispenseHead."""

        for pump in self.pumps.values():
            pump.init()

    @pump_action
    def pause(self):
        """Pause current pump operations."""

        for pump in self.pumps.values():
            pump.terminateCmd()

    @pump_action
    def stop(self):
        """Stop current pump operations. Differs from pause by resetting command chain, thus clearing multi-command operations."""

        for pump in self.pumps.values():
            pump.terminateCmd()
            pump.resetChain()

    @pump_action
    def resume(self):
        """Resume all active pumps."""

        for pump in self.pumps.values():
            pump.sendRcv("", execute=True)

    @pump_action
    def prime(self, pumps: list[str], prime_volume: int = 10000):
        """Prime all ports for designated XCaliburD pumps on the DispenseHead. Fills tubing with fluid from reservoir to ensure that ports are ready for use during influx routines.

        Args:
            pumps_to_prime (list[int]): List of syringe pump IDs to prime.
        """
        for pump in pumps:
            try:
                for port_id in self.ports:
                    self.pumps[pump].primePort(in_port=port_id, out_port=self.pumps[pump].head_port, volume_ul=prime_volume)

                self.pumps[pump].extract(from_port=self.pumps[pump].head_port, volume_ul=self.pumps[pump].air_gap_ul)

            except (ValueError, SyringeError, TecanAPITimeout, TecanAPIInvalidFrame, TecanAPIInvalidAddress) as e:
                msg: str = f"Low level hardware XCaliburD pump error encountered: {e}"
                logger.exception(msg, stack_info=True)
                raise DispenseHeadError(f"Low level hardware XCaliburD pump error encountered: {e}")

        self.primed = True

    @pump_action
    def aspirate(self, aspirate_commands: dict[str, int]) -> None:
        """Coordinates multi-pump aspirate operations for the DispenseHead. Port selection for each syringe pump is handling automatically by checking reservoir volumes.

        Args:
            aspirate_commands (list[int]): List of dispense volume commands. Volume index corresponds to syrige pump on the DispenseHead.

        Raises:
            DispenseHeadError: If any aspirate volume is negative.
            DispenseHeadWarning: If reservoirs are empty for any single syringe pump.
        """
        if not self.primed:
            raise DispenseHeadError(f"DispenseHead_{self.head_id} not primed")
        for pump, aspirate_volume in aspirate_commands.items():
            port_found, valid_port_id = self.getset_pump_port(pump)
            if not port_found:
                raise DispenseHeadError(
                    f"Reservoirs are empty for pump_{pump} on DispenseHead_{self.head_id}. Skipping aspirate..."
                )

            try:
                time_delay: float = self.pumps[pump].extract(from_port=valid_port_id, volume_ul=aspirate_volume, execute=True)
                time.sleep(time_delay * 1.5)
            except (ValueError, SyringeError, TecanAPITimeout, TecanAPIInvalidFrame, TecanAPIInvalidAddress) as e:
                msg: str = f"Low level hardware XCaliburD pump error encountered: {e}"
                logger.exception(msg, stack_info=True)
                raise DispenseHeadError(f"Low level hardware XCaliburD pump error encountered: {e}")

    @pump_action
    def dispense(self, dispense_commands: dict[str, int]):
        """Coordinates multi-pump dispense operations for the DispenseHead. Fluid is dispensed to pump's coonfigured head port id. Automatically handles

        Args:
            dispense_commands (list[int]): List of dispense volume commands. Volume index corresponds to syrige pump on the DispenseHead.

        Raises:
            DispenseHeadError: If any dispense volume is negative.
        """

        for pump, dispense_volume in dispense_commands.items():
            try:
                time_delay: float = self.pumps[pump].dispense(
                    to_port=self.pumps[pump].head_port, volume_ul=dispense_volume, execute=True
                )
                time.sleep(time_delay)
                time_delay: float = self.pumps[pump].extract(
                    from_port=self.pumps[pump].head_port, volume_ul=self.pumps[pump].air_gap_ul, execute=True
                )
            except (ValueError, SyringeError, TecanAPITimeout, TecanAPIInvalidFrame, TecanAPIInvalidAddress) as e:
                msg: str = f"Low level hardware XCaliburD pump error encountered: {e}"
                logger.exception(msg, stack_info=True)
                raise DispenseHeadError(f"Low level hardware XCaliburD pump error encountered: {e}")

    def to_dict(self) -> dict:
        """Convert the DispenseHead to a dictionary representation.

        Returns:
            dict: Dictionary containing the current state and configuration.
                Includes pump configurations, number of pumps, universality status,
                and number of dispensing windows.
        """

        return {
            "head_id": self.head_id,
            "active": self.active,
            "enabled": self.enabled,
            "pumps": {pump: self.to_dict_pump(pump) for pump in self.pumps},
            "reservoirs": {reservoir_id: asdict(reservoir) for reservoir_id, reservoir in self.reservoirs.items()},
        }
