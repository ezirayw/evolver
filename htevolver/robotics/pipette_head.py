import asyncio
import logging
from dataclasses import dataclass, field
from typing import Callable, ClassVar

from tecancavro.models import XCaliburD
from tecancavro.syringe import SyringeError, SyringeTimeout
from tecancavro.transport import TecanAPISerial

from htevolver.exceptions import PipetteHeadError
from htevolver.robotics.interfaces import PumpProtocol
from htevolver.shared import FluidTypes

logger = logging.getLogger(__name__)


@dataclass
class PumpPort:
    """Represents a fluidic port on a pump.

    Maintains information about the fluid type, volume, and state of a single port
    on a syringe pump.

    Attributes:
        id (int): Unique identifier for the port.
        fluid (FluidTypes): Type of fluid in the port.
        starting_volume (int): Initial volume in microliters (μL).
        current_volume (int): Current remaining volume in microliters (μL).
        primed (bool): Whether the port has been primed.
    """

    id: int
    fluid: FluidTypes
    starting_volume: int = field(repr=False)
    current_volume: int
    primed: bool

    @classmethod
    def create(cls, port_id: int, port_config: dict):
        """Create a new pump port instance.

        Args:
            port_id (int): Unique identifier for the port.
            port_config (dict): Configuration dictionary containing port parameters.
                Should include 'fluid', 'volume', and optionally 'primed'.

        Returns:
            PumpPort: A new instance of PumpPort.

        Examples:
            ```
            port = PumpPort.create(1, {"fluid": "MEDIA", "volume": 1000, "primed": False})
            ```
        """

        fluid = (
            FluidTypes[port_config.get("fluid", "empty").upper()]
            if port_config.get("fluid", "empty").upper() in FluidTypes.__members__
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
        """Update port configuration from a dictionary.

        Args:
            port_config (dict): Dictionary containing updated port parameters.
                May include 'fluid', 'volume', 'primed', etc.

        Examples:
            ```
            port.update({"volume": 500, "primed": True})
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
                logger.info(f"Updated PumpPort_{self.id} parameter: {config_parameter}={value}")

    def to_dict(self):
        """Convert the port to a dictionary representation.

        Returns:
            dict: Dictionary containing port state and configuration.
        """
        return {"id": self.id, "fluid": self.fluid.name, "current_volume": self.current_volume, "primed": self.primed}


@dataclass
class DummyPump:
    """A mock implementation of a pump for testing purposes.

    Simulates pump functionality without connecting to actual hardware.
    Logs operations instead of performing them.

    Attributes:
        enabled (bool): Simulated connection status.
        primary_fluid (FluidTypes): The main fluid type handled by this pump.
        ports (dict[int, PumpPort]): Dictionary mapping port IDs to PumpPort objects.
        head_port (int): Port ID of the dispensing head.
        active_port (int): Port ID currently selected for operation.
    """

    enabled: bool
    primary_fluid: FluidTypes
    ports: dict[int, PumpPort]
    head_port: int = field(repr=False)
    active_port: int

    @classmethod
    def create(cls, pump_id: int, pump_config: dict) -> "DummyPump":
        ports: dict[int, PumpPort] = {}
        for port_id, port_config in pump_config.get("ports", {}).items():
            ports[port_id] = PumpPort.create(port_id, port_config)
        return cls(
            primary_fluid=FluidTypes.EMPTY,
            ports=ports,
            head_port=pump_config.get("head_port", 0),
            active_port=0,
            enabled=pump_config.get("connect", False),
        )

    @staticmethod
    def find_serial_port(pump_id: int) -> str:
        "Simulate finding serial port. TODO: extend to virtual serial ports"
        logger.info(f"Simulate finding serial port for dummy pump_{pump_id}")
        return ""

    def enable(self) -> None:
        """Simulate enabling pump hardware.

        Logs the connection event but doesn't connect to physical hardware.
        """

        logger.info("Enabled dummy pump")

    def disable(self, delete: bool = False) -> None:
        """Simulate disabling pump hardware.

        Args:
            delete (bool, optional): Whether to delete the connection object. Defaults to False.
        """
        logger.info("Disabled dummy pump")

    def initialize(self) -> None:
        """Simulate initializing the pump hardware."""
        logger.info("Initialized dummy pump")

    def aspirate(self, volume: int) -> None:
        """Simulate aspirating fluid.

        Args:
            volume (int): Volume to aspirate in microliters (μL).
        """
        logger.info(f"Aspirating volume_{volume} on dummy pump")

    def dispense(self, volume: int) -> None:
        """Simulate dispensing fluid.

        Args:
            volume (int): Volume to dispense in microliters (μL).
        """
        logger.info(f"Dispensing volume_{volume} on dummy pump")

    def prime(self) -> None:
        """Simulate priming the pump."""
        logger.info("Priming dummy pump")

    def pause(self) -> None:
        """Simulate pausing the pump operation."""
        logger.info("Pausing dummy pump")

    def stop(self) -> None:
        """Simulate stopping the pump operation."""
        logger.info("Stopping dummy pump")

    def resume(self) -> None:
        """Simulate resuming a paused operation."""
        logger.info("Resuming dummy pump")

    def update(self, pump_config: dict = {}) -> None:
        """Simulate updating pump configuration.

        Args:
            pump_config (dict): Updated configuration dictionary.
        """
        logger.info(f"Updating dummy pump with {pump_config}")

    def to_dict(self) -> dict:
        """Convert the pump to a dictionary representation.

        Returns:
            dict: Dictionary with minimal dummy representation.
        """
        return {"yo": "dummy"}


#### DECORATORS ####
def pump_action(func: Callable):
    """Decorator for pump actions that require connection verification/error handling.

    Wraps pump operation functions to verify connection status and handle errors.

    Args:
        func (callable): The helper function to decorate.

    Returns:
        callable: The wrapped function which handles connection verification and errors.

    Raises:
        PipetteHeadError: If the pump is not connected or encounters an error.

    Examples:
        ```
        @pump_action
        def aspirate(self, volume):
            # Function implementation
        ```
    """

    def wrapper(self, *args, **kwargs):
        if self.enabled:
            try:
                func(self, *args, **kwargs)
            except (SyringeError, SyringeTimeout):
                raise PipetteHeadError(f"Error trying to run {func.__name__} on PipetteHead Pump_{self.id}")
        else:
            raise PipetteHeadError(f"PipetteHead Pump_{self.id} cannot run {func.__name__}, not connected")

    return wrapper


@dataclass
class XCaliburDPump:
    """Implementation of XCaliburD syringe pump for the HT-eVOLVER system.

    Controls a Tecan XCaliburD syringe pump via serial communication.

    Attributes:
        connected (bool): Connection status to the physical pump.
        hardware_api (XCaliburD): The software interface for the pump.
        primary_fluid (FluidTypes): The main fluid type handled by this pump.
        ports (dict[int, PumpPort]): Dictionary mapping port IDs to PumpPort objects.
        head_port (int): ID of the PumpPort that is hooked up to the PipetteHead.
        active_port (int): ID of the currently selection PumpPort.
    """

    enabled: bool
    hardware_api: XCaliburD = field(repr=False)
    primary_fluid: FluidTypes
    ports: dict[int, PumpPort]
    head_port: int = field(repr=False)
    active_port: int
    prime_volume: int = field(repr=False)
    max_volume: int = field(repr=False)

    @classmethod
    def create(cls, pump_id: int, pump_config: dict) -> "XCaliburDPump":
        """Create a new XCaliburD pump instance.

        Args:
            pump_id (int): Unique identifier for the pump.
            pump_config (dict): Configuration dictionary for the pump.
                Must include 'serial_port' and should include 'fluid', 'ports', etc.

        Returns:
            XCaliburDPump: A new instance of XCaliburDPump.

        Examples:
            ```
            config = {
                "serial_port": "/dev/ttyUSB0",
                "fluid": "MEDIA",
                "ports": {1: {"fluid": "MEDIA", "volume": 1000}}
            }
            pump = XCaliburDPump.create(0, config)
            ```
        """
        fluid = (
            FluidTypes[pump_config.get("fluid", "empty")]
            if pump_config.get("fluid", "empty") in FluidTypes.__members__
            else FluidTypes.EMPTY
        )
        ports: dict[int, PumpPort] = {}
        for port_id, port_config in pump_config.get("ports", {}).items():
            ports[port_id] = PumpPort.create(port_id, port_config)

        found_serial_port: str = ""
        if pump_config["serial_port"]:
            found_serial_port = pump_config["serial_port"]
        else:
            found_serial_port = XCaliburDPump.find_serial_port(pump_id)
        return cls(
            hardware_api=XCaliburD(
                com_link=TecanAPISerial(pump_id, ser_port=found_serial_port, ser_baud=9600),
            ),
            primary_fluid=fluid,
            ports=ports,
            head_port=pump_config.get("head_port", 0),
            active_port=0,
            enabled=False,
            prime_volume=pump_config.get("prime_volume", 5),
            max_volume=pump_config.get("max_volume", 1000),
        )

    @staticmethod
    def find_serial_port(pump_id: int) -> str:
        """Use the TecanAPISerial class to find the XCaliburD pump serial port

        Args:
            pump_id (int): The ID of the pump to find the serial port for

        Returns:
            str: The found serial port path (e.g. "/dev/ttyUSB0")

        Raises:
            PipetteHeadError: If the serial port for given ID cannot be found
        """

        found_pumps: dict[int, tuple] = TecanAPISerial.findSerialPumps()
        if pump_id in found_pumps:
            serial_port: str = found_pumps[pump_id][0]
            return serial_port
        else:
            raise PipetteHeadError(f"Cannot find serial port for XCaliburDPump {pump_id}")

    def enable(self):
        """Connect to the physical pump hardware.

        Sets the connected flag to True and re-establishes the hardware API connection if needed.
        """
        self.enabled = True
        # TODO check if hardware_api com_link exists, if not, re-establish

    def disable(self, delete: bool = False):
        """Disconnect from the physical pump hardware.

        Args:
            delete (bool, optional): If True, deletes the COM link to fully release resources.
                Defaults to False.
        """
        if delete:
            del self.hardware_api.com_link
        self.enabled = False

    def update(self, pump_config: dict):
        """Update pump configuration from a dictionary.

        Args:
            pump_config (dict): Dictionary containing updated pump parameters.
                May include 'primary_fluid' and per-port configuration updates.

        Examples:
            ```
            pump.update({"primary_fluid": "MEDIA", "ports": {1: {"volume": 500}}})
            ```
        """
        if pump_config["primary_fluid"] in FluidTypes.__members__:
            self.primary_fluid = FluidTypes[pump_config["primary_fluid"]]
        for port_id, port in self.ports.items():
            port.update(pump_config["ports"][port_id])

    def to_dict(self):
        """Convert the pump to a dictionary representation.

        Returns:
            dict: Dictionary containing the current state and configuration of the pump.
        """
        return {
            "primary_fluid": (self.primary_fluid.name, self.primary_fluid.value),
            "ports": {port_id: port.to_dict() for port_id, port in self.ports.items()},
            "head_port": self.head_port,
            "active_port": self.active_port,
            "enabled": self.enabled,
            "hardware_api": "XCaliburD",
        }

    @pump_action
    def initialize(self):
        """Initialize the XCaliburD pump hardware.

        Sends the initialization command to the pump.

        Raises:
            PipetteHeadError: If initialization fails or the pump is not connected.
        """
        self.hardware_api.init()

    @pump_action
    def aspirate(self, volume: int):
        """Extract fluid from the active port.

        Checks fluid volume before aspirating and switches to the next port
        if the current one is low on fluid.

        Args:
            volume (int): Volume to aspirate in microliters (μL).

        Raises:
            RoboticsRoutineError: If all ports are low on fluid.
            PipetteHeadError: If aspiration fails or the pump is not connected.
        """
        if volume > self.max_volume:
            raise PipetteHeadError("Aborting aspirate, desired volume greater than maximum.")
        if self.ports[self.active_port].current_volume < self.ports[self.active_port].starting_volume * 0.1:
            max_port = max(self.ports.keys())
            if self.active_port < max_port:
                self.active_port += 1
            else:
                raise PipetteHeadError("Aborting aspirate, need to exchange fluid reservoir(s).")

        self.hardware_api.extract(self.active_port, volume)
        delay = self.hardware_api.executeChain()
        self.hardware_api.waitReady(int(delay))

    @pump_action
    def dispense(self, volume: int):
        """Dispense fluid to the head port.

        Args:
            volume (int): Volume to dispense in microliters (μL).

        Raises:
            PipetteHeadError: If dispense fails or the pump is not connected.
        """
        if volume > self.max_volume:
            raise PipetteHeadError("Aborting dispense, desired volume greater than maximum.")
        self.hardware_api.dispense(self.head_port, volume + self.prime_volume)
        self.hardware_api.extract(self.head_port, volume)
        delay = self.hardware_api.executeChain()
        self.hardware_api.waitReady(int(delay))

    @pump_action
    def prime(self):
        """Prime all ports of the pump.

        Fills tubing from each port to the head port to remove air bubbles.
        Uses a fixed volume of 800μL per port.

        Raises:
            PipetteHeadError: If priming fails or the pump is not connected.
        """
        for port_id, port in self.ports.items():
            self.hardware_api.primePort(in_port=port_id, out_port=self.head_port, volume_ul=10000)
        self.hardware_api.extract(self.head_port, self.prime_volume)

    @pump_action
    def pause(self):
        """Pause the current pump operation.

        Sends a terminate command to the pump to pause the current operation.

        Raises:
            PipetteHeadError: If pausing fails or the pump is not connected.
        """
        self.hardware_api.terminateCmd()

    @pump_action
    def stop(self):
        """Stop the current pump operation and reset the command chain.

        Raises:
            PipetteHeadError: If stopping fails or the pump is not connected.
        """
        self.hardware_api.terminateCmd()
        self.hardware_api.resetChain()

    @pump_action
    def resume(self):
        """Resume a paused pump operation.

        Sends an empty command with execute=True to resume operation.

        Raises:
            PipetteHeadError: If resuming fails or the pump is not connected.
        """
        self.hardware_api.sendRcv("", execute=True)


@dataclass
class PipetteHead:
    """Manages a collection of syringe pumps for fluid handling.

    Coordinates multiple pumps for aspirating and dispensing fluids. Handles
    scheduling of operations and manages pump coordination.

    Attributes:
        pumps (tuple[PumpProtocol, ...]): Collection of pump objects.
        pump_num (int): Number of functional pumps.
        universal (bool): Whether all pumps handle the same fluid type.
        num_windows (int): Number of dispensing windows possible with the configuration.
        active_pumps (list[PumpProtocol]): Currently active pumps.
        pump_factory (ClassVar[dict[str, type[PumpProtocol]]]): Factory dictionary for creating pumps.
        dummy_pump_config (ClassVar[dict]): Default configuration for dummy pumps.
    """

    pumps: dict[int, PumpProtocol]
    pump_num: int
    universal: bool = field(repr=False)
    num_windows: int
    active_pumps: list[PumpProtocol]
    pump_factory: ClassVar[dict[str, type[PumpProtocol]]] = {"dummy": DummyPump, "XCaliburD": XCaliburDPump}
    dummy_pump_config: ClassVar[dict] = {
        "connect": False,
        "primary_fluid": FluidTypes.EMPTY,
        "ports": {1: {"connect": False, "primed": False, "volume": 0}, 2: {"connect": False, "primed": False, "volume": 0}},
        "head_port": 0,
        "active_port": 0,
    }

    @classmethod
    def create(cls, config: dict):
        """Create a new PipetteHead instance.

        Args:
            config (dict): Configuration dictionary for the pipette head.
                Must include a 'pumps' key with per-pump configuration dictionaries.

        Returns:
            PipetteHead: A new instance of PipetteHead with configured pumps.

        Examples:
            ```
            config = {
                "pumps": {
                    0: {"type": "XCaliburD", "serial_port": "/dev/ttyUSB0", "fluid": "MEDIA"},
                    1: {"type": "XCaliburD", "serial_port": "/dev/ttyUSB1", "fluid": "DRUG"}
                }
            }
            pipette_head = PipetteHead.create(config)
            ```
        """
        pumps: dict[int, PumpProtocol] = {}
        for pump_id in range(4):
            pump_config = config["pumps"][pump_id]
            pump_type_key = pump_config.get("type", "dummy")
            if pump_type_key != "dummy":
                pumps[pump_id] = cls.pump_factory[pump_type_key].create(pump_id, config["pumps"][pump_id])
            else:
                try:
                    pumps[pump_id] = cls.pump_factory[pump_type_key].create(pump_id, cls.dummy_pump_config)
                except PipetteHeadError:
                    raise

        pump_num: int = sum(1 for pump in pumps.values() if pump.primary_fluid == FluidTypes.EMPTY)
        universal: bool = all(pump.primary_fluid == pumps[0].primary_fluid for pump in pumps.values())
        num_windows: int = 0
        if pump_num != 0 and universal:
            num_windows = int((6 / pump_num) + 0.5)
        if pump_num != 0 and not universal:
            num_windows = 6 + (pump_num - 1)

        return cls(
            pumps=pumps,
            pump_num=pump_num,
            universal=universal,
            num_windows=num_windows,
            active_pumps=[],
        )

    def stop(self):
        """Stop all active pumps.

        Calls stop() on all pumps currently marked as active.
        """
        for pump in self.active_pumps:
            pump.stop()

    def pause(self):
        """Pause all active pumps.

        Calls pause() on all pumps currently marked as active.
        """
        for pump in self.active_pumps:
            try:
                pump.pause()
            except PipetteHeadError:
                raise

    def resume(self):
        """Resume all active pumps.

        Calls resume() on all pumps currently marked as active.
        """
        for pump in self.active_pumps:
            pump.resume()

    def enable(self, pump_list: list[int]):
        """Enable specified pumps.

        Args:
            pump_list (list[int]): List of syringe pump IDs to enable.

        Examples:
            ```
            pipette_head.enable([0, 1])  # Connect pumps 0 and 1
            ```
        """
        for pump_id in pump_list:
            self.pumps[pump_id].enable()

    def disable(self, pump_list: list[int]):
        """Disable specified pumps.

        Args:
            pump_list (list[int]): List of syringe pump IDs to disable.

        Examples:
            ```
            pipette_head.disable([0, 1])  # Disconnect pumps 0 and 1
            ```
        """
        for pump_id in pump_list:
            self.pumps[pump_id].disable()

    async def prime(self, pump_list: list[int]):
        """Prime the specified pumps.

        Adds each pump to active_pumps, calls prime(), and removes afterward.

        Args:
            pump_list (list[int]): List of syringe pump IDs to prime.

        Examples:
            ```
            await pipette_head.prime([0, 1])  # Prime pumps 0 and 1
            ```
        """
        for pump_id in pump_list:
            self.active_pumps.append(self.pumps[pump_id])
            self.pumps[pump_id].prime()
            self.active_pumps.remove(self.pumps[pump_id])

    async def aspirate(self, aspirate_commands: dict[int, int]):
        """Coordinates multi-pump aspirate operations.

        Concurrently executes PipetteHead aspirate calls based on the incoming commands. Assumes
        that PipetteHead pumps are on their own serial port, or else this function needs to be modified to
        run sequential pump operations in a separate thread to avoid blocking the event loop (i.e. asyncio.to_thread)
        Ensures that aspirate volumes are positive prior to executing.

        Args:
            aspirate_volumes (dict[int, int]): Maps aspirate volume to syringe pump ID key.

        Raises:
            PipetteHeadError: If any aspirate volume is negative.

        Examples:
            ```
            await pipette_head.aspirate([100, 0, 50, 0])
            # Pump 0 aspirates 100μL, pump 2 aspirates 50μL
            ```
        """
        for pump_id, aspirate_volume in aspirate_commands.items():
            if aspirate_volume < 0:
                raise PipetteHeadError(f"Negative volume input detected when trying to aspirate PipettteHead Pump ID {pump_id}")

        async def aspirate_task(pump_id, volume):
            pump: PumpProtocol = self.pumps[pump_id]
            self.active_pumps.append(pump)
            try:
                pump.aspirate(volume)
            except (SyringeError, SyringeTimeout) as e:
                raise PipetteHeadError(f"Error trying to aspirate PipetteHead Pump_{pump_id}") from e
            finally:
                self.active_pumps.remove(pump)

        async with asyncio.TaskGroup() as pipette_head_aspiration_tasks:
            for pump_id, volume in aspirate_commands.items():
                pipette_head_aspiration_tasks.create_task(aspirate_task(pump_id, volume))

    async def dispense(self, dispense_commands: dict[int, int]):
        """Coordinates multi-pump dispense operations.

        Concurrently executes PipetteHead dispense calls based on the incoming commands. Assumes
        that PipetteHead pumps are on their own serial port, or else this function needs to be modified to
        run sequential pump operations in a separate thread to avoid blocking the event loop (i.e. asyncio.to_thread)
        Ensures that dispense volumes are positive prior to executing.

        Args:
            dispense_commands (dict[int, int]): Maps aspirate volume to syringe pump ID key.

        Raises:
            PipetteHeadError: If any aspirate volume is negative.

        Examples:
            ```
            await pipette_head.dispense({0: 100, 1: 0, 2: 50, 3: 0})
            # Pump 0 aspirates 100μL, pump 2 aspirates 50μL
            ```
        """
        for pump_id, aspirate_volume in dispense_commands.items():
            if aspirate_volume < 0:
                raise PipetteHeadError(f"Negative volume input detected when trying to dispense PipettteHead Pump {pump_id}")

        async def dispense_task(pump_id, volume):
            pump: PumpProtocol = self.pumps[pump_id]
            self.active_pumps.append(pump)
            try:
                pump.dispense(volume)
            except (SyringeError, SyringeTimeout) as e:
                raise PipetteHeadError(f"Error trying to dispense PipetteHead Pump {pump_id}") from e
            finally:
                self.active_pumps.remove(pump)

        async with asyncio.TaskGroup() as pipette_head_aspiration_tasks:
            for pump_id, volume in dispense_commands.items():
                pipette_head_aspiration_tasks.create_task(dispense_task(pump_id, volume))

    async def initialize(self, pump_list: list[int]):
        """Initialize the specified pumps.

        Adds each pump to active_pumps, calls initialize(), and removes afterward.

        Args:
            pump_list (list[int]): List of syringe pump IDs to initialize.

        Examples:
            ```
            await pipette_head.initialize([0, 1])  # Initialize pumps 0 and 1
            ```
        """
        for pump_id in pump_list:
            self.active_pumps.append(self.pumps[pump_id])
            try:
                self.pumps[pump_id].initialize()
            finally:
                self.active_pumps.remove(self.pumps[pump_id])

    def to_dict(self) -> dict:
        """Convert the PipetteHead to a dictionary representation.

        Returns:
            dict: Dictionary containing the current state and configuration.
                Includes pump configurations, number of pumps, universality status,
                and number of dispensing windows.
        """

        return {
            "pumps": {pump_id: pump.to_dict() for pump_id, pump in self.pumps.items()},
            "pump_num": self.pump_num,
            "universal": self.universal,
            "num_windows": self.num_windows,
        }

    def update(self, pipette_head_config: dict):
        """Update the PipetteHead configuration.

        Updates each pump with its corresponding configuration section.

        Args:
            pipette_head_config (dict): Dictionary containing updated configuration.
                Must include a 'pumps' key with per-pump configuration updates.

        Examples:
            ```
            config_update = {
                "pumps": {
                    0: {"primary_fluid": "MEDIA"},
                    1: {"primary_fluid": "DRUG"}
                }
            }
            pipette_head.update(config_update)
            ```
        """
        for pump_id, pump in self.pumps.items():
            pump.update(pipette_head_config["pumps"][pump_id])
