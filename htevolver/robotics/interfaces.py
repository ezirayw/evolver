from typing import Protocol

from htevolver.shared import FluidTypes


class PumpProtocol(Protocol):
    """Protocol defining the interface for pump implementations.

    This protocol defines the required interface for any pump implementation to be
    compatible with the HT-eVOLVER system. Custom pump implementations must implement
    all methods in this protocol.

    Attributes:
        id (int): Unique identifier for the pump.
        enabled (bool): Connection status of the pump.
        primary_fluid (FluidTypes): The main fluid type handled by this pump.
    """

    id: int
    enabled: bool
    primary_fluid: FluidTypes

    @classmethod
    def create(cls, pump_id: int, pump_config: dict) -> "PumpProtocol":
        """Create a pump instance.

        Args:
            pump_id (int): Unique identifier for the pump.
            pump_config (dict): Dictionary containing pump configuration parameters.
                Should include connection settings, fluid type, port settings, etc.

        Returns:
            PumpProtocol: A new instance of the pump implementation.
        """
        ...

    @staticmethod
    def find_serial_port(pump_id: int) -> str:
        """Find the pump's serial port connection

        Automatic detection of the serial port the pump is using for communication.
        Obscures low-level HT-eVOLVER setup by user and enables automatic PipetteHead setup by server
        """
        ...

    def enable(self) -> None:
        """Enable pump hardware.

        Establishes a connection to the physical pump hardware.
        """
        ...

    def disable(self, delete: bool = False) -> None:
        """Disconnect from the pump hardware.

        Args:
            delete (bool, optional): Whether to delete the connection object. Defaults to False.
        """
        ...

    def initialize(self) -> None:
        """Initialize the pump hardware.

        Performs initial setup of the pump hardware, such as setting positions, speeds, etc.
        """
        ...

    def aspirate(self, volume: int) -> None:
        """Aspirate the specified volume.

        Args:
            volume (int): Volume to aspirate in microliters (μL).
        """
        ...

    def dispense(self, volume: int) -> None:
        """Dispense the specified volume.

        Args:
            volume (int): Volume to dispense in microliters (μL).
        """
        ...

    def prime(self) -> None:
        """Prime the pump for use.

        Fills the pump and tubing with fluid to remove air bubbles.
        """
        ...

    def pause(self) -> None:
        """Pause the current pump operation.

        Temporarily stops the current operation, allowing for later resumption.
        """
        ...

    def stop(self) -> None:
        """Stop the current pump operation.

        Completely terminates the current operation.
        """
        ...

    def resume(self) -> None:
        """Resume a paused operation.

        Continues execution from where a previous pause occurred.
        """
        ...

    def update(self, pump_config: dict) -> None:
        """Update pump configuration from a dictionary.

        Args:
            pump_config (dict): Dictionary containing updated configuration parameters.
        """
        ...

    def to_dict(self) -> dict:
        """Serialize the pump to a dictionary.

        Returns:
            dict: Dictionary containing the current state and configuration of the pump.
        """
        ...
