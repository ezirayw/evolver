from typing import Protocol


class DispenseHeadProtocol(Protocol):
    """Protocol defining the interface for pump implementations.

    This protocol defines the required interface for any pump implementation to be
    compatible with the HT-eVOLVER system. Custom pump implementations must implement
    all methods in this protocol.

    Attributes:
        enabled (bool): Connection status of the pump.
        primary_fluid (FluidTypes): The main fluid type handled by this pump.
    """

    head_id: int
    in_use: bool
    enabled: bool
    pump_number: int

    @classmethod
    def from_config(cls, head_config: dict) -> "DispenseHeadProtocol":
        """Create a pump DispenseHead instance from a configuration.

        Args:
            pump_config (dict): Dictionary containing pump configuration parameters.

        Returns:
            DispenseHeadProtocol: A new instance of the pump implementation.
        """
        ...

    def enable_head(self) -> None:
        """Enable the DispenseHead object, indicating that its ready for use."""
        ...

    def disable_head(self) -> None:
        """Disable the DispenseHead object."""
        ...

    def initialize_head(self) -> None:
        """Initialize the DispenseHead. Run any relevant setup functions here (i.e. initializing pumps) and enable DispenseHead"""

    def update_head(self, dispense_head_config: dict) -> None:
        """Update DispenseHead object from input config"""
        ...

    def validate_volume(self, input_volume: int) -> bool:
        """Validate an volume command against the DispenseHead configuration"""
        ...

    def pause(self) -> None:
        """Pause DispenseHead operations"""
        ...

    def stop(self) -> None:
        """Stop current DispenseHad pump operations"""
        ...

    def resume(self) -> None:
        """Resume paused DispenseHead pump operations"""
        ...

    def prime(self) -> None:
        """Prime pumps on DispenseHead"""
        ...

    async def aspirate(self, aspirate_commands: list[int]) -> None:
        """Execute aspiration commands on the DispenseHead.

        Args:
            aspirate_commands (dict[int, int]): Maps aspirate volume to syringe pump on the DisenseHead.
        """
        ...

    def dispense(self, dispense_commands: list[int]) -> None:
        """Execute dispense commands on the DispenseHead.

        Args:
            dispense_commands (dict[int, int]): Maps aspirate volume to syringe pump on the DisenseHead.
        """
        ...

    def to_dict(self) -> dict:
        """Serialize the DispenseHead object to a dictionary representation.

        Returns:
            dict: Dictionary containing the current state and configuration of the pump.
        """
        ...
