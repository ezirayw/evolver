from typing import Protocol

from htevolver.shared import FluidTypes


class PumpProtocol(Protocol):
    """Protocol defining the interface for pump implementations. Allows users to implement their own syringe pumps as long as they implement this interface"""

    id: int
    connected: bool
    primary_fluid: FluidTypes

    @classmethod
    def create(cls, pump_id: int, pump_config: dict) -> "PumpProtocol":
        """Create a pump instance"""
        ...

    def connect(self) -> None:
        """Connect to the pump hardware."""
        ...

    def disconnect(self, delete: bool = False) -> None:
        """Disconnect from the pump hardware."""
        ...

    def initialize(self) -> None:
        """Initialize the pump hardware."""
        ...

    def aspirate(self, volume: int) -> None:
        """Aspirate the specified volume."""
        ...

    def dispense(self, volume: int) -> None:
        """Dispense the specified volume."""
        ...

    def prime(self) -> None:
        """Prime the pump for use."""
        ...

    def pause(self) -> None:
        """Pause the current pump operation."""
        ...

    def stop(self) -> None:
        """Stop the current pump operation."""
        ...

    def resume(self) -> None:
        """Resume a paused operation."""
        ...

    def update(self, pump_config: dict) -> None:
        """Update pump configuration from a dictionary."""

    def to_dict(self) -> dict:
        """Serialize the pump to a dictionary"""
        ...
