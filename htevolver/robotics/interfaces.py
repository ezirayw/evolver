from typing import Any, Protocol

from htevolver.dependencies import CartesianMovement


class RobotArmProtocol(Protocol):
    """Protocol defining the interface for robotic arm integrations with HT-eVOLVER"""

    arm_api: Any
    connected: bool
    ip: str
    home_position: CartesianMovement
    standby_position: CartesianMovement

    @classmethod
    def from_config(cls, config: dict) -> "RobotArmProtocol":
        """Create a RobotArm instance from a configuration"""
        ...

    def initialize(self):
        """Run any initialization code prior to usage of the RobotArm."""
        ...

    def connect(self):
        """Connect to the RobotArm instance."""
        ...

    def disconnect(self):
        """Disconnect from the RobotArm instance."""
        ...

    def update(self, config: dict):
        """Update the RobotArm configuration."""
        ...

    def stop(self):
        """Stop the RobotArm during movement."""
        ...

    def pause(self):
        """Pause the RobotArm during movement."""
        ...

    def resume(self):
        """Resume RobotArm movement."""
        ...

    def move(self, coordinate: CartesianMovement):
        """Move the RobotArm to the specified coordinate."""
        ...

    def to_dict(self) -> dict:
        """Conver the RobotArm instance to a dictionary representation."""
        ...


class ToolChangeProtocol(Protocol):
    """Protocol defining the interface for HT-eVOLVER tool change stations."""

    ...


class DispenseHeadProtocol(Protocol):
    """Protocol defining the interface for pump implementations.

    Defines the required interface for any pump implementation to be
    compatible with HT-eVOLVER.

    Attributes:
        head_id (int): Port ID that is connected to the DispenseHead needle.
        active (bool): Boolean representing whether DispenseHead is in use.
        enabled (bool): Boolean representing whether DispenseHead is usable for robotic operations.
        pump_number (int): Number of syringe pumps on integrated onto the DispenseHead.

    """

    head_id: int
    active: bool
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

    def enable(self) -> None:
        """Enable the DispenseHead object, indicating that its ready for use."""
        ...

    def disable(self) -> None:
        """Disable the DispenseHead object."""
        ...

    def initialize(self) -> None:
        """Initialize the DispenseHead. Run any relevant setup functions here (i.e. initializing pumps) and enable DispenseHead"""

    def update_head(self, dispensehead_config: dict) -> None:
        """Update DispenseHead object from input config"""
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

    def aspirate(self, aspirate_commands: dict[str, int]) -> None:
        """Execute aspiration commands on the DispenseHead.

        Args:
            aspirate_commands (dict[int, int]): Maps aspirate volume to syringe pump on the DisenseHead.
        """
        ...

    def dispense(self, dispense_commands: dict[str, int]) -> None:
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
