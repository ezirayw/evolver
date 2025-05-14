import logging
from dataclasses import asdict, dataclass, field
from typing import ClassVar

import numpy as np
import skimage as ski
from skimage.transform import EuclideanTransform

from htevolver.robotics.xarm import xArmCoordinate

logger = logging.getLogger(__name__)


def dict_factory_smartstation(data):
    """Custom factory for converting SmartStation instances to dictionaries.

    Handles special conversion of EuclideanTransform objects to serializable form.

    Args:
        data: Iterable of key-value pairs from dataclass fields.

    Returns:
        dict: Dictionary with properly converted values.
    """
    result = {}
    for key, value in data:
        if isinstance(value, EuclideanTransform):
            result[key] = value.params
        else:
            result[key] = value
    return result


@dataclass
class VialCoordinate:
    """Represents a coordinate in the vial grid system.

    Stores 2D coordinates (x, y) for identifying positions in the vial system's
    coordinate space.

    Attributes:
        x (float): The x-coordinate value in the vial system.
        y (float): The y-coordinate value in the vial system.
    """

    x: float
    y: float


@dataclass
class xArmPlane:
    """Represents a plane in the xArm coordinate system with transformation capabilities.

    Manages the transformation between vial coordinates and xArm coordinates.
    Contains calibration points and Z-height for a specific movement plane.

    Attributes:
        vial0_x (float): X-coordinate of vial 0 in the xArm coordinate system.
        vial0_y (float): Y-coordinate of vial 0 in the xArm coordinate system.
        vial17_x (float): X-coordinate of vial 17 in the xArm coordinate system.
        vial17_y (float): Y-coordinate of vial 17 in the xArm coordinate system.
        z (float): Z-coordinate (height) of this plane in the xArm system.
        transform_matrix (EuclideanTransform): Transformation matrix for coordinate conversion.
            Automatically calculated during initialization.
    """

    vial0_x: float
    vial0_y: float
    vial17_x: float
    vial17_y: float
    z: float
    transform_matrix: EuclideanTransform = field(repr=False, init=False)

    def update(self, plane_config: dict):
        """Update the plane configuration parameters.

        Args:
            plane_config (dict): Dictionary containing updated plane parameters.
                May include 'vial0_x', 'vial0_y', 'vial17_x', 'vial17_y', 'z'.

        Examples:
            ```
            plane.update({
                "vial0_x": 150.5,
                "vial0_y": 200.3,
                "z": 50.0
            })
            ```
        """
        for config_parameter, value in plane_config.items():
            if hasattr(self, config_parameter) and getattr(self, config_parameter) != value:
                setattr(self, config_parameter, value)
            logger.debug(f"Updated xArm parameter: {config_parameter}={value}")
        self.rigid_transform()

    def rigid_transform(self):
        """Calculate the rigid transformation matrix between coordinate systems.

        Creates a transformation matrix for converting vial coordinates into
        xArm coordinates using the calibration points (vial0 and vial17).
        Uses scikit-image's EuclideanTransform to compute the transformation.
        """

        vial_coordinates = np.array([[0, 36], [90, 0]])
        vial_0 = np.array([self.vial0_x, self.vial0_y])
        vial_17 = np.array([self.vial17_x, self.vial17_y])
        np.array([vial_0, vial_17])
        tform = ski.transform.EuclideanTransform()
        tform.estimate(vial_coordinates, np.array([vial_0, vial_17]))
        self.transform_matrix = tform

    def vial_to_xarm(self, evolver_coordinates: VialCoordinate) -> xArmCoordinate:
        """Transform vial coordinates to xArm coordinates.

        Applies the rigid transformation matrix to convert from the evolver
        coordinate system to the xArm coordinate system.

        Args:
            evolver_coordinates (VialCoordinate): Coordinates in the evolver system.
                Example: VialCoordinate(x=18, y=36)

        Returns:
            xArmCoordinate: The transformed coordinates in the xArm system.

        Examples:
            ```
            vial_pos = VialCoordinate(x=18, y=36)
            arm_pos = plane.vial_to_xarm(vial_pos)
            print(f"xArm position: ({arm_pos.x}, {arm_pos.y}, {arm_pos.z})")
            ```
        """
        np_coordinates = np.array([[evolver_coordinates.x], [evolver_coordinates.y], [1]])
        transformed = np.dot(self.transform_matrix, np_coordinates)
        return xArmCoordinate(x=transformed[0][0], y=transformed[1][0], z=self.z)


@dataclass
class SmartStationRobotics:
    """Represents a Smart Station with transformation capabilities for robotics.

    Manages the coordinate transformations between the vial grid system and the
    xArm coordinate system for both in-vial and above-vial planes.

    Attributes:
        xArmPlane_in (xArmPlane): Transformation plane for in-vial positions.
        xArmPlane_out (xArmPlane): Transformation plane for above-vial positions.
        wash_location (VialCoordinate): Location of the wash station in vial coordinates.
        wash_depth (float): Depth for washing operations.
        vial_map (ClassVar[list[list[int]]]): Standard mapping of vial IDs in the grid layout.
    """

    xArmPlane_in: xArmPlane
    xArmPlane_out: xArmPlane
    wash_location: VialCoordinate
    wash_depth: float
    vial_map: ClassVar[list[list[int]]] = field(
        repr=False, default_factory=lambda: [[0, 1, 2, 3, 4, 5], [11, 10, 9, 8, 7, 6], [12, 13, 14, 15, 16, 17]]
    )

    @classmethod
    def create(cls, config: dict):
        """Create a new SmartStationRobotics instance.

        Args:
            config (dict): Configuration dictionary for the Smart Station.
                Must include 'plane_in' and 'plane_out' sections for the transformation planes.
                Should include 'wash_location' and 'wash_depth'.

        Returns:
            SmartStationRobotics: A new instance of SmartStationRobotics.

        Examples:
            ```
            config = {
                "plane_in": {
                    "vial0_x": 150.5, "vial0_y": 200.3,
                    "vial17_x": 240.7, "vial17_y": 170.4,
                    "z": 10.0
                },
                "plane_out": {
                    "vial0_x": 150.5, "vial0_y": 200.3,
                    "vial17_x": 240.7, "vial17_y": 170.4,
                    "z": 50.0
                },
                "wash_location": {"x": 72, "y": -29},
                "wash_depth": 20.0
            }
            station = SmartStationRobotics.create(config)
            ```
        """
        plane_in = xArmPlane(**config["plane_in"])
        plane_out = xArmPlane(**config["plane_out"])
        plane_in.rigid_transform()
        plane_out.rigid_transform()
        wash_location_x = config["wash_location"].get("x", 72)
        wash_location_y = config["wash_location"].get("y", -29)
        return cls(
            xArmPlane_in=plane_in,
            xArmPlane_out=plane_out,
            wash_location=VialCoordinate(x=wash_location_x, y=wash_location_y),
            wash_depth=config.get("wash_depth", 0),
        )

    def update(self, station_config: dict):
        """Update the SmartStation configuration.

        Args:
            station_config (dict): Dictionary containing updated configuration.
                Should include 'plane_in' and 'plane_out' sections.

        Examples:
            ```
            config_update = {
                "plane_in": {"z": 15.0},
                "plane_out": {"z": 55.0}
            }
            station.update(config_update)
            ```
        """
        """Update the SmartStation xArmPlane calibration points based on the input configuration."""
        self.xArmPlane_in.update(station_config["plane_in"])
        self.xArmPlane_out.update(station_config["plane_out"])

    def to_dict(self):
        """Convert the SmartStationRobotics to a dictionary representation.

        Returns:
            dict: Dictionary containing the current state and configuration.
                Uses dict_factory_smartstation to handle special types like EuclideanTransform.
        """
        return asdict(self, dict_factory=dict_factory_smartstation)
