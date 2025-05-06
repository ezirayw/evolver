import logging
from dataclasses import asdict, dataclass, field

import numpy as np
import skimage as ski
from skimage.transform import EuclideanTransform

from htevolver.robotics.xarm import xArmCoordinate

logger = logging.getLogger(__name__)


def dict_factory_smartstation(data):
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

    Attributes:
        x: The x-coordinate value.
        y: The y-coordinate value.
    """

    x: float
    y: float


@dataclass(kw_only=True)
class xArmPlane:
    vial0_x: float
    vial0_y: float
    vial17_x: float
    vial17_y: float
    z: float
    transform_matrix: EuclideanTransform = field(init=False)

    def update(self, plane_config: dict):
        for config_parameter, value in plane_config.items():
            if hasattr(self, config_parameter) and getattr(self, config_parameter) != value:
                setattr(self, config_parameter, value)
            logger.debug(f"Updated xArm parameter: {config_parameter}={value}")
        self.rigid_transform()

    def rigid_transform(self):
        """Calculates the rigid transformation matrix between coordinate systems.

        Creates a transformation matrix for converting vial coordinates into
        xArm coordinates using the calibration points.
        """

        vial_coordinates = np.array([[0, 36], [90, 0]])
        vial_0 = np.array([self.vial0_x, self.vial0_y])
        vial_17 = np.array([self.vial17_x, self.vial17_y])
        np.array([vial_0, vial_17])
        tform = ski.transform.EuclideanTransform()
        tform.estimate(vial_coordinates, np.array([vial_0, vial_17]))
        self.transform_matrix = tform

    def vial_to_xarm(self, evolver_coordinates: VialCoordinate) -> xArmCoordinate:
        """Transforms vial coordinates to xArm coordinates.

        Applies the rigid transformation matrix to convert from the evolver
        coordinate system to the xArm coordinate system.

        Args:
            evolver_coordinates (VialCoordinate): Coordinates in the evolver system.
                Example: VialCoordinate(x=18, y=36)

        Returns:
            xArmCoordinate: The transformed coordinates in the xArm system.
        """
        np_coordinates = np.array([[evolver_coordinates.x], [evolver_coordinates.y], [1]])
        transformed = np.dot(self.transform_matrix, np_coordinates)
        return xArmCoordinate(x=transformed[0][0], y=transformed[1][0], z=self.z)


@dataclass
class SmartStationRobotics:
    xArmPlane_in: xArmPlane
    xArmPlane_out: xArmPlane
    wash_location: VialCoordinate = field(default_factory=lambda: VialCoordinate(x=72, y=-29))
    wash_depth: float = field(init=False)
    vial_map: list[list[int]] = field(
        default_factory=lambda: [[0, 1, 2, 3, 4, 5], [11, 10, 9, 8, 7, 6], [12, 13, 14, 15, 16, 17]]
    )

    @classmethod
    def create(cls, config: dict):
        plane_in = xArmPlane(**config["plane_in"])
        plane_out = xArmPlane(**config["plane_out"])
        plane_in.rigid_transform()
        plane_out.rigid_transform()
        return cls(xArmPlane_in=plane_in, xArmPlane_out=plane_out)

    def update(self, station_config: dict):
        """Update the SmartStation xArmPlane calibration points based on the input configuration."""
        self.xArmPlane_in.update(station_config["plane_in"])
        self.xArmPlane_out.update(station_config["plane_out"])

    def to_dict(self):
        return asdict(self, dict_factory=dict_factory_smartstation)
