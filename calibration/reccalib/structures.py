from typing import Dict, Tuple, List
from dataclasses import dataclass

import numpy as np


@dataclass
class LidarSnapshot:
    """A single synchronous 2D scan from one LiDAR device"""
    device_id: str
    timestamp: float          # UNIX time or any synchronized clock
    points: np.ndarray        # shape (N, 2)  --  [[x, y], ...]


@dataclass
class CalibrationResult:
    """
    Stores homogeneous transforms between LiDAR frames.
    transforms[(A, B)] = T_AB  (maps points from frame A → frame B)
    """
    transforms: Dict[Tuple[str, str], np.ndarray]

    def get_transform(self, from_id: str, to_id: str) -> np.ndarray:
        """
        Return T_from→to.  If only the reverse exists, return its inverse.
        """
        if (from_id, to_id) in self.transforms:
            return self.transforms[(from_id, to_id)]
        if (to_id, from_id) in self.transforms:
            return np.linalg.inv(self.transforms[(to_id, from_id)])
        raise KeyError(f"No transform between {from_id!r} and {to_id!r}")

@dataclass
class Circle:
    """A circle defined by its center and radius."""
    center: np.ndarray  # shape (2,)
    radius: float

    def representing_polygon(self, num_points: int = 64) -> np.ndarray:
        """
        Return a polygonal representation of the circle.
        """
        angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
        return self.center + self.radius * np.column_stack((np.cos(angles), np.sin(angles)))