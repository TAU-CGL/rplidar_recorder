import numpy as np
from typing import Dict, Tuple

class LidarSnapshot:
    """
    Represents a single scan from a LiDAR device at a specific timestamp.
    """
    def __init__(self, device_id: str, timestamp: float, points: np.ndarray):
        """
        :param device_id: Unique ID of the LiDAR device
        :param timestamp: Time of the scan (e.g., UNIX time)
        :param points: N x 2 array of 2D points (x, y) from the scan
        """
        self.device_id = device_id
        self.timestamp = timestamp
        self.points = points


class CalibrationResult:
    """
    Stores transformation matrices between pairs of LiDAR devices.
    """
    def __init__(self, transforms: Dict[Tuple[str, str], np.ndarray]):
        """
        :param transforms: Dictionary mapping (from_id, to_id) to 3x3 homogeneous transform matrix
        """
        self.transforms = transforms

    def get_transform(self, from_id: str, to_id: str) -> np.ndarray:
        """
        Returns the transformation matrix from 'from_id' to 'to_id'.
        Raises a KeyError if the transform does not exist.
        """
        if (from_id, to_id) in self.transforms:
            return self.transforms[(from_id, to_id)]
        elif (to_id, from_id) in self.transforms:
            # Return inverse if only the reverse exists
            return np.linalg.inv(self.transforms[(to_id, from_id)])
        else:
            raise KeyError(f"No transformation available between {from_id} and {to_id}")
