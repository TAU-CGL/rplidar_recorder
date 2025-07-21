"""
lidar_calibration.py
--------------------
Single‑shot 2‑D LiDAR calibration using a circular target.

Features
--------
* Works for any number of LiDARs (N ≥ 2).
* DBSCAN to isolate object clusters.
* Least‑squares circle fitting; choose the cluster whose radius is
  closest to the known calibration‑object radius.
* Computes pair‑wise 3×3 homogeneous transforms (translation only).
* Export / import transforms as human‑readable JSON.

Dependencies
------------
pip install numpy scipy scikit‑learn
"""

from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

import json
import numpy as np
from sklearn.cluster import DBSCAN
from scipy.optimize import leastsq


# ----------------------------------------------------------------------
# Data containers
# ----------------------------------------------------------------------

@dataclass
class LidarSnapshot:
    """A single synchronous 2‑D scan from one LiDAR device."""
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


# ----------------------------------------------------------------------
# Circle fitting helpers
# ----------------------------------------------------------------------

def _fit_circle_least_squares(pts: np.ndarray) -> Tuple[np.ndarray, float]:
    """
    Algebraic circle fit (Taubin / least‑squares).
    Returns (center_xy, radius).
    """

    def calc_R(c):
        return np.sqrt((pts[:, 0] - c[0])**2 + (pts[:, 1] - c[1])**2)

    def residuals(c):
        return calc_R(c) - calc_R(c).mean()

    center0 = pts.mean(axis=0)
    center, _ = leastsq(residuals, center0)
    radius = calc_R(center).mean()
    return center, radius


# ----------------------------------------------------------------------
# Per‑scan circle detection
# ----------------------------------------------------------------------

def find_calibration_circle(
    points: np.ndarray,
    expected_radius: float,
    radius_tol: float = 0.05,
    eps: float = 0.05,
    min_samples: int = 8,
) -> Tuple[np.ndarray, float]:
    """
    Identify the calibration circle in a point cloud.

    Parameters
    ----------
    points : np.ndarray
        (N, 2) xy points from one LiDAR scan.
    expected_radius : float
        Known radius of the calibration object (same units as points).
    radius_tol : float
        Acceptable absolute deviation from expected_radius.
    eps, min_samples
        DBSCAN parameters.

    Returns
    -------
    center : np.ndarray
        1‑D array [x, y] for the best circle's center.
    radius : float
        Fitted radius (mainly for diagnostics).

    Raises
    ------
    ValueError if no suitable circle is found.
    """
    db = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
    labels = db.labels_

    best_center, best_r, best_err = None, None, np.inf

    for k in set(labels):
        if k == -1:          # noise
            continue
        cluster = points[labels == k]
        if len(cluster) < 3:
            continue

        center, radius = _fit_circle_least_squares(cluster)
        err = abs(radius - expected_radius)

        if err < best_err and err <= radius_tol:
            best_center, best_r, best_err = center, radius, err

    if best_center is None:
        raise ValueError("Calibration circle not found in this scan.")
    return best_center, best_r


# ----------------------------------------------------------------------
# Multi‑LiDAR calibration (translation‑only)
# ----------------------------------------------------------------------

def calibrate_lidars(
    snapshots: List[LidarSnapshot],
    expected_radius: float,
    radius_tol: float = 0.05,
    eps: float = 0.05,
    min_samples: int = 8,
) -> CalibrationResult:
    """
    Compute pair‑wise rigid transforms for N ≥ 2 LiDARs given a single
    synchronized scan per device and a circular target.

    Returns
    -------
    CalibrationResult
        Contains T_from→to for every ordered pair (including inverses).
    """
    centers: Dict[str, np.ndarray] = {}

    # Step 1: Detect circle center in each LiDAR frame
    for snap in snapshots:
        c, _ = find_calibration_circle(
            snap.points, expected_radius, radius_tol, eps, min_samples
        )
        centers[snap.device_id] = c

    # Step 2: Build translation matrices between every pair
    transforms: Dict[Tuple[str, str], np.ndarray] = {}
    ids = list(centers.keys())

    for i, src in enumerate(ids):
        for j, dst in enumerate(ids):
            if i == j:
                continue
            offset = centers[dst] - centers[src]
            T = np.array(
                [[1.0, 0.0, offset[0]],
                 [0.0, 1.0, offset[1]],
                 [0.0, 0.0, 1.0]],
                dtype=float,
            )
            transforms[(src, dst)] = T

    return CalibrationResult(transforms)


# ----------------------------------------------------------------------
# JSON (de‑)serialisation utilities
# ----------------------------------------------------------------------

def calibration_to_dict(calib: CalibrationResult) -> Dict[str, Dict[str, List[List[float]]]]:
    """Convert CalibrationResult to a JSON‑serialisable nested dict."""
    out: Dict[str, Dict[str, List[List[float]]]] = {}
    for (src, dst), mat in calib.transforms.items():
        out.setdefault(src, {})[dst] = mat.tolist()
    return out


def save_calibration_json(calib: CalibrationResult, path: str | Path, *, indent: int = 2) -> None:
    """Write CalibrationResult to disk as pretty‑printed JSON."""
    with open(path, "w") as f:
        json.dump(calibration_to_dict(calib), f, indent=indent)
    print(f"[✓] Saved calibration to {path}")


def load_calibration_json(path: str | Path) -> CalibrationResult:
    """Load a CalibrationResult from JSON created by save_calibration_json()."""
    with open(path, "r") as f:
        data = json.load(f)

    transforms: Dict[Tuple[str, str], np.ndarray] = {}
    for src, dsts in data.items():
        for dst, mat in dsts.items():
            transforms[(src, dst)] = np.asarray(mat, dtype=float)
    return CalibrationResult(transforms)


# ----------------------------------------------------------------------
# Quick synthetic demo (five scanners)
# ----------------------------------------------------------------------

if __name__ == "__main__":
    np.random.seed(1)
    true_center = np.array([1.5, 2.5])
    R = 0.12                                      # radius of calibration object

    # Define offsets for five LiDARs in their own local frames
    offsets = [
        np.array([ 0.0,  0.0]),
        np.array([-0.4,  0.3]),
        np.array([ 0.6, -0.2]),
        np.array([-0.5, -0.5]),
        np.array([ 0.3,  0.6]),
    ]

    def synthetic_scan(sensor_offset: np.ndarray) -> np.ndarray:
        """Generate circle pts + random background for a single LiDAR."""
        th = np.linspace(0, 2 * np.pi, 90)
        circle = true_center + np.vstack((np.cos(th), np.sin(th))).T * R
        circle += (np.random.rand(*circle.shape) - 0.5) * 0.002          # small noise
        circle -= sensor_offset            # translate into LiDAR's local frame
        background = (np.random.rand(200, 2) - 0.5) * 4
        return np.vstack((circle, background))

    snapshots: List[LidarSnapshot] = [
        LidarSnapshot(f"L{i+1}", 0.0, synthetic_scan(off)) for i, off in enumerate(offsets)
    ]

    calib = calibrate_lidars(snapshots, expected_radius=R)
    save_calibration_json(calib, "five_scanner_calib.json")

    print("Example transform L1 → L3\n", calib.get_transform("L1", "L3"))
