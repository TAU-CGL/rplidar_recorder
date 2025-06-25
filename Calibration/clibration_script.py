#!/usr/bin/env python3
"""
quick_calib_5sensors.py
=======================

Proof-of-concept for the kitchen-LiDAR calibration with **five** fake sensors.
It reuses the same math we’ll use on real scans:

•   subtract baseline → isolate cylinder points
•   fit a circle centre in each shot
•   Procrustes (SVD) to solve SE(2) transforms
"""

from __future__ import annotations
import math, random
from typing import List
import numpy as np

# ─── Geometry helpers ────────────────────────────────────────────────────────

def detect_new_points(baseline: np.ndarray,
                      shot: np.ndarray,
                      eps: float = 0.02) -> np.ndarray:
    """Keep points whose nearest baseline neighbour is > eps m away."""
    d2 = ((shot[:, None, :] - baseline[None, :, :]) ** 2).sum(-1)
    return shot[d2.min(1) > eps ** 2]

def fit_circle(pts: np.ndarray) -> np.ndarray:
    """Algebraic LS fit; returns centre (x, y)."""
    A = np.hstack([2 * pts, np.ones((pts.shape[0], 1))])
    b = (pts ** 2).sum(1)
    x0, y0, _ = np.linalg.lstsq(A, b, rcond=None)[0]
    return np.array([x0, y0])

def solve_se2(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """Closed-form Procrustes → 3 × 3 homogeneous that maps src → dst."""
    src_c, dst_c = src.mean(0), dst.mean(0)
    X, Y = src - src_c, dst - dst_c
    U, _, Vt = np.linalg.svd(X.T @ Y)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:         # reflection guard
        Vt[1] *= -1
        R = Vt.T @ U.T
    t = dst_c - R @ src_c
    T = np.eye(3, dtype=np.float32)
    T[:2, :2], T[:2, 2] = R, t
    return T

# ─── Synthetic scan generator ────────────────────────────────────────────────

def make_scan(n: int,
              cyl: tuple[float, float] | None,
              r: float = .12,
              seed: int | None = None) -> np.ndarray:
    rng = np.random.default_rng(seed)
    walls = rng.uniform(-2, 2, size=(n, 2))      # “kitchen walls”
    if cyl:
        ang = rng.uniform(0, 2 * math.pi, 180)
        circle = np.column_stack([cyl[0] + r * np.cos(ang),
                                  cyl[1] + r * np.sin(ang)])
        walls = np.vstack([walls, circle])
    return walls.astype(np.float32)

def random_se2() -> np.ndarray:
    θ = math.radians(random.uniform(0, 360))
    tx, ty = random.uniform(-1, 1), random.uniform(-1, 1)
    R = np.array([[math.cos(θ), -math.sin(θ)],
                  [math.sin(θ),  math.cos(θ)]])
    T = np.eye(3, dtype=np.float32)
    T[:2, :2], T[:2, 2] = R, (tx, ty)
    return T

def apply(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    hom = np.c_[pts, np.ones(len(pts))]
    return (T @ hom.T).T[:, :2]

# ─── Main demo ───────────────────────────────────────────────────────────────

def main(num_sensors: int = 5) -> None:
    np.set_printoptions(precision=3, suppress=True)

    # 1. Ground-truth transforms sensor_i → world (sensor 0 is identity)
    T_world: List[np.ndarray] = [np.eye(3, dtype=np.float32)]
    T_world += [random_se2() for _ in range(num_sensors - 1)]

    # 2. World scans
    baseline_w = make_scan(3_000, None,            seed=1)
    shot1_w    = make_scan(3_000, ( 0.8,  0.2),    seed=2)
    shot2_w    = make_scan(3_000, (-0.4, -0.9),    seed=3)

    # 3. Convert scans into each sensor’s local frame (world → sensor_i)
    baseline_s, shot1_s, shot2_s = [], [], []
    for T in T_world:
        invT = np.linalg.inv(T)
        baseline_s.append(apply(invT, baseline_w))
        shot1_s.append(apply(invT, shot1_w))
        shot2_s.append(apply(invT, shot2_w))

    # 4. Extract cylinder centres for each sensor
    centres = []
    for b, s1, s2 in zip(baseline_s, shot1_s, shot2_s):
        c1 = fit_circle(detect_new_points(b, s1))
        c2 = fit_circle(detect_new_points(b, s2))
        centres.append(np.stack([c1, c2]))

    # 5. Estimate sensor_i → sensor_0 transforms
    est_T: List[np.ndarray] = [np.eye(3, dtype=np.float32)]
    for i in range(1, num_sensors):
        est_T.append(solve_se2(centres[i], centres[0]))

    # 6. Compare with ground-truth
    print("Sensor i  |  rot-err (°)   trans-err (cm)")
    print("-------------------------------------------")
    for i in range(num_sensors):
        # true transform sensor_i → sensor_0
        T_true = np.linalg.inv(T_world[0]) @ T_world[i]
        dR = est_T[i][:2, :2] @ T_true[:2, :2].T
        rot_err = math.degrees(math.acos(np.clip((dR[0, 0] + dR[1, 1]) / 2, -1, 1)))
        trans_err = np.linalg.norm(est_T[i][:2, 2] - T_true[:2, 2]) * 100
        print(f"   {i:<6} |   {rot_err:8.5f}     {trans_err:8.4f}")

if __name__ == "__main__":
    random.seed(42)
    np.random.seed(42)
    main()
