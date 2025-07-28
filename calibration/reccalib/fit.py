from typing import Tuple

import numpy as np
from scipy.optimize import leastsq

from .structures import LidarSnapshot, Circle


def fit_circle_least_squares(snapshot: LidarSnapshot) -> Circle:
    pts = snapshot.points
    def calc_R(c):
        return np.sqrt((pts[:, 0] - c[0])**2 + (pts[:, 1] - c[1])**2)
    def residuals(c):
        return calc_R(c) - calc_R(c).mean()
    
    center0 = pts.mean(axis=0)
    center, _ = leastsq(residuals, center0)
    radius = calc_R(center).mean()
    return Circle(center=center, radius=radius)

def find_best_circle(snapshot: LidarSnapshot, radius: float) -> Circle:
    """
    Apply DBSCAN on lidar points. For each cluster, try to fit a circle.
    Return circle that has the best fit to the given radius.
    """
    from sklearn.cluster import DBSCAN

    # Cluster points using DBSCAN
    clustering = DBSCAN(eps=0.05, min_samples=3).fit(snapshot.points)
    labels = clustering.labels_

    best_circle = None
    best_error = float('inf')

    for label in set(labels):
        if label == -1:  # Skip noise points
            continue
        cluster_points = snapshot.points[labels == label]
        if len(cluster_points) < 3:  # Need at least 3 points to fit a circle
            continue
        
        circle = fit_circle_least_squares(LidarSnapshot(snapshot.device_id, snapshot.timestamp, cluster_points))
        # Error should be as close the the given radius as possible, but points should have low variance
        error = abs(circle.radius - radius) + 0.5 * np.std(np.linalg.norm(cluster_points - circle.center, axis=1))

        
        if error < best_error:
            best_error = error
            best_circle = circle

    return best_circle if best_circle else Circle(center=np.zeros(2), radius=0.0)


def find_best_transform(C1_: Circle, C2_: Circle, C1: Circle, C2: Circle) -> np.ndarray:
    """
    Find the best transform that maps C1_ to C2_ based on the original circles C1 and C2.
    """
    A = np.stack([C1_.center, C2_.center])
    B = np.stack([C1.center, C2.center])
    centroid_A = A.mean(axis=0)
    centroid_B = B.mean(axis=0)
    AA = A - centroid_A
    BB = B - centroid_B

    H = AA.T @ BB
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T

    t = centroid_B - R @ centroid_A
    transform = np.eye(3)
    transform[:2, :2] = R
    transform[:2, 2] = t
    return transform


def symb_lagrangian():
    """
    Symbolic computation of the Lagrangian and its derivatives (Jacobian & Hessian)
    """
    import sympy as sp

    ct, st, a, b, l = sp.symbols('ct st a b l')
    x1, y1, x2, y2 = sp.symbols('x1 y1 x2 y2')
    x1_, y1_, x2_, y2_ = sp.symbols('x1_ y1_ x2_ y2_')
    L = 0.5 * (ct * x1_ - st * y1_ + a - x1)**2 + (st * x1_ + ct * y1_ + b - y1)**2 + \
        0.5 * (ct * x2_ - st * y2_ + a - x2)**2 + (st * x2_ + ct * y2_ + b - y2)**2 + \
        l * (ct**2 + st**2 - 1)
    print("Lagrangian:", L)