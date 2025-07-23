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
    clustering = DBSCAN(eps=0.1, min_samples=5).fit(snapshot.points)
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
        error = abs(circle.radius - radius) + np.std(np.linalg.norm(cluster_points - circle.center, axis=1))

        
        if error < best_error:
            best_error = error
            best_circle = circle

    return best_circle if best_circle else Circle(center=np.zeros(2), radius=0.0)