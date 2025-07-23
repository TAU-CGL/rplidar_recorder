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