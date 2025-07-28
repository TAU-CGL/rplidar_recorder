import reccalib

import cv2
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

from data import *

R1 = 0.18/2
R2 = 0.28/2

if __name__ == "__main__":
    raw_lidars = [lidar1, lidar2, lidar3, lidar4, lidar5]
    # raw_lidars = raw_lidars[:2]  # For testing, use only the first two
    lidars = []
    for raw_lidar in raw_lidars:
        lidar = []
        for i in range(720):
            theta = i / (720 - 1) * 2 * np.pi
            val = raw_lidar[i]
            if val < 0:
                continue
            lidar.append((val * np.cos(theta), val * np.sin(theta)))
        lidars.append(np.array(lidar))

    # Fit circles to each lidar
    circles = {}
    for i, lidar in enumerate(lidars):
        ls = reccalib.LidarSnapshot(points=lidar, device_id="test_device", timestamp=0)
        C1_ = reccalib.find_best_circle(ls, R1)
        C2_ = reccalib.find_best_circle(ls, R2)
        circles[i] = (C1_, C2_)

        plt.scatter(lidar[:,0], lidar[:,1], s=1, label='Lidar points')
        plt.gca().add_patch(plt.Circle(C1_.center, C1_.radius, color='r', fill=False, label='C1 radius'))
        plt.gca().add_patch(plt.Circle(C2_.center, C2_.radius, color='g', fill=False, label='C2 radius')) 
        plt.show()

    probable_scans = [lidars[0], lidars[3], lidars[4]]
    
    from_idx, to_idx = 1,4
    plt.plot(lidars[to_idx][:,0], lidars[to_idx][:,1], 'bo', markersize=2)
    
    T = (reccalib.find_best_transform(*circles[from_idx], *circles[to_idx]))
    print(T)
    transformed = np.dot(lidars[from_idx], T[:2, :2].T) + T[:2, 2]
    plt.plot(transformed[:,0], transformed[:,1], 'ro', markersize=2)

    # Draw the circle of to_idx, and the transformed circle of from_idx
    plt.gca().add_patch(plt.Circle(circles[to_idx][0].center, circles[to_idx][0].radius, color='r', fill=False, label='C1 radius'))
    plt.gca().add_patch(plt.Circle(circles[to_idx][1].center, circles[to_idx][1].radius, color='g', fill=False, label='C2 radius')) 
    plt.gca().add_patch(plt.Circle(np.dot(circles[from_idx][0].center, T[:2, :2].T) + T[:2, 2], circles[from_idx][0].radius, color='r', fill=False, linestyle='--', label='C1 radius (transformed)'))
    plt.gca().add_patch(plt.Circle(np.dot(circles[from_idx][1].center, T[:2, :2].T) + T[:2, 2], circles[from_idx][1].radius, color='g', fill=False, linestyle='--', label='C2 radius (transformed)'))
    plt.show()

