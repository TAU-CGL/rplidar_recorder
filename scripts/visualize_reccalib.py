import reccalib

import sys
import json

import cv2
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

from data import *

R = 0.18

FILE1 = "scripts/raw/kitchenette/scans_2025-09-04-16-43-09.json"
FILE2 = "scripts/raw/kitchenette/scans_2025-09-04-16-51-11.json"

if __name__ == "__main__":

    with open(FILE1, 'r') as fp:
        lidars1 = json.load(fp)
    with open(FILE2, 'r') as fp:
        lidars2 = json.load(fp)
    devices = ["dev1", "dev2", "dev3", "dev4", "dev5"]

    lidars1_tmp = {
        "dev1": (2.35, -2.14),
        "dev2": (1.06, -3.72),
        "dev3": (-2.16, -3.88),
        "dev4": (0.407, -5.46),
        "dev5": (0.35, -1.49),
    }
    lidars2_tmp = {
        "dev1": (1.972, -3.50),
        "dev2": (2.454, -4.12),
        "dev3": (-0.74, -4.28),
        "dev4": (0.848, -4.08),
        "dev5": (0.376, -1.49),
    }
    dx = 1

    circle1 = {}
    circle2 = {}

    for lidars, lidars_tmp, circle in [(lidars1, lidars1_tmp, circle1), (lidars2, lidars2_tmp, circle2)]:
        for dev in devices:
            points = lidars[dev]
            filtered_points = []
            x_, y_ = lidars_tmp[dev]
            for x, y in points:
                if x < x_ - dx or x > x_ + dx:
                    continue
                if y < y_ - dx or y > y_ + dx:
                    continue
                filtered_points.append((x, y))
            filtered_points = np.array(filtered_points)
            points = np.array(lidars[dev])
            ls = reccalib.LidarSnapshot(points=filtered_points, device_id=dev, timestamp=0)
            circ = reccalib.find_best_circle(ls, R)
            circle[dev] = {"center": circ.center.tolist(), "radius": circ.radius}

            print(dev, circ)

            plt.scatter(points[:,0], points[:,1], s=1)
            plt.gca().add_patch(plt.Circle(circ.center, circ.radius, color='r', fill=False, label='C1 radius'))
            plt.show()

    print("CIRCLE1\n\n")
    print(json.dumps(circle1))
    print("\n\n----------------------\n\nCIRCLE2\n\n")
    print(json.dumps(circle2))

    sys.exit(0)


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

