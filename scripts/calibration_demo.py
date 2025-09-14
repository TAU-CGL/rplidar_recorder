import json

import numpy as np
import matplotlib.pyplot as plt

import reccalib


BASE_DIR = "scripts/raw/lab446a_20250828_1424"


with open(f'{BASE_DIR}/scans.json', 'r') as fp:
    scans = json.load(fp)
with open(f'{BASE_DIR}/circle1.json') as fp:
    circle1 = json.load(fp)
with open(f'{BASE_DIR}/circle2.json') as fp:
    circle2 = json.load(fp)


devices = ["dev1", "dev2", "dev3", "dev4", "dev5"]
lidars = [np.array(scans[dev]) for dev in devices]
circles = {}
for i, dev in enumerate(devices):
    c1 = circle1[dev]
    c1 = reccalib.Circle(center=np.array(c1['center']), radius=c1['radius'])
    c2 = circle2[dev]
    c2 = reccalib.Circle(center=np.array(c2['center']), radius=c2['radius'])
    circles[i] = (c1, c2)


for i in range(1, len(devices)):
    from_idx, to_idx = i, 0
    plt.plot(lidars[to_idx][:,0], lidars[to_idx][:,1], 'bo', markersize=2)

    T = (reccalib.find_best_transform(*circles[from_idx], *circles[to_idx]))
    transformed = np.dot(lidars[from_idx], T[:2, :2].T) + T[:2, 2]
    plt.plot(transformed[:,0], transformed[:,1], 'ro', markersize=2)

    # Draw the circle of to_idx, and the transformed circle of from_idx
    plt.gca().add_patch(plt.Circle(circles[to_idx][0].center, circles[to_idx][0].radius, color='r', fill=False, label='C1 radius'))
    plt.gca().add_patch(plt.Circle(circles[to_idx][1].center, circles[to_idx][1].radius, color='g', fill=False, label='C2 radius')) 
    plt.gca().add_patch(plt.Circle(np.dot(circles[from_idx][0].center, T[:2, :2].T) + T[:2, 2], circles[from_idx][0].radius, color='r', fill=False, linestyle='--', label='C1 radius (transformed)'))
    plt.gca().add_patch(plt.Circle(np.dot(circles[from_idx][1].center, T[:2, :2].T) + T[:2, 2], circles[from_idx][1].radius, color='g', fill=False, linestyle='--', label='C2 radius (transformed)'))
plt.show()