"""
A script that combines multiple, asynchronously generated lidar recording data into a single, united, calibrated (correctly transformed) data.
"""

import os
import json
import datetime
import itertools

import tqdm
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import cv2


RECORDING_ROOT_DIR = "/Volumes/My Passport/ICRA2026-DataCollection/Lab446"
TRANSFORMS_FILE = "scripts/raw/lab446a_20250828_1424/transforms.json"
UUID_TO_DEV = {
    "8dc0b0fc-cb63-4f6c-ad49-b6f27673fef9": "dev1",
    "6bf095b9-06a5-495b-88d9-194e6357eeb1": "dev2",
    "a4b8e68f-99b0-4c80-b2b6-de006906af2f": "dev3",
    "44d9d253-58cc-4dca-a412-bb4803eef6c9": "dev4",
    "fcc61f1f-0193-4e19-9242-edcd505ca4f7": "dev5"
}


def get_all_recording_files(root_dir):
    all_files = {}
    for dev_uuid in os.listdir(root_dir):
        if dev_uuid.startswith('.'):
            continue
        all_files[dev_uuid] = []
        dev_dir = os.path.join(root_dir, dev_uuid)
        for item in os.listdir(dev_dir):
            dir_path = os.path.join(dev_dir, item)
            if item.startswith('.') or not os.path.isdir(dir_path):
                continue
            for filename in os.listdir(dir_path):
                all_files[dev_uuid].append(os.path.join(dir_path, filename))
    return all_files

def parse_filename(filename):
    filename = os.path.basename(filename)
    base, idx = filename.split('_', 1)
    idx = int(idx.split('.')[0])
    start = datetime.datetime.strptime(base, "%Y-%m-%d-%H-%M-%S") + datetime.timedelta(minutes=idx*5)
    end = start + datetime.timedelta(minutes=5)
    return start, end

def finds_all_intersecting_tuples(all_files):
    """
    Sweep-line algorithm to find all tuples that can intersect with each other.
    """ 
    events = []
    print("Parsing events...")
    for dev_idx, files in tqdm.tqdm(enumerate(all_files.values())):
        for filename in files:
            start, end = parse_filename(filename)
            events.append((start, 'start', dev_idx, filename))
            events.append((end, 'end', dev_idx, filename))
    events.sort()

    active = [set() for _ in range(len(all_files))]
    result = set()
    print("Finding intersecting tuples...")
    for time, event_type, dev_idx, filename in tqdm.tqdm(events):
        if event_type == 'start':
            active[dev_idx].add(filename)
            if all(active):
                for combo in itertools.product(*active):
                    result.add(combo)
        else:
            active[dev_idx].discard(filename)
    
    return list(result)

def get_dev_name(filename):
    for dev_uuid in UUID_TO_DEV:
        if dev_uuid in filename:
            return UUID_TO_DEV[dev_uuid]
    return "unknown"

def read_npzs_from_tuple(npz_tuple):
    dfs = []
    for npz_file in npz_tuple:
        data = np.load(npz_file)
        df = pd.DataFrame({
            "ts": data["ts"],
            f"pcd_{get_dev_name(npz_file)}": list(data["pcd"])
        })
        dfs.append(df)
    return dfs

def ranges_to_points(points):
    points = np.array(points)
    if points.ndim == 1 and len(points) == 0:
        return np.array([]).reshape(0, 2)
    if points.ndim == 1:
        points = points.reshape(-1, 2)
    # Filter out NaN and Inf values
    valid_mask = ~(np.isnan(points).any(axis=1) | np.isinf(points).any(axis=1))
    points = points[valid_mask]
    return -points[:, [0, 1]]

def merge_npzs(dfs):
    merged_df = dfs[0]
    for df in dfs[1:]:
        merged_df = pd.merge_asof(merged_df, df, on='ts', direction='nearest')
    for col in merged_df.columns:
        if col.startswith('pcd_'):
            merged_df[col] = merged_df[col].apply(ranges_to_points)
    
    return merged_df

def transform_points(points, transform_matrix):
    if len(points) == 0:
        return points
    
    transform_matrix = np.array(transform_matrix)
    # Try different transformation approach - maybe matrix needs to be transposed
    homogeneous_points = np.hstack([points, np.ones((points.shape[0], 1))])
    transformed = np.dot(transform_matrix, homogeneous_points.T).T
    return transformed[:, :2]

def add_transformed_points(merged_df, transforms):
    pcd_columns = [col for col in merged_df.columns if col.startswith('pcd_')]
    devices = [col.replace('pcd_', '') for col in pcd_columns]
    devices = sorted(devices)
    
    if not devices:
        return merged_df
    
    reference_device = devices[0]
    merged_df[f'pcd_{reference_device}_transformed'] = merged_df[f'pcd_{reference_device}']
    
    for device in devices[1:]:
        if device in transforms and reference_device in transforms[device]:
            transform_matrix = np.array(transforms[device][reference_device])
            merged_df[f'pcd_{device}_transformed'] = merged_df[f'pcd_{device}'].apply(
                lambda points: transform_points(points, transform_matrix)
            )
        else:
            merged_df[f'pcd_{device}_transformed'] = merged_df[f'pcd_{device}']
    
    return merged_df

def create_video_from_dataframe(merged_df, output_path="video.mp4", fps=15):
    transformed_columns = [col for col in merged_df.columns if col.endswith('_transformed')]
    
    if not transformed_columns:
        print("No transformed point cloud columns found!")
        return
    
    all_points = []
    for _, row in merged_df.iterrows():
        frame_points = []
        for col in transformed_columns:
            points = row[col]
            if len(points) > 0:
                frame_points.extend(points)
        if frame_points:
            all_points.extend(frame_points)
    
    if not all_points:
        print("No points found to create video!")
        return
        
    all_points = np.array(all_points)
    x_min, x_max = all_points[:, 0].min(), all_points[:, 0].max()
    y_min, y_max = all_points[:, 1].min(), all_points[:, 1].max()
    
    margin = 0.5
    x_min, x_max = x_min - margin, x_max + margin
    y_min, y_max = y_min - margin, y_max + margin

    x_min, x_max = -2.5, 5.5
    y_min, y_max = -4, 4

    fig, ax = plt.subplots(figsize=(10, 10))
    
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_path, fourcc, fps, (1000, 1000))
    
    # Skip every 4th frame to speed up rendering (4x faster)
    frame_indices = range(0, len(merged_df), 4)
    print(f"Creating video with {len(frame_indices)} frames (skipping every 4th frame)...")
    for frame_idx, idx in tqdm.tqdm(enumerate(frame_indices), total=len(frame_indices)):
        row = merged_df.iloc[idx]
        ax.clear()
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_aspect('equal')
        ax.set_title(f'Combined Point Cloud - Frame {frame_idx}')
        
        colors = ['red', 'blue', 'green', 'orange', 'purple']
        color_idx = 0
        
        for col in transformed_columns:
            points = row[col]
            if len(points) > 0:
                ax.scatter(points[:, 0], points[:, 1], 
                          c=colors[color_idx % len(colors)], s=1, alpha=0.7,
                          label=col.replace('pcd_', '').replace('_transformed', ''))
                color_idx += 1
        
        ax.legend()
        
        fig.canvas.draw()
        image = np.asarray(fig.canvas.renderer.buffer_rgba())[:,:,:3]
        img_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        img_resized = cv2.resize(img_bgr, (1000, 1000))
        out.write(img_resized)
    
    out.release()
    plt.close(fig)
    print(f"Video saved to {output_path}")

if __name__ == "__main__":
    # print("Getting all recording files...")
    # all_files = get_all_recording_files(RECORDING_ROOT_DIR)
    # with open("all_files.json", 'w') as f:
    #     json.dump(all_files, f, indent=2)
    # print("Done.")
    with open("all_files.json", 'r') as f:
        all_files = json.load(f)
    intersecting_tuples = finds_all_intersecting_tuples(all_files)
    print(f"Found {len(intersecting_tuples)} intersecting tuples.")
    print(intersecting_tuples[0])

    with open(TRANSFORMS_FILE, 'r') as f:
        transforms = json.load(f)

    DATA_IDX = 20800

    data = read_npzs_from_tuple(intersecting_tuples[DATA_IDX])
    merged_df = merge_npzs(data)
    merged_df = add_transformed_points(merged_df, transforms)
    print(merged_df)    
    create_video_from_dataframe(merged_df, "video.mp4")