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

def ranges_to_points(ranges):
    points = []
    N = len(ranges)
    for i in range(N):
        if np.isnan(ranges[i]) or ranges[i] < 0:
            continue
        theta = (i / (N-1)) * 2 * np.pi
        x = ranges[i] * np.cos(theta)
        y = ranges[i] * np.sin(theta)
        points.append((x, y))
    return np.array(points)

def merge_npzs(dfs):
    merged_df = dfs[0]
    for df in dfs[1:]:
        merged_df = pd.merge_asof(merged_df, df, on='ts', direction='nearest')
    for col in merged_df.columns:
        if col.startswith('pcd_'):
            merged_df[col] = merged_df[col].apply(ranges_to_points)
    
    return merged_df

def transform_points(points, transforms):
    # TODO

def add_transformed_points(merged_df, transforms):
    # TODO

if __name__ == "__main__":
    print("Getting all recording files...")
    all_files = get_all_recording_files(RECORDING_ROOT_DIR)
    print("Done.")
    intersecting_tuples = finds_all_intersecting_tuples(all_files)
    print(f"Found {len(intersecting_tuples)} intersecting tuples.")
    print(intersecting_tuples[0])

    data = read_npzs_from_tuple(intersecting_tuples[0])
    merged_df = merge_npzs(data)
    print(merged_df)
