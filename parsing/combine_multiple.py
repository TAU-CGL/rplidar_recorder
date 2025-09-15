"""
A script that combines multiple, asynchronously generated lidar recording data into a single, united, calibrated (correctly transformed) data.
"""

import os
import json
import datetime
import itertools
import sys

import tqdm
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import cv2
from scipy.spatial.distance import cdist


# RECORDING_ROOT_DIR = "/Volumes/My Passport/ICRA2026-DataCollection/Lab446"
# OUT_DIR = "/Volumes/My Passport/ICRA2026-DataCollection/Lab446_Processed"
# TRANSFORMS_FILE = "scripts/raw/lab446a_20250828_1424/transforms.json"

RECORDING_ROOT_DIR = "/Volumes/My Passport/ICRA2026-DataCollection/Floor4Kitchenette/npz"
OUT_DIR = "/Volumes/My Passport/ICRA2026-DataCollection/Floor4Kitchenette_Processed"
TRANSFORMS_FILE = "scripts/raw/kitchenette/transforms.json"

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

def simple_icp(source_points, target_points, max_iterations=10, tolerance=1e-7):
    """Simple ICP implementation to refine transformation between point clouds"""
    if len(source_points) == 0 or len(target_points) == 0:
        return np.eye(3)
    
    # Subsample points if too many for speed
    if len(source_points) > 1000:
        indices = np.random.choice(len(source_points), 1000, replace=False)
        source_points = source_points[indices]
    if len(target_points) > 1000:
        indices = np.random.choice(len(target_points), 1000, replace=False)
        target_points = target_points[indices]
    
    current_source = source_points.copy()
    prev_error = float('inf')
    
    for iteration in range(max_iterations):
        # Find closest points
        distances = cdist(current_source, target_points)
        closest_indices = np.argmin(distances, axis=1)
        closest_target_points = target_points[closest_indices]
        
        # Compute centroids
        source_centroid = np.mean(current_source, axis=0)
        target_centroid = np.mean(closest_target_points, axis=0)
        
        # Center the points
        source_centered = current_source - source_centroid
        target_centered = closest_target_points - target_centroid
        
        # Compute rotation using SVD
        H = source_centered.T @ target_centered
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        
        # Ensure proper rotation matrix
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        
        # Compute translation
        t = target_centroid - R @ source_centroid
        
        # Apply transformation
        current_source = (R @ current_source.T).T + t
        
        # Check convergence
        error = np.mean(np.min(cdist(current_source, target_points), axis=1))
        if abs(prev_error - error) < tolerance:
            break
        prev_error = error
    
    # Build homogeneous transformation matrix
    transform = np.eye(3)
    transform[:2, :2] = R
    transform[:2, 2] = t
    
    return transform

def refine_transforms_with_icp(merged_df, transforms):
    """Refine transformation matrices using ICP on the first frame"""
    pcd_columns = [col for col in merged_df.columns if col.startswith('pcd_') and not col.endswith('_transformed')]
    devices = [col.replace('pcd_', '') for col in pcd_columns]
    devices = sorted(devices)
    
    if len(devices) < 2:
        return transforms
    
    reference_device = devices[0]
    first_row = merged_df.iloc[0]
    target_points = first_row[f'pcd_{reference_device}']
    
    refined_transforms = transforms.copy()
    
    print("Refining transforms with ICP...")
    for device in devices[1:]:
        if device in transforms and reference_device in transforms[device]:
            # Apply initial transform
            source_points = first_row[f'pcd_{device}']
            initial_transform = np.array(transforms[device][reference_device])
            transformed_source = transform_points(source_points, initial_transform)
            
            # Run ICP to refine
            icp_refinement = simple_icp(transformed_source, target_points)
            
            # Combine transforms
            combined_transform = icp_refinement @ initial_transform
            refined_transforms[device][reference_device] = combined_transform.tolist()
            
            print(f"Refined transform for {device} -> {reference_device}")
    
    return refined_transforms

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

    # Skip every 2nd frame to speed up rendering (2x faster)
    frame_indices = range(0, len(merged_df), 20)
    print(f"Creating video with {len(frame_indices)} frames (skipping every 2nd frame)...")
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

def save_batch_dataframes(merged_dfs_batch, batch_idx, output_dir):
    """Save a batch of merged dataframes in a highly compressed parquet format"""
    os.makedirs(output_dir, exist_ok=True)
    
    # Create filename based on batch index
    output_path = os.path.join(output_dir, f"merged_batch_{batch_idx:04d}.parquet")
    
    # Combine all dataframes in the batch
    combined_df = pd.concat(merged_dfs_batch, ignore_index=True)
    
    # Keep only transformed point clouds, timestamps, and tuple index
    columns_to_keep = ['ts', 'tuple_idx']
    transformed_cols = [col for col in combined_df.columns if col.endswith('_transformed')]
    columns_to_keep.extend(transformed_cols)
    
    # Filter to only the columns we want
    df_to_save = combined_df[columns_to_keep].copy()
    
    # Convert point cloud arrays to bytes for parquet compatibility
    for col in df_to_save.columns:
        if col.startswith('pcd_'):
            # Convert numpy arrays to bytes for compact storage, using float32 for smaller size
            df_to_save[col] = df_to_save[col].apply(
                lambda points: points.astype(np.float32).tobytes() if len(points) > 0 else b''
            )
    
    # Save as parquet with maximum compression
    df_to_save.to_parquet(
        output_path, 
        compression='zstd',  # Very good compression ratio
        compression_level=22,  # Maximum compression level for zstd
        index=False
    )
    return output_path

def load_merged_dataframe(parquet_path):
    """Load merged dataframe and reconstruct point cloud arrays"""
    df = pd.read_parquet(parquet_path)
    
    for col in df.columns:
        if col.startswith('pcd_'):
            # Convert bytes back to numpy arrays (now stored as float32)
            df[col] = df[col].apply(
                lambda data: np.frombuffer(data, dtype=np.float32).reshape(-1, 2) if len(data) > 0 else np.array([]).reshape(0, 2)
            )
    
    return df

if __name__ == "__main__":
    print("Getting all recording files...")
    all_files = get_all_recording_files(RECORDING_ROOT_DIR)
    with open("all_files.json", 'w') as f:
        json.dump(all_files, f, indent=2)
    print("Done.")
    with open("all_files.json", 'r') as f:
        all_files = json.load(f)
    intersecting_tuples = finds_all_intersecting_tuples(all_files)
    intersecting_tuples = sorted(intersecting_tuples, key=lambda x: x[0])
    print(f"Found {len(intersecting_tuples)} intersecting tuples.")

    with open(TRANSFORMS_FILE, 'r') as f:
        transforms = json.load(f)

    # Process all intersecting tuples in batches of 10
    BATCH_SIZE = 10
    num_batches = (len(intersecting_tuples) + BATCH_SIZE - 1) // BATCH_SIZE
    print(f"Processing {len(intersecting_tuples)} intersecting tuples in {num_batches} batches of {BATCH_SIZE}...")
    
    successful_batches = 0
    failed_tuples = 0
    
    for batch_idx in tqdm.tqdm(range(num_batches), desc="Processing batches"):
        start_idx = batch_idx * BATCH_SIZE
        end_idx = min((batch_idx + 1) * BATCH_SIZE, len(intersecting_tuples))
        batch_tuples = intersecting_tuples[start_idx:end_idx]
        
        print(f"\nBatch {batch_idx + 1}/{num_batches} (tuples {start_idx}-{end_idx-1})")
        
        # Process first tuple in batch to refine transforms with ICP
        if batch_tuples:
            try:
                print("  Loading first tuple for ICP refinement...")
                first_data = read_npzs_from_tuple(batch_tuples[0])
                first_merged_df = merge_npzs(first_data)
                print("  Running ICP refinement...")
                refined_transforms = refine_transforms_with_icp(first_merged_df, transforms)
                print("  ICP refinement complete")
            except Exception as e:
                print(f"  Error refining transforms for batch {batch_idx + 1}: {e}")
                refined_transforms = transforms
        else:
            refined_transforms = transforms
        
        # Process all tuples in this batch
        batch_merged_dfs = []
        batch_failed = 0
        
        print(f"  Processing {len(batch_tuples)} tuples in batch...")
        for tuple_idx, tuple_data in tqdm.tqdm(enumerate(batch_tuples), 
                                               desc="  Tuples in batch", 
                                               total=len(batch_tuples), 
                                               leave=False):
            try:
                # Read and merge data for this tuple
                data = read_npzs_from_tuple(tuple_data)
                merged_df = merge_npzs(data)
                
                # Apply refined transforms to all timestamps
                merged_df = add_transformed_points(merged_df, refined_transforms)

                # Display first union point cloud and exit
                if start_idx + tuple_idx == 0:
                    first_row = merged_df.iloc[0]
                    plt.figure(figsize=(10, 10))
                    colors = ['red', 'blue', 'green', 'orange', 'purple']
                    color_idx = 0
                    for col in merged_df.columns:
                        if col.endswith('_transformed'):
                            points = first_row[col]
                            if len(points) > 0:
                                plt.scatter(points[:, 0], points[:, 1], c=colors[color_idx % len(colors)], s=1, alpha=0.7, label=col.replace('pcd_', '').replace('_transformed', ''))
                                color_idx += 1
                    plt.legend()
                    plt.axis('equal')
                    plt.title('First Union Point Cloud')
                    plt.show()
                    # sys.exit(-1)

                # Add tuple index for reference
                merged_df['tuple_idx'] = start_idx + tuple_idx
                
                batch_merged_dfs.append(merged_df)
                
            except Exception as e:
                print(f"  Error processing tuple {start_idx + tuple_idx}: {e}")
                batch_failed += 1
                failed_tuples += 1
                continue
        
        # Save the entire batch as one file
        if batch_merged_dfs:
            try:
                print(f"  Saving batch with {len(batch_merged_dfs)} tuples...")
                output_path = save_batch_dataframes(batch_merged_dfs, batch_idx, OUT_DIR)
                successful_batches += 1
                print(f"  ✓ Saved batch {batch_idx + 1} to {os.path.basename(output_path)}")
            except Exception as e:
                print(f"  ✗ Error saving batch {batch_idx + 1}: {e}")
                failed_tuples += len(batch_merged_dfs)
        else:
            print(f"  ✗ No valid tuples in batch {batch_idx + 1}")
        
        if batch_failed > 0:
            print(f"  ⚠ Failed to process {batch_failed} tuples in batch {batch_idx + 1}")
        
        # Progress summary every 50 batches
        if (batch_idx + 1) % 50 == 0:
            print(f"\n--- Progress Summary ---")
            print(f"Completed: {batch_idx + 1}/{num_batches} batches ({(batch_idx + 1)/num_batches*100:.1f}%)")
            print(f"Successful batches: {successful_batches}")
            print(f"Failed tuples so far: {failed_tuples}")
            print("-----------------------\n")
    
    print(f"\n🎉 Processing complete!")
    print(f"Successfully processed batches: {successful_batches}/{num_batches}")
    print(f"Failed tuples: {failed_tuples}")
    print(f"Output directory: {OUT_DIR}")
    print(f"Expected output files: ~{num_batches} parquet files")