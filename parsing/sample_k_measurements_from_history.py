#!/usr/bin/env python3
"""
Script to process LIDAR data and create measurement comparison table.
Converted from sample_k_measurements_from_history.ipynb for ICRA submission.
"""

import numpy as np
import pandas as pd
from pathlib import Path
import sdsl as _sdsl
import os
from typing import List, Dict, Tuple, Optional
import glob
from tqdm.auto import tqdm
from concurrent.futures import ProcessPoolExecutor
import multiprocessing

def sample_valid_poses(env, n_samples=1, max_tries=1000):
    """
    Sample valid poses inside the given environment.
    Returns a list of up to n_samples poses.
    """
    poses = []
    bb = env.bounding_box()
    tries = 0
    while len(poses) < n_samples and tries < max_tries:
        q = bb.sample()
        if env.is_inside(q):
            poses.append(q)
        tries += 1
    return poses

def decode_pcd_array(b):
    """Decode binary point cloud data."""
    if isinstance(b, bytes):
        f = np.frombuffer(b, dtype=np.float32)
        return f.reshape(-1, 2) if len(f) % 2 == 0 else np.empty((0, 2))
    return np.empty((0, 2))

def merge_point_clouds(row, devices, x_clamp=20, y_clamp=5):
    """Merge point clouds from all devices with clamping."""
    all_points = []
    for dev in devices:
        col = f"pcd_{dev}_transformed"
        points = row[col]
        if isinstance(points, np.ndarray) and points.ndim == 2 and points.shape[1] == 2:
            valid = points[np.isfinite(points).all(axis=1)]
            # Clamp to remove outliers
            mask = (np.abs(valid[:, 0]) < x_clamp) & (np.abs(valid[:, 1]) < y_clamp)
            clamped = valid[mask]
            if len(clamped) > 0:
                all_points.append(clamped)
    
    if not all_points:
        return None, None
    
    merged_points_2d = np.vstack(all_points)
    merged_points_3d = np.hstack([
        merged_points_2d,
        np.zeros((merged_points_2d.shape[0], 1))
    ])
    
    return merged_points_2d, merged_points_3d

def calculate_distance_to_env(pose_x, pose_y, env_points_2d):
    """Calculate minimum distance from pose to environment points."""
    pose_point = np.array([pose_x, pose_y])
    distances = np.linalg.norm(env_points_2d - pose_point, axis=1)
    return np.min(distances)

def process_parquet_file_worker(args):
    """Worker function for parallel processing of parquet files."""
    parquet_file, devices, output_dir = args
    
    # Recreate odometry in worker (since it can't be pickled)
    L = 0.05
    odometry = [
        _sdsl.R3xS2(
            L * np.cos(i * np.pi / 16), L * np.sin(i * np.pi / 16), 0,
            np.cos(i * np.pi / 16), np.sin(i * np.pi / 16), 0
        ) for i in range(32)
    ]
    
    # Decode point cloud data
    df = pd.read_parquet(parquet_file)
    for dev in devices:
        col = f"pcd_{dev}_transformed"
        df[col] = df[col].apply(decode_pcd_array)
    
    if len(df) == 0:
        return 0, parquet_file.name
    
    # Create output filename
    output_filename = f"processed_{parquet_file.stem}.parquet"
    output_file = output_dir / output_filename
    
    comparison_results = []
    MIN_VALID_BEAMS = 8
    
    # Step 1: Use frame 0 as reference frame
    ref_frame = df.iloc[0]
    ref_merged_2d, ref_merged_3d = merge_point_clouds(ref_frame, devices)
    
    if ref_merged_3d is None:
        return 0, parquet_file.name
    
    ref_env = _sdsl.Env_R3_PCD(ref_merged_3d)
    
    # Step 2: Sample 100 poses in the reference frame
    sampled_poses = sample_valid_poses(ref_env, 100, max_tries=1000)
    
    if not sampled_poses:
        return 0, parquet_file.name
    
    # Step 3: For each sampled pose, compute reference measurements
    pose_ref_measurements = {}
    valid_poses = []
    
    for q in sampled_poses:
        ref_measurements = [ref_env.measure_distance(q.act(g)) for g in odometry[:16]]
        valid_beams = sum(d < 1e6 for d in ref_measurements)
        
        if valid_beams >= MIN_VALID_BEAMS:
            pose_ref_measurements[len(valid_poses)] = ref_measurements
            valid_poses.append(q)
    
    if not valid_poses:
        return 0, parquet_file.name
    
    # Step 4: Sample every 100th frame and compare measurements for each pose
    sampled_frames = df.iloc[::100]
    
    for frame_idx, (_, current_frame) in tqdm(enumerate(sampled_frames.iterrows()), 
                                              total=len(sampled_frames), 
                                              desc=f"{parquet_file.name}",
                                              leave=True):
        
        # Merge point clouds for current frame
        current_merged_2d, current_merged_3d = merge_point_clouds(current_frame, devices)
        if current_merged_3d is None:
            continue
            
        current_env = _sdsl.Env_R3_PCD(current_merged_3d)
        
        # For each valid pose, compare measurements between reference and current frame
        for pose_idx, q in enumerate(valid_poses):
            ref_measurements = pose_ref_measurements[pose_idx]
            current_measurements = [current_env.measure_distance(q.act(g)) for g in odometry[:16]]
            
            # Count measurements that "agree" (both valid and similar)
            agreements = 0
            total_diffs = []
            
            for ref_m, curr_m in zip(ref_measurements, current_measurements):
                if ref_m < 1e6 and curr_m < 1e6:  # Both measurements valid
                    diff = abs(curr_m - ref_m)
                    total_diffs.append(diff)
                    if diff < 0.1:  # Agreement threshold - you can adjust this
                        agreements += 1
            
            if len(total_diffs) < MIN_VALID_BEAMS:
                continue
                
            # Calculate distance to environment (using current frame)
            dist_to_env = calculate_distance_to_env(q.x(), q.y(), current_merged_2d)
            
            comparison_results.append({
                "x": q.x(),
                "y": q.y(),
                "theta": q.r(),
                "avg_error": np.mean(total_diffs),
                "dist_to_env": dist_to_env,
                "k_prime": agreements,  # Number of measurements that agree
                "frameIdx": current_frame.name
            })
    
    # Save results
    if comparison_results:
        result_df = pd.DataFrame(comparison_results)
        result_df.to_parquet(output_file, index=False)
        return len(comparison_results), parquet_file.name
    else:
        return 0, parquet_file.name

def main():
    # Configuration
    DATA_DIR = Path("/Volumes/My Passport/ICRA2026-DataCollection/Floor4Kitchenette_Processed")
    OUTPUT_DIR = Path("/Volumes/My Passport/ICRA2026-DataCollection/processed_results")
    
    # Create output directory
    OUTPUT_DIR.mkdir(exist_ok=True)
    
    # Define devices and odometry
    devices = ['dev1', 'dev2', 'dev3', 'dev4', 'dev5']
    L = 0.05
    odometry = [
        _sdsl.R3xS2(
            L * np.cos(i * np.pi / 16), L * np.sin(i * np.pi / 16), 0,
            np.cos(i * np.pi / 16), np.sin(i * np.pi / 16), 0
        ) for i in range(32)
    ]
    
    # Find all parquet files
    parquet_files = list(DATA_DIR.glob("merged_batch_*.parquet"))
    if not parquet_files:
        raise FileNotFoundError(f"No parquet files found in {DATA_DIR}")
    
    print(f"Found {len(parquet_files)} parquet files to process")
    
    print("Ready to process files - each will use its own frame 0 as reference")
    
    # Process all files in parallel
    print(f"Processing {len(parquet_files)} files in parallel...")
    print("Each file will use its own frame 0 as reference and sample 100 poses")
    
    # Prepare arguments for parallel processing
    process_args = [
        (pf, devices, OUTPUT_DIR)
        for pf in parquet_files
    ]
    
    # Use ProcessPoolExecutor for parallel processing
    n_cores = min(multiprocessing.cpu_count(), len(parquet_files))
    total_measurements = 0
    
    with ProcessPoolExecutor(max_workers=n_cores) as executor:
        # Submit all jobs and use tqdm to track progress
        futures = [executor.submit(process_parquet_file_worker, args) for args in process_args]
        
        # Track progress with tqdm - each file shows its own progress bar
        print("Each file will show its own progress bar below:")
        
        for future in tqdm(futures, desc="Files completed", unit="file", position=0):
            try:
                measurements, filename = future.result()
                total_measurements += measurements
                print(f"✓ Completed {filename}: {measurements} measurements")
            except Exception as e:
                print(f"✗ Error processing {filename}: {e}")
    
    print(f"\nProcessing complete!")
    print(f"Total measurements collected: {total_measurements}")
    print(f"Results saved to: {OUTPUT_DIR}")
    print(f"Created {len(parquet_files)} output files")
    
    # Create a summary file
    summary_file = OUTPUT_DIR / "processing_summary.txt"
    with open(summary_file, 'w') as f:
        f.write(f"Processing Summary\n")
        f.write(f"=================\n")
        f.write(f"Total parquet files processed: {len(parquet_files)}\n")
        f.write(f"Total output files created: {len(parquet_files)}\n")
        f.write(f"Total measurements: {total_measurements}\n")
        f.write(f"Reference frame: 1000\n")
        f.write(f"Reference pose: x={ref_pose.x():.4f}, y={ref_pose.y():.4f}, theta={ref_pose.r():.4f}\n")
        f.write(f"Output columns: [x, y, theta, avg_error, dist_to_env, k_prime, frameIdx]\n")

if __name__ == "__main__":
    main()