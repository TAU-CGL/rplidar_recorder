#!/usr/bin/env python3
"""
Script to analyze processed LIDAR measurement results.
Creates simplified dataframes with k_prime ratios relative to frame 0.
"""

import numpy as np
import pandas as pd
from pathlib import Path
import glob
from tqdm.auto import tqdm
import openpyxl  # For Excel writing

def process_result_file(parquet_file):
    """Process a single result file and compute k_prime ratios."""
    
    # Load the processed results
    df = pd.read_parquet(parquet_file)
    
    if len(df) == 0:
        return None
    
    # Group by pose (x, y, theta) to track the same pose across frames
    pose_groups = df.groupby(['x', 'y', 'theta'])
    
    simplified_rows = []
    
    for (x, y, theta), pose_data in pose_groups:
        # Get frame 0 data for this pose (reference)
        frame_0_data = pose_data[pose_data['frameIdx'] == 0]
        
        if len(frame_0_data) == 0:
            continue  # Skip poses that don't have frame 0 data
            
        # Get the k_prime value from frame 0 as reference
        frame_0_k_prime = frame_0_data['k_prime'].iloc[0]
        
        if frame_0_k_prime == 0:
            continue  # Skip poses with no valid measurements in frame 0
        
        # Process all other frames for this pose
        other_frames = pose_data[pose_data['frameIdx'] != 0]
        
        for _, row in other_frames.iterrows():
            k_prime_ratio = row['k_prime'] / frame_0_k_prime if frame_0_k_prime > 0 else 0
            
            simplified_rows.append({
                'x': x,
                'y': y,
                'theta': theta,
                'frameIdx': row['frameIdx'],
                'dist_to_env': row['dist_to_env'],
                'k_prime_ratio': k_prime_ratio,
                'frame_0_k_prime': frame_0_k_prime,  # For reference
                'current_k_prime': row['k_prime'],   # For reference
                'avg_error': row['avg_error']        # Keep this too
            })
    
    if simplified_rows:
        return pd.DataFrame(simplified_rows)
    else:
        return None

def main():
    # Configuration
    RESULTS_DIR = Path("/Volumes/My Passport/ICRA2026-DataCollection/processed_results")
    OUTPUT_DIR = Path("/Users/michaelbilevich/Projects/Research/rplidar-recorder/analyzed_results")  # Local disk
    
    # Create output directory
    OUTPUT_DIR.mkdir(exist_ok=True)
    
    # Find all processed parquet files
    parquet_files = list(RESULTS_DIR.glob("processed_*.parquet"))
    
    if not parquet_files:
        raise FileNotFoundError(f"No processed parquet files found in {RESULTS_DIR}")
    
    print(f"Found {len(parquet_files)} processed files to analyze")
    
    
    # Combine all processed files directly into one Excel file
    print("\nCombining all files into one Excel file...")
    
    combined_dfs = []
    
    for parquet_file in tqdm(parquet_files, desc="Processing and combining files"):
        try:
            simplified_df = process_result_file(parquet_file)
            
            if simplified_df is not None and len(simplified_df) > 0:
                simplified_df['source_file'] = parquet_file.stem  # Track source
                combined_dfs.append(simplified_df)
                print(f"✓ {parquet_file.name}: {len(simplified_df)} rows")
            else:
                print(f"⚠️ {parquet_file.name}: no valid data")
                
        except Exception as e:
            print(f"✗ Error processing {parquet_file.name}: {e}")
    
    if combined_dfs:
        # Combine all dataframes
        master_df = pd.concat(combined_dfs, ignore_index=True)
        
        # Save as single Parquet file (much faster than Excel)
        parquet_file = OUTPUT_DIR / "master_analysis.parquet"
        master_df.to_parquet(parquet_file, index=False)
        
        print(f"✓ Parquet file saved with {len(master_df)} rows")
        
        # Also save as CSV for easy viewing
        csv_file = OUTPUT_DIR / "master_analysis.csv"
        print("Also saving as CSV (this may take a moment for large files)...")
        master_df.to_csv(csv_file, index=False)
        
        print(f"✓ CSV file also saved for easy viewing")
        print(f"✓ Columns: {list(master_df.columns)}")
        
        # Print some statistics
        print(f"\nDataset Statistics:")
        print(f"- Total poses tracked: {master_df[['x', 'y', 'theta']].drop_duplicates().shape[0]}")
        print(f"- Frame indices range: {master_df['frameIdx'].min()} to {master_df['frameIdx'].max()}")
        print(f"- k_prime ratio range: {master_df['k_prime_ratio'].min():.3f} to {master_df['k_prime_ratio'].max():.3f}")
        print(f"- Average k_prime ratio: {master_df['k_prime_ratio'].mean():.3f}")
        print(f"- Distance to env range: {master_df['dist_to_env'].min():.3f}m to {master_df['dist_to_env'].max():.3f}m")
        
        print(f"\nFiles saved to: {OUTPUT_DIR}")
        print(f"- master_analysis.parquet (fast binary format)")
        print(f"- master_analysis.csv (human readable)")
    else:
        print("No valid data found in any files!")
    
    print(f"\nAnalysis complete! Results saved to: {OUTPUT_DIR}")

if __name__ == "__main__":
    main()