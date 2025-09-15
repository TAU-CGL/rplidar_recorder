#!/usr/bin/env python3
"""
npz_converter_local.py — **hard‑coded directory version**
--------------------------------------------------------
* Converts every `*.db3.zstd` ROS 2 bag under `~/Downloads/sftp` into a mirrored
  `.npz` tree under `~/Downloads/npz_output`.
* Uses **round‑robin** traversal so one file from each contraption folder is
  produced early.
* Skips bags that already have a matching `.npz` in the output tree.
* Requires ROS 2 Humble (`rosbag2_py`) — be sure to `source /opt/ros/humble/setup.bash`
  before running.

Run:
    source /opt/ros/humble/setup.bash
    python npz_converter_local.py

The generated `.npz` archives each contain:
    ts   – `int64  (N,)`          timestamps (nanoseconds)
    pcd  – `float32 (N, 2×Beams)` flattened Cartesian scan points
"""
from __future__ import annotations

import os
import random
import string
from pathlib import Path
from typing import Dict, List

import numpy as np
import tqdm
import zstandard as zstd
import rclpy
from sensor_msgs.msg import LaserScan
import rosbag2_py
from rclpy.serialization import deserialize_message

# ---------------------------------------------------------------------------
# Hard‑coded I/O locations  (edit here only)
# ---------------------------------------------------------------------------
INPUT_DIR = Path("/Volumes/My Passport/ICRA2026-DataCollection/Floor4Kitchenette/raw")
OUTPUT_DIR = Path("/Volumes/My Passport/ICRA2026-DataCollection/Floor4Kitchenette/npz")
OVERWRITE = False                                         # set True to redo

# ---------------------------------------------------------------------------
# Constants & temp‑file settings
# ---------------------------------------------------------------------------
ROSBAG_EXTENSION = ".db3.zstd"
NPZ_EXTENSION = ".npz"
TMP_DIR = Path("tmp")
TMP_PATTERN = "{}.db3"
ALPHABET = string.ascii_lowercase + string.digits


# ---------------------------------------------------------------------------
# Utility helpers
# ---------------------------------------------------------------------------

def random_stem(k: int = 8) -> str:
    return "".join(random.choices(ALPHABET, k=k))


def ensure_tmp_dir() -> None:
    TMP_DIR.mkdir(exist_ok=True)


def decompress_zstd(src: str) -> Path:
    dst = TMP_DIR / TMP_PATTERN.format(random_stem())
    with open(src, "rb") as fin, open(dst, "wb") as fout:
        zstd.ZstdDecompressor().copy_stream(fin, fout)
    return dst


# ---------------------------------------------------------------------------
# ROS bag reading utilities
# ---------------------------------------------------------------------------

def read_laserscan_messages(db3_path: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=db3_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                    output_serialization_format="cdr"),
    )
    while reader.has_next():
        topic, data, ts = reader.read_next()
        try:
            yield ts, deserialize_message(data, LaserScan)
        except Exception:
            continue  # skip non‑LaserScan topics
    del reader


# ---------------------------------------------------------------------------
# Scan → NumPy conversion
# ---------------------------------------------------------------------------

def scan_to_cartesian(msg: LaserScan) -> np.ndarray:
    n = len(msg.ranges)
    angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment
    ranges = np.asarray(msg.ranges, dtype=np.float32)

    mask = ~np.isfinite(ranges) | (ranges < msg.range_min) | (ranges > msg.range_max)
    ranges[mask] = np.nan

    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    return np.column_stack((x, y)).ravel()


# ---------------------------------------------------------------------------
# Single‑bag conversion routine
# ---------------------------------------------------------------------------

def convert_bag(src_bag: str, dst_npz: Path) -> bool:
    print(f"Processing: {src_bag}")
    tmp_db3 = decompress_zstd(src_bag)

    try:
        rows, ts_list = [], []
        for ts, scan in read_laserscan_messages(str(tmp_db3)):
            ts_list.append(ts)
            rows.append(scan_to_cartesian(scan))
        if not rows:
            print("  ⚠️  no LaserScan messages – skipped")
            return False
        arr = np.vstack(rows).astype(np.float32, copy=False)
        np.savez_compressed(dst_npz, ts=np.asarray(ts_list, dtype=np.int64), pcd=arr)
        print(f"  ✔  saved → {dst_npz.relative_to(OUTPUT_DIR)}")
        return True
    finally:
        if tmp_db3.exists():
            tmp_db3.unlink()


# ---------------------------------------------------------------------------
# Folder traversal & round‑robin queue builder
# ---------------------------------------------------------------------------

def gather_by_folder(root: Path) -> Dict[str, List[str]]:
    mapping: Dict[str, List[str]] = {}
    for bag in root.rglob(f"*{ROSBAG_EXTENSION}"):
        mapping.setdefault(str(bag.parent), []).append(str(bag))
    for lst in mapping.values():
        lst.sort()
    return mapping


def round_robin(mapping: Dict[str, List[str]]) -> List[str]:
    queue: List[str] = []
    idx, more = 0, True
    while more:
        more = False
        for lst in mapping.values():
            if idx < len(lst):
                queue.append(lst[idx])
                more = True
        idx += 1
    return queue


def relative_npz_path(root: Path, bag_path: str) -> Path:
    rel = Path(bag_path).relative_to(root)
    return rel.with_suffix("").with_suffix(NPZ_EXTENSION)


# ---------------------------------------------------------------------------
# End‑to‑end directory driver
# ---------------------------------------------------------------------------

def convert_all() -> None:
    ensure_tmp_dir()
    src_root = INPUT_DIR.expanduser().resolve()
    dst_root = OUTPUT_DIR.expanduser().resolve()
    dst_root.mkdir(parents=True, exist_ok=True)

    mapping = gather_by_folder(src_root)
    queue = round_robin(mapping)

    # skip previously converted unless OVERWRITE=True
    bags_to_do = [b for b in queue if OVERWRITE or not (dst_root / relative_npz_path(src_root, b)).exists()]

    if not bags_to_do:
        print("All up‑to‑date — nothing to do ✨")
        return

    print(f"Will convert {len(bags_to_do)} bag(s) → {dst_root}\n")
    for bag in tqdm.tqdm(bags_to_do, desc="Converting rosbags"):
        dst_npz = dst_root / relative_npz_path(src_root, bag)
        dst_npz.parent.mkdir(parents=True, exist_ok=True)
        try:
            convert_bag(bag, dst_npz)
        except Exception as exc:
            print(f"❌  failed {bag}: {exc}")


# ---------------------------------------------------------------------------
# Script entry‑point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    rclpy.init(args=None)  # ensure rosbag2_py works without warnings
    convert_all()
