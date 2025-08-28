import os
import string
import random
from typing import List
from stat import S_ISDIR, S_ISREG

import tqdm
import paramiko
import zstandard
import numpy as np
import matplotlib.pyplot as plt

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import LaserScan


SFTP_HOST = os.environ["SFTP_HOST"]
SFTP_PORT = int(os.environ["SFTP_PORT"])
SFTP_USERNAME = os.environ["SFTP_USERNAME"]
SFTP_PASSWORD = os.environ["SFTP_PASSWORD"]

ROOT_PATH = "/home/rplidar_recorder"
ROSBAG_EXTENSION = ".db3.zstd"
NPZ_EXTENSION = ".npz"

TEMP_DB3 = "tmp/{}.db3"
alphabet = string.ascii_lowercase + string.digits
def random_choice():
    return ''.join(random.choices(alphabet, k=8))

def decompress_zstd(input_file: str):
    with open(input_file, "rb") as fp:
        decomp = zstandard.ZstdDecompressor()
        tmp_filename = TEMP_DB3.format(random_choice())
        with open(tmp_filename, "wb") as fp_:
            decomp.copy_stream(fp, fp_)
        return tmp_filename

# Based on example from: https://mcap.dev/guides/python/ros2
def read_messages(input_bag: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=input_bag, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg = deserialize_message(data, LaserScan)
        yield topic, msg, timestamp
    del reader

def process_message(msg: LaserScan):
    """
    Convert message data to numpy array, of the form:
    (x0, y0, x1, y1, ...., x719, y719)
    """
    arrlen = 2 * len(msg.ranges)
    arr = np.zeros(arrlen, dtype=np.float64)
    
    # Convert ranges to x, y coordinates
    for i in range(len(msg.ranges)):
        angle = msg.angle_min + i * msg.angle_increment
        arr[2 * i    ] = msg.ranges[i] * np.cos(angle)
        arr[2 * i + 1] = msg.ranges[i] * np.sin(angle)
    return arr

def rosbag_to_pcd(input_bag: str):
    """
    Convert entire rosbag file to numpy array representind point clouds (+ corresponding timestamps)
    """
    tmp_filename = decompress_zstd(input_bag)
    rows = []
    timestamps = []
    for _, msg, timestamp in read_messages(tmp_filename):
        timestamps.append(timestamp)
        rows.append(process_message(msg))
    arr = np.vstack(rows)
    timestamps = np.array(timestamps, dtype=np.int64)
    np.savez_compressed(input_bag.replace(".db3.zstd", ".npz"), ts = timestamps, pcd=arr)
    os.remove(tmp_filename)

def _get_sftp_client():
    """
    Returns an SFTP client connected to the remote server.
    """
    print("Connecting to SFTP server...")
    transport = paramiko.Transport((SFTP_HOST, SFTP_PORT))
    transport.connect(None, SFTP_USERNAME, SFTP_PASSWORD)
    sftp = paramiko.SFTPClient.from_transport(transport)
    print("Connected.")
    return sftp

def listdir_r(sftp, remotedir, res=None) -> List[str]:
    """
    Returns a list of full paths of rosbags & npz files in the remote SFTP server.
    We of course traverse recursively.
    """
    print(f"Listing directory: {remotedir}")
    if res is None:
        res = []
    # Based on: https://stackoverflow.com/questions/56671446/python-paramiko-directory-walk-over-sftp
    try:
        for entry in sftp.listdir_attr(remotedir):
            remotepath = remotedir + "/" + entry.filename
            print("\t", remotepath)
            mode = entry.st_mode
            if S_ISDIR(mode):
                listdir_r(sftp, remotepath, res)
            elif S_ISREG(mode):
                res.append(remotepath)
    except:
        pass

    return res

def get_unnpzed_files() -> List[str]:
    """
    Return a list of full paths of rosbags that are "unnzped" (i.e., do not have a corresponding npz file)    
    """
    sftp = _get_sftp_client()
    remote_files = listdir_r(sftp, ROOT_PATH)
    remote_files_set = set(remote_files)
    unnpzed_files = []
    for file in remote_files:
        if file.endswith(ROSBAG_EXTENSION):
            npz_file = file.replace(ROSBAG_EXTENSION, NPZ_EXTENSION)
            if npz_file not in remote_files_set:
                unnpzed_files.append(file)
    sftp.close()
    return unnpzed_files


def rosbag_to_pcd_sftp(remotepath: str):
    """
    Get a remote path in the SFTP server of a rosbag file.
    Download it to the current dir (with the same name), convert it into a pcd npz,
    and upload it back to the SFTP server.
    """
    sftp = _get_sftp_client()
    local_path = "./tmp/" + os.path.basename(remotepath)
    print(f"Downloading {remotepath} to {local_path}")
    sftp.get(remotepath, local_path)

    rosbag_to_pcd(local_path)

    npz_path = local_path.replace(ROSBAG_EXTENSION, NPZ_EXTENSION)
    print(f"Uploading {npz_path} to {remotepath.replace(ROSBAG_EXTENSION, NPZ_EXTENSION)}")
    sftp.put(npz_path, remotepath.replace(ROSBAG_EXTENSION, NPZ_EXTENSION))

    os.remove(local_path)
    os.remove(npz_path)
    sftp.close()


if __name__ == "__main__":
    while True:
        unnpzed_files = get_unnpzed_files()
        print("Unnpzed files:")
        for file in unnpzed_files:
            print(file)
        if not unnpzed_files:
            print("No unnpzed files found.")
        else:
            print(f"Total: {len(unnpzed_files)} unnpzed files found.")

        for remotepath in tqdm.tqdm(unnpzed_files, desc="Converting unnpzed files"):
            try:
                rosbag_to_pcd_sftp(remotepath)
            except Exception as e:
                print(f"Error processing {remotepath}: {e}")
                continue

        print("All done.")