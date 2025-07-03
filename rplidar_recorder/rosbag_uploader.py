import os
import pathlib

import rclpy
import requests
from rclpy.node import Node

class RosbagUploader(Node):
    def __init__(self):
        super().__init__('rosbag_uploader')

        self.SERVER_URL = os.environ["SERVER_URL"]
        self.ROSBAGS_FOLDER = "/home/ubuntu/rosbags"
        self.UUID_FILE = "/home/ubuntu/.contraption_uuid"
        self.REMOTE_ROOT = "/home/rplidar_recorder"
        with open(self.UUID_FILE, "r") as f:
            self.contraption_uuid = f.read().strip()
        
        # Run every hour
        self.timer = self.create_timer(10, self.upload_rosbag)
        self.get_logger().info("RosbagUploader node started.")
    
    def upload_rosbag(self):
        # This function should implement the logic to upload the rosbag file
        # For now, we will just log that the upload is happening
        self.get_logger().info(f"Uploading rosbag for contraption {self.contraption_uuid}...")

        remote_path = os.path.join(self.REMOTE_ROOT, self.contraption_uuid)
        for dirfile in os.listdir(self.ROSBAGS_FOLDER):
            local_path = os.path.join(self.ROSBAGS_FOLDER, dirfile)
            for filename in os.listdir(local_path):
                try:
                    if not filename.endswith('.zstd'):
                        continue
                    local_file_path = os.path.join(local_path, filename)
                    remote_file_path = os.path.join(remote_path, dirfile, filename)
                    self.get_logger().info(f"Uploading {local_file_path} to {remote_file_path}...")
                    
                    local_file = pathlib.Path(local_file_path)
                    with local_file.open("rb") as fp:
                        r = requests.post(
                            f"{self.SERVER_URL}/api/contraption/bag/upload",
                            files={"file": (local_file.name, fp, "application/octet-stream")},
                            data={"remote_path": remote_file_path},
                            timeout=(30, 900),
                        )
                        if r.status_code != 200:
                            raise Exception(r.text)
                    os.remove(local_file_path)

                except Exception as e:
                    self.get_logger().error(f"Failed to upload {local_file_path} to {remote_file_path}: {e}")
                    continue
    
        self.get_logger().info(f"Rosbag uploaded successfully for contraption {self.contraption_uuid}.")

def main(args=None):
    rclpy.init(args=args)
    node = RosbagUploader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()