import os

import rclpy
import paramiko
from rclpy.node import Node

class RosbagUploader(Node):
    def __init__(self):
        super().__init__('rosbag_uploader')

        self.ROSBAGS_FOLDER = "/home/ubuntu/rosbags"
        self.UUID_FILE = "/home/ubuntu/.contraption_uuid"
        self.REMOTE_ROOT = "/home/rplidar_recorder"
        with open(self.UUID_FILE, "r") as f:
            self.contraption_uuid = f.read().strip()
        
        # Run every 5 minutes
        self.timer = self.create_timer(10, self.upload_rosbag)
        self.get_logger().info("RosbagUploader node started.")
    
    def upload_rosbag(self):
        # This function should implement the logic to upload the rosbag file
        # For now, we will just log that the upload is happening
        self.get_logger().info(f"Uploading rosbag for contraption {self.contraption_uuid}...")

        host, port = os.environ["SFTP_HOST"], 23
        transport = paramiko.Transport((host, port))
        username, password = os.environ["SFTP_USERNAME"], os.environ["SFTP_PASSWORD"]
        transport.connect(None, username, password)
        sftp = paramiko.SFTPClient.from_transport(transport)

        remote_path = os.path.join(self.REMOTE_ROOT, self.contraption_uuid)
        if self.contraption_uuid not in sftp.listdir(self.REMOTE_ROOT):
            self.get_logger().info(f"Creating remote directory {remote_path}...")
            sftp.mkdir(remote_path)
        for dirfile in os.listdir(self.ROSBAGS_FOLDER):
            local_path = os.path.join(self.ROSBAGS_FOLDER, dirfile)
            for filename in os.listdir(local_path):
                if not filename.endswith('.zstd'):
                    continue
                local_file_path = os.path.join(local_path, filename)
                remote_file_path = os.path.join(remote_path, dirfile, filename)
                if dirfile not in sftp.listdir(remote_path):
                    self.get_logger().info(f"Creating remote directory {os.path.join(remote_path, dirfile)}...")
                    sftp.mkdir(os.path.join(remote_path, dirfile))
                self.get_logger().info(f"Uploading {local_file_path} to {remote_file_path}...")
                sftp.put(local_file_path, remote_file_path)
                os.remove(local_file_path)  # Remove the local file after upload
        
        sftp.close()
        transport.close()
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