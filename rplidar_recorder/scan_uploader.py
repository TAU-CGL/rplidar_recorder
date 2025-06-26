    """
    ROS node that uploads /scan topics once every minute to the endpoint "URL/api/contraption/scan"
    """
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoS
from sensor_msgs.msg import LaserScan
import requests
import json
from datetime import datetime


class ScanUploader(Node):
    def __init__(self):
        super().__init__('scan_uploader')

        self.SERVER_URL = os.environ["SERVER_URL"]
        self.UUID_FILE = "/home/ubuntu/.contraption_uuid"
        with open(self.UUID_FILE, "w") as f:
            self.contraption_uuid = f.read()

        qos_profile = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )
        self.get_logger().info("ScanUploader node started.")
        self.last_sent = datetime.min()

    # The following code sends *ALL* messages, we need to send only one message per minute.
    def scan_callback(self, msg: LaserScan):
        now = datetime.now()
        if (now - self.last_sent).total_seconds() < 60:
            return
        timestamp = datetime.now().isoformat()
        data = {
            'contraption_uuid': self.contraption_uuid,
            'timestamp': timestamp,
            'ranges': {
                'min_range': msg.range_min,
                'max_range': msg.range_max,
                'ranges': list(msg.ranges),
                'intensities': list(msg.intensities) if msg.intensities else []
            }
        }
        response = requests.post(f'{self.SERVER_URL}/api/contraption/scan', json=data)
        if response.status_code == 200:
            self.get_logger().info(f"Scan uploaded successfully at {timestamp}.")
        else:
            self.get_logger().error(f"Failed to upload scan: {response.text}")

def main(args=None):
    rclpy.init(args=args)
    scan_uploader = ScanUploader()
    try:
        rclpy.spin(scan_uploader)
    except KeyboardInterrupt:
        pass
    finally:
        scan_uploader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()