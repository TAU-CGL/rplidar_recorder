#!/bin/bash

# On first boot - make sure the contraption is registered in the system
# and save its name to ~/.contraption_uuid
export SERVER_URL="<server_url>"
python3 /root/ros_ws/src/rplidar_recorder/scripts/init_name.py

# Setup USB permissions for the RPLIDAR device
sudo chmod 777 /dev/ttyUSB0

# Source (prepare) the ROS2 environment - global humble install and local ros workspace
source /opt/ros/humble/setup.bash
source /root/ros_ws/install/setup.bash

# Start the ROS2 nodes (single launch file, many nodes)
ros2 launch /root/ros_ws/src/rplidar_recorder/launch/recorder.py &