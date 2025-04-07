#!/bin/bash
sudo chmod 777 /dev/ttyUSB0
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros_ws/install/setup.bash
ros2 launch rplidar_recorder recorder.py