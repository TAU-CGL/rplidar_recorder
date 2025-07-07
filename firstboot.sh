#!/bin/bash



##################################################
# Initial boot - make sure apt is correctly updated
# For the Ubuntu image for respberry pi, 
# you sometimes may need to run update twice.
##################################################
sudo apt-get update
sudo apt-get update

#######################
# Install ROS2 Humble
#######################
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-ros-base -y
sudo apt install ros-dev-tools -y

##############################################
# Install RPLIDAR ROS2 & recorder packages
# If this script is run with sudo, this will
# install everything in `/root/ros_ws`
##############################################
source /opt/ros/humble/setup.bash
cd ~ && mkdir ros_ws && mkdir ros_ws/src && cd ros_ws/src
git clone https://github.com/Slamtec/rplidar_ros.git
cd rplidar_ros && git checkout ros2 && cd ..
git clone https://github.com/TAU-CGL/rplidar_recorder.git
cd ~/ros_ws && colcon build

################################
# Install misc dependencies
################################
sudo apt-get install python3-gpiozero -y

################################
# Add boot script to crontab
################################
sudo chmod a+x /root/ros_ws/src/rplidar_recorder/scripts/launch.bash
line=@reboot /root/ros_ws/src/rplidar_recorder/scripts/launch.bash
(sudo crontab -l; echo "$line") | sudo crontab -