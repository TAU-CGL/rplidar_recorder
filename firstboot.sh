#!/bin/bash
sudo apt-get update -y
sudo apt-get update -y
sudo apt-get upgrade -y
sudo apt-get update -y
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update -y
sudo apt upgrade -y
sudo apt install ros-humble-ros-base
sudo apt install ros-dev-tools
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
cd ~ && mkdir ros_ws && mkdir ros_ws/src && cd ros_ws/src
git clone https://github.com/Slamtec/rplidar_ros.git
cd rplidar_ros && gco ros2 && cd ..
git clone https://github.com/TAU-CGL/rplidar_recorder.git
cd ~/ros_ws && colcon build
echo "source ~/ros_ws/install/setup.bash" >> ~/.bashrc
sudo apt-get install python3-gpiozero -y
sudo chmod a+x /home/ubuntu/ros_ws/src/rplidar_recorder/scripts/launch.bash
line=@reboot /home/ubuntu/ros_ws/src/rplidar_recorder/scripts/launch.bash
(sudo crontab -l; echo "$line") | sudo crontab -