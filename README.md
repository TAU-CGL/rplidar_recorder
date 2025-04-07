# rplidar-recorder
ROS2 package for continous recording of 2D LiDAR data, as well as an open source CAD model, easily 3D printed.
We describe the step-by-step process of construction the LiDAR recording device, which can be simply plugged into any power source,
and it starts recording two-dimensional point dlouc data.
We also provide post-processing code that transforms the `*.mcap.zstd` files recorded by the `rosbag` into machine-learning-friendly data.

![Contraption](https://raw.githubusercontent.com/TAU-CGL/rplidar_recorder/refs/heads/main/docs/recorder.png)

## Hardware

The device can be constructed with any Ubuntu micro-computer, and any two-dimensional LiDAR scanner.
However, for this repository (and the corresponding CAD models) we used the following:

* Raspberry Pi 4 Model B (2GB RAM)
* A 128 GB microSD card (While class performance can be varied, we used a V30 card)
* Any compatible USB C power adapter or power bank
* SLAMTEC RPLIDAR-C1 Laser Range Scanner
* HDMI cable + microHDMI to HDMI Adaptor
* Any HDMI compatible screen
* USB Keyboard and mouse
* PLA (we used black)
* 4x M3x?? screws
* 4x M2x?? screws
* 4x Raspberry pi standard mount screws
* 1x 3mm Red & 1X 3mm Green LEDs (see next section)
* 2x 220 Ohm resistors


### Preparing LEDs


## Installation

### Automatic Setup

TBD.

### Manual Setup

1. Download Ubuntu 22.04 server for Raspberry Pi, via the Raspberry Pi Imager [https://www.raspberrypi.com/software/](https://www.raspberrypi.com/software/), and flash it onto the microSD card.
2. Connect the RPi to the power, the screen and the keyboard.
3. Upon first login, the username and password are both `ubuntu`. You will be asked to change the password.
4. Optional: Increase font size. Run: `sudo nano /etc/default/console-setup` and change the value of `FONTSIZE` to `FONTSIZE=16x32`, then `sudo update-initramfs -u` and `sudo reboot`.
5. If you are not connected to the network by Ethernet, you need to setup a WiFi connection with netplan [https://dev.to/joeneville_/configure-ubuntu-wifi-with-netplan-4je0](https://dev.to/joeneville_/configure-ubuntu-wifi-with-netplan-4je0).
6. Run `sudo apt-get update` and `sudo apt-get upgrade`.
7. Optional: Install [ohmybash](https://github.com/ohmybash/oh-my-bash), change `di` color in `LS_COLORS` to `di:01;96:`:
    ```
    echo "export LS_COLORS=\"$LS_COLORS\"" >> ~/.bashrc
    ```
    Then edit the value of `di`. Change (in `.bashrc`) the ohmybash theme to `OSH_THEME="bakke"`

8. Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) with build tools, and add it the `.bashrc`: `echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc`.
9. Optional: Install a minimal GUI for running `rviz2`.
    * Run: `sudo apt-get install xorg openbox`
    * Run: `sudo apt-get install tint2 xterm kitty`
    * Run: `echo "exec openbox-session" >> ~/.xinitrc`
    * Run: `mkdir ~/.config/openbox && echo "tint2" >> ~/.config/openbox/autostart`
10. Create the ROS2 workspace:
    * Run: `cd ~ && mkdir ros_ws && mkdir ros_ws/src && cd ros_ws/src
    * Clone `rplidar_ros`: `git clone https://github.com/Slamtec/rplidar_ros.git`
    * Switch `rplidar_ros` to the `ros2` branch: `cd rplidar_ros && gco ros2 && cd ..`
    * Clone `rplidar_recorder`: `git clone https://github.com/TAU-CGL/rplidar_recorder.git`
    * Build the workspace: `cd ~/ros_ws && colcon build`
    * Source the workspace: `echo "source ~/ros_ws/install/setup.bash" >> ~/.bashrc`
10. Miscelenous steps:
    * Install `sudo apt-get install python3-gpiozero`
11. Run: `sudo crontab -e` and then add the line `@reboot /home/ubuntu/ros_ws/src/rplidar_recorder/chmod_ttyUSB.bash`
