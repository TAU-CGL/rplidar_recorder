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

