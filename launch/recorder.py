from launch import LaunchDescription
from launch_ros.actions import Node

from gpiozero import LED
LED_PIN = 12

def generate_launch_description():
    LED(LED_PIN).on()
    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{'channel_type':"serial",
                        'serial_port': "/dev/ttyUSB0",
                        'serial_baudrate': "460800",
                        'frame_id': "laser",
                        'inverted': "false",
                        'angle_compensate': "true",
                        'scan_mode': "Standard"}],
            output='screen'),
        Node(
            package='rplidar_recorder',
            executable='led_blinker',
            name='led_blinker',
            output='screen'
        ),
    ])