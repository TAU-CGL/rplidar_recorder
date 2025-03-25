from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.events.process import ProcessStarted
from launch.event_handlers.on_process_start import OnProcessStart
from launch_ros.actions import Node, Shutdown

from gpiozero import LED
LED_PIN = 16 # Red

def generate_launch_description():
    LED(LED_PIN).on()

    rplidar_ros = Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            exec_name='rplidar_node',
            parameters=[{'channel_type':"serial",
                        'serial_port': "/dev/ttyUSB0",
                        'serial_baudrate': "460800",
                        'frame_id': "laser",
                        'inverted': "false",
                        'angle_compensate': "true",
                        'scan_mode': "Standard"}],
            on_exit=Shutdown(),
            output='screen')
    led_blinker = Node(
            package='rplidar_recorder',
            executable='led_blinker',
            name='led_blinker',
            output='screen'
        )
    

    return LaunchDescription([
        RegisterEventHandler(event_handler=OnProcessStart(target_action=rplidar_ros, on_start=led_blinker)),
        rplidar_ros,
    ])