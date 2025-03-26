from launch import LaunchDescription
from launch.actions import RegisterEventHandler, Shutdown
from launch.events.process import ProcessStarted
from launch.event_handlers.on_process_start import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():
    rplidar_ros = Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            exec_name='rplidar_node',
            parameters=[{'channel_type':"serial",
                        'serial_port': "/dev/ttyUSB0",
                        'serial_baudrate': 460800,
                        'frame_id': "laser",
                        'inverted': False,
                        'angle_compensate': True,
                        'scan_mode': "Standard"}],
            on_exit=Shutdown(),
            output='screen')
    
    red_led = Node(
        package='rplidar_recorder',
        executable='red_led',
        name='red_led',
        output='screen'
    )
    green_led_blinker = Node(
        package='rplidar_recorder',
        executable='led_blinker',
        name='led_blinker',
        output='screen'
    )
    frame_link = Node( # For displaying the laser scan in rviz
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "laser"],
    )
    bag = Node(
        package="rosbag",
        executable="record",
        arguments=["/scan", "-d", "10", "-o", "/home/ubuntu/rosbags", "compression-mode", "file", "compression-format", "zstd"]
    )
    

    return LaunchDescription([
        RegisterEventHandler(event_handler=OnProcessStart(target_action=red_led, on_start=rplidar_ros)),
        RegisterEventHandler(event_handler=OnProcessStart(target_action=rplidar_ros, on_start=green_led_blinker)),
        RegisterEventHandler(event_handler=OnProcessStart(target_action=rplidar_ros, on_start=bag)),
        red_led,
        frame_link,
    ])