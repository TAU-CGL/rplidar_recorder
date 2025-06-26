import datetime

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, Shutdown, ExecuteProcess
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
    scan_uploader = Node(
        package='rplidar_recorder',
        executable='scan_uploader',
        name='scan_uploader',
        output='screen'
    )
    rosbag_uploader = Node(
        package='rplidar_recorder',
        executable='rosbag_uploader',
        name='rosbag_uploader',
        output='screen'
    )


    today = datetime.datetime.now()
    bag_name = today.strftime("%Y-%m-%d-%H-%M-%S")
    bag = ExecuteProcess(
        cmd=["ros2", "bag", "record", "/scan", "-d", "300", "-o", f"/home/ubuntu/rosbags/{bag_name}/", "--compression-mode", "file", "--compression-format", "zstd"],
        output="screen",
    )
    

    return LaunchDescription([
        RegisterEventHandler(event_handler=OnProcessStart(target_action=rplidar_ros, on_start=green_led_blinker)),
        RegisterEventHandler(event_handler=OnProcessStart(target_action=rplidar_ros, on_start=bag)),
        RegisterEventHandler(event_handler=OnProcessStart(target_action=rplidar_ros, on_start=scan_uploader)),
        RegisterEventHandler(event_handler=OnProcessStart(target_action=rplidar_ros, on_start=rosbag_uploader)),
        rplidar_ros,
        frame_link,
    ])