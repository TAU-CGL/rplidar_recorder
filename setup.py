from setuptools import find_packages, setup

package_name = 'rplidar_recorder'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mickelbil84',
    maintainer_email='mickelbil84@gmail.com',
    description='ROS2 package for continous recording of 2D LiDAR data',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "led_blinker = rplidar_recorder.led_blinker:main",
        ],
    },
)
