import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_driver',
            namespace='',
            output='screen',

            parameters=[{
                'port': '/dev/ttyUSB0',
                'frame_id': 'laser_frame',

                # Driver configuration
                'ignore_array': ' ',
                'baudrate': 128000,
                'lidar_type': 1,
                'device_type': 0,
                'sample_rate': 9,
                'abnormal_check_count': 4,
                'resolution_fixed': True,
                'reversion': False,
                'auto_reconnect': True,

                # Angles (degrees)
                'angle_min': -180.0,
                'angle_max': 180.0,

                # Ranges (meters)
                'range_min': 0.08,
                'range_max': 12.0,

                # Rotation frequency (Hz)
                'frequency': 10.0,

                # Output mode
                'invalid_range_is_inf': False,
            }],

            # Ensure the output topic is always /scan
            remappings=[
                ('/scan', '/scan')
            ]
        )
    ])

