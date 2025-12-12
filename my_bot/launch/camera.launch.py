import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='webcam',
            namespace='',
            output='screen',

            parameters=[{
                'image_size': [640, 480],
                'camera_frame_id': 'camera_link_optical',
                'video_device': '/dev/video0'
            }],

            # Recommended remappings for ROS2 compatibility
            remappings=[
                ('/image_raw', '/camera/image_raw'),
                ('/camera_info', '/camera/camera_info')
            ]
        )
    ])
