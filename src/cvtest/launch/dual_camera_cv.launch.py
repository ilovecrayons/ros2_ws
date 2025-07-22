#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Downward camera (camera 0) - for linear regression
        Node(
            package='camera_ros',
            executable='camera_node',
            name='downward_camera',
            parameters=[
                {'camera_id': 0},
                {'frame_id': 'downward_camera_frame'},
                {'fps': 30.0},
                {'width': 640},
                {'height': 480}
            ],
            remappings=[
                ('image_raw', '/camera/downward/image_raw'),
                ('camera_info', '/camera/downward/camera_info')
            ],
            output='screen'
        ),
        
        # Front-facing camera (camera 1) - for color segmentation
        Node(
            package='camera_ros',
            executable='camera_node',
            name='front_camera',
            parameters=[
                {'camera_id': 1},
                {'frame_id': 'front_camera_frame'},
                {'fps': 30.0},
                {'width': 640},
                {'height': 480}
            ],
            remappings=[
                ('image_raw', '/camera/front/image_raw'),
                ('camera_info', '/camera/front/camera_info')
            ],
            output='screen'
        ),
        
        # CV processing node
        Node(
            package='cvtest',
            executable='cv_node',
            name='cv_processor',
            output='screen'
        )
    ])
