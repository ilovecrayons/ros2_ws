#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Downward camera - for linear regression
        Node(
            package='camera_ros',
            executable='camera_node',
            name='downward_camera',
            namespace='downwardCamera',
            parameters=[
                {'camera': '/base/axi/pcie@120000/rp1/i2c@88000/imx708@1a'},
                {'frame_id': 'downward_camera_frame'},
                {'fps': 30.0},
                {'width': 640},
                {'height': 480}
            ],
            output='screen'
        ),
        
        # Front-facing camera - for color segmentation
        Node(
            package='camera_ros',
            executable='camera_node',
            name='front_camera',
            namespace='frontCamera',
            parameters=[
                {'camera': '/base/axi/pcie@120000/rp1/i2c@80000/imx708@1a'},
                {'frame_id': 'front_camera_frame'},
                {'fps': 30.0},
                {'width': 640},
                {'height': 480}
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
