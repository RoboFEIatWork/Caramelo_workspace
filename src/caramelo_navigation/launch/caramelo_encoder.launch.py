#!/usr/bin/env python3
"""
CARAMELO ENCODER LAUNCH
Launch apenas para encoder readings e joint states
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Encoder node
    encoder_node = Node(
        package='caramelo_controller',
        executable='encoder_node',
        name='encoder_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'publish_rate': 50.0,
        }]
    )
    
    # Joint state publisher para encoder
    joint_state_node = Node(
        package='caramelo_controller',
        executable='encoder_joint_state_node', 
        name='encoder_joint_state_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )
    
    return LaunchDescription([
        encoder_node,
        joint_state_node,
    ])
