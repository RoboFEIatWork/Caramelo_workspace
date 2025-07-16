#!/usr/bin/env python3
"""
ODOMETRY FUSION LAUNCH - Fusão Sensorial de Odometria

FUNÇÃO:
=======
Fusão inteligente entre ESP32 Encoders + ZED2i IMU para odometria robusta.

ENTRADAS:
=========
- /joint_states (do encoder_bringup.launch.py)
- /zed/zed_node/imu/data (do imu_zed_bringup.launch.py)

SAÍDAS:
=======
- /odom (nav_msgs/Odometry) - Odometria fundida
- /tf (odom -> base_footprint)

PRÉ-REQUISITOS:
==============
1. ros2 launch caramelo_bringup encoder_bringup.launch.py
2. ros2 launch caramelo_bringup imu_zed_bringup.launch.py
3. ros2 launch caramelo_bringup robot_state.launch.py

USO:
====
ros2 launch caramelo_bringup odometry_fusion.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch para fusão de odometria robusta
    """
    
    # Package directory
    caramelo_bringup_share = get_package_share_directory('caramelo_bringup')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Configuração
    robust_odometry_config = os.path.join(
        caramelo_bringup_share, 'config', 'robust_odometry_config.yaml'
    )
    
    # Nó de fusão sensorial robusta
    robust_odometry_node = Node(
        package='caramelo_bringup',
        executable='robust_odometry_node',
        name='robust_odometry',
        output='screen',
        parameters=[
            robust_odometry_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/joint_states', '/joint_states'),
            ('/zed/imu/data', '/zed/zed_node/imu/data'),
            ('/odom', '/odom'),
        ]
    )
    
    return LaunchDescription([
        # Arguments
        declare_use_sim_time_cmd,
        
        # Fusion node
        robust_odometry_node,
    ])
