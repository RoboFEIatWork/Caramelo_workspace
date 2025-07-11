#!/usr/bin/env python3
"""
LAUNCH MINIMAL NAV2 - SÓ O ESSENCIAL QUE FUNCIONA

HARDWARE:
1. Terminal 1: ros2 launch caramelo_bringup encoder_bringup.launch.py  
2. Terminal 2: ros2 launch caramelo_bringup pwm_bringup.launch.py
3. Terminal 3: ros2 launch caramelo_navigation simple_nav2.launch.py

FUNÇÃO: Carrega LiDAR + Nav2 + RViz

USO:
ros2 launch caramelo_navigation simple_nav2.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Pacotes
    caramelo_bringup_dir = get_package_share_directory('caramelo_bringup')
    caramelo_navigation_dir = get_package_share_directory('caramelo_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Arquivos
    map_file = '/home/work/Caramelo_workspace/maps/arena_fei/map.yaml'
    nav2_params_file = os.path.join(caramelo_navigation_dir, 'config', 'nav2_static_map_only.yaml')
    rviz_config_file = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    # ========================================
    # 1. LIDAR
    # ========================================
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(caramelo_bringup_dir, 'launch', 'lidar_bringup.launch.py')
        ])
    )
    
    # ========================================
    # 2. NAV2 COMPLETO  
    # ========================================
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ]),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_params_file,
            'use_sim_time': 'false',
            'autostart': 'true',
            'use_lifecycle_mgr': 'true'
        }.items()
    )
    
    # ========================================
    # 3. RVIZ 
    # ========================================
    rviz_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_file],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        lidar_launch,
        nav2_launch,
        rviz_node,
    ])
