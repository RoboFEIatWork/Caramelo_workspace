#!/usr/bin/env python3
"""
Launch FUNCIONAL para navegação Nav2 - VERSÃO SIMPLES QUE FUNCIONA

HARDWARE deve estar rodando em terminais separados:
1. ros2 launch caramelo_bringup encoder_bringup.launch.py  
2. ros2 launch caramelo_bringup pwm_bringup.launch.py

Este launch inicia:
- LiDAR RPLidar S2
- Map Server (mapa estático)
- AMCL (localização)  
- Nav2 Planning & Control
- RViz com configuração Nav2

Uso:
ros2 launch caramelo_navigation navigation_only.launch.py
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
    
    # Diretórios dos pacotes
    caramelo_bringup_dir = get_package_share_directory('caramelo_bringup')
    caramelo_navigation_dir = get_package_share_directory('caramelo_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Arquivos de configuração - PATHS ABSOLUTOS SIMPLES
    map_file = '/home/work/Caramelo_workspace/maps/arena_fei/map.yaml'
    nav2_params_file = os.path.join(caramelo_navigation_dir, 'config', 'nav2_static_map_only.yaml')
    rviz_config_file = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    # 1. LiDAR (RPLidar S2)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(caramelo_bringup_dir, 'launch', 'lidar_bringup.launch.py')
        ])
    )
    
    # 2. Nav2 Stack Completo - SINTAXE CORRETA
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ]),
        launch_arguments=[
            ('map', map_file),
            ('params_file', nav2_params_file),
            ('use_sim_time', 'false'),
            ('autostart', 'true'),
            ('use_lifecycle_mgr', 'true')
        ]
    )
    
    # 3. RViz para visualização
    rviz_node = TimerAction(
        period=8.0,  
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
    
    # 4. Node de navegação autônoma por waypoints
    waypoint_follower = TimerAction(
        period=15.0,  # Aguarda Nav2 inicializar
        actions=[
            Node(
                package='caramelo_navigation',
                executable='simple_waypoint_follower',
                name='simple_waypoint_follower',
                parameters=[{
                    'arena': 'arena_fei',
                    'auto_start': True,
                    'loop_mission': False
                }],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        lidar_launch,
        nav2_launch,
        rviz_node,
        waypoint_follower,
    ])
