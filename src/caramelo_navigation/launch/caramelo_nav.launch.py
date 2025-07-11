#!/usr/bin/env python3
"""
NAVEGAÇÃO AUTÔNOMA CARAMELO - SISTEMA REAL FUNCIONAL

HARDWARE deve estar rodando antes:
1. Terminal 1: ros2 launch caramelo_bringup pwm_bringup.launch.py
2. Terminal 2: ros2 launch caramelo_bringup encoder_bringup.launch.py

Este launch:
- Inicia LiDAR RPLidar S2 
- Carrega mapa estático
- Inicia AMCL para localização
- Inicia Nav2 stack completo
- Inicia RViz
- Inicia navegação por waypoints

Uso:
ros2 launch caramelo_navigation caramelo_nav.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    
    # Diretórios
    caramelo_bringup_dir = get_package_share_directory('caramelo_bringup')
    caramelo_navigation_dir = get_package_share_directory('caramelo_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Arquivos
    map_yaml = '/home/work/Caramelo_workspace/maps/arena_fei/map.yaml'
    nav2_params = os.path.join(caramelo_navigation_dir, 'config', 'caramelo_nav2.yaml')
    rviz_config = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    # 1. LiDAR
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(caramelo_bringup_dir, 'launch', 'lidar_bringup.launch.py')
        )
    )
    
    # 2. Nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments=[
            ('map', map_yaml),
            ('params_file', nav2_params),
            ('use_sim_time', 'false'),
            ('autostart', 'true')
        ]
    )
    
    # 3. RViz (com delay)
    rviz = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config],
                output='screen'
            )
        ]
    )
    
    # 4. Twist Converter (CRÍTICO para movimento!)
    # Converte /cmd_vel (do Nav2) para /mecanum_drive_controller/cmd_vel
    twist_converter = Node(
        package='caramelo_bringup',
        executable='twist_converter_node',
        name='twist_converter',
        output='screen'
    )
    
    # 5. Navegação autônoma (com delay maior)
    waypoint_nav = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='caramelo_navigation',
                executable='caramelo_waypoint_nav',
                output='screen',
                parameters=[{
                    'mission_file': '/home/work/Caramelo_workspace/maps/arena_fei/mission.yaml',
                    'auto_start': True
                }]
            )
        ]
    )

    return LaunchDescription([
        lidar,
        nav2,
        rviz,
        twist_converter,  # ADICIONADO!
        waypoint_nav
    ])
