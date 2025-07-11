#!/usr/bin/env python3
"""
CARAMELO - SISTEMA COMPLETO AUTÔNOMO
=====================================

FAIL-PROOF: Inicializa TODO o sistema em um único comando!

Hardware + Navegação + RViz + Waypoints

Após pkill ros2, rode apenas:
ros2 launch caramelo_navigation caramelo_complete.launch.py

O que este launch faz:
1. Hardware: PWM + Encoder (obrigatório para movimento)
2. LiDAR: RPLidar S2 para localização AMCL  
3. Twist Converter: /cmd_vel -> /mecanum_drive_controller/cmd_vel
4. Nav2: Mapa estático + AMCL + Planning + Control
5. RViz: Visualização
6. Navegação autônoma: Waypoints automáticos

SEQUÊNCIA DE INICIALIZAÇÃO:
- Hardware: 0s 
- LiDAR: 2s
- Twist Converter: 4s
- Nav2: 6s
- RViz: 12s  
- Navegação: 20s (permite AMCL convergir)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    
    # Pacotes
    caramelo_bringup_dir = get_package_share_directory('caramelo_bringup')
    caramelo_navigation_dir = get_package_share_directory('caramelo_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Configurações
    map_yaml = '/home/work/Caramelo_workspace/maps/arena_fei/map.yaml'
    nav2_params = os.path.join(caramelo_navigation_dir, 'config', 'caramelo_nav2.yaml')
    rviz_config = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    mission_file = '/home/work/Caramelo_workspace/maps/arena_fei/mission.yaml'
    
    # ========== HARDWARE ==========
    # PWM (motores)
    pwm_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(caramelo_bringup_dir, 'launch', 'pwm_bringup.launch.py')
        )
    )
    
    # Encoder (odometria)
    encoder_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(caramelo_bringup_dir, 'launch', 'encoder_bringup.launch.py')
        )
    )
    
    # ========== SENSORES ==========
    # LiDAR (delay 2s para hardware estabilizar)
    lidar = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(caramelo_bringup_dir, 'launch', 'lidar_bringup.launch.py')
                )
            )
        ]
    )
    
    # ========== CONVERSORES ==========
    # Twist Converter (delay 4s - CRÍTICO!)
    # Converte /cmd_vel do Nav2 para /mecanum_drive_controller/cmd_vel
    twist_converter = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='caramelo_bringup',
                executable='twist_converter_node',
                name='twist_converter',
                output='screen'
            )
        ]
    )
    
    # ========== NAVEGAÇÃO ==========
    # Nav2 Stack (delay 6s para hardware + sensores prontos)
    nav2 = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
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
        ]
    )
    
    # ========== VISUALIZAÇÃO ==========
    # RViz (delay 12s para Nav2 estar ativo)
    rviz = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config],
                output='screen'
            )
        ]
    )
    
    # ========== AUTONOMIA ==========
    # Navegação por waypoints (delay 20s para AMCL convergir)
    waypoint_navigation = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='caramelo_navigation',
                executable='caramelo_waypoint_nav',
                name='caramelo_waypoint_navigator',
                output='screen',
                parameters=[{
                    'mission_file': mission_file,
                    'auto_start': True,
                    'publish_initial_pose': True
                }]
            )
        ]
    )

    return LaunchDescription([
        # Hardware (imediato)
        pwm_hardware,
        encoder_hardware,
        
        # Sensores (2s)
        lidar,
        
        # Conversores (4s) 
        twist_converter,
        
        # Navegação (6s)
        nav2,
        
        # Visualização (12s)
        rviz,
        
        # Autonomia (20s)
        waypoint_navigation
    ])
