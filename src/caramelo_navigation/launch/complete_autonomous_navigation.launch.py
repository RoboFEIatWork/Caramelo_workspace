#!/usr/bin/env python3
"""
Launch unificado para navegação autônoma completa do robô Caramelo.

Inicia TUDO necessário em um único comando:
- Hardware (encoders + PWM + LiDAR)  
- Nav2 stack completo
- Navegação automática por waypoints

Uso:
ros2 launch caramelo_navigation complete_autonomous_navigation.launch.py arena:=arena_fei
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # ========================================
    # ARGUMENTOS DE LAUNCH
    # ========================================
    
    arena_arg = DeclareLaunchArgument(
        'arena',
        default_value='arena_fei',
        description='Nome da pasta da arena (ex: arena_fei, ambiente_escritorio)'
    )
    
    loop_arg = DeclareLaunchArgument(
        'loop_mission',
        default_value='false',
        description='Se true, repete a missão infinitamente'
    )
    
    # ========================================
    # PATHS E CONFIGURAÇÕES
    # ========================================
    
    arena = LaunchConfiguration('arena')
    loop_mission = LaunchConfiguration('loop_mission')
    
    # Paths importantes
    bringup_share = FindPackageShare('caramelo_bringup')
    nav_share = FindPackageShare('caramelo_navigation')
    
    # Arquivos de configuração
    nav2_params_file = PathJoinSubstitution([
        nav_share, 'config', 'nav2_static_map_only.yaml'
    ])
    
    rviz_config_file = PathJoinSubstitution([
        nav_share, 'rviz', 'nav2_default_view.rviz'
    ])
    
    # Mapa dinâmico baseado na arena
    maps_base = '/home/work/Caramelo_workspace/maps'
    map_file = [maps_base, '/', arena, '/map.yaml']
    
    # ========================================
    # 1. HARDWARE DO ROBÔ
    # ========================================
    
    # Encoder + Odometria
    encoder_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bringup_share, 'launch', 'encoder_bringup.launch.py'])
        ])
    )
    
    # PWM + Controle dos motores
    pwm_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bringup_share, 'launch', 'pwm_bringup.launch.py'])
        ])
    )
    
    # LiDAR
    lidar_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bringup_share, 'launch', 'lidar_bringup.launch.py'])
        ])
    )
    
    # ========================================
    # 2. NAV2 STACK COMPLETO
    # ========================================
    
    # Nav2 com mapa estático
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_params_file,
            'use_sim_time': 'false',
            'autostart': 'true'
        }.items()
    )
    
    # ========================================
    # 3. NAVEGAÇÃO POR WAYPOINTS
    # ========================================
    
    # Navegador simples por waypoints (com delay para Nav2 inicializar)
    waypoint_navigator = TimerAction(
        period=8.0,  # Aguarda 8s para Nav2 estar pronto
        actions=[
            Node(
                package='caramelo_navigation',
                executable='simple_waypoint_follower',
                name='simple_waypoint_follower',
                parameters=[{
                    'arena': arena,
                    'loop_mission': loop_mission,
                    'auto_start': True
                }],
                output='screen'
            )
        ]
    )
    
    # ========================================
    # 4. RVIZ (OPCIONAL)
    # ========================================
    
    rviz_node = TimerAction(
        period=5.0,  # Aguarda 5s para não conflitar com Nav2
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_file],
                parameters=[{'use_sim_time': False}],
                output='screen'
            )
        ]
    )
    
    # ========================================
    # AGRUPAMENTO FINAL
    # ========================================
    
    # Hardware em grupo (inicia primeiro)
    hardware_group = GroupAction([
        encoder_bringup,
        pwm_bringup,
        lidar_bringup
    ])
    
    # Navegação em grupo (inicia após hardware)
    navigation_group = TimerAction(
        period=3.0,  # Aguarda hardware estar pronto
        actions=[
            nav2_bringup,
            waypoint_navigator
        ]
    )
    
    return LaunchDescription([
        # Argumentos
        arena_arg,
        loop_arg,
        
        # Grupos de inicialização
        hardware_group,
        navigation_group,
        rviz_node,
    ])
