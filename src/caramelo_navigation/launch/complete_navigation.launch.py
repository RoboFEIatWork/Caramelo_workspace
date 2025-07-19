#!/usr/bin/env python3
"""
CARAMELO COMPLETE NAVIGATION LAUNCH
==================================

Launch completo que inicia TUDO: hardware + navegação + servidor de workstations.
Use este quando quiser iniciar o sistema completo de uma vez.

Este launch inicia:
1. Hardware: LIDAR, Encoders, PWM
2. Nav2 stack completo 
3. Workstation Navigation Server
4. RViz para visualização

Uso:
    ros2 launch caramelo_navigation complete_navigation.launch.py arena:=arena_fei

Para testar navegação:
    ros2 run caramelo_navigation navigation_test_client WS01 --wait

Autor: GitHub Copilot
Data: 2025-01-18
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # === ARGUMENTOS ===
    use_sim_time = LaunchConfiguration('use_sim_time')
    arena = LaunchConfiguration('arena')
    
    # Caminhos dos arquivos
    pkg_nav = get_package_share_directory('caramelo_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # === DECLARAÇÃO DE ARGUMENTOS ===
    declare_arena_cmd = DeclareLaunchArgument(
        'arena',
        default_value='arena_fei',
        description='Nome da arena (pasta em /maps/)'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # === CAMINHOS DINÂMICOS ===
    map_yaml_file = PathJoinSubstitution([
        '/home/work/Caramelo_workspace/maps',
        arena,
        'map.yaml'
    ])
    
    nav2_config_file = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    rviz_config_file = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    # === HARDWARE LAUNCHES ===
    
    # 1. LIDAR Launch
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('caramelo_bringup'),
                'launch',
                'lidar_bringup.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # 2. Odometry Launch (Encoders)
    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('caramelo_bringup'),
                'launch',
                'odometry_bringup.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # 3. PWM/Actuators Launch
    actuators_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('caramelo_bringup'),
                'launch',
                'actuators_bringup.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # === NAVEGAÇÃO NAV2 ===
    nav2_launch = TimerAction(
        period=5.0,  # Aguardar hardware inicializar
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('nav2_bringup'),
                        'launch',
                        'bringup_launch.py'
                    ])
                ]),
                launch_arguments={
                    'map': map_yaml_file,
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_config_file,
                    'autostart': 'true',
                }.items()
            )
        ]
    )
    
    # === WORKSTATION NAVIGATION SERVER ===
    workstation_nav_server = TimerAction(
        period=15.0,  # Aguardar hardware + Nav2 inicializarem
        actions=[
            Node(
                package='caramelo_navigation',
                executable='workstation_navigation_server',
                name='workstation_navigation_server',
                output='screen',
                parameters=[{
                    'arena': arena,
                    'use_sim_time': use_sim_time,
                }]
            )
        ]
    )
    
    # === VISUALIZAÇÃO ===
    rviz_cmd = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_file],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    # === LOGS INFORMATIVOS ===
    start_log = LogInfo(
        msg=[
            '🚀 CARAMELO COMPLETE NAVIGATION SYSTEM',
            '\\n   Arena: ', arena,
            '\\n   Map: ', map_yaml_file,
            '\\n   Config: ', nav2_config_file,
            '\\n\\n🔧 HARDWARE INCLUÍDO:',
            '\\n   ✓ LIDAR (RPLidar S2)',
            '\\n   ✓ Encoders (Odometry)',
            '\\n   ✓ PWM (Actuators)',
            '\\n\\n📋 SERVIÇOS DISPONÍVEIS:',
            '\\n   /navigate_to_workstation - Navegação por workstation',
            '\\n\\n🧪 PARA TESTAR:',
            '\\n   ros2 run caramelo_navigation navigation_test_client WS01 --wait',
            '\\n\\n⏱️  INICIALIZAÇÃO ESCALONADA:',
            '\\n   0s: Hardware',
            '\\n   5s: Nav2',
            '\\n   10s: RViz',
            '\\n   15s: Workstation Server'
        ]
    )
    
    return LaunchDescription([
        # Argumentos
        declare_arena_cmd,
        declare_use_sim_time_cmd,
        
        # Logs
        start_log,
        
        # Hardware (imediato)
        lidar_launch,
        odometry_launch,
        actuators_launch,
        
        # Navegação (após hardware)
        nav2_launch,
        workstation_nav_server,
        
        # Visualização
        rviz_cmd,
    ])
