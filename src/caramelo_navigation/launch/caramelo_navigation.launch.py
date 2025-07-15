#!/usr/bin/env python3
"""
CARAMELO NAVIGATION - Launch principal para competição RoboCup@Work

FUNCIONALIDADES:
- Aceita parâmetro de arena (arena_fei, arena_competicao, etc)
- Carrega mapa automaticamente da arena especificada
- A* path planner para planejamento rápido
- DWB controller para navegação holonômica
- AMCL otimizado para correção rotacional
- Sistema de waypoints e workstations

HARDWARE NECESSÁRIO (rodar antes):
Terminal 1: ros2 launch caramelo_navigation caramelo_hardware.launch.py
Terminal 2: ros2 launch caramelo_navigation caramelo_nav2.launch.py arena:=arena_fei

USO:
ros2 launch caramelo_navigation caramelo_navigation.launch.py arena:=arena_fei
ros2 launch caramelo_navigation caramelo_navigation.launch.py arena:=arena_competicao
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    
    # Argumentos
    arena_arg = DeclareLaunchArgument(
        'arena',
        default_value='arena_fei',
        description='Nome da arena (pasta dentro de maps/). Ex: arena_fei, arena_competicao'
    )
    
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value='',
        description='Caminho completo para map.yaml (opcional - se não especificado, usa maps/[arena]/map.yaml)'
    )
    
    arena = LaunchConfiguration('arena')
    map_yaml = LaunchConfiguration('map_yaml')
    
    # Diretórios
    caramelo_navigation_dir = get_package_share_directory('caramelo_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Arquivos de configuração
    nav2_config_file = os.path.join(caramelo_navigation_dir, 'config', 'caramelo_nav2.yaml')
    rviz_config_file = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    # === NAVEGAÇÃO AUTÔNOMA POR WAYPOINTS ===
    waypoint_navigator_node = Node(
        package='caramelo_navigation',
        executable='caramelo_waypoint_nav.py',
        name='caramelo_waypoint_nav',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'arena': arena,  # ⭐ Passa o parâmetro de arena
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel_raw'),  # Para safety filter
        ]
    )
    
    # === RVIZ PARA NAVEGAÇÃO ===
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # === SAFETY FILTER ===
    safety_filter_node = Node(
        package='caramelo_navigation', 
        executable='cmd_vel_safety_filter.py',
        name='cmd_vel_safety_filter',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'max_linear_vel': 0.5,      # Velocidade máxima para competição
            'max_angular_vel': 1.0,
            'emergency_stop_distance': 0.10,  # ⭐ 10cm - próximo para WS
            'warning_distance': 0.25,          # ⭐ 25cm - aviso antecipado
        }]
    )
    
    # === CMD VEL MONITOR ===
    cmd_vel_monitor_node = Node(
        package='caramelo_navigation',
        executable='cmd_vel_monitor.py', 
        name='cmd_vel_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )
    
    return LaunchDescription([
        # Argumentos
        arena_arg,
        map_yaml_arg,
        
        # Nodes
        waypoint_navigator_node,
        rviz_node,
        safety_filter_node, 
        cmd_vel_monitor_node,
    ])
