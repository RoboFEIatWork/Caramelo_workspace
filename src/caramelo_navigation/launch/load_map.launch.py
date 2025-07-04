#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch para carregar um mapa salvo.
    
    Uso: ros2 launch caramelo_navigation load_map.launch.py map_file:=mapa_20250704_145039.yaml
    """
    
    # Argumentos
    map_file = LaunchConfiguration('map_file')
    
    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value='mapa_20250704_145039.yaml',
        description='Nome do arquivo de mapa YAML'
    )
    
    # Map Server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'use_sim_time': False
        }]
    )
    
    # Lifecycle Manager
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server']
        }]
    )
    
    return LaunchDescription([
        declare_map_file_cmd,
        map_server_node,
        lifecycle_manager_node
    ])
