#!/usr/bin/env python3
"""
Launch TESTE apenas Nav2 (sem LiDAR) para debugar
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    
    # Argumentos
    arena_arg = DeclareLaunchArgument(
        'arena',
        default_value='arena_fei',
        description='Nome da pasta da arena'
    )
    
    arena = LaunchConfiguration('arena')
    
    # Diretórios dos pacotes
    caramelo_navigation_dir = get_package_share_directory('caramelo_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Arquivos de configuração
    maps_base_path = '/home/work/Caramelo_workspace/maps'
    map_file = PathJoinSubstitution([maps_base_path, arena, 'map.yaml'])
    nav2_params_file = os.path.join(caramelo_navigation_dir, 'config', 'nav2_static_map_only.yaml')
    
    # Nav2 Stack Completo
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

    return LaunchDescription([
        arena_arg,
        nav2_launch,
    ])
