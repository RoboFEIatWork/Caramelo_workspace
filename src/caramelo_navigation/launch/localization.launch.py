#!/usr/bin/env python3
"""
Launch para LOCALIZAÇÃO baseado nas melhores práticas do AntoBrandi
Adaptado para o robô Caramelo

Este launch inicia:
- map_server (mapa estático)
- amcl (localização)
- lifecycle_manager para localização
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    arena_arg = DeclareLaunchArgument(
        'arena',
        default_value='arena_fei',
        description='Nome da pasta da arena'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false"  # Robot real
    )

    arena = LaunchConfiguration('arena')
    use_sim_time = LaunchConfiguration("use_sim_time")
    lifecycle_nodes = ["map_server", "amcl"]
    caramelo_navigation_pkg = get_package_share_directory("caramelo_navigation")

    # Arquivos de configuração
    maps_base_path = '/home/work/Caramelo_workspace/maps'
    map_file = PathJoinSubstitution([maps_base_path, arena, 'map.yaml'])

    # Map Server
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_file},
            {"use_sim_time": use_sim_time}
        ],
    )
    
    # AMCL
    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            os.path.join(
                caramelo_navigation_pkg,
                "config",
                "amcl.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )

    # Lifecycle Manager para localização
    lifecycle_manager_localization = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )

    return LaunchDescription([
        arena_arg,
        use_sim_time_arg,
        map_server,
        amcl,
        lifecycle_manager_localization,
    ])
