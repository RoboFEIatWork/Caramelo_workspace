#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch para carregar mapa com publicação contínua.
    """
    
    # Caminho absoluto para o mapa
    map_file_path = '/home/work/Caramelo_workspace/mapa_20250704_145039.yaml'
    
    # Map Server com publicação contínua
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file_path,
            'use_sim_time': False,
            'topic_name': 'map',
            'frame_id': 'map'
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
    
    # Republisher para publicação contínua
    republisher_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_republisher',
        output='screen',
        parameters=[{
            'yaml_filename': map_file_path,
            'use_sim_time': False
        }],
        remappings=[
            ('/map', '/map_continuous')
        ]
    )
    
    return LaunchDescription([
        map_server_node,
        lifecycle_manager_node
    ])
