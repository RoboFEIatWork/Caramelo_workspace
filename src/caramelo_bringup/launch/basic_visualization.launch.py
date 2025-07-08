#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch para visualização básica sem mapa - apenas LIDAR e robot model.
    
    Responsabilidades:
    - Inicializar RViz com Fixed Frame = odom
    - Visualizar robot model, laser scan, TF
    - Sem componentes de navegação/mapa
    """
    
    # Argumentos de launch
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar tempo de simulação')
    
    # Nó do RViz - Config básico
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('caramelo_bringup'),
            'rviz', 
            'caramelo_basic.rviz'
        ])],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        rviz_node,
    ])
