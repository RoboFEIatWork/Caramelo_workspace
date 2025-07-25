#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch para inicializar apenas o RViz para visualização.
    
    Responsabilidades:
    - Inicializar RViz com configuração específica do Caramelo
    - Configuração para visualizar:
    - Robot model
    - Laser scan
    - Odometria
    - TF tree
    """
    
    # Argumentos de launch
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar tempo de simulação')
    
    # Nó do RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('caramelo_bringup'), 'rviz', 'caramelo_basic_visualization.rviz')],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        rviz_node,
    ])
