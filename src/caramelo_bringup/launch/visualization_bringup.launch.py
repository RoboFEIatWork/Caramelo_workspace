#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch para inicializar apenas o RViz para visualização.
    
    Responsabilidades:
    - Inicializar RViz com configuração específica do Caramelo
    - Configuração para visualizar:
      - Robot model
      - Laser scan
      - Odometria
      - Mapas (se disponíveis)
      - TF tree
    """
    
    # Argumentos de launch
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar tempo de simulação')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('caramelo_bringup'),
            'config', 
            'caramelo_complete.rviz'
        ]),
        description='Arquivo de configuração do RViz para o Caramelo')
    
    # Nó do RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_rviz_config_file_cmd,
        rviz_node,
    ])
