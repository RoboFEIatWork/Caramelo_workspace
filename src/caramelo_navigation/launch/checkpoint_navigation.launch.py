#!/usr/bin/env python3
"""
Launch file para o sistema de navegação por checkpoints do robô Caramelo
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Gera a descrição do launch"""
    
    # Parâmetros de launch
    package_dir = get_package_share_directory('caramelo_navigation')
    config_dir = os.path.join(package_dir, 'config')
    
    # Argumentos de launch
    workstations_file_arg = DeclareLaunchArgument(
        'workstations_file',
        default_value=os.path.join(config_dir, 'workstations.yaml'),
        description='Arquivo YAML com configurações das workstations'
    )
    
    checkpoints_file_arg = DeclareLaunchArgument(
        'checkpoints_file',
        default_value=os.path.join(config_dir, 'checkpoints.json'),
        description='Arquivo JSON com checkpoints para navegação'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Frame de referência para navegação'
    )
    
    # Nó do navegador de checkpoints
    checkpoint_navigator = Node(
        package='caramelo_navigation',
        executable='checkpoint_navigator',
        name='checkpoint_navigator',
        parameters=[{
            'workstations_file': LaunchConfiguration('workstations_file'),
            'checkpoints_file': LaunchConfiguration('checkpoints_file'),
            'frame_id': LaunchConfiguration('frame_id')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        workstations_file_arg,
        checkpoints_file_arg,
        frame_id_arg,
        checkpoint_navigator
    ])

if __name__ == '__main__':
    generate_launch_description()
