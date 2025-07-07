#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Diretório do pacote
    pkg_dir = get_package_share_directory('caramelo_navigation')
    
    # Argumentos configuráveis
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join('/home/work/Caramelo_workspace', 'mapa_20250704_145039.yaml'),
        description='Caminho completo para o arquivo de mapa (.yaml)'
    )
    
    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value=os.path.join('/home/work/Caramelo_workspace/src/caramelo_navigation/config', 'waypoints.json'),
        description='Caminho completo para o arquivo de waypoints (.json)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (false for real robot)'
    )
    
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='',
        description='Namespace for robot topics'
    )
    
    # Incluir navegação com mapa (sem SLAM)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'map': LaunchConfiguration('map_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'namespace': LaunchConfiguration('robot_namespace')
        }.items()
    )
    
    # Navegador autônomo de waypoints
    waypoint_navigator = Node(
        package='caramelo_navigation',
        executable='autonomous_waypoint_navigator',
        name='autonomous_waypoint_navigator',
        namespace=LaunchConfiguration('robot_namespace'),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'waypoints_file': LaunchConfiguration('waypoints_file'),
            'map_file': LaunchConfiguration('map_file')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        map_file_arg,
        waypoints_file_arg,
        use_sim_time_arg,
        robot_namespace_arg,
        navigation_launch,
        waypoint_navigator
    ])
