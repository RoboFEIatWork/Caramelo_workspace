#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file principal para o sistema completo do robô Caramelo
    para RoboCup@Work.
    
    EXECUTE EM TERMINAIS SEPARADOS:
    1. Terminal 1: ros2 launch caramelo_bringup encoder_bringup.launch.py
    2. Terminal 2: ros2 launch caramelo_bringup pwm_bringup.launch.py
    3. Terminal 3: ros2 launch caramelo_tasks system.launch.py
    
    Este launch inclui:
    - Nav2 completo (navegação autônoma)
    - Task Executor (execução de tarefas YAML)
    - RViz para visualização
    """
    
    # Configurações
    map_file = LaunchConfiguration('map_file')
    task_file = LaunchConfiguration('task_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Argumentos
    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(get_package_share_directory('caramelo_navigation'),
                                   'maps', 'map.yaml'),
        description='Caminho para o arquivo de mapa')
        
    declare_task_file_cmd = DeclareLaunchArgument(
        'task_file',
        default_value=os.path.join(get_package_share_directory('caramelo_tasks'),
                                   'config', 'tasks.yaml'),
        description='Caminho para o arquivo de tarefas')
        
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar tempo de simulação')

    # 1. Nav2 (navegação autônoma)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('caramelo_navigation'),
                         'launch', 'navigation_launch.py')),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time
        }.items()
    )

    # 2. Task Executor
    task_executor_node = Node(
        package='caramelo_tasks',
        executable='task_executor',
        name='task_executor',
        output='screen',
        parameters=[
            {'task_file': task_file},
            {'use_sim_time': use_sim_time}
        ]
    )

    # 3. RViz para visualização
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('caramelo_navigation'),
                                      'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_map_file_cmd,
        declare_task_file_cmd, 
        declare_use_sim_time_cmd,
        nav2_launch,
        task_executor_node,
        rviz_node,
    ])
