#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Argumentos de entrada
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='/home/work/Caramelo_workspace/mapa_20250704_145039.yaml',
        description='Path to the map file'
    )
    
    task_file_arg = DeclareLaunchArgument(
        'task_file',
        default_value='tasks.yaml',
        description='Name of the task file in caramelo_tasks/config/'
    )
    
    # Configurações
    map_file = LaunchConfiguration('map_file')
    task_file = LaunchConfiguration('task_file')
    
    # Paths dos pacotes
    nav_launch_dir = os.path.join(get_package_share_directory('caramelo_navigation'), 'launch')
    
    # 1. Navigation Stack (Nav2 + AMCL + Map Server)
    navigation_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav_launch_dir, '/navigation_launch.py']),
        launch_arguments={
            'map_file': map_file,
            'use_sim_time': 'false'
        }.items()
    )
    
    # 2. Task Executor (com delay para aguardar Nav2 inicializar)
    task_executor = TimerAction(
        period=5.0,  # Aguarda 5 segundos para Nav2 inicializar
        actions=[
            Node(
                package='caramelo_tasks',
                executable='task_executor_node',
                name='task_executor',
                output='screen',
                parameters=[
                    {'task_file': task_file},
                    {'use_sim_time': False}
                ]
            )
        ]
    )
    
    # Log de inicialização
    startup_log = LogInfo(
        msg=[
            '\\n🏆 =================================================================',
            '\\n🤖 CARAMELO ROBOCUP@WORK COMPETITION MODE',
            '\\n🗺️  Map: ', map_file,
            '\\n📋 Tasks: ', task_file,
            '\\n⚠️  MAKE SURE PWM AND ENCODER BRINGUP ARE RUNNING!',
            '\\n🚀 Starting autonomous competition execution...',
            '\\n=================================================================\\n'
        ]
    )
    
    return LaunchDescription([
        # Argumentos
        map_file_arg,
        task_file_arg,
        
        # Log de inicialização
        startup_log,
        
        # Componentes do sistema
        navigation_stack,
        task_executor,
        
        # Log final
        LogInfo(msg='🎯 Competition system ready! Robot will start autonomous execution in 5 seconds.')
    ])
