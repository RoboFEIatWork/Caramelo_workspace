#!/usr/bin/env python3
"""
CARAMELO TASK NAVIGATOR LAUNCH
Launch para execução de missões baseadas em task.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Argumentos
    task_name = LaunchConfiguration('task')
    arena = LaunchConfiguration('arena')
    
    # Caminhos
    pkg_tasks = get_package_share_directory('caramelo_tasks')
    
    # Argumentos de launch
    declare_task_cmd = DeclareLaunchArgument(
        'task',
        default_value='BMT',
        description='Nome da task (BMT, BTT1, BTT2, ATT1, ATT2)'
    )
    
    declare_arena_cmd = DeclareLaunchArgument(
        'arena',
        default_value='teste_robocup25',
        description='Nome da arena'
    )
    
    # Task Navigator Node
    task_navigator_node = Node(
        package='caramelo_tasks',
        executable='task_navigator',
        name='task_navigator',
        output='screen',
        parameters=[{
            'task_file': ['/home/work/Caramelo_workspace/src/caramelo_tasks/', task_name, '/task.yaml'],
            'arena': arena,
            'waypoints_file': ['/home/work/Caramelo_workspace/maps/', arena, '/waypoints.json'],
        }]
    )
    
    # Log informativo
    start_log = LogInfo(
        msg=[
            "🚀 CARAMELO TASK NAVIGATOR\n",
            "   Task: ", task_name, "\n",
            "   Arena: ", arena, "\n",
            "   Aguardando Nav2 estar ativo..."
        ]
    )
    
    return LaunchDescription([
        # Argumentos
        declare_task_cmd,
        declare_arena_cmd,
        
        # Log
        start_log,
        
        # Nodes
        task_navigator_node,
    ])
