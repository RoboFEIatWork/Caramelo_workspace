#!/usr/bin/env python3
"""
CARAMELO COMPETITION LAUNCHER - Sistema Universal de Competição

Launcher modular que aceita parâmetros de arena e task:
- Arena: especifica qual mapa usar (ex: arena_fei, arena_larc, etc)
- Task: especifica qual competição (BMT, BTT1, BTT2, ATT1, ATT2, Final)

Uso:
ros2 launch caramelo_tasks competition.launch.py arena:=arena_fei task:=BMT

Autor: GitHub Copilot
Data: 2025-07-12
"""

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Gera o launch description para competições do Caramelo"""
    
    # Argumentos de entrada
    arena_arg = DeclareLaunchArgument(
        'arena',
        default_value='arena_fei',
        description='Nome da arena (pasta em maps/)'
    )
    
    task_arg = DeclareLaunchArgument(
        'task', 
        default_value='BMT',
        choices=['BMT', 'BTT1', 'BTT2', 'ATT1', 'ATT2', 'Final'],
        description='Tipo de competição'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Ativa modo debug com logs extras'
    )
    
    # Configurações
    arena_name = LaunchConfiguration('arena')
    task_name = LaunchConfiguration('task')
    debug_mode = LaunchConfiguration('debug')
    
    # Caminhos dinâmicos
    caramelo_tasks_pkg = FindPackageShare('caramelo_tasks')
    caramelo_navigation_pkg = FindPackageShare('caramelo_navigation')
    
    # Path do mapa baseado na arena
    map_yaml_path = PathJoinSubstitution([
        '/home/work/Caramelo_workspace/maps',
        arena_name,
        'map.yaml'
    ])
    
    # Path dos waypoints baseado na arena
    waypoints_json_path = PathJoinSubstitution([
        '/home/work/Caramelo_workspace/maps', 
        arena_name,
        'waypoints.json'
    ])
    
    # Path da missão baseado na task
    mission_yaml_path = PathJoinSubstitution([
        caramelo_tasks_pkg,
        task_name,
        'mission.yaml'
    ])
    
    # Launch da navegação com parâmetros dinâmicos
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                caramelo_navigation_pkg,
                'launch',
                'caramelo_navigation.launch.py'
            ])
        ]),
        launch_arguments={
            'map_yaml': map_yaml_path,
            'mission_file': mission_yaml_path,
            'waypoints_file': waypoints_json_path,
            'auto_start': 'true',
            'use_rviz': 'true'
        }.items()
    )
    
    # Launch específico da task (se existir)
    task_specific_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                caramelo_tasks_pkg,
                task_name,
                'task_specific.launch.py'
            ])
        ]),
        condition=IfCondition(
            # Verifica se arquivo existe (implementar lógica se necessário)
            'true'  # Por enquanto sempre inclui
        )
    )
    
    # Logs informativos
    start_log = LogInfo(
        msg=[
            '🚀 CARAMELO COMPETITION SYSTEM',
            '\\n   Arena: ', arena_name,
            '\\n   Task: ', task_name,
            '\\n   Map: ', map_yaml_path,
            '\\n   Mission: ', mission_yaml_path,
            '\\n   Waypoints: ', waypoints_json_path
        ]
    )
    
    return LaunchDescription([
        # Argumentos
        arena_arg,
        task_arg, 
        debug_arg,
        
        # Logs
        start_log,
        
        # Launches
        navigation_launch,
        # task_specific_launch,  # Comentado até implementar tasks específicas
    ])
