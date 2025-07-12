#!/usr/bin/env python3
"""
SISTEMA COMPLETO SEGURO - CARAMELO
Launch que inicializa todos os componentes em ordem segura
e só inicia navegação quando tudo estiver estável
"""

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Caminhos dos pacotes
    caramelo_bringup_path = FindPackageShare('caramelo_bringup')
    caramelo_navigation_path = FindPackageShare('caramelo_navigation')
    
    # Argumentos
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # 1. ENCODER (imediato)
    encoder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            caramelo_bringup_path, '/launch/encoder_bringup.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # 2. PWM (após 3 segundos)
    pwm_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    caramelo_bringup_path, '/launch/pwm_bringup.launch.py'
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time
                }.items()
            )
        ]
    )
    
    # 3. LIDAR (após 6 segundos)
    lidar_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    caramelo_bringup_path, '/launch/lidar_bringup.launch.py'
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time
                }.items()
            )
        ]
    )
    
    # 4. NAVEGAÇÃO (após 10 segundos)
    navigation_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    caramelo_navigation_path, '/launch/caramelo_navigation.launch.py'
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time
                }.items()
            )
        ]
    )
    
    # 5. WAYPOINT NAVIGATION (após 20 segundos - aguarda Nav2 estabilizar)
    waypoint_navigation = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='caramelo_navigation',
                executable='caramelo_waypoint_nav',
                name='caramelo_waypoint_nav',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'auto_start': True,  # Vai aguardar localização antes de iniciar
                    'publish_initial_pose': True,
                }]
            )
        ]
    )
    
    # Log de início
    start_log = ExecuteProcess(
        cmd=['echo', '🚀 INICIANDO SISTEMA COMPLETO CARAMELO - MODO SEGURO'],
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        start_log,
        encoder_launch,      # T+0s:  Encoder
        pwm_launch,          # T+3s:  PWM
        lidar_launch,        # T+6s:  LIDAR
        navigation_launch,   # T+10s: Nav2
        waypoint_navigation, # T+20s: Waypoints (com verificação de localização)
    ])
