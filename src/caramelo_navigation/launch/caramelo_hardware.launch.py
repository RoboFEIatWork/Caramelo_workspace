#!/usr/bin/env python3
"""
TERMINAL 1: HARDWARE DO ROBÔ CARAMELO
===================================

Este launch ativa TODO o hardware necessário:
- Encoders (odometria)
- PWM (motores)
- LiDAR (navegação)
- Twist Converter (movimento físico)

Uso:
ros2 launch caramelo_navigation caramelo_hardware.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    
    # Pacotes
    caramelo_bringup_dir = get_package_share_directory('caramelo_bringup')
    
    # ========== HARDWARE IMEDIATO ==========
    
    # PWM (motores)
    pwm_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(caramelo_bringup_dir, 'launch', 'pwm_bringup.launch.py')
        )
    )
    
    # Encoder (odometria)
    encoder_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(caramelo_bringup_dir, 'launch', 'encoder_bringup.launch.py')
        )
    )
    
    # ========== SENSORES (delay 2s) ==========
    
    # LiDAR
    lidar = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(caramelo_bringup_dir, 'launch', 'lidar_bringup.launch.py')
                )
            )
        ]
    )
    
    # ========== CONVERSORES (delay 4s) ==========
    
    # Twist Converter - CRÍTICO para movimento físico!
    twist_converter = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='caramelo_bringup',
                executable='twist_converter_node',
                name='twist_converter',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        # Hardware imediato
        pwm_hardware,
        encoder_hardware,
        
        # Sensores (2s)
        lidar,
        
        # Conversores (4s)
        twist_converter
    ])
