#!/usr/bin/env python3
"""
TERMINAL 2: NAVEGAÇÃO NAV2 COM MAPA
==================================

Este launch ativa a navegação Nav2:
- Map Server (mapa estático)
- AMCL (localização)
- Nav2 stack completo
- AMCL Initializer (força frame 'map')

IMPORTANTE: Execute APÓS o hardware estar rodando!

Uso:
ros2 launch caramelo_navigation caramelo_nav2.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    
    # Pacotes
    caramelo_navigation_dir = get_package_share_directory('caramelo_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Arquivos de configuração
    map_yaml = '/home/work/Caramelo_workspace/maps/arena_fei/map.yaml'
    nav2_params = os.path.join(caramelo_navigation_dir, 'config', 'caramelo_nav2.yaml')
    
    # ========== NAV2 STACK ==========
    
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments=[
            ('map', map_yaml),
            ('params_file', nav2_params),
            ('use_sim_time', 'false'),
            ('autostart', 'true')
        ]
    )
    
    # ========== AMCL INITIALIZER ==========
    # Força inicialização do AMCL para criar frame 'map'
    
    amcl_initializer = TimerAction(
        period=8.0,  # Aguarda Nav2 estar pronto
        actions=[
            Node(
                package='caramelo_navigation',
                executable='amcl_initializer',
                name='amcl_initializer',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        # Nav2 stack completo
        nav2,
        
        # AMCL initializer (8s delay)
        amcl_initializer
    ])
