#!/usr/bin/env python3
"""
TERMINAL 3: VISUALIZAÇÃO E NAVEGAÇÃO AUTÔNOMA
============================================

Este launch ativa:
- RViz (visualização)
- Navegação autônoma por waypoints

IMPORTANTE: Execute APÓS hardware + Nav2 estarem rodando!

Uso:
ros2 launch caramelo_navigation caramelo_autonomous.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    
    # Pacotes
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Configurações
    rviz_config = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    mission_file = '/home/work/Caramelo_workspace/maps/arena_fei/mission.yaml'
    
    # ========== VISUALIZAÇÃO ==========
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    # ========== NAVEGAÇÃO AUTÔNOMA ==========
    # Aguarda RViz carregar e AMCL convergir
    
    waypoint_navigation = TimerAction(
        period=10.0,  # Aguarda RViz + AMCL
        actions=[
            Node(
                package='caramelo_navigation',
                executable='caramelo_waypoint_nav',
                name='caramelo_waypoint_navigator',
                output='screen',
                parameters=[{
                    'mission_file': mission_file,
                    'auto_start': True,
                    'publish_initial_pose': False  # Já foi publicada pelo initializer
                }]
            )
        ]
    )

    return LaunchDescription([
        # Visualização imediata
        rviz,
        
        # Navegação autônoma (10s delay)
        waypoint_navigation
    ])
