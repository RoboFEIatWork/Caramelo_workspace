#!/usr/bin/env python3
"""
🗺️ TELEOP MAPPING CARAMELO - OCUPANCY GRID DE ALTA PRECISÃO

Sistema completo para mapeamento manual com SlamToolbox otimizado.
Arquitetura: map → odom → base_footprint → base_link → laser_frame

PASSO-A-PASSO PARA MAPEAMENTO:
==============================

TERMINAL 1 - SENSORES E ODOMETRIA:
  ros2 launch caramelo_bringup odometry_bringup.launch.py

TERMINAL 2 - ATUADORES/PWM:  
  ros2 launch caramelo_bringup actuators_bringup.launch.py

TERMINAL 3 - LIDAR:
  ros2 launch caramelo_bringup lidar_bringup.launch.py

TERMINAL 4 - ESTE MAPEAMENTO:
  ros2 launch caramelo_navigation teleop_mapping.launch.py arena:=NOME_ARENA

🎮 CONTROLES DE MAPEAMENTO:
==========================
  i = frente     u = diagonal ↖    o = diagonal ↗
  j = esquerda   k = PARAR         l = direita  
  m = ré         , = ré-esq        . = ré-dir

  q/z = ↑/↓ velocidade linear    w/x = ↑/↓ velocidade angular

 SALVAR MAPA FINAL:
====================
# Em outro terminal após terminar o mapeamento:
mkdir -p ~/Caramelo_workspace/maps/NOME_ARENA
ros2 run nav2_map_server map_saver_cli -f ~/Caramelo_workspace/maps/NOME_ARENA/map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch limpo para mapeamento manual com SLAM
    """
    
    # Configurações
    use_sim_time = LaunchConfiguration('use_sim_time')
    arena = LaunchConfiguration('arena')
    
    # Caminhos
    nav_pkg = get_package_share_directory('caramelo_navigation')
    rviz_config_file = os.path.join(nav_pkg, 'rviz', 'slam_config.rviz')
    slam_params_file = os.path.join(nav_pkg, 'config', 'mapper_params_online_async.yaml')
    
    # Argumentos
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true')
    
    declare_arena_cmd = DeclareLaunchArgument(
        'arena',
        default_value='teste_lab',
        description='Nome da arena para salvar o mapa')
    
    # 1. SLAM TOOLBOX - COMO NO COMMIT FUNCIONAL f14bccc
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # 2. LIFECYCLE MANAGER - COMO NO COMMIT FUNCIONAL
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['slam_toolbox']
        }]
    )

    # 3. CONVERSOR TWIST
    twist_converter_node = Node(
        package='caramelo_bringup',
        executable='twist_converter_node',
        name='twist_converter',
        output='screen'
    )
    
    # 4. TELEOP KEYBOARD
    teleop_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e'
    )
    
    # 5. RVIZ (CONFIGURAÇÃO SIMPLES COMO NO COMMIT FUNCIONAL)
    rviz_node = TimerAction(
        period=3.0,  
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_slam',
                arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                }]
            )
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_arena_cmd,
        slam_toolbox_node,
        lifecycle_manager_node,
        twist_converter_node,
        teleop_keyboard_node,
        rviz_node,
    ])
