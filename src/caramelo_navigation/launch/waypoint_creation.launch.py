#!/usr/bin/env python3
"""
WAYPOINT CREATION LAUNCH - CARAMELO NAVIGATION (WORKSTATION-READY)
================================================================

Launch file interativo para criação de waypoints/workstations usando RViz.
Agora com suporte específico para workstations da RoboCup@Work.

Usage:
    ros2 launch caramelo_navigation waypoint_creation.launch.py arena:=arena_fei

Features:
- Map server com mapa pré-existente
- Robot description para visualização  
- Interface interativa no RViz
- Criação de workstations com posicionamento preciso (8cm da mesa)
- Validação de free space 80x80cm
- Exportação automática para waypoints.json na pasta da arena

Como usar para WORKSTATIONS:
1. Execute: ros2 launch caramelo_navigation waypoint_creation.launch.py arena:=arena_fei
2. Use "2D Pose Estimate" para marcar o CENTRO da mesa de trabalho
3. Use "2D Nav Goal" para definir a posição de DOCKING do robô (8cm da mesa)
4. Digite nome da WS (ex: "WS01", "WS02")
5. Os waypoints são salvos automaticamente em maps/[arena]/waypoints.json

IMPORTANTE: Agora otimizado para criar posições de docking nas workstations!
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    # Caminhos
    caramelo_nav_dir = get_package_share_directory('caramelo_navigation')
    caramelo_desc_dir = get_package_share_directory('caramelo_description')
    
    # Parâmetros
    arena = LaunchConfiguration('arena')
    map_file = LaunchConfiguration('map_file')
    
    # Argumentos obrigatórios
    declare_arena_cmd = DeclareLaunchArgument(
        'arena',
        default_value='arena_fei',
        description='Nome da arena (ex: arena_fei, hotel, laboratorio). Map files must exist in maps/[arena]/ folder.')
    
    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Caminho COMPLETO para o arquivo map.yaml (ex: /home/work/Caramelo_workspace/maps/arena_robocup25/map.yaml)')
    
    declare_arena_mode_cmd = DeclareLaunchArgument(
        'arena_mode',
        default_value='true',
        description='Usar modo arena (true) ou map_file direto (false)')
    
    # URDF do robô
    urdf_file = os.path.join(caramelo_desc_dir, 'URDF', 'robot.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Construir caminho para a pasta do mapa baseado na arena
    map_folder_path = PathJoinSubstitution([
        '/home/work/Caramelo_workspace/maps',
        arena
    ])
    
    # Construir caminho completo para o arquivo map.yaml
    map_file_path = PathJoinSubstitution([
        map_folder_path,
        'map.yaml'
    ])
    
    # Map server único que escolhe o caminho baseado na condição
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{
            'yaml_filename': map_file,  # Usa map_file diretamente
            'use_sim_time': False
        }],
        output='screen'
    )
    
    # 2. Lifecycle manager para o map server
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server']
        }],
        output='screen'
    )
    
    # 3. Robot state publisher (para mostrar o robô no RViz)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }],
        output='screen'
    )
    
    # 4. Joint state publisher (para o robô não ficar "quebrado")
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': False
        }],
        output='screen'
    )
    
    # 5. TF static: map -> odom
    tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    
    # 6. Interactive Robot Positioner 
    interactive_positioner = Node(
        package='caramelo_navigation',
        executable='interactive_robot_positioner',
        name='interactive_robot_positioner',
        parameters=[{
            'map_file': map_file,  # Usa map_file diretamente
            'map_folder': map_folder_path
        }],
        output='screen'
    )
    
    # 7. RViz com configuração final - aguarda 5 segundos
    rviz_config = os.path.join(caramelo_nav_dir, 'config', 'checkpoint_creator_final.rviz')
    
    rviz_delayed = TimerAction(
        period=5.0,  # Aguarda 5 segundos
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        declare_arena_cmd,
        declare_map_file_cmd,
        declare_arena_mode_cmd,
        map_server,
        lifecycle_manager,
        robot_state_publisher,
        joint_state_publisher,
        tf_map_odom,
        interactive_positioner,
        rviz_delayed
    ])


if __name__ == '__main__':
    generate_launch_description()
