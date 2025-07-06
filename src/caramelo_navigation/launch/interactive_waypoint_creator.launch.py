#!/usr/bin/env python3
"""
Launch INTERATIVO - Sistema completo para posicionar robô e criar waypoints
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """
    Launch INTERATIVO - Sistema completo para posicionar robô e criar waypoints
    
    INCLUI:
    - Map server com o mapa
    - Robot state publisher com o URDF do robô
    - Interactive robot positioner (move o robô virtual)
    - RViz configurado
    
    COMO USAR:
    1. ros2 launch caramelo_navigation interactive_waypoint_creator.launch.py
    2. Aguarde o RViz abrir
    3. Use "2D Pose Estimate" para MOVER o robô virtual
    4. Ajuste a posição até ficar satisfeito
    5. Use "2D Nav Goal" para SALVAR o waypoint
    6. Repita para criar mais waypoints!
    """
    
    # Caminhos
    caramelo_nav_dir = get_package_share_directory('caramelo_navigation')
    caramelo_desc_dir = get_package_share_directory('caramelo_description')
    
    # Arquivo do mapa
    map_file = '/home/work/Caramelo_workspace/mapa_20250704_145039.yaml'
    
    # URDF do robô
    urdf_file = os.path.join(caramelo_desc_dir, 'URDF', 'robot.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # 1. Map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{
            'yaml_filename': map_file,
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
    
    # 6. Interactive Robot Positioner (substitui o transform odom -> base_link)
    interactive_positioner = Node(
        package='caramelo_navigation',
        executable='interactive_robot_positioner',
        name='interactive_robot_positioner',
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
