#!/usr/bin/env python3
"""
CARAMELO SLAM + NAVEGAÇÃO SIMULTÂNEA
Launch para mapeamento E navegação em tempo real
Tolerante a odometria ruim e delays USB/ESP32
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Argumentos
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_config_file = LaunchConfiguration('slam_params_file')
    nav2_config_file = LaunchConfiguration('nav2_params_file')
    arena = LaunchConfiguration('arena')
    
    # Caminhos dos arquivos
    pkg_nav = get_package_share_directory('caramelo_navigation')
    
    # Argumentos de launch
    declare_arena_cmd = DeclareLaunchArgument(
        'arena',
        default_value='arena_real',
        description='Nome da arena para navegação (será criada durante SLAM)'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_nav, 'config', 'slam_params.yaml'),
        description='Full path to SLAM params file'
    )
    
    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(pkg_nav, 'config', 'caramelo_nav2.yaml'),
        description='Full path to Nav2 params file'
    )
    
    # ==========================================================================
    # NODES ESSENCIAIS PARA NAVEGAÇÃO
    # ==========================================================================
    
    # 1. Twist Converter (ESSENCIAL para movimento físico)
    twist_converter_node = Node(
        package='caramelo_bringup',
        executable='twist_converter_node',
        name='twist_converter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 2. SLAM Toolbox - MAPEAMENTO + LOCALIZAÇÃO EM TEMPO REAL
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',  # SLAM síncrono para navegação
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config_file, {'use_sim_time': use_sim_time}],
        remappings=[('/scan', '/scan')]
    )
    
    # 3. Controller Server com DWB
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_config_file, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', '/cmd_vel_raw')]  # vai para safety filter
    )
    
    # 4. Planner Server
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_config_file, {'use_sim_time': use_sim_time}]
    )
    
    # 5. Behavior Server
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_config_file, {'use_sim_time': use_sim_time}]
    )
    
    # 6. BT Navigator
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_config_file, {'use_sim_time': use_sim_time}]
    )
    
    # 7. Waypoint Follower
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_config_file, {'use_sim_time': use_sim_time}]
    )
    
    # 8. Smoother Server
    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_config_file, {'use_sim_time': use_sim_time}]
    )
    
    # 9. Velocity Smoother
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_config_file, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', '/cmd_vel_raw'),       # recebe do controller
                   ('/cmd_vel_smoothed', '/cmd_vel_raw')] # envia suavizado
    )
    
    # 10. Lifecycle Manager - gerencia todos os nodes Nav2
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server', 
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'smoother_server',
                'velocity_smoother'
            ]
        }]
    )
    
    # ==========================================================================
    # NODES DE SEGURANÇA E MONITORAMENTO
    # ==========================================================================
    
    # Safety Filter - protege contra comandos perigosos
    cmd_vel_safety_filter_node = Node(
        package='caramelo_bringup',
        executable='cmd_vel_safety_filter_node',
        name='cmd_vel_safety_filter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_raw', '/cmd_vel_raw'),    # recebe do Nav2
                   ('/cmd_vel', '/cmd_vel')]              # envia filtrado
    )
    
    # Monitor de comandos
    cmd_vel_monitor_node = Node(
        package='caramelo_bringup', 
        executable='cmd_vel_monitor_node',
        name='cmd_vel_monitor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # ==========================================================================
    # ACTIONS E TIMERS
    # ==========================================================================
    
    # Informação sobre inicialização
    log_info_slam = LogInfo(
        msg='🗺️ SLAM + NAVEGAÇÃO: Sistema iniciando...'
    )
    
    log_info_nav2 = TimerAction(
        period=5.0,
        actions=[LogInfo(msg='🚀 Nav2 stack ativo para navegação em SLAM!')]
    )
    
    # RViz para visualização
    rviz_node = TimerAction(
        period=8.0,  # inicia após SLAM estar ativo
        actions=[
            LogInfo(msg='🖥️ Iniciando RViz para visualização...'),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', os.path.join(pkg_nav, 'rviz', 'caramelo_navigation.rviz')],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    # Navegação por waypoints automática (após SLAM inicializar)
    waypoint_nav_node = TimerAction(
        period=10.0,  # aguarda SLAM inicializar
        actions=[
            LogInfo(msg='🎯 Iniciando navegação automática em modo SLAM!'),
            Node(
                package='caramelo_navigation',
                executable='caramelo_waypoint_nav',
                name='caramelo_waypoint_nav',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'auto_start': True},
                    {'publish_initial_pose': False},  # SLAM não precisa de pose inicial
                    {'slam_mode': True}  # modo SLAM ativo
                ]
            )
        ]
    )
    
    # ==========================================================================
    # LAUNCH DESCRIPTION
    # ==========================================================================
    
    return LaunchDescription([
        # Argumentos
        declare_arena_cmd,
        declare_use_sim_time_cmd,
        declare_slam_params_file_cmd,
        declare_nav2_params_file_cmd,
        
        # Informações
        log_info_slam,
        
        # Nodes essenciais
        twist_converter_node,
        slam_toolbox_node,              # SLAM em vez de map_server + AMCL
        controller_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        smoother_server_node,
        velocity_smoother_node,
        lifecycle_manager_node,
        
        # Nodes de segurança
        cmd_vel_safety_filter_node,
        cmd_vel_monitor_node,
        
        # Timers
        log_info_nav2,
        rviz_node,
        waypoint_nav_node
    ])


if __name__ == '__main__':
    generate_launch_description()
