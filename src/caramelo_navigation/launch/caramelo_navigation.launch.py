#!/usr/bin/env python3
"""
CARAMELO NAVEGAÇÃO COMPLETA
Launch completo para navegação autônoma incluindo:
- Nav2 stack completo
- AMCL para localização
- Map server para mapa estático  
- TF map → odom (através do AMCL)
- Twist converter para movimento físico
- RViz para visualização
- Waypoint navigation automático
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
    map_yaml_file = LaunchConfiguration('map')
    nav2_config_file = LaunchConfiguration('params_file')
    arena = LaunchConfiguration('arena')
    
    # Caminhos dos arquivos
    pkg_nav = get_package_share_directory('caramelo_navigation')
    
    # Argumentos de launch
    declare_arena_cmd = DeclareLaunchArgument(
        'arena',
        default_value='teste_lab',
        description='Nome da arena (pasta dentro de maps/). Ex: arena_fei, teste_lab'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=['/home/work/Caramelo_workspace/maps/', arena, '/map.yaml'],
        description='Full path to map yaml file to load'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_nav, 'config', 'nav2_params.yaml'),
        description='Full path to param file to load'
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
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )
    
    # 2. Map Server (carrega mapa estático)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file
        }]
    )
    
    # 3. AMCL (localização + TF map→odom)
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_config_file, {'use_sim_time': use_sim_time}]
    )
    
    # 4. Nav2 Controller Server
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[nav2_config_file, {'use_sim_time': use_sim_time}]
    )
    
    # 5. Nav2 Planner Server
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_config_file, {'use_sim_time': use_sim_time}]
    )
    
    # 6. Nav2 Behavior Server
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        output='screen',
        parameters=[nav2_config_file, {'use_sim_time': use_sim_time}]
    )
    
    # 7. Nav2 BT Navigator
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_config_file, {'use_sim_time': use_sim_time}]
    )
    
    # 8. Nav2 Waypoint Follower
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_config_file, {'use_sim_time': use_sim_time}]
    )
    
    # 9. Nav2 Lifecycle Manager
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower'
            ]
        }]
    )
    
    # ==========================================================================
    # INICIALIZAÇÃO E VISUALIZAÇÃO (COM DELAY)
    # ==========================================================================
    
    # 10. AMCL Initializer (publica pose inicial automaticamente)
    amcl_initializer_node = TimerAction(
        period=8.0,  # Aguarda Nav2 inicializar
        actions=[
            Node(
                package='caramelo_navigation',
                executable='amcl_initializer',
                name='amcl_initializer',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'initial_pose_x': 0.0,
                    'initial_pose_y': 0.0,
                    'initial_pose_yaw': 0.0
                }]
            )
        ]
    )
    
    # 11. RViz (visualização)
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_file = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    rviz_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_file]
            )
        ]
    )
    
    # 12. Waypoint Navigation (navegação automática)
    waypoint_nav_node = TimerAction(
        period=15.0,  # Aguarda tudo inicializar
        actions=[
            Node(
                package='caramelo_navigation',
                executable='caramelo_waypoint_nav',
                name='caramelo_waypoint_navigator',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'arena': arena,
                    'mission_file': ['/home/work/Caramelo_workspace/maps/', arena, '/mission.yaml'],
                    'waypoints_file': ['/home/work/Caramelo_workspace/maps/', arena, '/workstations.json'],
                    'auto_start': True,
                    'publish_initial_pose': True,
                }],
                remappings=[
                    ('/cmd_vel', '/cmd_vel_raw'),  # Para safety filter
                ]
            )
        ]
    )
    
    # 13. Safety Filter (filtro de segurança para velocidades)
    safety_filter_node = Node(
        package='caramelo_navigation', 
        executable='cmd_vel_safety_filter',
        name='cmd_vel_safety_filter',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_linear_vel': 0.5,      # Velocidade máxima para competição
            'max_angular_vel': 1.0,
            'emergency_stop_distance': 0.10,  # 10cm - próximo para WS
            'warning_distance': 0.25,          # 25cm - aviso antecipado
        }],
        remappings=[
            ('/cmd_vel_raw', '/cmd_vel_raw'),  # Input do waypoint navigator
            ('/cmd_vel', '/cmd_vel'),          # Output para motores
        ]
    )
    
    # 14. CMD VEL Monitor (monitor de comandos)
    cmd_vel_monitor_node = Node(
        package='caramelo_navigation',
        executable='cmd_vel_monitor',
        name='cmd_vel_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    # ==========================================================================
    # LOGS INFORMATIVOS
    # ==========================================================================
    
    startup_log = LogInfo(
        msg="🚀 CARAMELO NAVEGAÇÃO: Iniciando sistema completo..."
    )
    
    nav2_ready_log = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="📍 Nav2 stack inicializando...")
        ]
    )
    
    amcl_ready_log = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg="🗺️  AMCL e mapa carregados, publicando pose inicial...")
        ]
    )
    
    waypoint_ready_log = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg="🎯 Iniciando navegação automática por waypoints!")
        ]
    )
    
    return LaunchDescription([
        # Argumentos
        declare_arena_cmd,
        declare_use_sim_time_cmd,
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        
        # Logs
        startup_log,
        nav2_ready_log,
        amcl_ready_log,
        waypoint_ready_log,
        
        # Nodes essenciais (imediatos)
        twist_converter_node,
        map_server_node,
        amcl_node,
        controller_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        lifecycle_manager_node,
        safety_filter_node,
        cmd_vel_monitor_node,
        
        # Nodes com delay
        amcl_initializer_node,
        rviz_node,
        waypoint_nav_node,
    ])
