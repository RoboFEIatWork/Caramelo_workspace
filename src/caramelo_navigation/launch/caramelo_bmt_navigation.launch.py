#!/usr/bin/env python3
"""
CARAMELO BMT NAVIGATION LAUNCH - Sistema Completo Integrado
Baseado no sistema funcional do commit 7e8a309 + melhorias BMT

Sistema que inicia:
1. Nav2 stack completo com AMCL
2. Map server para mapa estático
3. Sistema BMT de waypoints com tasks
4. RViz para visualização
5. Inicialização automática escalonada

Autor: GitHub Copilot + Sistema Funcional 7e8a309
Data: 2025-07-18
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
    
    # === ARGUMENTOS ===
    use_sim_time = LaunchConfiguration('use_sim_time')
    arena = LaunchConfiguration('arena')
    map_yaml_file = LaunchConfiguration('map')
    nav2_config_file = LaunchConfiguration('params_file')
    waypoints_file = LaunchConfiguration('waypoints_file')
    task_file = LaunchConfiguration('task_file')
    task_type = LaunchConfiguration('task_type')
    mission_mode = LaunchConfiguration('mission_mode')
    auto_start = LaunchConfiguration('auto_start')
    
    # === CAMINHOS ===
    pkg_nav = get_package_share_directory('caramelo_navigation')
    
    # === DECLARAÇÃO DE ARGUMENTOS ===
    declare_arena_cmd = DeclareLaunchArgument(
        'arena',
        description='Arena name (OBRIGATÓRIO): lab_fei, arena_robocup25, teste_robocup25, etc.'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=['/home/work/Caramelo_workspace/maps/', arena, '/map.yaml'],
        description='Full path to map yaml file to load (dynamic based on arena)'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_nav, 'config', 'caramelo_nav2_functional.yaml'),
        description='Full path to param file to load'
    )
    
    declare_waypoints_file_cmd = DeclareLaunchArgument(
        'waypoints_file',
        default_value=['/home/work/Caramelo_workspace/maps/', arena, '/waypoints.json'],
        description='Full path to waypoints JSON file (dynamic based on arena)'
    )
    
    declare_task_file_cmd = DeclareLaunchArgument(
        'task_file',
        default_value='/home/work/Caramelo_workspace/src/caramelo_tasks/BMT/task.yaml',
        description='Full path to BMT task YAML file'
    )
    
    declare_task_type_cmd = DeclareLaunchArgument(
        'task_type',
        default_value='BMT',
        description='Task type: BMT, BTT1, or BTT2'
    )
    
    declare_mission_mode_cmd = DeclareLaunchArgument(
        'mission_mode',
        default_value='bmt',
        description='Mission mode: bmt (BMT RoboCup format), robocup (visit all waypoints) or simple (visit all waypoints)'
    )
    
    declare_auto_start_cmd = DeclareLaunchArgument(
        'auto_start',
        default_value='true',
        description='Auto start mission after localization'
    )
    
    # ==========================================================================
    # NODES ESSENCIAIS PARA NAVEGAÇÃO (baseado no sistema funcional)
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
    
    # 6. Nav2 Smoother Server
    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_config_file, {'use_sim_time': use_sim_time}]
    )
    
    # 7. Nav2 Behavior Server
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        output='screen',
        parameters=[nav2_config_file, {'use_sim_time': use_sim_time}]
    )
    
    # 8. Nav2 BT Navigator
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_config_file, {'use_sim_time': use_sim_time}]
    )
    
    # 9. Nav2 Waypoint Follower
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_config_file, {'use_sim_time': use_sim_time}]
    )
    
    # 10. Nav2 Lifecycle Manager
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
                'smoother_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower'
            ]
        }]
    )
    
    # ==========================================================================
    # SISTEMA BMT INTEGRADO (com delays baseados no sistema funcional)
    # ==========================================================================
    
    # 11. BMT Waypoint Navigator (principal - substitui sistema antigo)
    bmt_waypoint_nav_node = TimerAction(
        period=10.0,  # Aguarda Nav2 estar totalmente pronto
        actions=[
            Node(
                package='caramelo_navigation',
                executable='caramelo_bmt_waypoint_nav',
                name='caramelo_bmt_waypoint_navigator',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'waypoints_file': waypoints_file,
                    'task_file': task_file,
                    'task_type': task_type,
                    'mission_mode': mission_mode,
                    'auto_start': auto_start,
                    'initial_pose_x': 0.0,
                    'initial_pose_y': 0.0,
                    'initial_pose_yaw': 0.0
                }]
            )
        ]
    )
    
    # ==========================================================================
    # VISUALIZAÇÃO E MONITORAMENTO
    # ==========================================================================
    
    # 12. RViz (visualização)
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_file = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    rviz_node = TimerAction(
        period=8.0,
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
    
    # 13. Status Monitor BMT (opcional)
    bmt_status_monitor = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='caramelo_navigation',
                executable='bmt_status_monitor',
                name='bmt_status_monitor',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                }]
            )
        ]
    )
    
    # ==========================================================================
    # LOGS INFORMATIVOS (baseados no sistema funcional)
    # ==========================================================================
    
    startup_log = LogInfo(
        msg="🚀 CARAMELO BMT NAVIGATION: Iniciando sistema completo integrado..."
    )
    
    nav2_ready_log = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="📍 Nav2 stack inicializando...")
        ]
    )
    
    bmt_ready_log = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg="🤖 BMT Waypoint Navigation iniciando...")
        ]
    )
    
    system_ready_log = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg="✅ Sistema BMT completo ativo! Navegação autônoma por tasks.")
        ]
    )
    
    return LaunchDescription([
        # === ARGUMENTOS ===
        declare_arena_cmd,  # OBRIGATÓRIO
        declare_use_sim_time_cmd,
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_waypoints_file_cmd,
        declare_task_file_cmd,
        declare_task_type_cmd,
        declare_mission_mode_cmd,
        declare_auto_start_cmd,
        
        # === LOGS ===
        startup_log,
        nav2_ready_log,
        bmt_ready_log,
        system_ready_log,
        
        # === NAVEGAÇÃO BÁSICA (baseada no sistema funcional) ===
        twist_converter_node,
        map_server_node,
        amcl_node,
        controller_server_node,
        planner_server_node,
        smoother_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        lifecycle_manager_node,
        
        # === SISTEMA BMT (com delays) ===
        bmt_waypoint_nav_node,
        
        # === VISUALIZAÇÃO ===
        rviz_node,
        # bmt_status_monitor,  # Comentado até implementar
    ])
