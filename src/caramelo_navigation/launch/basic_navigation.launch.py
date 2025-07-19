#!/usr/bin/env python3
"""
CARAMELO BASIC NAVIGATION LAUNCH
===============================

Launch baseado no padrão funcional do caramelo_navigation.launch.py
Inicia todos os componentes Nav2 individualmente para máximo controle.

Este launch inicia:
1. Map Server - carrega o mapa estático
2. AMCL - localização por filtro de partículas
3. Controller Server - controle de movimento DWB 
4. Planner Server - planejamento global NavFn
5. Behavior Server - comportamentos de recuperação
6. BT Navigator - coordenação via behavior trees
7. Workstation Navigation Server - navegação por workstations
8. RViz - visualização

Uso:
    ros2 launch caramelo_navigation basic_navigation.launch.py arena:=arena_robocup25

Para testar navegação:
    ros2 topic pub --once /navigate_to_workstation std_msgs/String "data: WS01"

Baseado no commit funcional fd8ea47
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    
    # === ARGUMENTOS ===
    use_sim_time = LaunchConfiguration('use_sim_time')
    arena = LaunchConfiguration('arena')
    
    # Caminhos dos arquivos
    pkg_nav = get_package_share_directory('caramelo_navigation')
    
    # === DECLARAÇÃO DE ARGUMENTOS ===
    declare_arena_cmd = DeclareLaunchArgument(
        'arena',
        default_value='arena_robocup25',
        description='Nome da arena (pasta em /maps/)'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # === CAMINHOS DINÂMICOS ===
    map_yaml_path = PathJoinSubstitution([
        '/home/work/Caramelo_workspace/maps',
        arena,
        'map.yaml'
    ])
    
    nav2_params_path = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    rviz_config_path = os.path.join(pkg_nav, 'rviz', 'navigation_complete.rviz')  # Usar config completo
    
    # ==========================================================================
    # NAV2 STACK INDIVIDUAL (baseado no commit funcional fd8ea47)
    # ==========================================================================
    
    # 1. Map Server (mapa estático da arena)
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_path
        }]
    )
    
    # 2. AMCL (localização por filtro de partículas + scan matching)
    amcl_cmd = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_path]
    )
    
    # 3. Controller Server
    controller_cmd = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_path]
    )
    
    # 4. Planner Server
    planner_cmd = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_path]
    )
    
    # 5. Behavior Server
    behaviors_cmd = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_path]
    )
    
    # 6. BT Navigator - REMOVIDO: causa conflito de IDs
    # bt_navigator_cmd = Node(
    #     package='nav2_bt_navigator',
    #     executable='bt_navigator',
    #     name='bt_navigator',
    #     output='screen',
    #     parameters=[nav2_params_path]
    # )
    
    # 7. Waypoint Follower
    waypoint_follower_cmd = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_path]
    )
    
    # 8. Velocity Smoother
    velocity_smoother_cmd = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_path]
    )
    
    # 8. Twist Converter Node - Converte /cmd_vel para /mecanum_drive_controller/cmd_vel
    twist_converter_cmd = Node(
        package='caramelo_bringup',
        executable='twist_converter_node',
        name='twist_converter_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('/cmd_vel_in', '/cmd_vel'),
            ('/cmd_vel_out', '/mecanum_drive_controller/cmd_vel')
        ]
    )
    
    # 9. Lifecycle Manager
    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,  # MUDANÇA PRINCIPAL: autostart True
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'behavior_server',
                'waypoint_follower',
                'velocity_smoother'
                # REMOVIDO: 'bt_navigator' - causa conflito
            ]
        }]
    )    # === WORKSTATION NAVIGATION SERVER ===
    workstation_nav_server = TimerAction(
        period=10.0,  # Aguardar Nav2 inicializar completamente
        actions=[
            Node(
                package='caramelo_navigation',
                executable='workstation_navigation_server_simple',
                name='workstation_navigation_server',
                output='screen',
                parameters=[{
                    'arena': arena,
                    'use_sim_time': use_sim_time,
                }]
            )
        ]
    )
    
    # === VISUALIZAÇÃO ===
    rviz_cmd = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_path],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    # === LOGS INFORMATIVOS ===
    start_log = LogInfo(
        msg=[
            '🚀 CARAMELO BASIC NAVIGATION SYSTEM',
            '\\n   Arena: ', arena,
            '\\n   Map: ', map_yaml_path,
            '\\n   Config: ', nav2_params_path,
            '\\n\\n📋 SISTEMA INCLUÍDO:',
            '\\n   ✅ Map Server (carrega mapa)',
            '\\n   ✅ AMCL (localização)',
            '\\n   ✅ Nav2 Planner (planejamento de rota)',
            '\\n   ✅ Nav2 Controller (controle de movimento)',
            '\\n   ✅ Nav2 Recoveries (recuperação de falhas)',
            '\\n   ✅ Nav2 BT Navigator (árvore de comportamento)',
            '\\n   ✅ Workstation Navigation Server',
            '\\n   ✅ RViz2 (visualização)',
            '\\n\\n📡 TÓPICOS DISPONÍVEIS:',
            '\\n   /navigate_to_workstation (std_msgs/String) - Enviar comando',
            '\\n   /navigation_status (std_msgs/String) - Status da navegação',
            '\\n\\n🧪 PARA TESTAR:',
            '\\n   ros2 run caramelo_navigation navigation_test_client_simple WS01',
            '\\n   ros2 topic pub --once /navigate_to_workstation std_msgs/String "data: WS07"',
            '\\n\\n⚠️  IMPORTANTE: Este launch NÃO inclui hardware!',
            '\\n   Execute separadamente: encoders, PWM, LIDAR, base_controller'
        ]
    )
    
    return LaunchDescription([
        # Argumentos
        declare_arena_cmd,
        declare_use_sim_time_cmd,
        
        # Logs
        start_log,
        
        # Nav2 Stack Individual (baseado no commit funcional fd8ea47)
        map_server_cmd,
        amcl_cmd,
        controller_cmd,
        planner_cmd,
        behaviors_cmd,
        # bt_navigator_cmd,  # REMOVIDO: causa conflito
        waypoint_follower_cmd,
        velocity_smoother_cmd,
        twist_converter_cmd,  # ADICIONADO: Conversor cmd_vel
        lifecycle_manager_cmd,
        
        # Workstation Navigation Server
        workstation_nav_server,
        
        # Visualização  
        rviz_cmd,
    ])
