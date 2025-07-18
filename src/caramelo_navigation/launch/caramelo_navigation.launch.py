#!/usr/bin/env python3
"""
CARAMELO NAVEGAÇÃO COMPLETA COM SISTEMA DE TASKS
Launch completo para navegação autônoma incluindo:
- Nav2 stack completo com AMCL (localização)
- Map Server (mapa estático da arena)
- Sistema de tasks automático
- Filtro de laser scan (90°-270°)
- Raio de inflação corrigido (0.01m)

Uso: ros2 launch caramelo_navigation caramelo_navigation.launch.py arena:=teste_robocup25 task:=BMT
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
    arena = LaunchConfiguration('arena')
    task = LaunchConfiguration('task')
    
    # Caminhos dos arquivos
    pkg_nav = get_package_share_directory('caramelo_navigation')
    
    # Argumentos de launch (sistema BMT atual)
    declare_arena_cmd = DeclareLaunchArgument(
        'arena',
        default_value='teste_robocup25',
        description='Nome da arena (pasta dentro de maps/). Ex: arena_fei, teste_robocup25'
    )
    
    declare_task_cmd = DeclareLaunchArgument(
        'task',
        default_value='BMT',
        description='Tipo de task (BMT, BTT1, BTT2, ATT1, ATT2)'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    # Construir caminhos dinâmicos baseados na arena (mantendo flexibilidade)
    map_yaml_path = PathJoinSubstitution([
        '/home/work/Caramelo_workspace/maps',
        arena,
        'map.yaml'
    ])
    
    # Usar configuração nav2 correta (baseado no que funcionava)
    nav2_params_path = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    rviz_config_path = os.path.join(pkg_nav, 'rviz', 'navigation_complete.rviz')
    
    # ==========================================================================
    # NAVEGAÇÃO COM AMCL + MAPA ESTÁTICO (baseado no commit funcional fd8ea47)
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
    
    # 3. Laser Scan Filter (filtra scan: 90°-270°, min 5cm)
    laser_filter_cmd = Node(
        package='caramelo_navigation',
        executable='laser_scan_filter',
        name='laser_scan_filter',
        output='screen'
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
    
    # 6. BT Navigator
    bt_navigator_cmd = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_path]
    )
    
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
    
    # 9. Lifecycle Manager
    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'map_server',    # Mapa estático da arena  
                'amcl',          # Localização por filtro de partículas
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother'
            ]
        }]
    )
    
    # ==========================================================================
    # INTEGRAÇÃO E SISTEMA DE TASKS
    # ==========================================================================
    
    # 10. Twist Converter (Twist -> TwistStamped)
    twist_converter_cmd = Node(
        package='caramelo_bringup',
        executable='twist_converter_node',
        name='twist_converter_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # 11. Task Navigation Manager (Sistema BMT - Planejamento de Rota)
    task_navigation_manager = TimerAction(
        period=10.0,  # Aguardar Nav2 + AMCL inicializar
        actions=[
            Node(
                package='caramelo_navigation',
                executable='task_navigation_manager.py',
                name='task_navigation_manager',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'arena': arena,
                    'task_type': task,
                }]
            )
        ]
    )
    
    # ==========================================================================
    # VISUALIZAÇÃO
    # ==========================================================================
    
    # 12. RViz (visualização)
    rviz_cmd = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_path]
            )
        ]
    )
    
    # ==========================================================================
    # LOGS INFORMATIVOS
    # ==========================================================================
    
    startup_log = LogInfo(
        msg="🚀 CARAMELO NAVEGAÇÃO: Sistema de tasks iniciando..."
    )
    
    return LaunchDescription([
        # Argumentos
        declare_arena_cmd,
        declare_task_cmd,
        declare_use_sim_time_cmd,
        
        # Logs
        startup_log,
        
        # Nav2 Stack Completo (AMCL + Map Server)
        map_server_cmd,
        amcl_cmd,
        laser_filter_cmd,
        controller_cmd,
        planner_cmd,
        behaviors_cmd,
        bt_navigator_cmd,
        waypoint_follower_cmd,
        velocity_smoother_cmd,
        lifecycle_manager_cmd,
        
        # Integração
        twist_converter_cmd,
        
        # Sistema BMT (Planejamento + Execução)
        task_navigation_manager,
        
        # Visualização
        rviz_cmd,
    ])
