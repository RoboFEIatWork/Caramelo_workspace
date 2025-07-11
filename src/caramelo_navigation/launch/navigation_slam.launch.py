#!/usr/bin/env python3
"""
NAVIGATION WITH SLAM LAUNCH - CARAMELO NAVIGATION
=================================================

Launch file completo para navegação com SLAM baseado no tutorial Automatic Addison.
Inicia todo o stack Nav2 + SLAM + RViz para navegação autônoma em tempo real.

Usage:
    ros2 launch caramelo_navigation navigation_slam.launch.py

Features:
- SLAM Toolbox para mapeamento em tempo real
- Stack Nav2 completo (controller, planner, behavior, bt_navigator)
- Map server dinâmico conectado ao SLAM
- AMCL desabilitado (não necessário durante SLAM)
- RViz com configuração de navegação
- Velocity relay para comunicação com mecanum drive controller

Como usar:
1. Execute o launch file
2. Aguarde o RViz abrir
3. Use "2D Nav Goal" para navegar e mapear simultaneamente
4. O robô navega evitando obstáculos enquanto constrói o mapa
5. Salve o mapa quando terminar o mapeamento

IMPORTANTE: Para navegação com mapa existente, use navigation_launch.py
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    
    # Configurações
    caramelo_nav_dir = get_package_share_directory('caramelo_navigation')
    caramelo_desc_dir = get_package_share_directory('caramelo_description')
    
    # Parâmetros
    nav2_params_file = os.path.join(caramelo_nav_dir, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(caramelo_nav_dir, 'config', 'slam_params.yaml')
    rviz_config_file = os.path.join(caramelo_nav_dir, 'rviz', 'navigation_slam.rviz')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    
    # Argumentos de launch
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation time')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true', description='Automatically startup the nav2 stack')
    
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='true', description='Whether to use composed bringup')
    
    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container', description='Container name for composition')
    
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='false', description='Whether to respawn if a node crashes')
    
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level')
    
    # URDF do robô
    urdf_file = os.path.join(caramelo_desc_dir, 'URDF', 'robot.urdf.xacro')
    
    # Map fully qualified names to relative ones
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    
    # Configuração dos parâmetros
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params_file,
            root_key='',
            param_rewrites={},
            convert_types=True),
        allow_substs=True)
    
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    
    # 1. SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
        remappings=remappings,
    )
    
    # 2. Velocity relay node (Twist -> TwistStamped)
    velocity_relay_node = Node(
        package='caramelo_navigation',
        executable='twist_converter_node',
        name='twist_converter_nav',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': '/cmd_vel',
            'output_topic': '/mecanum_drive_controller/cmd_vel'
        }],
    )
    
    # 3. Nós composáveis do Nav2
    composable_nodes = [
        ComposableNode(
            package='nav2_controller',
            plugin='nav2_controller::ControllerServer',
            name='controller_server',
            parameters=[configured_params],
            remappings=remappings),
        ComposableNode(
            package='nav2_smoother',
            plugin='nav2_smoother::SmootherServer',
            name='smoother_server',
            parameters=[configured_params],
            remappings=remappings),
        ComposableNode(
            package='nav2_planner',
            plugin='nav2_planner::PlannerServer',
            name='planner_server',
            parameters=[configured_params],
            remappings=remappings),
        ComposableNode(
            package='nav2_behaviors',
            plugin='behavior_server::BehaviorServer',
            name='behavior_server',
            parameters=[configured_params],
            remappings=remappings),
        ComposableNode(
            package='nav2_bt_navigator',
            plugin='nav2_bt_navigator::BtNavigator',
            name='bt_navigator',
            parameters=[configured_params],
            remappings=remappings),
        ComposableNode(
            package='nav2_waypoint_follower',
            plugin='nav2_waypoint_follower::WaypointFollower',
            name='waypoint_follower',
            parameters=[configured_params],
            remappings=remappings),
        ComposableNode(
            package='nav2_velocity_smoother',
            plugin='nav2_velocity_smoother::VelocitySmoother',
            name='velocity_smoother',
            parameters=[configured_params],
            remappings=remappings),
        ComposableNode(
            package='nav2_collision_monitor',
            plugin='nav2_collision_monitor::CollisionMonitor',
            name='collision_monitor',
            parameters=[configured_params],
            remappings=remappings),
        ComposableNode(
            package='nav2_lifecycle_manager',
            plugin='nav2_lifecycle_manager::LifecycleManager',
            name='lifecycle_manager_navigation',
            parameters=[{
                'autostart': autostart,
                'node_names': [
                    'controller_server',
                    'smoother_server', 
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother',
                    'collision_monitor'
                ]
            }]),
    ]
    
    # 4. Container para nós composáveis
    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name,
        composable_node_descriptions=composable_nodes,
    )
    
    # 5. Container
    container = Node(
        condition=IfCondition(use_composition),
        name=container_name,
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[configured_params, {'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
        output='screen',
    )
    
    # 6. RViz - aguarda 5 segundos para tudo inicializar
    rviz_delayed = TimerAction(
        period=8.0,  # Aguarda 8 segundos
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_file] if Path(rviz_config_file).exists() 
                          else [],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        # Ambiente
        stdout_linebuf_envvar,
        
        # Argumentos
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        declare_use_composition_cmd,
        declare_container_name_cmd,
        declare_use_respawn_cmd,
        declare_log_level_cmd,
        
        # Nós
        slam_toolbox_node,
        velocity_relay_node,
        container,
        load_composable_nodes,
        rviz_delayed,
    ])


if __name__ == '__main__':
    generate_launch_description()
