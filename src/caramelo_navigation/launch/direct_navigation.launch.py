#!/usr/bin/env python3
"""
Launch file para navegação básica com servidor direto
Inclui Nav2 stack completo + workstation navigation server direto
Contorna problemas com bt_navigator
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Argumentos de launch
    arena_arg = DeclareLaunchArgument(
        'arena',
        default_value='arena_robocup25',
        description='Nome da arena/mapa a ser usado'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar tempo de simulação'
    )
    
    # Obter parâmetros
    arena = LaunchConfiguration('arena')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Paths
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    caramelo_nav_dir = get_package_share_directory('caramelo_navigation')
    
    # Arquivos de configuração
    map_file = PathJoinSubstitution([
        FindPackageShare('caramelo_navigation'),
        'maps',
        arena,
        'map.yaml'
    ])
    
    nav2_params_file = os.path.join(caramelo_nav_dir, 'config', 'nav2_params.yaml')
    rviz_config_file = os.path.join(caramelo_nav_dir, 'rviz', 'nav2_default_view.rviz')
    
    # === INFORMAÇÕES DO SISTEMA ===
    system_info = LogInfo(
        msg=[
            '\n🚀 CARAMELO NAVIGATION SYSTEM (DIRECT MODE)',
            '\n   Arena: ', arena,
            '\n   Map: ', map_file,
            '\n   Config: ', nav2_params_file,
            '\n',
            '\n📋 SISTEMA INCLUÍDO:',
            '\n   ✅ Map Server (carrega mapa)',
            '\n   ✅ AMCL (localização)',
            '\n   ✅ Nav2 Planner (planejamento de rota)',
            '\n   ✅ Nav2 Controller (controle de movimento)',
            '\n   ✅ Nav2 Recoveries (recuperação de falhas)',
            '\n   ❌ Nav2 BT Navigator (desabilitado - causa conflitos)',
            '\n   ✅ Workstation Navigation Server (modo direto)',
            '\n   ✅ RViz2 (visualização)',
            '\n',
            '\n📡 TÓPICOS DISPONÍVEIS:',
            '\n   /navigate_to_workstation (std_msgs/String) - Enviar comando',
            '\n   /navigation_status (std_msgs/String) - Status da navegação',
            '\n',
            '\n🧪 PARA TESTAR:',
            '\n   ros2 run caramelo_navigation navigation_test_client_simple WS01',
            '\n   ros2 topic pub --once /navigate_to_workstation std_msgs/String "data: WS07"',
            '\n',
            '\n⚠️  IMPORTANTE: Este launch NÃO inclui hardware!',
            '\n   Execute separadamente: encoders, PWM, LIDAR, base_controller',
            '\n'
        ]
    )
    
    # === NAV2 STACK (SEM BT_NAVIGATOR) ===
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'true'
        }.items()
    )
    
    # === WORKSTATION NAVIGATION SERVER (DIRECT) ===
    workstation_server = Node(
        package='caramelo_navigation',
        executable='workstation_navigation_server_direct',
        name='workstation_navigation_server_direct',
        parameters=[
            {'arena': arena},
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )
    
    # === RVIZ2 ===
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        arena_arg,
        use_sim_time_arg,
        system_info,
        nav2_bringup,
        workstation_server,
        rviz_node,
    ])
