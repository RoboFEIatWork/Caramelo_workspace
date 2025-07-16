#!/usr/bin/env python3
"""
SLAM TELEOP MAPPING - SISTEMA IMPECÁVEL
Launch otimizado para mapeamento por teleoperação com RPLIDAR S2
- SLAM Toolbox configurado para odometria ruim
- Filtro de laser scan otimizado para RPLIDAR S2  
- Tolerante a delays USB/ESP32
- Sem dependências de safety filter
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Argumentos
    use_sim_time = LaunchConfiguration('use_sim_time')
    arena = LaunchConfiguration('arena')
    
    # Caminhos
    pkg_nav = get_package_share_directory('caramelo_navigation')
    
    # Argumentos de launch
    declare_arena_cmd = DeclareLaunchArgument(
        'arena',
        default_value='arena_robocup25',
        description='Nome da arena para salvar o mapa'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # ==========================================================================
    # HARDWARE LAUNCHES (usando launches existentes com parâmetros)
    # ==========================================================================
    
    # 1. RPLIDAR S2 Launch (com filtros e parâmetros otimizados)
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('caramelo_bringup'),
                'launch',
                'lidar_bringup.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # 2. Encoder Launch (odometria)
    encoder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('caramelo_bringup'),
                'launch',
                'encoder_bringup.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # 3. PWM/Movement Launch  
    pwm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('caramelo_bringup'),
                'launch',
                'pwm_bringup.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # ==========================================================================
    # SISTEMA BÁSICO PARA MAPEAMENTO IMPECÁVEL
    # ==========================================================================
    
    # 1. Twist Converter (ESSENCIAL para movimento físico)
    twist_converter_node = Node(
        package='caramelo_bringup',
        executable='twist_converter_node',
        name='twist_converter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 2. SLAM Toolbox (OTIMIZADO para RPLIDAR S2 + odometria ruim)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg_nav, 'config', 'slam_params.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/scan', '/scan_filtered')  # Usa scan filtrado
        ]
    )
    
    # 3. Filtro de Laser Scan (OTIMIZADO para RPLIDAR S2)
    laser_filter_node = Node(
        package='caramelo_navigation',
        executable='laser_scan_filter',
        name='laser_scan_filter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('input_scan', '/scan'),
            ('filtered_scan', '/scan_filtered')
        ]
    )
    
    # 4. Monitor de CMD_VEL (básico)
    cmd_vel_monitor_node = Node(
        package='caramelo_navigation',
        executable='cmd_vel_monitor',
        name='cmd_vel_monitor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 5. Teleop Keyboard para mapeamento manual
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 6. RViz para visualização
    rviz_config_file = os.path.join(pkg_nav, 'rviz', 'slam_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 7. Map Saver (para salvar mapas)
    map_saver_node = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_saver',
        output='screen',
        arguments=[
            '-f', PathJoinSubstitution([
                '/home/work/Caramelo_workspace/maps',
                arena,
                'map'
            ])
        ],
        condition=IfCondition('false')  # Só ativa quando necessário
    )
    
    # ==========================================================================
    # INFORMAÇÕES DE INICIALIZAÇÃO
    # ==========================================================================
    
    info_start = LogInfo(msg="🗺️ SLAM TELEOP MAPPING: Sistema iniciando com TUDO incluído...")
    info_ready = LogInfo(msg="🎮 Sistema completo pronto! RPLIDAR + Encoder + PWM + SLAM + Teleop!")
    info_controls = LogInfo(msg="📋 Controles: i=frente, k=parar, j=esquerda, l=direita, u=diagonal, o=diagonal")
    info_save = LogInfo(msg="💾 Para salvar: ros2 run nav2_map_server map_saver_cli -f ~/Caramelo_workspace/maps/SEU_MAPA")
    
    # ==========================================================================
    # LAUNCH DESCRIPTION
    # ==========================================================================
    
    ld = LaunchDescription()
    
    # Argumentos
    ld.add_action(declare_arena_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    
    # Informações
    ld.add_action(info_start)
    
    # Hardware launches (com parâmetros e filtros)
    ld.add_action(rplidar_launch)
    ld.add_action(encoder_launch)
    ld.add_action(pwm_launch)
    
    # Nodes principais
    ld.add_action(twist_converter_node)
    ld.add_action(laser_filter_node)
    ld.add_action(slam_toolbox_node)
    ld.add_action(cmd_vel_monitor_node)
    ld.add_action(teleop_node)
    ld.add_action(rviz_node)
    ld.add_action(map_saver_node)
    
    # Informações finais
    ld.add_action(info_ready)
    ld.add_action(info_controls)
    ld.add_action(info_save)
    
    return ld
