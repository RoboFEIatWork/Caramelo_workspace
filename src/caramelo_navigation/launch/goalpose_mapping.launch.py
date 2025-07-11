#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch file para mapeamento usando goals no RViz (clique para navegar).
    
    IMPORTANTE: Este launch NÃO inclui encoders e PWM!
    Execute separadamente em terminais diferentes:
    1. Terminal 1: ros2 launch caramelo_bringup encoder_bringup.launch.py
    2. Terminal 2: ros2 launch caramelo_bringup pwm_bringup.launch.py  
    3. Terminal 3: ros2 launch caramelo_navigation goalpose_mapping.launch.py
    
    Como usar:
    1. Clique em "2D Nav Goal" no RViz
    2. Clique no mapa onde quer que o robô vá
    3. O robô navegará para lá enquanto constrói o mapa
    4. Para salvar: ros2 run nav2_map_server map_saver_cli -f goalpose_map
    """
    
    # Configurações
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Declara argumentos de launch
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar tempo de simulação')

    # 1. Mapeamento SLAM (com LIDAR e filtro)
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('caramelo_navigation'),
                         'launch', 'mapping_launch.py'))
    )

    # 2. Twist Converter (dedicado para este fluxo - evita conflitos)
    twist_converter_node = Node(
        package='caramelo_bringup',
        executable='twist_converter_node',
        name='twist_converter_manual_mapping',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel_unstamped', '/cmd_vel'),
            ('/cmd_vel_stamped', '/cmd_vel_stamped_manual')
        ]
    )

    # 3. Navegador de waypoints simples
    simple_waypoint_navigator = Node(
        package='caramelo_navigation',
        executable='simple_waypoint_navigator',
        name='simple_waypoint_navigator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 4. Filtro de segurança cmd_vel
    cmd_vel_safety_filter = Node(
        package='caramelo_navigation',
        executable='cmd_vel_safety_filter',
        name='cmd_vel_safety_filter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 5. Monitor cmd_vel
    cmd_vel_monitor = Node(
        package='caramelo_navigation',
        executable='cmd_vel_monitor',
        name='cmd_vel_monitor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        mapping_launch,              # LIDAR + SLAM + RViz + filtro
        twist_converter_node,        # Twist converter dedicado
        simple_waypoint_navigator,   # Navegador por waypoints
        cmd_vel_safety_filter,       # Filtro de segurança
        cmd_vel_monitor,             # Monitor de velocidade
    ])
