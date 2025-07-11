#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """
    Launch SIMPLES para navegação autônoma com waypoints.
    
    ANTES DE RODAR:
    1. Terminal 1: ros2 launch caramelo_bringup encoder_bringup.launch.py
    2. Terminal 2: ros2 launch caramelo_bringup pwm_bringup.launch.py  
    3. Terminal 3: este launch
    """
    
    # Parâmetros
    map_folder = LaunchConfiguration('map_folder', default='arena_fei')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Argumentos
    declare_map_folder = DeclareLaunchArgument(
        'map_folder',
        default_value='arena_fei',
        description='Pasta do mapa'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar sim time'
    )

    # ========================================
    # 1. MAPA E LOCALIZAÇÃO (Nav2 padrão)
    # ========================================
    
    # Usar o Nav2 bringup padrão que JÁ FUNCIONA
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch'),
            '/bringup_launch.py'
        ]),
        launch_arguments=[(k, v) for k, v in {
            'map': '/home/work/Caramelo_workspace/maps/arena_fei/map.yaml',
            'params_file': '/home/work/Caramelo_workspace/src/caramelo_navigation/config/nav2_params_minimal.yaml',
            'use_sim_time': 'false',
            'autostart': 'true'
        }.items()],
    )

    # ========================================
    # 2. LIDAR
    # ========================================
    
    # LIDAR
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB2',
            'serial_baudrate': 1000000,
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': 'Standard',
            'inverted': True,
            'use_sim_time': False
        }]
    )

    # TF do laser
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf',
        arguments=['0.292', '0', '0.026', '0.0', '0.0', '0.0', 'base_link', 'laser_frame']
    )

    # ========================================
    # 2.1. FILTRO DE LIDAR (90° a 270°)
    # ========================================
    laser_scan_filter_node = Node(
        package='caramelo_navigation',
        executable='laser_scan_filter',
        name='laser_scan_filter',
        output='screen'
    )

    # ========================================
    # 3. ROBOT DESCRIPTION
    # ========================================
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', '/home/work/Caramelo_workspace/src/caramelo_bringup/urdf/caramelo_real.urdf.xacro']),
                value_type=str
            )
        }]
    )

    # ========================================
    # 4. TWIST CONVERTER
    # ========================================
    
    # Converter para mecanum drive
    twist_converter = Node(
        package='caramelo_bringup',
        executable='twist_converter_node',
        name='twist_converter',
        output='screen',
        remappings=[
            ('/cmd_vel_unstamped', '/cmd_vel'),
            ('/cmd_vel_stamped', '/mecanum_drive_controller/cmd_vel')
        ]
    )

    # ========================================
    # 5. NAVEGADOR AUTÔNOMO
    # ========================================
    
    # Navegador de waypoints (com delay para Nav2 estar pronto)
    waypoint_navigator = TimerAction(
        period=10.0,  # 10 segundos de delay
        actions=[
            Node(
                package='caramelo_navigation',
                executable='autonomous_waypoint_navigator',
                name='waypoint_navigator',
                output='screen',
                parameters=[{
                    'map_folder': map_folder
                }]
            )
        ]
    )

    # ========================================
    # 6. RVIZ (opcional)
    # ========================================
    
    rviz = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', '/opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz'],
                output='screen'
            )
        ]
    )

    # Removido global_localization_service automático para evitar crash
    return LaunchDescription([
        declare_map_folder,
        declare_use_sim_time,
        nav2_bringup,
        rplidar_node,
        laser_scan_filter_node,
        laser_tf,
        robot_state_publisher,
        twist_converter,
        waypoint_navigator,
        rviz
    ])
