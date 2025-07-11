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
    Launch SIMPLES para navegação até um ponto específico.
    
    ANTES DE RODAR:
    1. Terminal 1: ros2 launch caramelo_bringup encoder_bringup.launch.py
    2. Terminal 2: ros2 launch caramelo_bringup pwm_bringup.launch.py  
    3. Terminal 3: este launch
    """
    
    # Parâmetros
    goal_x = LaunchConfiguration('goal_x', default='2.0')
    goal_y = LaunchConfiguration('goal_y', default='1.0')
    goal_yaw = LaunchConfiguration('goal_yaw', default='0.0')
    
    # Argumentos
    declare_goal_x = DeclareLaunchArgument(
        'goal_x',
        default_value='2.0',
        description='Coordenada X do objetivo'
    )
    
    declare_goal_y = DeclareLaunchArgument(
        'goal_y',
        default_value='1.0',
        description='Coordenada Y do objetivo'
    )
    
    declare_goal_yaw = DeclareLaunchArgument(
        'goal_yaw',
        default_value='0.0',
        description='Rotação do objetivo (radianos)'
    )

    # ========================================
    # 1. NAV2 BRINGUP (com mapa e parâmetros)
    # ========================================
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch'),
            '/bringup_launch.py'
        ]),
        launch_arguments=[
            ('map', '/home/work/Caramelo_workspace/maps/arena_fei/map.yaml'),
            ('params_file', '/home/work/Caramelo_workspace/src/caramelo_navigation/config/nav2_params_clean.yaml'),
            ('use_sim_time', 'false'),
            ('autostart', 'true')
        ],
    )

    # ========================================
    # 2. LIDAR E FILTRO
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

    # Filtro de LIDAR
    laser_scan_filter_node = Node(
        package='caramelo_navigation',
        executable='laser_scan_filter',
        name='laser_scan_filter',
        output='screen'
    )

    # ========================================
    # 3. ROBOT DESCRIPTION
    # ========================================
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
    # 5. NAVEGADOR SIMPLES
    # ========================================
    simple_navigator = TimerAction(
        period=8.0,  # 8 segundos de delay para Nav2 estar pronto
        actions=[
            Node(
                package='caramelo_navigation',
                executable='simple_goal_navigator',
                name='simple_goal_navigator',
                output='screen',
                parameters=[{
                    'goal_x': goal_x,
                    'goal_y': goal_y,
                    'goal_yaw': goal_yaw
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

    return LaunchDescription([
        declare_goal_x,
        declare_goal_y,
        declare_goal_yaw,
        nav2_bringup,
        rplidar_node,
        laser_scan_filter_node,
        laser_tf,
        robot_state_publisher,
        twist_converter,
        simple_navigator,
        rviz
    ])
