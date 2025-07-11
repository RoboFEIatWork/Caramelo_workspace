#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """
    Launch COMPLETO para navegação autônoma com mapa estático.
    
    O robô:
    - Inicia sempre na posição (0,0) do mapa
    - USA LIDAR APENAS para localização (AMCL)
    - USA APENAS o mapa estático para obstacle avoidance
    - Navega pelos waypoints definidos no JSON da arena
    
    ANTES DE RODAR:
    1. Terminal 1: ros2 launch caramelo_bringup encoder_bringup.launch.py
    2. Terminal 2: ros2 launch caramelo_bringup pwm_bringup.launch.py  
    3. Terminal 3: este launch
    
    EXEMPLOS DE USO:
    ros2 launch caramelo_navigation autonomous_map_navigation.launch.py arena:=arena_fei
    ros2 launch caramelo_navigation autonomous_map_navigation.launch.py arena:=arena_robocup25 waypoint_file:=waypoints.json
    """
    
    # Parâmetros configuráveis
    arena = LaunchConfiguration('arena', default='arena_fei')
    waypoint_file = LaunchConfiguration('waypoint_file', default='waypoints.json')
    loop_mission = LaunchConfiguration('loop_mission', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # Argumentos do launch
    declare_arena = DeclareLaunchArgument(
        'arena',
        default_value='arena_fei',
        description='Nome da pasta da arena dentro de maps/ (ex: arena_fei, arena_robocup25)'
    )
    
    declare_waypoint_file = DeclareLaunchArgument(
        'waypoint_file',
        default_value='waypoints.json',
        description='Nome do arquivo de waypoints dentro da pasta da arena'
    )
    
    declare_loop_mission = DeclareLaunchArgument(
        'loop_mission',
        default_value='false',
        description='Se true, repete a missão continuamente'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Se true, abre o RViz'
    )

    # Caminhos dinâmicos baseados na arena
    maps_base_path = '/home/work/Caramelo_workspace/maps'
    
    # ========================================
    # 1. NAV2 BRINGUP (MAPA ESTÁTICO APENAS)
    # ========================================
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch'),
            '/bringup_launch.py'
        ]),
        launch_arguments=[
            ('map', [maps_base_path, '/', arena, '/map.yaml']),
            ('params_file', '/home/work/Caramelo_workspace/src/caramelo_navigation/config/nav2_static_map_only.yaml'),
            ('use_sim_time', 'false'),
            ('autostart', 'true')
        ],
    )

    # ========================================
    # 2. SENSORES E TF
    # ========================================
    
    # LIDAR (APENAS para localização)
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

    # Filtro de LIDAR (180° frontal para melhor localização)
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
    # 4. CONTROLE DE MOVIMENTO
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
    # 5. NAVEGADOR AUTÔNOMO (MAPA ESTÁTICO)
    # ========================================
    map_waypoint_navigator = TimerAction(
        period=10.0,  # 10 segundos de delay para Nav2 estar completamente pronto
        actions=[
            Node(
                package='caramelo_navigation',
                executable='map_based_waypoint_navigator',
                name='map_based_waypoint_navigator',
                output='screen',
                parameters=[{
                    'arena': arena,
                    'waypoint_file': waypoint_file,
                    'loop_mission': loop_mission
                }]
            )
        ]
    )

    # ========================================
    # 6. RVIZ (OPCIONAL)
    # ========================================
    rviz_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', '/opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz'],
                output='screen',
                condition=IfCondition(use_rviz)
            )
        ]
    )

    return LaunchDescription([
        declare_arena,
        declare_waypoint_file,
        declare_loop_mission,
        declare_use_rviz,
        
        # Nodes principais
        nav2_bringup,
        rplidar_node,
        laser_scan_filter_node,
        laser_tf,
        robot_state_publisher,
        twist_converter,
        map_waypoint_navigator,
        rviz_node
    ])
