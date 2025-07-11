#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch file para navegação autônoma usando Nav2 Simple Commander e waypoints JSON.
    
    IMPORTANTE: Este launch NÃO inclui encoders e PWM!
    Execute separadamente em terminais diferentes:
    1. Terminal 1: ros2 launch caramelo_bringup encoder_bringup.launch.py
    2. Terminal 2: ros2 launch caramelo_bringup pwm_bringup.launch.py  
    3. Terminal 3: ros2 launch caramelo_navigation autonomous_navigation_v2.launch.py map_folder:=arena_fei
    
    Como usar:
    1. Certifique-se que o mapa e waypoints_simple.json existem na pasta especificada
    2. O robô navegará automaticamente pelos waypoints usando Nav2 Simple Commander
    3. Use LIDAR para evitar obstáculos dinâmicos
    """
    
    # Configurações
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_folder = LaunchConfiguration('map_folder', default='arena_fei')
    loop_mission = LaunchConfiguration('loop_mission', default='false')
    
    # Argumentos de launch
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar tempo de simulação')
        
    declare_map_folder_cmd = DeclareLaunchArgument(
        'map_folder',
        default_value='arena_fei',
        description='Pasta com mapa e waypoints (dentro de ~/Caramelo_workspace/maps/)')
        
    declare_loop_mission_cmd = DeclareLaunchArgument(
        'loop_mission',
        default_value='false',
        description='Repetir missão em loop')

    # ===============================================
    # 1. TRANSFORMAÇÕES TF ESSENCIAIS
    # ===============================================
    
    # Robot Description (URDF) - NECESSÁRIO para TF!
    caramelo_bringup_path = FindPackageShare('caramelo_bringup')
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            caramelo_bringup_path,
            "urdf",
            "caramelo_real.urdf.xacro"
        ])
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}
    
    # Robot State Publisher - publica TF do URDF (base_footprint -> base_link, etc.)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        respawn=True
    )
    
    # Odom TF Publisher - publica odom -> base_footprint
    odom_tf_publisher = Node(
        package='caramelo_bringup',
        executable='odom_tf_publisher_node',
        name='odom_tf_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'initial_x': 0.0,
            'initial_y': 0.0,
            'initial_z': 0.0,
            'parent_frame': 'odom',
            'child_frame': 'base_footprint',
            'publish_rate': 50.0
        }]
    )

    # ===============================================
    # 2. MAPA E LOCALIZAÇÃO
    # ===============================================
    
    # Caminhos do mapa
    map_yaml_file = PathJoinSubstitution([
        '/home/work/Caramelo_workspace/maps',
        map_folder,
        'map.yaml'
    ])
    
    # Parâmetros do Nav2
    nav2_params_file = os.path.join(
        get_package_share_directory('caramelo_navigation'),
        'config', 'nav2_params_minimal.yaml'
    )
    
    # Nav2 Bringup completo (Map Server + AMCL + Navigation)
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'true',
            'use_composition': 'True'
        }.items()
    )

    # ===============================================
    # 3. SENSORES
    # ===============================================
    
    # LIDAR para evitação de obstáculos - COM INVERSÃO
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
            'inverted': True,  # LIDAR está de ponta cabeça no robô real
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('/scan', '/scan_raw')  # Publicar no tópico raw
        ]
    )

    # Filtro de LIDAR para usar apenas 90° a 270°
    laser_filter_node = Node(
        package='caramelo_navigation',
        executable='laser_scan_filter',
        name='laser_scan_filter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Static Transform: base_link -> laser (sem rotação - inverted=true corrige orientação)
    static_transform_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_laser',
        arguments=['0.0', '0.0', '0.1', '0.0', '0.0', '0.0', 'base_link', 'laser_frame'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ===============================================
    # 4. CONTROLE
    # ===============================================
    
    # Twist Converter para mecanum drive
    twist_converter_node = Node(
        package='caramelo_bringup',
        executable='twist_converter_node',
        name='twist_converter_autonomous',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel_unstamped', '/cmd_vel'),  # Recebe do Nav2
            ('/cmd_vel_stamped', '/mecanum_drive_controller/cmd_vel')  # Publica para mecanum drive
        ]
    )

    # ===============================================
    # 5. NAVEGAÇÃO AUTÔNOMA
    # ===============================================
    
    # Navegador autônomo de waypoints - COM DELAY PARA AGUARDAR NAV2
    autonomous_navigator = TimerAction(
        period=15.0,  # Aguardar 15 segundos para Nav2 estar completamente ativo
        actions=[
            Node(
                package='caramelo_navigation',
                executable='autonomous_waypoint_navigator',
                name='autonomous_waypoint_navigator',
                output='screen',
                parameters=[{
                    'map_folder': map_folder,
                    'loop_mission': loop_mission
                }]
            )
        ]
    )

    # ===============================================
    # 6. VISUALIZAÇÃO
    # ===============================================
    
    # RViz para visualização (usando config padrão do Nav2)
    rviz_config_file = PathJoinSubstitution([
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz'
    ])
    
    rviz_node = TimerAction(
        period=8.0,  # Espera 8 segundos para Nav2 inicializar
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_file],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )

    # ===============================================
    # 7. SEGURANÇA
    # ===============================================
    
    # Monitor de segurança
    safety_monitor = Node(
        package='caramelo_navigation',
        executable='cmd_vel_monitor',
        name='safety_monitor',
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_map_folder_cmd,
        declare_loop_mission_cmd,
        
        # Transformações TF essenciais
        robot_state_publisher,
        odom_tf_publisher,
        
        # Nav2 completo (Map + AMCL + Navigation)
        nav2_bringup_launch,
        
        # Sensores
        rplidar_node,
        laser_filter_node,
        static_transform_laser,
        
        # Controle
        twist_converter_node,
        
        # Navegação autônoma
        autonomous_navigator,
        
        # Visualização
        rviz_node,
        
        # Segurança
        safety_monitor,
    ])
