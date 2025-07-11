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
    Launch file para navegação autônoma usando mapa existente e waypoints.
    
    IMPORTANTE: Este launch NÃO inclui encoders e PWM!
    Execute separadamente em terminais diferentes:
    1. Terminal 1: ros2 launch caramelo_bringup encoder_bringup.launch.py
    2. Terminal 2: ros2 launch caramelo_bringup pwm_bringup.launch.py  
    3. Terminal 3: ros2 launch caramelo_navigation autonomous_navigation.launch.py map_folder:=arena_fei
    
    Como usar:
    1. Certifique-se que o mapa e waypoints.json existem na pasta especificada
    2. O robô navegará automaticamente por todos os waypoints
    3. Use LIDAR para evitar obstáculos dinâmicos
    4. O sistema para em caso de emergência
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
    # 0. TRANSFORMAÇÕES TF ESSENCIAIS (faltavam!)
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

    # 1. Nav2 navigation nodes INDIVIDUAIS (sem collision_monitor e docking_server)
    # Aqui vamos lançar apenas os nós essenciais do Nav2
    nav2_bringup_params = os.path.join(
        get_package_share_directory('caramelo_navigation'),
        'config', 'nav2_params_minimal.yaml'
    )
    
    # Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_bringup_params, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', '/cmd_vel')]  # Para o twist_converter
    )
    
    # Planner Server  
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_bringup_params, {'use_sim_time': use_sim_time}]
    )
    
    # Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_bringup_params, {'use_sim_time': use_sim_time}]
    )
    
    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_bringup_params, {'use_sim_time': use_sim_time}]
    )
    
    # Waypoint Follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_bringup_params, {'use_sim_time': use_sim_time}]
    )
    
    # Velocity Smoother
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_bringup_params, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', '/cmd_vel'),
                   ('/cmd_vel_smoothed', '/cmd_vel')]
    )
    
    # Lifecycle Manager for Navigation
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[nav2_bringup_params,
                   {'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': ['controller_server', 
                                   'planner_server',
                                   'behavior_server',
                                   'bt_navigator',
                                   'waypoint_follower', 
                                   'velocity_smoother']}]
    )

    # 1.1 Map server e AMCL usando localization_launch
    map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'localization_launch.py')),
        launch_arguments={
            'map': ['/home/work/Caramelo_workspace/maps/', map_folder, '/map.yaml'],
            'use_sim_time': use_sim_time,
            'params_file': nav2_bringup_params,
            'autostart': 'true'
        }.items()
    )

    # 3. LIDAR para evitação de obstáculos - COM INVERSÃO
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

    # 3.1. Filtro de LIDAR para usar apenas 90° a 270° - COM TAXA REDUZIDA
    laser_filter_node = Node(
        package='caramelo_navigation',
        executable='laser_scan_filter',
        name='laser_scan_filter',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_rate': 5.0  # Reduzir taxa de publicação para 5Hz
        }]
    )

    # 4. Static Transform: base_link -> laser - COM TOLERÂNCIA MAIOR
    static_transform_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_laser',
        arguments=['0.0', '0.0', '0.1', '0.0', '0.0', '0.0', 'base_link', 'laser_frame'],
        parameters=[{
            'use_sim_time': use_sim_time,
            'transform_tolerance': 1.0  # Aumentar tolerância para 1 segundo
        }]
    )

    # 5. RViz para visualização - COM DELAY MAIOR E CONFIGURAÇÕES OTIMIZADAS
    rviz_config_file = PathJoinSubstitution([
        get_package_share_directory('caramelo_navigation'),
        'rviz',
        'autonomous_navigation.rviz'
    ])
    
    rviz_node = TimerAction(
        period=10.0,  # Esperar 10 segundos para tudo estar estabilizado
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_file],
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'tf_buffer_size': 120,  # Buffer maior para TF
                    'transform_timeout': 2.0  # Timeout maior para transforms
                }],
                output='screen'
            )
        ]
    )

    # 6. Twist Converter para mecanum drive - IGUAL AO GOALPOSE
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

    # 7. Navegador autônomo de waypoints - COM DELAY MAIOR
    # Aguardar Nav2 stack estar completamente ativo antes de iniciar navegação
    autonomous_navigator = TimerAction(
        period=35.0,  # Delay aumentado para 35 segundos
        actions=[
            Node(
                package='caramelo_navigation',
                executable='autonomous_waypoint_navigator',
                name='autonomous_waypoint_navigator',
                output='screen',
                parameters=[{
                    'map_folder': map_folder,
                    'loop_mission': loop_mission,
                    'use_sim_time': use_sim_time
                }]
            )
        ]
    )

    # 8. Monitor de segurança
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
        
        # Transformações TF essenciais (estavam faltando!)
        robot_state_publisher,
        odom_tf_publisher,
        
        # Map Server e AMCL
        map_server_launch,
        
        # Nav2 navigation nodes individuais (sem collision_monitor)
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager_navigation,
        
        # Sensores
        rplidar_node,
        laser_filter_node,
        static_transform_laser,
        
        # Visualização
        rviz_node,
        
        # Controle
        twist_converter_node,
        autonomous_navigator,
        safety_monitor,
    ])
