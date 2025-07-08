#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
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
    
    Para salvar o mapa (novo ambiente):
    1. mkdir -p ~/Caramelo_workspace/maps/nome_ambiente
    2. cd ~/Caramelo_workspace/maps/nome_ambiente
    3. ros2 run nav2_map_server map_saver_cli -f map
    
    Exemplo:
    mkdir -p ~/Caramelo_workspace/maps/sala_conferencia
    cd ~/Caramelo_workspace/maps/sala_conferencia
    ros2 run nav2_map_server map_saver_cli -f map
    """
    # Diretórios
    nav_dir = get_package_share_directory('caramelo_navigation')
    bringup_dir = get_package_share_directory('nav2_bringup')
    caramelo_bringup_dir = get_package_share_directory('caramelo_bringup')

    # Parâmetros
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # Argumentos de launch
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(nav_dir, 'config', 'slam_params.yaml'),
        description='Arquivo de parâmetros do SLAM')

    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(nav_dir, 'config', 'nav2_params.yaml'),
        description='Arquivo de parâmetros do Nav2')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(nav_dir, 'config', 'mapping.rviz'),
        description='Arquivo de configuração do RViz para mapeamento')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar tempo de simulação')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Autostart Nav2')

    # 0. LIDAR (precisa estar ativo para SLAM)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(caramelo_bringup_dir, 'launch', 'lidar_bringup.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items())

    # 1. SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                         'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': use_sim_time
        }.items())

    # 2. Nav2 (usando bringup completo mas sem map file)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': autostart,
            'map_subscribe_transient_local': 'true'
        }.items())

    # 3. Goal Pose Mapping Node
    goalpose_mapping_node = Node(
        package='caramelo_navigation',
        executable='goalpose_mapping',
        name='goalpose_mapping',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    # 4. Twist Converter
    twist_converter_cmd = Node(
        package='caramelo_bringup',
        executable='twist_converter_node',
        name='twist_converter_mapping',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel', '/cmd_vel')
        ])

    # 5. RViz (usando o visualization_bringup do caramelo_bringup)
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(caramelo_bringup_dir, 'launch', 'visualization_bringup.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz_config_file': os.path.join(caramelo_bringup_dir, 'rviz', 'caramelo_navigation.rviz')
        }.items())

    # 6. Navegação autônoma por waypoints após mapeamento
    waypoint_navigator_cmd = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='caramelo_navigation',
                executable='autonomous_waypoint_navigator',
                name='waypoint_navigator',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'waypoints_file': os.path.join(nav_dir, 'config', 'waypoints.json'),
                    'map_file': os.path.join(nav_dir, '..', '..', 'mapa_20250704_145039.yaml')
                }])
        ])

    return LaunchDescription([
        declare_slam_params_file_cmd,
        declare_nav2_params_file_cmd,
        declare_rviz_config_file_cmd,
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        lidar_launch,
        slam_launch,
        nav2_launch,
        goalpose_mapping_node,
        twist_converter_cmd,
        rviz_cmd,
        waypoint_navigator_cmd
    ])
