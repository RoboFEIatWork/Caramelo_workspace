#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file para mapeamento do ambiente com SLAM.
    
    IMPORTANTE: Este launch NÃO inclui encoder e PWM!
    Execute separadamente em terminais diferentes:
    1. Terminal 1: ros2 launch caramelo_bringup encoder_bringup.launch.py
    2. Terminal 2: ros2 launch caramelo_bringup pwm_bringup.launch.py
    3. Terminal 3: ros2 launch caramelo_navigation mapping_launch.py
    
    Este launch inicia:
    - LIDAR (rplidar) - DADOS BRUTOS como sensor de proximidade
    - SLAM Toolbox - configurado para RPLidar S2
    - RViz para visualização
    """
    
    # Configurações
    package_name = 'caramelo_navigation'
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declara argumentos de launch
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory(package_name),
                                   'config', 'slam_params.yaml'),
        description='Arquivo de parâmetros do SLAM')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar tempo de simulação')

    # LIDAR (rplidar oficial) - publica em /scan
    start_lidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rplidar_ros'),
                         'launch', 'rplidar_s2_launch.py')),
        launch_arguments={
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB2',
            'serial_baudrate': '1000000',
            'frame_id': 'laser_frame',
            'inverted': 'true',
            'angle_compensate': 'true',
            'scan_mode': 'DenseBoost'
        }.items()
    )

    # SEM FILTRO LIDAR - RPLidar S2 é muito bom, usar dados brutos
    # Também usamos como sensor de proximidade (não ignorar < 15cm)

    # Robot Localization EKF - DESABILITADO para mapeamento mais limpo
    # start_ekf_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory(package_name),
    # Robot Localization EKF - DESABILITADO para mapeamento mais limpo
    # start_ekf_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory(package_name),
    #                      'launch', 'ekf_launch.py'))
    # )

    # NOTA: twist_converter_node foi removido daqui pois já está incluído
    # no teleop_keyboard.launch.py que é chamado pelo teleop_mapping.launch.py
    # Evita conflito de nós com mesmo nome rodando simultaneamente

    # SLAM Toolbox - USA dados brutos do LIDAR
    start_slam_toolbox_node = Node(
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
        # Usa /scan diretamente (dados brutos do RPLidar S2)
    )

    # Lifecycle Manager para SLAM
    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{'autostart': True},
                    {'node_names': ['slam_toolbox']}]
    )

    # RViz com configuração para mapeamento
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('caramelo_bringup'), 'rviz', 'caramelo_navigation.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Cria a descrição de launch
    ld = LaunchDescription()

    # Declara argumentos de launch
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Adiciona ações
    ld.add_action(start_lidar_cmd)           # LIDAR (dados brutos, sem filtro)
    # ld.add_action(start_ekf_cmd)            # EKF - DESABILITADO
    # twist_converter incluído no teleop_keyboard.launch (via teleop_mapping)
    ld.add_action(start_slam_toolbox_node)   # SLAM (usa scan bruto)
    ld.add_action(start_lifecycle_manager_cmd) # Lifecycle manager
    ld.add_action(start_rviz_cmd)            # RViz

    return ld
