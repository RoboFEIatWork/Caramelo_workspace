#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch para inicializar apenas a IMU da ZED 2i.
    
    Responsabilidades:
    - Publicar dados IMU da ZED em /zed/zed_node/imu/data
    - Publicar TF do sensor IMU
    - Configuração otimizada para performance
    
    Tópicos publicados:
    - /zed/zed_node/imu/data (sensor_msgs/Imu)
    """
    
    # Argumentos de launch
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar tempo de simulação')
    
    # Configuração para IMU da ZED
    config_path = os.path.join(
        get_package_share_directory('caramelo_navigation'),
        'config',
        'zed_imu_only.yaml'
    )
    
    # Nó da ZED otimizado apenas para IMU
    zed_imu_node = Node(
        package='zed_wrapper',
        executable='zed_wrapper',
        name='zed_node',
        output='screen',
        parameters=[
            config_path,
            {
                'use_sim_time': use_sim_time,
                'general.camera_model': 'zed2i',
                'general.publish_tf': False,  # EKF vai gerenciar TF
                'general.publish_map_tf': False,
                'pos_tracking.pos_tracking_enabled': False,
                'sensors.publish_imu_tf': True,
                'sensors.sensors_pub_rate': 100.0,  # 100Hz para IMU
                'video.grab_frame_rate': 15,  # Baixo FPS para economizar
            }
        ],
        remappings=[
            ('~/imu/data', '/zed/zed_node/imu/data'),
        ]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        zed_imu_node,
    ])
