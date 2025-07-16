#!/usr/bin/env python3
"""
CARAMELO SENSORS BRINGUP - Todos os Sensores do Robô Caramelo

SENSORES INCLUÍDOS:
==================
1. ESP32 Encoders - Leitura de posição das 4 rodas mecanum
2. RPLIDAR S2 - Laser scan 360° para navegação
3. ZED2i IMU - Orientação e aceleração

TÓPICOS PUBLICADOS:
==================
- /joint_states (sensor_msgs/JointState) - Posição das rodas
- /scan (sensor_msgs/LaserScan) - Laser scan filtrado
- /zed/zed_node/imu/data (sensor_msgs/Imu) - Dados da IMU

USO:
====
# Todos os sensores:
ros2 launch caramelo_bringup caramelo_sensors_bringup.launch.py

# Sensores individuais:
ros2 launch caramelo_bringup encoder_bringup.launch.py
ros2 launch caramelo_bringup lidar_bringup.launch.py
ros2 launch caramelo_bringup imu_zed_bringup.launch.py

ARGUMENTOS:
===========
- use_sim_time: false (padrão para robô real)
- enable_lidar: true (ativar RPLIDAR S2)
- enable_imu: true (ativar IMU ZED2i)
- enable_encoders: true (ativar encoders ESP32)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch de todos os sensores do robô"""
    
    # Package directories
    caramelo_bringup_share = get_package_share_directory('caramelo_bringup')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_imu = LaunchConfiguration('enable_imu')
    enable_encoders = LaunchConfiguration('enable_encoders')
    
    # Declare arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar tempo de simulação')
    
    declare_enable_lidar_cmd = DeclareLaunchArgument(
        'enable_lidar',
        default_value='true',
        description='Ativar RPLIDAR S2')
    
    declare_enable_imu_cmd = DeclareLaunchArgument(
        'enable_imu',
        default_value='true',
        description='Ativar IMU ZED2i')
    
    declare_enable_encoders_cmd = DeclareLaunchArgument(
        'enable_encoders',
        default_value='true',
        description='Ativar encoders ESP32')
    
    # ===============================================
    # 1. ESP32 Encoders
    # ===============================================
    encoders_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            caramelo_bringup_share, '/launch/encoder_bringup.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(enable_encoders)
    )
    
    # ===============================================
    # 2. RPLIDAR S2
    # ===============================================
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            caramelo_bringup_share, '/launch/lidar_bringup.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(enable_lidar)
    )
    
    # ===============================================
    # 3. ZED2i IMU
    # ===============================================
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            caramelo_bringup_share, '/launch/imu_zed_bringup.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(enable_imu)
    )

    return LaunchDescription([
        # Argumentos
        declare_use_sim_time_cmd,
        declare_enable_lidar_cmd,
        declare_enable_imu_cmd,
        declare_enable_encoders_cmd,
        
        # Sensores em paralelo:
        encoders_launch,    # ESP32 Encoders
        lidar_launch,       # RPLIDAR S2
        imu_launch,         # ZED2i IMU
    ])
