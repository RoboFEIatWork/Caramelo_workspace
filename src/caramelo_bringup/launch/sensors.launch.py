#!/usr/bin/env python3
"""
CARAMELO SENSORS BRINGUP - Sensores + Odometria + Robot State do Robô Caramelo

COMPONENTES INCLUÍDOS:
=====================
0. Robot State Publisher - TF/URDF (BASE OBRIGATÓRIA)
1. ESP32 Encoders - Leitura de posição das 4 rodas mecanum
2. RPLIDAR S2 - Laser scan 360° para navegação
3. ZED2i IMU - Orientação e aceleração
4. Fusão de Odometria - EKF com ESP32 + ZED2i IMU

TÓPICOS PUBLICADOS:
==================
- /robot_description (std_msgs/String) - URDF do robô
- /tf, /tf_static (tf2_msgs/TFMessage) - Árvore de transformações
- /joint_states (sensor_msgs/JointState) - Posição das rodas
- /scan (sensor_msgs/LaserScan) - Laser scan filtrado
- /zed/zed_node/imu/data (sensor_msgs/Imu) - Dados da IMU
- /odom (nav_msgs/Odometry) - Odometria fusionada
- /odom -> /base_link TF - Transformação de odometria

USO:
====
# Sistema completo (sensores + odometria + robot state):
ros2 launch caramelo_bringup sensors.launch.py

# Sensores individuais (requer robot_state separado):
ros2 launch caramelo_bringup robot_state.launch.py
ros2 launch caramelo_bringup encoder_bringup.launch.py
ros2 launch caramelo_bringup lidar_bringup.launch.py
ros2 launch caramelo_bringup imu_zed_bringup.launch.py
ros2 launch caramelo_bringup odometry_fusion.launch.py

ARGUMENTOS:
===========
- use_sim_time: false (padrão para robô real)
- enable_lidar: true (ativar RPLIDAR S2)
- enable_imu: true (ativar IMU ZED2i)
- enable_encoders: true (ativar encoders ESP32)
- enable_odometry: true (ativar fusão de odometria)
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
    enable_odometry = LaunchConfiguration('enable_odometry')
    
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
    
    declare_enable_odometry_cmd = DeclareLaunchArgument(
        'enable_odometry',
        default_value='true',
        description='Ativar fusão de odometria')
    
    # ===============================================
    # 0. Robot State Publisher (BASE OBRIGATÓRIA)
    # ===============================================
    robot_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            caramelo_bringup_share, '/launch/robot_state.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
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

    # ===============================================
    # 4. Fusão de Odometria (EKF)
    # ===============================================
    odometry_fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            caramelo_bringup_share, '/launch/odometry_fusion.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(enable_odometry)
    )

    return LaunchDescription([
        # Argumentos
        declare_use_sim_time_cmd,
        declare_enable_lidar_cmd,
        declare_enable_imu_cmd,
        declare_enable_encoders_cmd,
        declare_enable_odometry_cmd,
        
        # Componentes em ordem:
        robot_state_launch,     # 0. Robot State (BASE TF/URDF)
        encoders_launch,        # 1. ESP32 Encoders
        lidar_launch,           # 2. RPLIDAR S2
        imu_launch,             # 3. ZED2i IMU
        odometry_fusion_launch, # 4. Fusão de Odometria
    ])
