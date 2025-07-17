#!/usr/bin/env python3
"""
🔄 ODOMETRY BRINGUP - Sistema de Odometria Completo Caramelo

FUNÇÃO PRINCIPAL:
================
Launch focado em ODOMETRIA que inicia todos os sensores necessários:
- Leitura de encoders ESP32 
- Fusão de odometria (encoder + IMU ZED)
- Joint State Publisher
- Robot Model (URDF)
- IMU ZED2i

⚠️ IMPORTANTE: NÃO inclui PWM Controller!
PWM Controller deve ser iniciado separadamente via:
ros2 launch caramelo_bringup actuators_bringup.launch.py

MOTIVO DA SEPARAÇÃO:
===================
Comunicação serial ESP32 pode conflitar se encoder_reader e pwm_controller
rodarem simultaneamente no mesmo processo.

COMPONENTES:
============
1. Robot State Publisher - URDF + TF tree completa
2. Encoder Reader Node - Lê dados raw ESP32 encoders (/dev/ttyUSB1)
3. IMU ZED2i - Dados inerciais para fusão
4. Odometry Fusion Node - Fusão encoder + IMU → odometria real
5. Joint State Publisher - Estados das rodas 

TF TREE RESULTANTE:
==================
odom → base_footprint → base_link → [laser_frame, wheel_joints, zed_camera, etc]

TÓPICOS PRINCIPAIS:
==================
SAÍDAS:
- /odom (nav_msgs/Odometry) - Odometria real calculada
- /joint_states (sensor_msgs/JointState) - Estados das rodas
- /tf → TF tree completa para navegação
- /encoder_data - Dados raw dos encoders

USO:
====
# ODOMETRIA (este launch):
ros2 launch caramelo_bringup odometry_bringup.launch.py

# ATUADORES (separado):
ros2 launch caramelo_bringup actuators_bringup.launch.py

DEPENDÊNCIAS:
============
- ESP32 Encoders em /dev/ttyUSB1
- ZED2i conectada (IMU)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Argumentos do launch
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Package paths
    caramelo_bringup_share = FindPackageShare('caramelo_bringup')
    
    # =====================================================
    # 1. ROBOT DESCRIPTION (URDF)
    # =====================================================
    robot_description_content = Command([
        'xacro ', 
        PathJoinSubstitution([
            caramelo_bringup_share,
            'urdf',
            'caramelo_real.urdf.xacro'
        ])
    ])
    
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }
    
    # =====================================================
    # 2. ROBOT STATE PUBLISHER
    # =====================================================
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        respawn=True
    )
    
    # =====================================================
    # 3. ENCODER READER NODE - ESP32 Encoders
    # =====================================================
    encoder_reader_node = Node(
        package='caramelo_bringup',
        executable='encoder_reader_node',
        name='encoder_reader_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                caramelo_bringup_share,
                'config',
                'encoder_params.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ],
        respawn=True,
        respawn_delay=2.0
    )
    
    # =====================================================
    # 4. IMU ZED2i - Incluir launch do IMU
    # =====================================================
    imu_zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                caramelo_bringup_share,
                'launch',
                'imu_zed_bringup.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # =====================================================
    # 5. ODOMETRY FUSION NODE - Fusão Encoder + IMU
    # =====================================================
    odometry_fusion_node = Node(
        package='caramelo_bringup',
        executable='odometry_fusion_node',
        name='odometry_fusion_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                caramelo_bringup_share,
                'config',
                'encoder_params.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ],
        respawn=True,
        respawn_delay=2.0
    )

    # =====================================================
    # 6. JOINT STATE PUBLISHER - Estados das Rodas
    # =====================================================
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        respawn=True
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        robot_state_publisher_node,
        encoder_reader_node,
        imu_zed_launch,
        odometry_fusion_node,
        joint_state_publisher_node,
    ])
