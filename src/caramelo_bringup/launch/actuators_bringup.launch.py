#!/usr/bin/env python3
"""
ACTUATORS BRINGUP - Sistema de Atuadores do Robô Caramelo

ATUADORES CONTROLADOS:
======================
1. PWM Controller Node - Controle PWM das 4 rodas mecanum via ESP32
2. Controller Manager - ROS2 Control
3. Mecanum Drive Controller - Plugin de controle

COMUNICAÇÃO:
============
- ESP32 PWM em /dev/ttyUSB0 (pwm_controller_node)
- ros2_control interface para navegação

TÓPICOS SUBSCRITOS:
==================
- /cmd_vel (geometry_msgs/Twist) - Comandos de velocidade
- /mecanum_controller/commands (Float64MultiArray) - Comandos diretos

TÓPICOS PUBLICADOS:
==================
- /motor_status (std_msgs/Bool) - Status da conexão ESP32

USO:
====
ros2 launch caramelo_bringup actuators_bringup.launch.py

⚠️ IMPORTANTE:
==============
Execute SEPARADO do odometry_bringup.launch.py para evitar 
conflitos de comunicação serial ESP32.

DEPENDÊNCIAS:
============
- robot_description deve estar ativo (robot_state.launch.py)
- ESP32 PWM em /dev/ttyUSB0
- Arquivo: config/robot_controllers.yaml e motor_params.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Argumentos do launch
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Caminhos dos pacotes
    caramelo_bringup_path = FindPackageShare('caramelo_bringup')
    
    # ===============================================
    # 1. Configuração dos controladores
    # ===============================================
    robot_controllers = PathJoinSubstitution([
        caramelo_bringup_path,
        "config",
        "robot_controllers.yaml"
    ])
    
    # ===============================================
    # 2. Robot Description (necessário para ros2_control)
    # ===============================================
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
    
    # ===============================================
    # 3. Controller Manager
    # ===============================================
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers, {'use_sim_time': use_sim_time}],
        output="screen",
        prefix='bash -c "sleep 2; $0 $@"',  # Aguarda robot_description
        respawn=True
    )
    
    # ===============================================
    # 4. PWM Controller Node (ESP32 PWM)
    # ===============================================
    pwm_controller_node = Node(
        package='caramelo_bringup',
        executable='pwm_controller_node',
        name='pwm_controller_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                caramelo_bringup_path,
                'config',
                'motor_params.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ],
        prefix='bash -c "sleep 3; $0 $@"',  # Aguarda controller_manager
        respawn=True
    )
    
    # ===============================================
    # 5. Mecanum Drive Controller Spawner
    # ===============================================
    mecanum_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mecanum_drive_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen",
        prefix='bash -c "sleep 5; $0 $@"'  # Aguarda hw_interface
    )
    
    # ===============================================
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Ordem de inicialização com delays:
        controller_manager,                      # 1. Controller Manager (2s delay)
        pwm_controller_node,                     # 2. PWM Controller (3s delay)
        mecanum_drive_controller_spawner,        # 3. Mecanum Controller (5s delay)
    ])
