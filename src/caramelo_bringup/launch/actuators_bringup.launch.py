#!/usr/bin/env python3
"""
ACTUATORS BRINGUP - Sistema de Atuadores do Robô Caramelo

ATUADORES CONTROLADOS:
======================
1. ESP32 PWM - Controle das 4 rodas mecanum
2. Controller Manager - ROS2 Control
3. Mecanum Drive Controller - Plugin de controle

TÓPICOS SUBSCRITOS:
==================
- /cmd_vel (geometry_msgs/Twist) - Comandos de velocidade

TÓPICOS PUBLICADOS:
==================
- /joint_states (sensor_msgs/JointState) - Estados dos joints
- /mecanum_drive_controller/cmd_vel - Comandos convertidos

USO:
====
ros2 launch caramelo_bringup actuators_bringup.launch.py

DEPENDÊNCIAS:
============
- robot_description deve estar ativo (robot_state.launch.py)
- Arquivo de configuração: config/robot_controllers.yaml
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
    # 4. Hardware Interface Node (ESP32 PWM)
    # ===============================================
    hw_interface_node = Node(
        package='caramelo_bringup',
        executable='caramelo_hw_interface_node',
        name='caramelo_hw_interface_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
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
        hw_interface_node,                      # 2. Hardware Interface (3s delay)
        mecanum_drive_controller_spawner,       # 3. Mecanum Controller (5s delay)
    ])
