#!/usr/bin/env python3

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
    # 1. URDF/Robot Description
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
    # 2. Robot State Publisher
    # ===============================================
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        respawn=True
    )
    
    
    # ===============================================
    # 4. Encoder Node (Odometria real)
    # ===============================================
    # Parâmetros físicos do robô Caramelo:
    # - Distância entre centros das rodas esquerda/direita: 31 cm
    # - Distância entre eixos (frente/trás): 47 cm  
    # - Diâmetro das rodas: 10 cm (raio 5 cm)
    # - Gear box: 1 volta da roda = 28 voltas do motor
    # - Encoder: 1 volta da roda = 57344 pulsos do encoder
    
    encoder_config = PathJoinSubstitution([
        caramelo_bringup_path,
        "config",
        "encoder_config.yaml"
    ])
    
    encoder_node = Node(
        package='caramelo_bringup',
        executable='encoder_joint_state_node',
        name='encoder_joint_state_node',
        output='screen',
        parameters=[
            encoder_config,
            {'use_sim_time': use_sim_time}
        ],
        respawn=True,
        respawn_delay=2.0
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Ordem de inicialização:
        robot_state_publisher,           # 1. URDF primeiro (inclui base_footprint -> base_link)
        encoder_node,                   # 2. Inicia node dos encoders (odom -> base_footprint dinâmico)
    ])
