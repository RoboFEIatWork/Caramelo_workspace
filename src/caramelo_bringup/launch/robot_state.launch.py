#!/usr/bin/env python3
"""
ROBOT STATE LAUNCH - URDF e TF Tree do Robô

FUNÇÃO:
=======
Carrega o URDF do robô e publica a árvore TF estática.

SAÍDAS:
=======
- /robot_description (std_msgs/String) - URDF do robô
- TF estático: base_footprint -> base_link -> sensores/rodas

USO:
====
ros2 launch caramelo_bringup robot_state.launch.py

VISUALIZAÇÃO:
============
rviz2 -d [config] para ver TF tree
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """
    Launch para robot state publisher
    """
    
    # Package directory
    caramelo_bringup_share = get_package_share_directory('caramelo_bringup')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # URDF path
    urdf_xacro_path = os.path.join(
        caramelo_bringup_share, 'urdf', 'caramelo_real.urdf.xacro'
    )
    
    # Process XACRO to URDF
    robot_description = Command(['xacro ', urdf_xacro_path])
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(robot_description, value_type=str),
        }]
    )
    
    return LaunchDescription([
        # Arguments
        declare_use_sim_time_cmd,
        
        # Robot state
        robot_state_publisher,
    ])
