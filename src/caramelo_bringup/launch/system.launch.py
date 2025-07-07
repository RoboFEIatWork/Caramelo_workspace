#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Argumentos de entrada
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='/home/work/Caramelo_workspace/mapa_20250704_145039.yaml',
        description='Path to the map file'
    )
    
    task_file_arg = DeclareLaunchArgument(
        'task_file',
        default_value='tasks.yaml',
        description='Name of the task file in caramelo_tasks/config/'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )
    
    initial_pose_x_arg = DeclareLaunchArgument(
        'initial_pose_x',
        default_value='0.0',
        description='Initial X position'
    )
    
    initial_pose_y_arg = DeclareLaunchArgument(
        'initial_pose_y',
        default_value='0.0',
        description='Initial Y position'
    )
    
    initial_pose_yaw_arg = DeclareLaunchArgument(
        'initial_pose_yaw',
        default_value='0.0',
        description='Initial Yaw orientation'
    )
    
    # Configurações
    map_file = LaunchConfiguration('map_file')
    task_file = LaunchConfiguration('task_file')
    use_rviz = LaunchConfiguration('use_rviz')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    
    # Paths dos pacotes
    nav_launch_dir = os.path.join(get_package_share_directory('caramelo_navigation'), 'launch')
    description_launch_dir = os.path.join(get_package_share_directory('caramelo_description'), 'launch')
    
    # 1. Robot Description (URDF)
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([description_launch_dir, '/robot_description.launch.py']),
        launch_arguments={'use_sim_time': 'false'}.items()
    )
    
    # 2. Navigation Stack (Nav2 + AMCL + Map Server)
    navigation_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav_launch_dir, '/navigation_launch.py']),
        launch_arguments={
            'map_file': map_file,
            'use_sim_time': 'false',
            'initial_pose_x': initial_pose_x,
            'initial_pose_y': initial_pose_y,
            'initial_pose_yaw': initial_pose_yaw
        }.items()
    )
    
    # 3. Task Executor
    task_executor = Node(
        package='caramelo_tasks',
        executable='task_executor_node',
        name='task_executor',
        output='screen',
        parameters=[
            {'task_file': task_file},
            {'use_sim_time': False}
        ]
    )
    
    # 4. RViz (opcional)
    rviz_config = PathJoinSubstitution([
        FindPackageShare('caramelo_description'),
        'rviz',
        'caramelo.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        condition=LaunchConfiguration('use_rviz')
    )
    
    # Log de inicialização
    startup_log = LogInfo(
        msg=[
            '\\n🚀 =================================================================',
            '\\n🤖 CARAMELO ROBOCUP@WORK SYSTEM STARTING...',
            '\\n🗺️  Map: ', map_file,
            '\\n📋 Tasks: ', task_file,
            '\\n📍 Initial Pose: (', initial_pose_x, ', ', initial_pose_y, ', ', initial_pose_yaw, ')',
            '\\n🖥️  RViz: ', use_rviz,
            '\\n=================================================================\\n'
        ]
    )
    
    return LaunchDescription([
        # Argumentos
        map_file_arg,
        task_file_arg,
        use_rviz_arg,
        initial_pose_x_arg,
        initial_pose_y_arg,
        initial_pose_yaw_arg,
        
        # Log de inicialização
        startup_log,
        
        # Componentes do sistema
        robot_description,
        navigation_stack,
        task_executor,
        rviz_node,
        
        # Log final
        LogInfo(msg='🎉 All components launched! System ready for autonomous operation.')
    ])
