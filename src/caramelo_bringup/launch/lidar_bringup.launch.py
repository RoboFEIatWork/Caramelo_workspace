#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch para RPLidar S2 do robô Caramelo.
    
    Este launch inicia:
    1. Driver do RPLidar S2 na porta USB2
    2. Filtro laser para limitar ângulo a 180° frontal
    3. Configurações otimizadas para navegação
    """
    
    # Argumentos do launch
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Filtro Python para limitar ângulo e ignorar pontos < 0.2m
    laser_scan_filter_node = Node(
        package='caramelo_navigation',
        executable='laser_scan_filter',
        name='laser_scan_filter',
        output='screen',
        remappings=[
            ('/scan_raw', 'scan_raw'),   # Input: scan bruto do RPLidar
            ('/scan', '/scan')           # Output: scan filtrado para navegação
        ]
    )
    
    # Nó do RPLidar S2 com configurações corretas
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',                   # Executável correto para S2
        name='rplidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',                # Tipo de canal
            'serial_port': '/dev/ttyUSB2',           # LIDAR na porta USB2
            'serial_baudrate': 1000000,              # Baudrate correto para S2 (1M)
            'frame_id': 'laser_frame',               # Frame do LIDAR no TF tree
            'inverted': True,                        # IMPORTANTE: inverted=true para S2
            'angle_compensate': True,                # Compensação de ângulo ativada
            'scan_mode': 'DenseBoost',               # Modo de scan do S2
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('scan', 'scan_raw')                     # Publicar como scan_raw para filtrar
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        rplidar_node,
        laser_scan_filter_node
    ])
