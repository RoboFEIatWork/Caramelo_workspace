#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file para mapeamento usando teleop_keyboard.
    
    EXECUTE EM TERMINAIS SEPARADOS (PADRÃO):
    1. Terminal 1: ros2 launch caramelo_bringup encoder_bringup.launch.py
    2. Terminal 2: ros2 launch caramelo_bringup pwm_bringup.launch.py
    3. Terminal 3: ros2 launch caramelo_navigation teleop_mapping.launch.py
    
    Como usar:
    1. Use as teclas do teclado para mover o robô:
       - w/s: frente/trás
       - a/d: esquerda/direita
       - q/e: rotação
    2. O mapa será construído automaticamente conforme você navega
    
    Para salvar o mapa (novo ambiente):
    1. mkdir -p ~/Caramelo_workspace/maps/nome_ambiente
    2. cd ~/Caramelo_workspace/maps/nome_ambiente
    3. ros2 run nav2_map_server map_saver_cli -f map
    
    Exemplo:
    mkdir -p ~/Caramelo_workspace/maps/laboratorio_fei
    cd ~/Caramelo_workspace/maps/laboratorio_fei
    ros2 run nav2_map_server map_saver_cli -f map
    """
    
    # Configurações
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Declara argumentos
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar tempo de simulação')

    # 1. Mapeamento SLAM (com filtro de LIDAR)
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('caramelo_navigation'),
                         'launch', 'mapping_launch.py'))
    )

    # 2. Teleop Keyboard (para controle manual)
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('caramelo_bringup'),
                         'launch', 'teleop_keyboard.launch.py'))
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        mapping_launch,         # LIDAR + SLAM + RViz + twist_converter + filtro
        teleop_launch,          # Controle por teclado
    ])
