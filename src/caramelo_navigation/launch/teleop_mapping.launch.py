#!/usr/bin/env python3
"""
🗺️ TELEOP MAPPING IMPECÁVEL - RPLIDAR S2 OTIMIZADO

Sistema 100% otimizado para mapeamento manual de alta precisão.
Configurado especificamente para RPLIDAR S2 com máxima tolerância a delays.

ARQUITETURA ROBUSTA:
===================
Este launch combina apenas os componentes essenciais para mapeamento:
- RPLIDAR S2 (dados brutos, 10Hz, resolução 0.12°)
- SLAM Toolbox (parâmetros otimizados para delays USB)
- Teleop Keyboard (controle suave e responsivo)
- RViz2 (visualização em tempo real)

⚠️ HARDWARE OBRIGATÓRIO EM OUTROS TERMINAIS:
============================================

TERMINAL 1 - ENCODER/ODOMETRIA:
  cd ~/Caramelo_workspace && source install/setup.bash
  ros2 launch caramelo_bringup encoder_bringup.launch.py

TERMINAL 2 - PWM/MOVIMENTO:  
  cd ~/Caramelo_workspace && source install/setup.bash
  ros2 launch caramelo_bringup pwm_bringup.launch.py

TERMINAL 3 - ESTE MAPEAMENTO:
  cd ~/Caramelo_workspace && source install/setup.bash
  ros2 launch caramelo_navigation teleop_mapping.launch.py arena:=NOME_ARENA

🎮 CONTROLES DE MAPEAMENTO:
==========================
Teclas de movimento:
  i = frente            , = ré-esquerda
  k = PARAR             . = ré-direita  
  j = esquerda          m = ré
  l = direita
  u = diagonal ↖        o = diagonal ↗

Controle de velocidade:
  q/z = aumentar/diminuir velocidade linear
  w/x = aumentar/diminuir velocidade angular

🏆 METODOLOGIA PARA MAPEAMENTO PERFEITO:
=======================================
1. 🐌 VELOCIDADE BAIXA - RPLIDAR S2 precisa tempo para processar
2. 🔄 FECHAR LOOPS - sempre retornar por caminhos já mapeados
3. 👁️ MONITOR RVIZ - verificar alinhamento constante dos scans
4. 🚪 EXPLORAÇÃO TOTAL - cada canto, corredor e obstáculo
5. ⏸️ PAUSE QUANDO NECESSÁRIO - melhor pausar que corrigir depois
6. 📐 ROTAÇÕES SUAVES - evitar movimentos bruscos no eixo angular
7. 🎯 COVERAGE COMPLETO - garantir 100% da área mapeada

💾 SALVAMENTO DO MAPA:
=====================
# Comando para salvar (execute em outro terminal):
ros2 run nav2_map_server map_saver_cli -f ~/Caramelo_workspace/maps/NOME_ARENA/map

# O mapa será salvo automaticamente como:
# - map.pgm (imagem do mapa)
# - map.yaml (metadados e configurações)

🏟️ EXEMPLOS DE ARENAS:
======================
# Arena principal do laboratório:
ros2 launch caramelo_navigation teleop_mapping.launch.py arena:=laboratorio_fei

# Arena oficial da competição:  
ros2 launch caramelo_navigation teleop_mapping.launch.py arena:=robocup_2025

# Arena de testes e desenvolvimento:
ros2 launch caramelo_navigation teleop_mapping.launch.py arena:=teste_desenvolvimento

# Arena hotel (exemplo já criado):
ros2 launch caramelo_navigation teleop_mapping.launch.py arena:=hotel

📊 MONITORAMENTO EM RVIZ:
========================
- /scan: pontos vermelhos do RPLIDAR S2 
- /map: mapa sendo construído em tempo real
- /tf: transformações entre frames (base_link, laser_frame, odom, map)
- trajectory: trajetória percorrida (linha verde)

⚡ OTIMIZAÇÕES IMPLEMENTADAS:
============================
- SLAM configurado para delays de até 500ms (USB/ESP32)
- Tolerância alta a odometria ruim 
- Taxa de publicação otimizada para 10Hz do RPLIDAR
- Filtros adaptados para ambiente real
- Interface responsiva mesmo com lag
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    TELEOP MAPPING LAUNCH - Sistema impecável para mapeamento manual
    
    Componentes incluídos:
    1. RPLIDAR S2 (via mapping_launch.py)
    2. SLAM Toolbox com parâmetros otimizados
    3. RViz2 com configuração específica para mapeamento
    4. Teleop Keyboard para controle responsivo
    5. Twist Converter para compatibilidade mecanum
    
    NOTA: Encoder e PWM devem rodar em terminais separados para
    máxima estabilidade e controle independente.
    """
    
    # Configurações de launch
    use_sim_time = LaunchConfiguration('use_sim_time')
    arena = LaunchConfiguration('arena')
    
    # Argumentos de entrada
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar tempo de simulação (false para hardware real)')
    
    declare_arena_cmd = DeclareLaunchArgument(
        'arena',
        default_value='default',
        description='Nome da arena para organização de mapas')

    # 1. Sistema de mapeamento completo (LIDAR + SLAM + RViz)
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('caramelo_navigation'),
                         'launch', 'mapping_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # 2. Sistema de teleoperação (Keyboard + Twist Converter)
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('caramelo_bringup'),
                         'launch', 'teleop_keyboard.launch.py'))
    )

    return LaunchDescription([
        # Declaração de argumentos
        declare_use_sim_time_cmd,
        declare_arena_cmd,
        
        # Sistemas principais
        mapping_launch,    # RPLIDAR S2 + SLAM Toolbox + RViz2
        teleop_launch,     # Teleop Keyboard + Twist Converter
    ])
