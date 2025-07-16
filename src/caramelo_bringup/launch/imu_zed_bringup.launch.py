#!/usr/bin/env python3
"""
🎯 ZED2i IMU-ONLY LAUNCH - Sensor IMU Apenas

Sistema otimizado para extrair APENAS dados do IMU da ZED2i
sem dependência de GPU NVIDIA. Focado em odometria robusta.

CONFIGURAÇÃO:
=============
- Camera: ZED2i com IMU integrado BMI055
- Objetivo: Apenas /zed/imu/data para fusão sensorial
- Sem visão: Desabilitado streams de imagem/depth
- CPU-only: Funciona sem GPU NVIDIA

DADOS PUBLICADOS:
================
- /zed/imu/data: sensor_msgs/Imu
  - Angular velocity (gyroscope)
  - Linear acceleration (accelerometer)  
  - Orientation (se disponível)

USO:
====
ros2 launch caramelo_bringup zed_imu_only.launch.py

FUSÃO SENSORIAL:
===============
Este IMU será combinado com encoders ESP32 via robot_localization
para odometria ultra-robusta mesmo com falhas de comunicação.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    ZED2i IMU-Only Launch - Extração pura de dados inerciais
    """
    
    # Package directories
    zed_wrapper_share = get_package_share_directory('zed_wrapper')
    
    # Launch arguments
    camera_model = LaunchConfiguration('camera_model')
    
    declare_camera_model_cmd = DeclareLaunchArgument(
        'camera_model',
        default_value='zed2i',
        description='Modelo da camera ZED (zed2i)')
    
    # ZED Wrapper com configuração minimal para IMU apenas
    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            zed_wrapper_share, '/launch/zed_camera.launch.py'
        ]),
        launch_arguments={
            'camera_model': camera_model,
            'publish_tf': 'false',           # Não publicar TF - deixar para URDF
            'publish_imu_tf': 'false',       # Não publicar TF do IMU - deixar para URDF
            'publish_map_tf': 'false',       # Não publicar map TF
            'base_frame': 'base_footprint',  # Frame base do robô
            'camera_name': 'zed',            # Nome do namespace
            'camera_frame': 'zed_camera',    # Frame da câmera (do URDF)
            'imu_frame': 'zed_camera_imu_link', # Frame do IMU (do URDF)
            'camera_name': 'zed',            # Nome do namespace
            'sensors_image_sync': 'false',   # Desabilitar sync de imagem
            'depth_mode': 'NONE',            # SEM processamento de depth
            'rgb_enabled': 'false',          # SEM stream RGB
            'depth_enabled': 'false',        # SEM stream depth
            'point_cloud_enabled': 'false',  # SEM point cloud
            'pos_tracking_enabled': 'false', # SEM SLAM interno ZED
            'mapping_enabled': 'false',      # SEM mapping interno
            'obj_det_enabled': 'false',      # SEM object detection
            'body_trk_enabled': 'false',     # SEM body tracking
            'stream_enabled': 'false',       # SEM streaming
            'sensors_enabled': 'true',       # APENAS sensores (IMU)
            'sensors_pub_rate': '200.0',     # IMU a 200Hz para precisão
            'grab_frame_rate': '30',         # Frame rate mínimo
            'grab_resolution': 'VGA',        # Resolução mínima
            'gpu_id': '-1',                  # FORÇAR CPU-only mode
        }.items()
    )
    
    return LaunchDescription([
        # Argumentos
        declare_camera_model_cmd,
        
        # ZED IMU-only wrapper
        zed_wrapper_launch,
    ])
