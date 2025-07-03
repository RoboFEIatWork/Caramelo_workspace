#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarFilter(Node):
    def __init__(self):
        super().__init__('lidar_filter')
        
        # Subscriber para scan bruto
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan_raw', self.scan_callback, 10)
        
        # Publisher para scan filtrado
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        # Parâmetros de filtro para robô Caramelo
        self.min_range = 0.20  # 20cm mínimo (remove carcaça/suportes do robô)
        self.max_range = 8.0   # 8m máximo
        
        # Filtros angulares para remover partes da carcaça do Caramelo
        # O robô Caramelo tem dimensões: ~30cm largura x ~40cm comprimento
        # LIDAR montado no centro, alguns suportes podem aparecer
        self.blocked_angles = [
            # Exemplo: se houver suportes em ângulos específicos
            # (-3.14159/6, -3.14159/8),  # ~-30° a -22.5° (lado esquerdo)
            # (3.14159/8, 3.14159/6),    # ~22.5° a 30° (lado direito)
            # Adicione conforme necessário após observar o scan
        ]
        
        self.get_logger().info('🔧 Filtro LIDAR iniciado!')
        self.get_logger().info(f'📏 Filtro de distância: {self.min_range}m - {self.max_range}m')
        
    def scan_callback(self, msg):
        """Filtra dados do LIDAR"""
        # Cria cópia da mensagem
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = self.min_range  # Atualiza range mínimo
        filtered_msg.range_max = self.max_range  # Atualiza range máximo
        
        # Processa ranges
        ranges = np.array(msg.ranges)
        
        # 1. Filtro de distância - remove pontos muito próximos (carcaça)
        ranges = np.where(ranges < self.min_range, float('inf'), ranges)
        ranges = np.where(ranges > self.max_range, float('inf'), ranges)
        
        # 2. Filtro de ângulos bloqueados (se necessário)
        for angle_start, angle_end in self.blocked_angles:
            start_idx = int((angle_start - msg.angle_min) / msg.angle_increment)
            end_idx = int((angle_end - msg.angle_min) / msg.angle_increment)
            start_idx = max(0, min(len(ranges), start_idx))
            end_idx = max(0, min(len(ranges), end_idx))
            ranges[start_idx:end_idx] = float('inf')
        
        # 3. Filtro de ruído - remove pontos isolados
        filtered_ranges = self.noise_filter(ranges)
        
        filtered_msg.ranges = filtered_ranges.tolist()
        filtered_msg.intensities = msg.intensities  # Manter intensidades
        
        # Publicar scan filtrado
        self.scan_pub.publish(filtered_msg)
        
    def noise_filter(self, ranges):
        """Remove pontos isolados que podem ser ruído"""
        filtered = ranges.copy()
        
        # Filtro simples: se um ponto está muito diferente dos vizinhos, remove
        for i in range(1, len(ranges) - 1):
            if not np.isinf(ranges[i]):
                # Verifica vizinhos
                prev_valid = ranges[i-1] if not np.isinf(ranges[i-1]) else None
                next_valid = ranges[i+1] if not np.isinf(ranges[i+1]) else None
                
                # Se o ponto está muito diferente dos vizinhos (>30cm), considera ruído
                if prev_valid is not None and abs(ranges[i] - prev_valid) > 0.3:
                    if next_valid is not None and abs(ranges[i] - next_valid) > 0.3:
                        filtered[i] = float('inf')
        
        return filtered


def main(args=None):
    rclpy.init(args=args)
    lidar_filter = LidarFilter()
    try:
        rclpy.spin(lidar_filter)
    except KeyboardInterrupt:
        lidar_filter.get_logger().info('🛑 Filtro LIDAR finalizado')
    lidar_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
