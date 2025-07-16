#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LaserScanFilter(Node):
    """
    Filtro OTIMIZADO para RPLIDAR S2 - mantém setor 90° a 270° (180° frontal).
    
    RPLIDAR S2 specs:
    - 10Hz, 0.12° resolução, 0.05-30m alcance, ±5cm precisão
    - inverted=true, DenseBoost mode, angle_compensate=true
    - Filtro remove auto-detecção (0°-90° e 270°-360°) e pontos < 5cm
    """
    
    def __init__(self):
        super().__init__('laser_scan_filter')
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_raw',  # Entrada do RPLIDAR S2 original
            self.scan_callback,
            10)
        
        self.publisher = self.create_publisher(
            LaserScan,
            '/scan',      # Saída filtrada para SLAM/Nav2
            10)
        
        # Parâmetros específicos para RPLIDAR S2
        self.min_valid_distance = 0.05  # 5cm - blind range do S2
        self.max_valid_distance = 25.0  # 25m - margem segura dos 30m do S2
        self.intensity_threshold = 10   # Threshold para intensidades válidas
        
        self.get_logger().info('🔍 Filtro RPLIDAR S2 iniciado: 90° a 270° (180° úteis)')
        
    def scan_callback(self, msg):
        """
        Filtra scan do RPLIDAR S2:
        1. Mantém apenas 90° a 270° (180° frontal)
        2. Remove pontos < 5cm (blind range)
        3. Remove pontos > 25m (margem segura)
        4. Filtra por intensidade se disponível
        """
        if not msg.ranges:
            return
        
        total_points = len(msg.ranges)
        
        # Setor útil: 90° a 270° (25% a 75% do array circular)
        start_idx = int(total_points * 0.25)  # 90°
        end_idx = int(total_points * 0.75)    # 270°
        
        # Criar mensagem filtrada
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = max(msg.range_min, self.min_valid_distance)
        filtered_msg.range_max = min(msg.range_max, self.max_valid_distance)
        
        # Filtrar ranges: manter setor útil e distâncias válidas
        filtered_msg.ranges = []
        for i, r in enumerate(msg.ranges):
            if start_idx <= i < end_idx:
                # Dentro do setor útil (90°-270°)
                if self.min_valid_distance <= r <= self.max_valid_distance:
                    # Distância válida para RPLIDAR S2
                    filtered_msg.ranges.append(r)
                else:
                    # Fora do range válido -> infinito
                    filtered_msg.ranges.append(float('inf'))
            else:
                # Fora do setor útil -> infinito
                filtered_msg.ranges.append(float('inf'))
        
        # Filtrar intensidades se disponíveis
        if msg.intensities and len(msg.intensities) == total_points:
            filtered_msg.intensities = []
            for i, intensity in enumerate(msg.intensities):
                if (start_idx <= i < end_idx and 
                    self.min_valid_distance <= msg.ranges[i] <= self.max_valid_distance and
                    intensity >= self.intensity_threshold):
                    filtered_msg.intensities.append(intensity)
                else:
                    filtered_msg.intensities.append(0.0)
        else:
            # Se não há intensidades, criar array vazio
            filtered_msg.intensities = []
        
        # Log único na inicialização
        if not hasattr(self, '_logged'):
            self._logged = True
            useful_points = end_idx - start_idx
            total_angle = 180.0  # 90° a 270° = 180°
            angular_res = total_angle / useful_points if useful_points > 0 else 0
            
            self.get_logger().info(f'🔍 RPLIDAR S2 filtro ativo:')
            self.get_logger().info(f'   Setor útil: 90° a 270° (180° frontal)')
            self.get_logger().info(f'   Pontos úteis: {useful_points}/{total_points}')
            self.get_logger().info(f'   Resolução efetiva: ~{angular_res:.2f}°/ponto')
            self.get_logger().info(f'   Range válido: {self.min_valid_distance}m a {self.max_valid_distance}m')
        
        self.publisher.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    filter_node = LaserScanFilter()
    
    try:
        rclpy.spin(filter_node)
    except KeyboardInterrupt:
        pass
    finally:
        filter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
