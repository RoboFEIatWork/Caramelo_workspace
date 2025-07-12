#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LaserScanFilter(Node):
    """
    Filtro de LIDAR que mantém apenas o setor 90° a 270°.
    Remove a detecção do próprio robô (0° a 90° e 270° a 360°).
    """
    
    def __init__(self):
        super().__init__('laser_scan_filter')
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_raw',  # Entrada do LIDAR original
            self.scan_callback,
            10)
        
        self.publisher = self.create_publisher(
            LaserScan,
            '/scan',      # Saída filtrada
            10)
        
        self.get_logger().info('🔍 Filtro de LIDAR iniciado: 90° a 270°')
        
    def scan_callback(self, msg):
        """Filtra o scan para usar apenas 90° a 270° e ignora pontos < 0.05m (define como infinito)"""
        if not msg.ranges:
            return
        
        total_points = len(msg.ranges)
        start_idx = int(total_points * 0.25)  # 90°
        end_idx = int(total_points * 0.75)    # 270°
        min_dist = 0.05  # 5cm
        
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max
        
        filtered_msg.ranges = [
            r if start_idx <= i < end_idx and r >= min_dist else float('inf')
            for i, r in enumerate(msg.ranges)
        ]
        filtered_msg.intensities = [
            msg.intensities[i] if msg.intensities and len(msg.intensities) == total_points and start_idx <= i < end_idx and msg.ranges[i] >= min_dist else 0.0
            for i in range(total_points)
        ]
        
        if not hasattr(self, '_logged'):
            self._logged = True
            self.get_logger().info(f'🔍 Filtro ativo: ignorando {start_idx} pontos iniciais e {total_points-end_idx} finais, e todos < 0.05m')
            self.get_logger().info(f'   Setor útil: índices {start_idx} a {end_idx}')
        
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
