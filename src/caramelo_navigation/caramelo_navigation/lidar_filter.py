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
        
        # === PARÂMETROS PARA RPLIDAR S2 ===
        self.min_range = 0.15   # 15cm mínimo (RPLidar S2)
        self.max_range = 30.0   # 30m máximo (RPLidar S2)
        
        # FILTRO ANGULAR: Apenas frente e laterais (90° a 270°)
        # Para RPLidar: 0° = frente, 90° = esquerda, 180° = trás, 270° = direita
        # Queremos de 90° a 270° (sem a parte traseira)
        self.angle_filter_min = math.pi/2    # 90° (esquerda)
        self.angle_filter_max = 3*math.pi/2  # 270° (direita)
        
        self.get_logger().info('🔧 Filtro LIDAR SIMPLES iniciado para RPLidar S2!')
        self.get_logger().info(f'📏 Range: {self.min_range}m - {self.max_range}m')
        self.get_logger().info(f'📐 Ângulos: 90° a 270° (frente e laterais)')
        
    def scan_callback(self, msg):
        """Filtro simples apenas para ângulos e distância"""
        # Cria cópia da mensagem
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = self.min_range
        filtered_msg.range_max = self.max_range
        
        # Processa ranges
        ranges = np.array(msg.ranges)
        
        # === FILTROS SIMPLES (SEM CRASH) ===
        
        # 1. Filtro de distância básico
        ranges = np.where(ranges < self.min_range, float('inf'), ranges)
        ranges = np.where(ranges > self.max_range, float('inf'), ranges)
        
        # 2. FILTRO ANGULAR: Apenas 90° a 270°
        ranges = self.angular_filter(ranges, msg.angle_min, msg.angle_increment)
        
        filtered_msg.ranges = ranges.tolist()
        filtered_msg.intensities = msg.intensities
        
        self.scan_pub.publish(filtered_msg)
        
    def angular_filter(self, ranges, angle_min, angle_increment):
        """Filtra apenas ângulos de 90° a 270°"""
        filtered = ranges.copy()
        
        for i in range(len(ranges)):
            current_angle = angle_min + i * angle_increment
            
            # Normaliza ângulo para 0 a 2π
            while current_angle < 0:
                current_angle += 2 * math.pi
            while current_angle >= 2 * math.pi:
                current_angle -= 2 * math.pi
            
            # Se não está entre 90° e 270°, remove
            if not (self.angle_filter_min <= current_angle <= self.angle_filter_max):
                filtered[i] = float('inf')
                
        return filtered


def main(args=None):
    rclpy.init(args=args)
    lidar_filter = LidarFilter()
    try:
        rclpy.spin(lidar_filter)
    except KeyboardInterrupt:
        lidar_filter.get_logger().info('� Filtro LIDAR finalizado')
    lidar_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        
    def scan_callback(self, msg):
        """Filtra dados do LIDAR com múltiplos filtros anti-ruído"""
        # Cria cópia da mensagem
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = self.min_range
        filtered_msg.range_max = self.max_range
        
        # Processa ranges
        ranges = np.array(msg.ranges)
        
        # === SEQUÊNCIA DE FILTROS SIMPLIFICADA ===
        
        # 1. Filtro de distância básico
        ranges = np.where(ranges < self.min_range, float('inf'), ranges)
        ranges = np.where(ranges > self.max_range, float('inf'), ranges)
        
        # 2. FILTRO ANGULAR: Apenas frente e laterais (90° a 270°)
        ranges = self.angular_filter(ranges, msg.angle_min, msg.angle_increment)
        
        # 3. Filtro mediano simples
        ranges = self.median_filter(ranges)
        
        # 4. Filtro de outliers simples
        ranges = self.simple_outlier_filter(ranges)
        
        filtered_msg.ranges = ranges.tolist()
        filtered_msg.intensities = msg.intensities
        
        self.scan_pub.publish(filtered_msg)
        
    def median_filter(self, ranges):
        """Aplica filtro mediano para suavizar dados"""
        filtered = ranges.copy()
        half_size = self.median_filter_size // 2
        
        for i in range(half_size, len(ranges) - half_size):
            if not np.isinf(ranges[i]):
                window = ranges[i-half_size:i+half_size+1]
                valid_points = window[~np.isinf(window)]
                if len(valid_points) >= 2:
                    filtered[i] = np.median(valid_points)
        
        return filtered
        
    def outlier_filter(self, ranges):
        """Remove pontos que diferem muito dos vizinhos"""
        filtered = ranges.copy()
        
        for i in range(1, len(ranges) - 1):
            if not np.isinf(ranges[i]):
                # Pega vizinhos válidos
                neighbors = []
                for j in range(max(0, i-2), min(len(ranges), i+3)):
                    if j != i and not np.isinf(ranges[j]):
                        neighbors.append(ranges[j])
                
                if len(neighbors) >= 2:
                    mean_neighbor = np.mean(neighbors)
                    # Se o ponto difere muito da média dos vizinhos, remove
                    if abs(ranges[i] - mean_neighbor) > self.max_range_diff:
                        filtered[i] = float('inf')
        
        return filtered
        
    def cluster_filter(self, ranges):
        """Remove clusters muito pequenos (provavelmente ruído)"""
        filtered = ranges.copy()
        
        # Identifica clusters de pontos válidos
        in_cluster = False
        cluster_start = 0
        
        for i in range(len(ranges)):
            if not np.isinf(ranges[i]):
                if not in_cluster:
                    in_cluster = True
                    cluster_start = i
            else:
                if in_cluster:
                    # Fim do cluster
                    cluster_size = i - cluster_start
                    if cluster_size < self.min_points_cluster:
                        # Cluster muito pequeno, remove
                        filtered[cluster_start:i] = float('inf')
                    in_cluster = False
        
        # Verifica último cluster
        if in_cluster:
            cluster_size = len(ranges) - cluster_start
            if cluster_size < self.min_points_cluster:
                filtered[cluster_start:] = float('inf')
        
        return filtered
        
    def temporal_filter(self, ranges):
        """Filtra pontos que não são consistentes no tempo"""
        # Adiciona scan atual ao histórico
        self.scan_history.append(ranges.copy())
        
        # Mantém apenas os últimos N scans
        if len(self.scan_history) > self.history_size:
            self.scan_history.pop(0)
        
        # Se não temos histórico suficiente, retorna como está
        if len(self.scan_history) < 2:
            return ranges
        
        filtered = ranges.copy()
        
        # Para cada ponto, verifica consistência no tempo
        for i in range(len(ranges)):
            if not np.isinf(ranges[i]):
                # Conta quantos scans anteriores veem um ponto próximo nesta posição
                consistent_count = 0
                
                for historical_scan in self.scan_history[:-1]:  # Exclui o atual
                    if not np.isinf(historical_scan[i]):
                        # Se a diferença é pequena, considera consistente
                        if abs(ranges[i] - historical_scan[i]) < 0.3:
                            consistent_count += 1
                
                # Se não é consistente o suficiente, remove
                consistency_ratio = consistent_count / (len(self.scan_history) - 1)
                if consistency_ratio < self.consistency_threshold:
                    filtered[i] = float('inf')
        
        return filtered
    
    def isolated_points_filter(self, ranges):
        """
        FILTRO PRINCIPAL ANTI-RUÍDO: Remove pontos que não têm vizinhos suficientes
        Este é o filtro mais importante para evitar pontos soltos no mapa
        """
        filtered = ranges.copy()
        
        for i in range(len(ranges)):
            if not np.isinf(ranges[i]):
                # Conta vizinhos próximos em distância (não apenas posição angular)
                neighbor_count = 0
                current_range = ranges[i]
                
                # Busca vizinhos em um raio angular ao redor do ponto
                for j in range(max(0, i - self.neighbor_search_radius), 
                             min(len(ranges), i + self.neighbor_search_radius + 1)):
                    if j != i and not np.isinf(ranges[j]):
                        # Verifica se o vizinho está próximo em distância (não só ângulo)
                        range_diff = abs(ranges[j] - current_range)
                        if range_diff <= self.neighbor_distance_threshold:
                            neighbor_count += 1
                
                # Se não tem vizinhos suficientes, REJEITA o ponto
                if neighbor_count < self.min_neighbors_required:
                    filtered[i] = float('inf')
                    # Log para debug (opcional)
                    # self.get_logger().debug(f'Rejeitando ponto isolado no índice {i}: {neighbor_count} vizinhos')
        
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
