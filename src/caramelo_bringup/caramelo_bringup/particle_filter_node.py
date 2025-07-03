#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class SimpleParticleFilter(Node):
    def __init__(self):
        super().__init__('simple_particle_filter')
        
        # Publishers
        self.filtered_odom_pub = self.create_publisher(Odometry, '/odom_filtered', 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/pose_filtered', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Filtro de partículas simples
        self.num_particles = 100
        self.particles = self.initialize_particles()
        self.weights = np.ones(self.num_particles) / self.num_particles
        
        # Estado filtrado
        self.filtered_x = 0.0
        self.filtered_y = 0.0
        self.filtered_theta = 0.0
        self.covariance = np.eye(3) * 0.1  # Covariância inicial
        
        # Dados dos sensores
        self.last_odom = None
        self.current_scan = None
        
        # Parâmetros do filtro
        self.motion_noise_std = 0.02    # Ruído do movimento
        self.measurement_noise_std = 0.1 # Ruído do sensor
        self.resample_threshold = 0.5    # Threshold para reamostragem
        
        self.get_logger().info('🔬 Filtro de Partículas iniciado!')
        
    def initialize_particles(self):
        """Inicializa partículas ao redor da origem"""
        particles = np.zeros((self.num_particles, 3))  # [x, y, theta]
        particles[:, 0] = np.random.normal(0.0, 0.1, self.num_particles)  # x
        particles[:, 1] = np.random.normal(0.0, 0.1, self.num_particles)  # y  
        particles[:, 2] = np.random.normal(0.0, 0.1, self.num_particles)  # theta
        return particles
        
    def odom_callback(self, msg):
        """Atualização de movimento das partículas"""
        if self.last_odom is None:
            self.last_odom = msg
            return
            
        # Calcular movimento desde a última odometria
        dx = msg.pose.pose.position.x - self.last_odom.pose.pose.position.x
        dy = msg.pose.pose.position.y - self.last_odom.pose.pose.position.y
        
        # Converter quaternion para yaw
        quat_curr = msg.pose.pose.orientation
        quat_last = self.last_odom.pose.pose.orientation
        
        theta_curr = self.quat_to_yaw(quat_curr)
        theta_last = self.quat_to_yaw(quat_last)
        dtheta = self.normalize_angle(theta_curr - theta_last)
        
        # Propagar partículas com ruído
        self.predict_particles(dx, dy, dtheta)
        
        # Se temos dados do lidar, fazer correção
        if self.current_scan is not None:
            self.update_particles()
            
        # Estimar pose filtrada
        self.estimate_pose()
        
        # Publicar resultado
        self.publish_filtered_odometry(msg.header.stamp)
        
        self.last_odom = msg
        
    def scan_callback(self, msg):
        """Armazena dados do lidar para correção"""
        self.current_scan = msg
        
    def predict_particles(self, dx, dy, dtheta):
        """Predição das partículas com modelo de movimento"""
        for i in range(self.num_particles):
            # Adicionar ruído gaussiano ao movimento
            noise_x = np.random.normal(0, self.motion_noise_std)
            noise_y = np.random.normal(0, self.motion_noise_std)
            noise_theta = np.random.normal(0, self.motion_noise_std)
            
            # Atualizar partícula
            self.particles[i, 0] += dx + noise_x
            self.particles[i, 1] += dy + noise_y
            self.particles[i, 2] = self.normalize_angle(self.particles[i, 2] + dtheta + noise_theta)
            
    def update_particles(self):
        """Atualização dos pesos das partículas baseada no lidar"""
        if self.current_scan is None:
            return
            
        # Simplificação: usar consistência do lidar como peso
        scan_ranges = np.array(self.current_scan.ranges)
        valid_ranges = scan_ranges[np.isfinite(scan_ranges)]
        
        if len(valid_ranges) == 0:
            return
            
        # Métrica simples: variabilidade dos ranges (paredes devem ser consistentes)
        scan_std = np.std(valid_ranges)
        scan_mean = np.mean(valid_ranges)
        
        # Atualizar pesos (partículas mais consistentes têm maior peso)
        for i in range(self.num_particles):
            # Peso baseado na posição da partícula
            # Simplificação: partículas próximas ao centro têm peso ligeiramente maior
            dist_from_center = np.sqrt(self.particles[i, 0]**2 + self.particles[i, 1]**2)
            
            # Peso inversamente proporcional à distância + ruído
            weight = 1.0 / (1.0 + dist_from_center * 0.1)
            weight += np.random.normal(0, self.measurement_noise_std)
            
            self.weights[i] = max(weight, 0.001)  # Peso mínimo
            
        # Normalizar pesos
        self.weights /= np.sum(self.weights)
        
        # Reamostragem se necessário
        effective_particles = 1.0 / np.sum(self.weights**2)
        if effective_particles < self.num_particles * self.resample_threshold:
            self.resample_particles()
            
    def resample_particles(self):
        """Reamostragem das partículas"""
        indices = np.random.choice(self.num_particles, self.num_particles, p=self.weights)
        self.particles = self.particles[indices]
        self.weights = np.ones(self.num_particles) / self.num_particles
        
        self.get_logger().debug('🔄 Reamostragem de partículas realizada')
        
    def estimate_pose(self):
        """Estima pose filtrada a partir das partículas"""
        # Média ponderada das partículas
        self.filtered_x = np.average(self.particles[:, 0], weights=self.weights)
        self.filtered_y = np.average(self.particles[:, 1], weights=self.weights)
        
        # Para ângulos, usar média circular
        sin_theta = np.average(np.sin(self.particles[:, 2]), weights=self.weights)
        cos_theta = np.average(np.cos(self.particles[:, 2]), weights=self.weights)
        self.filtered_theta = math.atan2(sin_theta, cos_theta)
        
        # Estimar covariância
        weighted_diff_x = (self.particles[:, 0] - self.filtered_x) * np.sqrt(self.weights)
        weighted_diff_y = (self.particles[:, 1] - self.filtered_y) * np.sqrt(self.weights)
        
        self.covariance[0, 0] = np.sum(weighted_diff_x**2)  # var_x
        self.covariance[1, 1] = np.sum(weighted_diff_y**2)  # var_y
        self.covariance[2, 2] = 0.1  # Variância angular aproximada
        
    def publish_filtered_odometry(self, timestamp):
        """Publica odometria filtrada"""
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        
        # Pose filtrada
        odom_msg.pose.pose.position.x = self.filtered_x
        odom_msg.pose.pose.position.y = self.filtered_y
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientação
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.filtered_theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.filtered_theta / 2.0)
        
        # Covariância
        for i in range(3):
            for j in range(3):
                if i < 2 and j < 2:  # x, y
                    odom_msg.pose.covariance[i*6 + j] = self.covariance[i, j]
                elif i == 5 and j == 5:  # yaw
                    odom_msg.pose.covariance[i*6 + j] = self.covariance[2, 2]
                    
        self.filtered_odom_pub.publish(odom_msg)
        
    def quat_to_yaw(self, quat):
        """Converte quaternion para yaw"""
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def normalize_angle(self, angle):
        """Normaliza ângulo para [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    pf = SimpleParticleFilter()
    try:
        rclpy.spin(pf)
    except KeyboardInterrupt:
        pass
    pf.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
