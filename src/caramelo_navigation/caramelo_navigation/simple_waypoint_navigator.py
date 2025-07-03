#!/usr/bin/env python3

import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class SimpleWaypointNavigator(Node):
    def __init__(self):
        super().__init__('simple_waypoint_navigator')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)  # Usar filtro de segurança
        self.status_pub = self.create_publisher(String, '/navigation_status', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Estado atual
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # Dados do lidar para detecção de obstáculos
        self.current_scan = None
        self.min_obstacle_distance = 0.5  # AUMENTADO: 50cm mínimo de distância para maior segurança
        
        # Waypoint atual
        self.goal_x = None
        self.goal_y = None
        self.goal_theta = None
        
        # Parâmetros de navegação
        self.linear_tolerance = 0.15  # 15cm de tolerância para ser mais seguro
        self.angular_tolerance = 0.2  # ~11° de tolerância
        self.max_linear_speed = 0.20  # REDUZIDO: 20cm/s máximo para maior segurança
        self.max_angular_speed = 0.3  # REDUZIDO: 0.3 rad/s máximo
        
        # Timer para controle
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('🎯 Simple Waypoint Navigator iniciado!')
        self.get_logger().info('💡 Sistema com detecção de obstáculos ativado!')
        self.get_logger().info('🎮 Publique goals em /goal_pose para navegar')
        
    def scan_callback(self, msg):
        """Atualiza dados do lidar para detecção de obstáculos"""
        self.current_scan = msg
        
    def check_obstacles_ahead(self):
        """Verifica se há obstáculos à frente usando dados do lidar"""
        if self.current_scan is None:
            return False
            
        # Verificar apenas a região frontal (±45° à frente para maior segurança)
        ranges = np.array(self.current_scan.ranges)
        angle_min = self.current_scan.angle_min
        angle_increment = self.current_scan.angle_increment
        
        # Calcular índices para ±45° (π/4 rad) - região mais ampla para detecção
        front_angle_range = math.pi / 4
        total_angles = len(ranges)
        center_index = total_angles // 2
        front_indices = int(front_angle_range / angle_increment)
        
        start_idx = max(0, center_index - front_indices)
        end_idx = min(total_angles, center_index + front_indices)
        
        front_ranges = ranges[start_idx:end_idx]
        
        # Remover valores infinitos e inválidos
        valid_ranges = front_ranges[np.isfinite(front_ranges)]
        valid_ranges = valid_ranges[valid_ranges > 0.05]  # Ignorar ruído muito baixo
        
        if len(valid_ranges) == 0:
            return False
            
        min_distance = np.min(valid_ranges)
        
        # Se a distância mínima for menor que o threshold, há obstáculo
        obstacle_detected = min_distance < self.min_obstacle_distance
        
        if obstacle_detected:
            self.get_logger().warn(f'⚠️ OBSTÁCULO DETECTADO! Distância mínima: {min_distance:.2f}m (< {self.min_obstacle_distance}m)')
            
        return obstacle_detected
        
    def odom_callback(self, msg):
        """Atualiza posição atual do robô"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Converter quaternion para yaw
        quat = msg.pose.pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)
        
    def goal_callback(self, msg):
        """Recebe novo objetivo de navegação"""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        
        # Converter quaternion do goal para yaw
        quat = msg.pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        self.goal_theta = math.atan2(siny_cosp, cosy_cosp)
        
        self.get_logger().info(f'🎯 Novo objetivo: ({self.goal_x:.2f}, {self.goal_y:.2f}, {math.degrees(self.goal_theta):.1f}°)')
        
        status_msg = String()
        status_msg.data = f"NAVIGATING_TO_{self.goal_x:.2f}_{self.goal_y:.2f}"
        self.status_pub.publish(status_msg)
        
    def control_loop(self):
        """Loop principal de controle de navegação"""
        if self.goal_x is None or self.goal_y is None:
            return
            
        # PROTEÇÃO CONTRA OBSTÁCULOS - PRIORIDADE MÁXIMA
        if self.check_obstacles_ahead():
            # PARAR IMEDIATAMENTE se há obstáculo à frente
            cmd = Twist()  # Velocidades zero
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            
            # Cancelar objetivo atual
            self.get_logger().warn('🛑 COLISÃO EVITADA! Objetivo cancelado por segurança.')
            status_msg = String()
            status_msg.data = "OBSTACLE_DETECTED_GOAL_CANCELLED"
            self.status_pub.publish(status_msg)
            
            # Limpar objetivo
            self.goal_x = None
            self.goal_y = None
            self.goal_theta = None
            return
            
        # Calcular distância e ângulo para o objetivo
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance = math.sqrt(dx*dx + dy*dy)
        angle_to_goal = math.atan2(dy, dx)
        
        # Diferença angular
        angle_error = self.normalize_angle(angle_to_goal - self.current_theta)
        
        cmd = Twist()
        
        # Se chegou na posição
        if distance < self.linear_tolerance:
            # Ajustar orientação final
            if self.goal_theta is not None:
                final_angle_error = self.normalize_angle(self.goal_theta - self.current_theta)
                if abs(final_angle_error) > self.angular_tolerance:
                    cmd.angular.z = self.clamp(final_angle_error * 2.0, -self.max_angular_speed, self.max_angular_speed)
                else:
                    # Chegou no objetivo!
                    self.get_logger().info('✅ Objetivo alcançado!')
                    status_msg = String()
                    status_msg.data = "GOAL_REACHED"
                    self.status_pub.publish(status_msg)
                    self.goal_x = None
                    self.goal_y = None
                    self.goal_theta = None
            else:
                # Sem orientação específica, chegou!
                self.get_logger().info('✅ Posição alcançada!')
                status_msg = String()
                status_msg.data = "POSITION_REACHED"
                self.status_pub.publish(status_msg)
                self.goal_x = None
                self.goal_y = None
        else:
            # Navegar para o objetivo
            # Primeiro girar na direção certa
            if abs(angle_error) > 0.3:  # ~17°
                cmd.angular.z = self.clamp(angle_error * 2.0, -self.max_angular_speed, self.max_angular_speed)
                cmd.linear.x = 0.0  # Não andar enquanto gira muito
            else:
                # Andar para frente com pequenas correções angulares
                # VERIFICAÇÃO DUPLA: só andar se não há obstáculos
                if not self.check_obstacles_ahead():
                    cmd.linear.x = self.clamp(distance * 2.0, 0.1, self.max_linear_speed)
                    cmd.angular.z = self.clamp(angle_error * 1.0, -self.max_angular_speed/2, self.max_angular_speed/2)
                else:
                    # Obstáculo detectado - apenas rotação é permitida
                    cmd.linear.x = 0.0
                    cmd.angular.z = self.clamp(angle_error * 1.0, -self.max_angular_speed/2, self.max_angular_speed/2)
        
        # ÚLTIMA VERIFICAÇÃO DE SEGURANÇA antes de publicar
        if cmd.linear.x > 0.0 and self.check_obstacles_ahead():
            self.get_logger().warn('🛑 Bloqueando movimento linear por obstáculo!')
            cmd.linear.x = 0.0
            
        self.cmd_vel_pub.publish(cmd)
        
        # Debug info
        if self.goal_x is not None:
            self.get_logger().debug(f'Pos: ({self.current_x:.2f}, {self.current_y:.2f}) | '
                                   f'Goal: ({self.goal_x:.2f}, {self.goal_y:.2f}) | '
                                   f'Dist: {distance:.2f}m | Ang: {math.degrees(angle_error):.1f}°')
    
    def normalize_angle(self, angle):
        """Normaliza ângulo para [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def clamp(self, value, min_val, max_val):
        """Limita valor entre min e max"""
        return max(min_val, min(value, max_val))

def main(args=None):
    rclpy.init(args=args)
    navigator = SimpleWaypointNavigator()
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
