#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class CmdVelSafetyFilter(Node):
    """
    Filtro de segurança para comandos de velocidade.
    
    - Recebe comandos em /cmd_vel_raw
    - Aplica filtros de segurança
    - Publica comandos seguros em /cmd_vel
    """
    
    def __init__(self):
        super().__init__('cmd_vel_safety_filter')
        
        # Subscriber para comandos brutos
        self.cmd_raw_sub = self.create_subscription(
            Twist, '/cmd_vel_raw', self.cmd_raw_callback, 10)
        
        # Subscriber para status de navegação (para controle de prioridade)
        self.nav_status_sub = self.create_subscription(
            String, '/navigation_status', self.nav_status_callback, 10)
        
        # Publisher para comandos filtrados
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Estado do sistema
        self.last_cmd_time = time.time()
        self.navigation_active = False
        self.last_nav_status = "IDLE"
        
        # Limites de segurança
        self.max_linear_speed = 0.3   # 30cm/s máximo global
        self.max_angular_speed = 0.5  # 0.5 rad/s máximo global
        
        # Timer para timeout de comandos
        self.safety_timer = self.create_timer(0.5, self.safety_check)
        
        self.get_logger().info('🛡️ Filtro de Segurança CMD_VEL iniciado!')
        self.get_logger().info('📥 Comandos recebidos em: /cmd_vel_raw')
        self.get_logger().info('📤 Comandos filtrados em: /cmd_vel')
        
    def nav_status_callback(self, msg):
        """Monitora status da navegação"""
        self.last_nav_status = msg.data
        self.navigation_active = "NAVIGATING" in msg.data or "EXPLORING" in msg.data
        
    def cmd_raw_callback(self, msg):
        """Processa e filtra comandos de velocidade"""
        current_time = time.time()
        self.last_cmd_time = current_time
        
        # Aplicar limites de segurança
        filtered_cmd = Twist()
        
        # Limitar velocidades máximas
        filtered_cmd.linear.x = max(-self.max_linear_speed, 
                                   min(self.max_linear_speed, msg.linear.x))
        filtered_cmd.linear.y = max(-self.max_linear_speed, 
                                   min(self.max_linear_speed, msg.linear.y))
        filtered_cmd.angular.z = max(-self.max_angular_speed, 
                                    min(self.max_angular_speed, msg.angular.z))
        
        # Log de comandos significativos
        if abs(filtered_cmd.linear.x) > 0.01 or abs(filtered_cmd.angular.z) > 0.01:
            self.get_logger().debug(f'🛡️ CMD filtrado: linear={filtered_cmd.linear.x:.3f}, '
                                   f'angular={filtered_cmd.angular.z:.3f} | Status: {self.last_nav_status}')
        
        # Publicar comando filtrado
        self.cmd_vel_pub.publish(filtered_cmd)
        
    def safety_check(self):
        """Verificação de segurança periódica"""
        current_time = time.time()
        time_since_last = current_time - self.last_cmd_time
        
        # Se não há comandos há muito tempo, publicar comando zero para garantir parada
        if time_since_last > 2.0:
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            
            if time_since_last > 10.0 and self.navigation_active:
                self.get_logger().warn('⚠️ Navegação ativa mas sem comandos há muito tempo!')


def main(args=None):
    rclpy.init(args=args)
    safety_filter = CmdVelSafetyFilter()
    try:
        rclpy.spin(safety_filter)
    except KeyboardInterrupt:
        safety_filter.get_logger().info('🛑 Filtro de segurança finalizado')
    safety_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
