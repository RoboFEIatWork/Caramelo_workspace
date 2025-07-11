#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Header


class CmdVelMonitor(Node):
    def __init__(self):
        super().__init__('cmd_vel_monitor')
        
        # Subscriber para monitorar comandos de velocidade
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.last_cmd_time = time.time()
        self.cmd_count = 0
        
        # Timer para estatísticas
        self.stats_timer = self.create_timer(5.0, self.print_stats)
        
        self.get_logger().info('� Monitor de /cmd_vel iniciado!')
        self.get_logger().info('� Estatísticas a cada 5 segundos')
        
    def cmd_vel_callback(self, msg):
        """Monitor de comandos /cmd_vel"""
        current_time = time.time()
        self.cmd_count += 1
        
        # Log apenas se há movimento significativo
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.get_logger().info(f'🎮 CMD_VEL: linear={msg.linear.x:.3f}, angular={msg.angular.z:.3f}')
        
        self.last_cmd_time = current_time
        
    def print_stats(self):
        """Imprime estatísticas de uso"""
        current_time = time.time()
        time_since_last = current_time - self.last_cmd_time
        
        self.get_logger().info(f'📊 Comandos recebidos: {self.cmd_count} | '
                              f'Último comando há {time_since_last:.1f}s')
        
        if time_since_last > 10.0:
            self.get_logger().info('💤 Robô inativo há mais de 10 segundos')


def main(args=None):
    rclpy.init(args=args)
    monitor = CmdVelMonitor()
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info('🛑 Monitor finalizado')
    monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
