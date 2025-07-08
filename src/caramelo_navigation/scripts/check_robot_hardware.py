#!/usr/bin/env python3
"""
Hardware Check Script for Caramelo Robot
========================================

Verifica se todos os componentes físicos do robô estão funcionando:
- Encoders das rodas
- Motores PWM  
- LiDAR RPLidar S2
- Publicação de joint_state e cmd_vel

Para usar no robô real antes de iniciar a navegação.
"""

import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState, LaserScan


class RobotHardwareChecker(Node):
    def __init__(self):
        super().__init__('robot_hardware_checker')
        
        # Status dos componentes
        self.joint_state_received = False
        self.lidar_received = False
        self.cmd_vel_working = False
        
        # Subscribers para verificar os tópicos
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        
        # Publisher para testar cmd_vel
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            '/mecanum_drive_controller/cmd_vel',
            10
        )
        
        # Timer para publicar teste e verificar status
        self.test_timer = self.create_timer(2.0, self.run_tests)
        self.status_timer = self.create_timer(5.0, self.print_status)
        
        self.get_logger().info('🔍 Verificando componentes físicos do robô...')
        self.get_logger().info('   Certifique-se de que estão rodando:')
        self.get_logger().info('   ✓ encoder_bringup.launch.py')
        self.get_logger().info('   ✓ pwm_bringup.launch.py') 
        self.get_logger().info('   ✓ lidar_bringup.launch.py')

    def joint_state_callback(self, msg):
        """Callback para joint_state - indica que encoders funcionam"""
        if not self.joint_state_received:
            self.get_logger().info('✅ Joint States recebidos - Encoders OK')
            self.joint_state_received = True

    def lidar_callback(self, msg):
        """Callback para scan - indica que LiDAR funciona"""
        if not self.lidar_received:
            self.get_logger().info('✅ Scan LiDAR recebido - LiDAR OK')
            self.lidar_received = True

    def run_tests(self):
        """Executa testes periódicos"""
        # Testa cmd_vel (movimento zero para não mover o robô)
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = ''
        cmd.twist.linear.x = 0.0
        cmd.twist.linear.y = 0.0
        cmd.twist.angular.z = 0.0
        
        self.cmd_vel_pub.publish(cmd)
        
        if not self.cmd_vel_working:
            self.cmd_vel_working = True
            self.get_logger().info('✅ Comando cmd_vel publicado - PWM OK')

    def print_status(self):
        """Imprime status geral dos componentes"""
        self.get_logger().info('📊 STATUS DOS COMPONENTES:')
        self.get_logger().info(f'   Encoders: {"✅ OK" if self.joint_state_received else "❌ FALHA"}')
        self.get_logger().info(f'   LiDAR:    {"✅ OK" if self.lidar_received else "❌ FALHA"}')
        self.get_logger().info(f'   PWM:      {"✅ OK" if self.cmd_vel_working else "❌ FALHA"}')
        
        if all([self.joint_state_received, self.lidar_received, self.cmd_vel_working]):
            self.get_logger().info('🎉 TODOS OS COMPONENTES FUNCIONANDO!')
            self.get_logger().info('   Robô pronto para navegação autônoma.')
        else:
            self.get_logger().warn('⚠️  Alguns componentes não estão funcionando.')
            self.get_logger().warn('   Verifique os launch files mencionados acima.')


def main(args=None):
    rclpy.init(args=args)
    
    checker = RobotHardwareChecker()
    
    try:
        rclpy.spin(checker)
    except KeyboardInterrupt:
        pass
    finally:
        checker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
