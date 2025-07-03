#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class GoalTester(Node):
    def __init__(self):
        super().__init__('goal_tester')
        
        # Publisher para goal
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Timer para enviar goal de teste
        self.timer = self.create_timer(5.0, self.send_test_goal)
        self.goal_count = 0
        
        self.get_logger().info('🎯 Goal Tester iniciado! Enviando goals de teste a cada 5s...')
        
    def send_test_goal(self):
        """Envia goals de teste simples"""
        goals = [
            (1.0, 0.0),    # 1 metro para frente
            (1.0, 1.0),    # diagonal
            (0.0, 1.0),    # 1 metro para a esquerda
            (0.0, 0.0),    # voltar para origem
        ]
        
        if self.goal_count < len(goals):
            x, y = goals[self.goal_count]
            
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'odom'
            
            goal_msg.pose.position.x = x
            goal_msg.pose.position.y = y
            goal_msg.pose.position.z = 0.0
            
            # Orientação padrão
            goal_msg.pose.orientation.w = 1.0
            
            self.goal_pub.publish(goal_msg)
            
            self.get_logger().info(f'🎯 Enviando goal {self.goal_count + 1}: ({x:.1f}, {y:.1f})')
            self.goal_count += 1
        else:
            self.get_logger().info('✅ Teste de goals completo!')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    tester = GoalTester()
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
