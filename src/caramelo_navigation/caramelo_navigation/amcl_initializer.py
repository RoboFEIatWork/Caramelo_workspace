#!/usr/bin/env python3
"""
AMCL INITIALIZER - Força inicialização do AMCL

Este node publica a pose inicial automaticamente para garantir
que o AMCL converja e crie o frame 'map'.

Autor: GitHub Copilot
Data: 2025-07-11
"""

import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class AMCLInitializer(Node):
    def __init__(self):
        super().__init__('amcl_initializer')
        
        # Publisher para pose inicial
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )
        
        # Aguarda um pouco para garantir que o AMCL está ativo
        self.timer = self.create_timer(3.0, self.publish_initial_pose)
        self.published = False
        
        self.get_logger().info("🎯 AMCL Initializer iniciado - aguardando 3s...")
        
    def publish_initial_pose(self):
        """Publica pose inicial para forçar convergência do AMCL"""
        if self.published:
            return
            
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Pose inicial na origem do mapa
        initial_pose.pose.pose.position.x = 0.0
        initial_pose.pose.pose.position.y = 0.0
        initial_pose.pose.pose.position.z = 0.0
        initial_pose.pose.pose.orientation.x = 0.0
        initial_pose.pose.pose.orientation.y = 0.0
        initial_pose.pose.pose.orientation.z = 0.0
        initial_pose.pose.pose.orientation.w = 1.0
        
        # Covariância relativamente baixa (robô está bem localizado)
        initial_pose.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787
        ]
        
        self.initial_pose_pub.publish(initial_pose)
        self.get_logger().info("✅ Pose inicial publicada para AMCL - Frame 'map' deve aparecer!")
        
        # Publica várias vezes para garantir que o AMCL receba
        for i in range(5):
            time.sleep(0.2)
            self.initial_pose_pub.publish(initial_pose)
            
        self.published = True
        self.timer.cancel()
        
        self.get_logger().info("🎯 AMCL inicializado com sucesso!")


def main(args=None):
    rclpy.init(args=args)
    node = AMCLInitializer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
