#!/usr/bin/env python3
"""
AMCL Initializer - Inicialização automática do AMCL
Publica pose inicial para estabelecer o frame map->odom
"""

import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_srvs.srv import Empty


class AMCLInitializer(Node):
    
    def __init__(self):
        super().__init__('amcl_initializer')
        
        # Parâmetros com valores padrão
        self.declare_parameter('initial_pose_x', 0.0)
        self.declare_parameter('initial_pose_y', 0.0) 
        self.declare_parameter('initial_pose_yaw', 0.0)
        
        self.initial_x = self.get_parameter('initial_pose_x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('initial_pose_y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('initial_pose_yaw').get_parameter_value().double_value
        
        # Publisher para pose inicial
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )
        
        # Cliente para global localization
        self.global_loc_client = self.create_client(Empty, '/global_localization')
        
        self.get_logger().info(f"🎯 AMCL Initializer: pose inicial ({self.initial_x}, {self.initial_y}, {self.initial_yaw})")
        
        # Timer para inicialização
        self.timer = self.create_timer(2.0, self.initialize_amcl)
        self.initialization_attempts = 0
        self.max_attempts = 5
        
    def initialize_amcl(self):
        """Inicializa AMCL com pose inicial"""
        
        if self.initialization_attempts >= self.max_attempts:
            self.get_logger().info("✅ AMCL inicialização completa")
            self.timer.cancel()
            return
            
        self.initialization_attempts += 1
        
        # Criar mensagem de pose inicial
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        # Posição
        pose_msg.pose.pose.position.x = self.initial_x
        pose_msg.pose.pose.position.y = self.initial_y
        pose_msg.pose.pose.position.z = 0.0
        
        # Orientação (quaternion do yaw)
        import math
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = math.sin(self.initial_yaw / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(self.initial_yaw / 2.0)
        
        # Covariância (incerteza inicial)
        pose_msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,      # x
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,      # y
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,       # z (não usado)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,       # roll (não usado)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,       # pitch (não usado)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.068      # yaw
        ]
        
        # Publicar pose inicial
        self.pose_pub.publish(pose_msg)
        self.get_logger().info(f"🎯 Tentativa {self.initialization_attempts}: Pose inicial publicada")
        
        # Também chamar global localization se disponível
        if self.global_loc_client.service_is_ready():
            req = Empty.Request()
            future = self.global_loc_client.call_async(req)
            self.get_logger().info("🌍 Global localization ativada")


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
