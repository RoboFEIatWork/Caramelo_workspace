#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import json
import time


class CheckpointNavigator(Node):
    def __init__(self):
        super().__init__('checkpoint_navigator')
        
        # Action client para navegação
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Carrega checkpoints
        self.checkpoints = self.load_checkpoints()
        self.current_checkpoint = 0
        
        self.get_logger().info(f'🤖 Checkpoint Navigator iniciado!')
        self.get_logger().info(f'📍 {len(self.checkpoints)} checkpoints carregados')
        
        # Aguarda Nav2
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('🔄 Aguardando Nav2...')
        
        self.get_logger().info('✅ Nav2 pronto! Use os comandos:')
        self.get_logger().info('   ros2 service call /navigate_to_checkpoint std_srvs/srv/Trigger')
        
    def load_checkpoints(self):
        """Carrega checkpoints do arquivo JSON"""
        try:
            with open('/home/work/Caramelo_workspace/checkpoints.json', 'r') as f:
                data = json.load(f)
                return data['checkpoints']
        except Exception as e:
            self.get_logger().error(f'❌ Erro ao carregar checkpoints: {e}')
            return []
    
    def navigate_to_checkpoint(self, checkpoint_index):
        """Navega para um checkpoint específico"""
        if checkpoint_index >= len(self.checkpoints):
            self.get_logger().error(f'❌ Checkpoint {checkpoint_index} não existe')
            return False
        
        checkpoint = self.checkpoints[checkpoint_index]
        
        # Cria goal de navegação
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Posição
        goal.pose.pose.position.x = checkpoint['position']['x']
        goal.pose.pose.position.y = checkpoint['position']['y']
        goal.pose.pose.position.z = checkpoint['position']['z']
        
        # Orientação
        goal.pose.pose.orientation.x = checkpoint['orientation']['x']
        goal.pose.pose.orientation.y = checkpoint['orientation']['y']
        goal.pose.pose.orientation.z = checkpoint['orientation']['z']
        goal.pose.pose.orientation.w = checkpoint['orientation']['w']
        
        self.get_logger().info(f'🚀 Navegando para: {checkpoint["name"]}')
        self.get_logger().info(f'📍 Posição: ({checkpoint["position"]["x"]:.2f}, {checkpoint["position"]["y"]:.2f})')
        
        # Envia objetivo
        future = self.nav_client.send_goal_async(goal)
        return future
    
    def list_checkpoints(self):
        """Lista todos os checkpoints"""
        self.get_logger().info('📋 Checkpoints disponíveis:')
        for i, cp in enumerate(self.checkpoints):
            self.get_logger().info(f'  {i}: {cp["name"]} - {cp["description"]}')


def main(args=None):
    rclpy.init(args=args)
    
    navigator = CheckpointNavigator()
    
    # Lista checkpoints
    navigator.list_checkpoints()
    
    # Exemplo: navegar para primeiro checkpoint
    if len(navigator.checkpoints) > 0:
        navigator.get_logger().info('🎯 Para navegar, use:')
        navigator.get_logger().info('   # Em outro terminal:')
        navigator.get_logger().info('   ros2 topic pub /navigate_to_checkpoint std_msgs/msg/Int32 "data: 0"')
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('🛑 Navegador interrompido')
    
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
