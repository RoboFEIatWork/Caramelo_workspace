#!/usr/bin/env python3
"""
Goal Pose Mapping - Permite controlar o robô via goal_pose do RViz durante mapeamento
"""

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class GoalPoseMapping(Node):
    def __init__(self):
        super().__init__('goalpose_mapping')
        
        # Action client para Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Subscriber para goal_pose do RViz
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10
        )
        
        # Publisher para cmd_vel (fallback direto)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Estado atual
        self.current_goal_handle = None
        
        self.get_logger().info("🎮 Goal Pose Mapping iniciado!")
        self.get_logger().info("📍 Use '2D Nav Goal' no RViz para mover o robô durante mapeamento")
        self.get_logger().info("⚠️ ATENÇÃO: Certifique-se que PWM e Encoder estão rodando!")
        
    def goal_pose_callback(self, msg):
        """Callback quando recebe goal_pose do RViz"""
        self.get_logger().info(f"🎯 Novo goal recebido: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")
        
        # Cancelar goal anterior se existir
        if self.current_goal_handle:
            self.get_logger().info("🛑 Cancelando goal anterior")
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None
        
        # Enviar novo goal para Nav2
        self.send_nav_goal(msg)
    
    def send_nav_goal(self, goal_pose):
        """Enviar goal para o Nav2"""
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("⚠️ Nav2 server não disponível - mapeamento sem navegação")
            return False
        
        # Criar goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.get_logger().info(f"🚀 Enviando goal para Nav2...")
        
        # Enviar goal
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        
        return True
    
    def goal_response_callback(self, future):
        """Callback quando o goal é aceito/rejeitado"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejeitado pelo Nav2!")
            return
        
        self.get_logger().info("✅ Goal aceito pelo Nav2!")
        self.current_goal_handle = goal_handle
        
        # Aguardar resultado
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """Callback quando a navegação termina"""
        result = future.result()
        
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("✅ Goal alcançado com sucesso!")
        elif result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("🛑 Goal cancelado")
        else:
            self.get_logger().warn(f"⚠️ Goal falhou: {result.status}")
        
        self.current_goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    
    node = GoalPoseMapping()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Parar o robô
        cmd = Twist()
        node.cmd_vel_pub.publish(cmd)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
