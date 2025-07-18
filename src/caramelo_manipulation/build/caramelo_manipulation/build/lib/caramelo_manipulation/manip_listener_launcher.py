import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess

class ManipListener(Node):
    def __init__(self):
        super().__init__('manip_listener')
        self.subscription = self.create_subscription(
            String,
            'manip_cmd',
            self.listener_callback,
            10
        )
        self.get_logger().info("🟡 Aguardando comando no tópico /manip_cmd...")

    def listener_callback(self, msg):
        if msg.data.strip().lower() == 'start':
            self.get_logger().info("🟢 Comando recebido. Rodando caramelo_manip.py com venv 3.10")
            try:
                subprocess.run([
                    '/home/linux24-04/Caramelo_workspace/venv310/bin/python3',
                    '/home/linux24-04/Caramelo_workspace/src/caramelo_manipulation/caramelo_manipulation/caramelo_manip.py'
                ], check=True)
            except subprocess.CalledProcessError:
                self.get_logger().error("❌ Erro durante execução da manipulação.")
            else:
                self.get_logger().info("✅ Manipulação finalizada com sucesso.")

def main(args=None):
    rclpy.init(args=args)
    node = ManipListener()
    rclpy.spin(node)
    rclpy.shutdown()
