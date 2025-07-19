import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os

class ManipListener(Node):
    def __init__(self):
        super().__init__('manip_listener')
        self.subscription = self.create_subscription(
            String,
            '/manip_cmd',
            self.listener_callback,
            10
        )
        self.get_logger().info("🟡 Aguardando comando no tópico /manip_cmd...")

    def listener_callback(self, msg):
        if msg.data.strip().lower() == 'start':
            self.get_logger().info("🟢 Comando recebido. Rodando caramelo_manip.py com venv 3.10")

            venv_python = os.path.expanduser('~/Caramelo_workspace/venv310/bin/python3')
            script_path = os.path.expanduser('~/Caramelo_workspace/src/caramelo_manipulation/caramelo_manipulation/caramelo_manip.py')

            try:
                subprocess.run([venv_python, script_path], check=True)
                self.get_logger().info("✅ Execução finalizada com sucesso.")
            except subprocess.CalledProcessError as e:
                self.get_logger().error(f"❌ Erro ao executar caramelo_manip.py: {e}")
        else:
            self.get_logger().warn(f"⚠️ Comando desconhecido recebido: '{msg.data}'")

def main(args=None):
    rclpy.init(args=args)
    node = ManipListener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
