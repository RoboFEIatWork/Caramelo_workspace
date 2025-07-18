import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import subprocess

class ManipService(Node):
    def __init__(self):
        super().__init__('manip_service_node')
        self.srv = self.create_service(Trigger, 'start_manip', self.callback)
        self.get_logger().info("🟢 Serviço /start_manip disponível.")

    def callback(self, request, response):
        self.get_logger().info("🚀 Serviço acionado! Executando sequência de manipulação...")

        try:
            result = subprocess.run([
                '/home/linux24-04/Caramelo_workspace/venv310/bin/python3',
                '/home/linux24-04/Caramelo_workspace/src/caramelo_manipulation/caramelo_manipulation/caramelo_manip.py'
            ], check=True)

            response.success = True
            response.message = "✅ Manipulação executada com sucesso."
        except subprocess.CalledProcessError as e:
            response.success = False
            response.message = f"❌ Erro ao executar a manipulação: {e}"

        return response

def main(args=None):
    rclpy.init(args=args)
    node = ManipService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
