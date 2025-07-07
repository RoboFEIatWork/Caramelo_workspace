#!/usr/bin/env python3
"""
Interface de Manipulação para o Task Executor
Fornece funções mock e reais para manipulação de objetos
"""

import random
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String


class ManipulationInterface(Node):
    def __init__(self):
        super().__init__('manipulation_interface')
        
        # Publishers para comunicação com o manipulador
        self.command_pub = self.create_publisher(String, '/manipulation_command', 10)
        
        # Subscribers para status do manipulador
        self.status_sub = self.create_subscription(
            String,
            '/manipulation_status',
            self.status_callback,
            10
        )
        
        # Estado interno
        self.last_status = "idle"
        self.manipulation_success = False
        
        self.get_logger().info("🦾 Manipulation Interface iniciada!")
        
    def status_callback(self, msg):
        """Callback para status do manipulador"""
        self.last_status = msg.data
        if msg.data == "success":
            self.manipulation_success = True
        elif msg.data == "failed":
            self.manipulation_success = False
    
    def detect_object(self, object_name: str) -> bool:
        """
        Detecta se um objeto está presente
        Por enquanto é mock - retorna True com 70% de chance
        """
        self.get_logger().info(f"🔍 Detectando objeto: {object_name}")
        
        # Simula tempo de detecção
        time.sleep(1.0)
        
        # Mock: 70% chance de sucesso
        success = random.random() < 0.7
        
        if success:
            self.get_logger().info(f"✅ Objeto {object_name} detectado!")
        else:
            self.get_logger().warn(f"❌ Objeto {object_name} não encontrado")
        
        return success
    
    def pick_object(self, object_name: str) -> bool:
        """
        Executa movimento de pegar objeto
        Por enquanto é mock - retorna True com 60% de chance
        """
        self.get_logger().info(f"🤏 Pegando objeto: {object_name}")
        
        # Envia comando para o manipulador
        cmd = String()
        cmd.data = f"pick:{object_name}"
        self.command_pub.publish(cmd)
        
        # Simula tempo de manipulação
        time.sleep(2.0)
        
        # Mock: 60% chance de sucesso
        success = random.random() < 0.6
        
        if success:
            self.get_logger().info(f"✅ Objeto {object_name} coletado com sucesso!")
        else:
            self.get_logger().warn(f"❌ Falha ao coletar {object_name}")
        
        return success
    
    def place_object(self, object_name: str) -> bool:
        """
        Executa movimento de colocar objeto
        Por enquanto é mock - retorna True com 80% de chance
        """
        self.get_logger().info(f"📦 Colocando objeto: {object_name}")
        
        # Envia comando para o manipulador
        cmd = String()
        cmd.data = f"place:{object_name}"
        self.command_pub.publish(cmd)
        
        # Simula tempo de manipulação
        time.sleep(2.0)
        
        # Mock: 80% chance de sucesso (colocar é mais fácil que pegar)
        success = random.random() < 0.8
        
        if success:
            self.get_logger().info(f"✅ Objeto {object_name} colocado com sucesso!")
        else:
            self.get_logger().warn(f"❌ Falha ao colocar {object_name}")
        
        return success
    
    def execute_manipulation_sequence(self, action: str, object_name: str) -> bool:
        """
        Executa sequência completa de manipulação
        """
        self.get_logger().info(f"🔄 Executando {action} para {object_name}")
        
        if action == "pick":
            # Sequência de pegar: detectar -> pegar
            if self.detect_object(object_name):
                return self.pick_object(object_name)
            else:
                return False
                
        elif action == "place":
            # Sequência de colocar: apenas colocar
            return self.place_object(object_name)
        
        else:
            self.get_logger().error(f"❌ Ação desconhecida: {action}")
            return False
    
    def get_manipulation_status(self) -> str:
        """Retorna status atual da manipulação"""
        return self.last_status
    
    def reset_manipulation(self):
        """Reseta estado da manipulação"""
        self.last_status = "idle"
        self.manipulation_success = False
        
        # Envia comando de reset
        cmd = String()
        cmd.data = "reset"
        self.command_pub.publish(cmd)
        
        self.get_logger().info("🔄 Manipulação resetada")


def main(args=None):
    rclpy.init(args=args)
    
    interface = ManipulationInterface()
    
    try:
        rclpy.spin(interface)
    except KeyboardInterrupt:
        interface.get_logger().info("🛑 Manipulation Interface interrompida")
    
    interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
