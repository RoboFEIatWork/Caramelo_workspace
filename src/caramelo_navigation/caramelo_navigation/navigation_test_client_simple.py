#!/usr/bin/env python3
"""
NAVIGATION TEST CLIENT - VERSÃO SIMPLES
=======================================

Cliente simples para testar o servidor de navegação usando tópicos.

Uso:
    ros2 run caramelo_navigation navigation_test_client_simple WS01
    ros2 run caramelo_navigation navigation_test_client_simple WS02

Autor: GitHub Copilot
Data: 2025-01-18
"""

import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class NavigationTestClientSimple(Node):
    """Cliente simples para navegação por workstations."""
    
    def __init__(self):
        super().__init__('navigation_test_client_simple')
        
        # Publisher para comandos
        self.navigation_publisher = self.create_publisher(
            String,
            'navigate_to_workstation',
            10
        )
        
        # Subscriber para status
        self.status_subscriber = self.create_subscription(
            String,
            'navigation_status',
            self._status_callback,
            10
        )
        
        self.last_status = {}
        self.navigation_complete = False
        
        self.get_logger().info("🧪 Navigation Test Client (Simple) iniciado")
    
    def _status_callback(self, msg):
        """Callback para receber status da navegação."""
        try:
            status_data = json.loads(msg.data)
            
            # Mostrar apenas mudanças importantes de status
            if status_data.get('status') != self.last_status.get('status'):
                status = status_data.get('status', 'UNKNOWN')
                message = status_data.get('message', 'No message')
                
                if status == "READY":
                    self.get_logger().info(f"✅ {message}")
                elif status == "NAVIGATING":
                    self.get_logger().info(f"🚶 {message}")
                elif status == "SUCCESS":
                    self.get_logger().info(f"🎉 {message}")
                    self.navigation_complete = True
                elif status == "FAILED":
                    self.get_logger().error(f"❌ {message}")
                    self.navigation_complete = True
                elif status == "ERROR":
                    self.get_logger().error(f"❌ {message}")
                    self.navigation_complete = True
                elif status == "TIMEOUT":
                    self.get_logger().error(f"⏰ {message}")
                    self.navigation_complete = True
                elif status == "BUSY":
                    self.get_logger().warn(f"⚠️  {message}")
            
            self.last_status = status_data
            
        except json.JSONDecodeError:
            self.get_logger().warn(f"Received invalid JSON status: {msg.data}")
    
    def navigate_to_workstation(self, workstation_name):
        """Envia comando de navegação."""
        self.get_logger().info(f"🎯 Enviando comando: navegar para {workstation_name}")
        
        msg = String()
        msg.data = workstation_name
        self.navigation_publisher.publish(msg)
        
        self.get_logger().info("⏳ Aguardando resposta...")
        
        # Aguardar status de confirmação
        timeout = 60.0  # 60 segundos timeout
        start_time = time.time()
        
        while rclpy.ok() and not self.navigation_complete and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.navigation_complete:
            if self.last_status.get('status') == 'SUCCESS':
                self.get_logger().info("✅ Navegação concluída com sucesso!")
                return True
            else:
                self.get_logger().error("❌ Navegação falhou!")
                return False
        else:
            self.get_logger().error("⏰ Timeout aguardando resposta do servidor!")
            return False


def main():
    if len(sys.argv) < 2:
        print("Uso: ros2 run caramelo_navigation navigation_test_client_simple <workstation>")
        print("Exemplo: ros2 run caramelo_navigation navigation_test_client_simple WS01")
        sys.exit(1)
    
    workstation = sys.argv[1]
    
    rclpy.init()
    client = None
    
    try:
        client = NavigationTestClientSimple()
        
        # Aguardar um pouco para estabelecer conexões
        time.sleep(1.0)
        
        # Executar navegação
        success = client.navigate_to_workstation(workstation)
        
        if success:
            print(f"✅ Teste concluído: navegação para {workstation} bem-sucedida!")
        else:
            print(f"❌ Teste falhou: navegação para {workstation} falhou!")
            sys.exit(1)
    
    except KeyboardInterrupt:
        print("\n⚠️  Teste interrompido pelo usuário")
    
    finally:
        try:
            if client is not None:
                client.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
