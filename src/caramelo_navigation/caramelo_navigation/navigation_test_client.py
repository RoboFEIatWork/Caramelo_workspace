#!/usr/bin/env python3
"""
NAVIGATION TEST CLIENT
=====================

Cliente de teste para o servidor de navegação por workstations.
Permite testar a navegação manualmente enviando comandos específicos.

Uso:
    # Navegar para WS01 (assíncrono)
    ros2 run caramelo_navigation navigation_test_client WS01
    
    # Navegar para WS02 aguardando conclusão
    ros2 run caramelo_navigation navigation_test_client WS02 --wait
    
    # Navegar com arena específica
    ros2 run caramelo_navigation navigation_test_client START --arena arena_robocup25
    
    # Navegar com timeout customizado
    ros2 run caramelo_navigation navigation_test_client FINISH --wait --timeout 180

Autor: GitHub Copilot
Data: 2025-01-18
"""

import argparse
import sys

import rclpy
from rclpy.node import Node

try:
    from caramelo_navigation.srv import NavigateToWorkstation
except ImportError:
    # Fallback para desenvolvimento
    from std_srvs.srv import Empty as NavigateToWorkstation


class NavigationTestClient(Node):
    """
    Cliente de teste para navegação por workstations.
    """
    
    def __init__(self):
        super().__init__('navigation_test_client')
        
        # Criar cliente do serviço
        self.navigation_client = self.create_client(
            NavigateToWorkstation, 
            'navigate_to_workstation'
        )
        
        self.get_logger().info("🧪 Navigation Test Client iniciado")
        self.get_logger().info("   Aguardando serviço /navigate_to_workstation...")
        
        # Aguardar serviço ficar disponível
        if not self.navigation_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("❌ Serviço /navigate_to_workstation não disponível!")
            self.get_logger().error("   Certifique-se de que o workstation_navigation_server está rodando:")
            self.get_logger().error("   ros2 run caramelo_navigation workstation_navigation_server")
            sys.exit(1)
        
        self.get_logger().info("✅ Serviço disponível!")
    
    def navigate_to_workstation(self, workstation_name, arena_name="", wait_completion=False, timeout=120.0):
        """Envia comando de navegação para workstation."""
        
        self.get_logger().info(f"🎯 Enviando comando de navegação:")
        self.get_logger().info(f"   Workstation: {workstation_name}")
        self.get_logger().info(f"   Arena: {arena_name if arena_name else 'padrão'}")
        self.get_logger().info(f"   Aguardar conclusão: {wait_completion}")
        self.get_logger().info(f"   Timeout: {timeout}s")
        
        # Criar request
        request = NavigateToWorkstation.Request()
        request.workstation_name = workstation_name
        request.arena_name = arena_name
        request.wait_for_completion = wait_completion
        request.timeout = timeout
        
        # Enviar request
        future = self.navigation_client.call_async(request)
        
        self.get_logger().info("⏳ Enviando comando...")
        
        # Aguardar resposta
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            
            if response.success:
                self.get_logger().info(f"🎉 SUCESSO: {response.message}")
                if wait_completion:
                    self.get_logger().info(f"   Tempo decorrido: {response.time_elapsed:.2f}s")
                    self.get_logger().info(f"   Distância: {response.distance_traveled:.2f}m")
            else:
                self.get_logger().error(f"❌ FALHA: {response.message}")
            
            return response.success
        else:
            self.get_logger().error("❌ Falha na comunicação com o serviço")
            return False


def main():
    """Main function com argumentos de linha de comando."""
    
    parser = argparse.ArgumentParser(description='Cliente de teste para navegação por workstations')
    parser.add_argument('workstation', help='Nome da workstation (ex: WS01, WS02, START, FINISH)')
    parser.add_argument('--arena', default='', help='Nome da arena (opcional)')
    parser.add_argument('--wait', action='store_true', help='Aguardar conclusão da navegação')
    parser.add_argument('--timeout', type=float, default=120.0, help='Timeout em segundos (padrão: 120)')
    
    # Parse apenas os argumentos conhecidos para ignorar argumentos do ROS
    args, unknown = parser.parse_known_args()
    
    # Inicializar ROS
    rclpy.init(args=sys.argv)
    
    try:
        # Criar cliente
        client = NavigationTestClient()
        
        # Executar navegação
        success = client.navigate_to_workstation(
            workstation_name=args.workstation,
            arena_name=args.arena,
            wait_completion=args.wait,
            timeout=args.timeout
        )
        
        if success:
            client.get_logger().info("✅ Teste concluído com sucesso!")
        else:
            client.get_logger().error("❌ Teste falhou!")
            sys.exit(1)
    
    except KeyboardInterrupt:
        print("\n⚠️  Teste interrompido pelo usuário")
    
    finally:
        try:
            client.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
