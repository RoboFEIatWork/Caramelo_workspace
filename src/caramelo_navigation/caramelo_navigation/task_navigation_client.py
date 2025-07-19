#!/usr/bin/env python3
"""
TASK NAVIGATION CLIENT
=====================

Cliente do servidor de navegação para uso no pacote caramelo_tasks.
Este módulo fornece uma interface simples para navegação por workstations.

Exemplo de uso no caramelo_tasks:

    from caramelo_navigation.task_navigation_client import TaskNavigationClient
    
    nav_client = TaskNavigationClient()
    
    # Navegar para WS01 (assíncrono)
    success = nav_client.navigate_to_workstation("WS01")
    
    # Navegar para WS02 aguardando conclusão
    success = nav_client.navigate_to_workstation("WS02", wait=True)
    
    # Navegar com timeout específico
    success = nav_client.navigate_to_workstation("FINISH", wait=True, timeout=180)

Autor: GitHub Copilot  
Data: 2025-01-18
"""

import rclpy
from rclpy.node import Node

try:
    from caramelo_navigation.srv import NavigateToWorkstation
except ImportError:
    # Durante desenvolvimento, usar um placeholder
    NavigateToWorkstation = None


class TaskNavigationClient:
    """
    Cliente simplificado para navegação por workstations.
    Projetado para uso fácil no pacote caramelo_tasks.
    """
    
    def __init__(self, node=None, arena=""):
        """
        Inicializa o cliente de navegação.
        
        Args:
            node: Nó ROS2 existente (opcional)
            arena: Nome da arena padrão (opcional)
        """
        self.external_node = node is not None
        self.node = node if node else Node('task_navigation_client')
        self.arena = arena
        
        # Criar cliente do serviço
        if NavigateToWorkstation:
            self.navigation_client = self.node.create_client(
                NavigateToWorkstation, 
                'navigate_to_workstation'
            )
        else:
            self.navigation_client = None
        
        self.service_available = False
        self._check_service()
    
    def _check_service(self):
        """Verifica se o serviço está disponível."""
        if not self.navigation_client:
            return False
        
        if self.navigation_client.wait_for_service(timeout_sec=2.0):
            self.service_available = True
            self.node.get_logger().info("✅ Serviço de navegação disponível")
            return True
        else:
            self.node.get_logger().warn("⚠️  Serviço /navigate_to_workstation não disponível")
            return False
    
    def navigate_to_workstation(self, workstation_name, arena="", wait=False, timeout=120.0):
        """
        Navega para uma workstation específica.
        
        Args:
            workstation_name (str): Nome da workstation (ex: "WS01", "START", "FINISH")
            arena (str): Nome da arena (usa padrão se vazio)
            wait (bool): Aguardar conclusão da navegação
            timeout (float): Timeout em segundos
            
        Returns:
            bool: True se navegação foi bem sucedida
        """
        if not self.service_available:
            if not self._check_service():
                self.node.get_logger().error("❌ Serviço de navegação não disponível!")
                self.node.get_logger().error("   Execute: ros2 launch caramelo_navigation basic_navigation.launch.py")
                return False
        
        if not NavigateToWorkstation:
            self.node.get_logger().error("❌ Serviço NavigateToWorkstation não compilado!")
            return False
        
        arena_to_use = arena if arena else self.arena
        
        self.node.get_logger().info(f"🎯 Navegando para {workstation_name}")
        if arena_to_use:
            self.node.get_logger().info(f"   Arena: {arena_to_use}")
        
        # Criar request
        request = NavigateToWorkstation.Request()
        request.workstation_name = workstation_name
        request.arena_name = arena_to_use
        request.wait_for_completion = wait
        request.timeout = timeout
        
        # Enviar request
        future = self.navigation_client.call_async(request)
        
        # Aguardar resposta
        rclpy.spin_until_future_complete(self.node, future)
        
        if future.result() is not None:
            response = future.result()
            
            if response.success:
                self.node.get_logger().info(f"✅ Navegação para {workstation_name}: {response.message}")
                if wait:
                    self.node.get_logger().info(f"   Tempo: {response.time_elapsed:.2f}s")
                return True
            else:
                self.node.get_logger().error(f"❌ Falha navegação para {workstation_name}: {response.message}")
                return False
        else:
            self.node.get_logger().error("❌ Falha na comunicação com serviço de navegação")
            return False
    
    def is_service_available(self):
        """Retorna True se o serviço está disponível."""
        return self.service_available
    
    def set_default_arena(self, arena):
        """Define a arena padrão."""
        self.arena = arena
        self.node.get_logger().info(f"🗺️  Arena padrão definida: {arena}")
    
    def __del__(self):
        """Destrutor - limpa recursos se necessário."""
        if not self.external_node and hasattr(self, 'node'):
            try:
                self.node.destroy_node()
            except:
                pass


# Função de conveniência para uso simples
def navigate_to_ws(workstation_name, arena="", wait=False, timeout=120.0):
    """
    Função de conveniência para navegação rápida.
    Cria e destrói o cliente automaticamente.
    
    Args:
        workstation_name (str): Nome da workstation
        arena (str): Nome da arena (opcional)
        wait (bool): Aguardar conclusão
        timeout (float): Timeout em segundos
        
    Returns:
        bool: Sucesso da navegação
    """
    rclpy.init()
    try:
        client = TaskNavigationClient(arena=arena)
        success = client.navigate_to_workstation(workstation_name, wait=wait, timeout=timeout)
        return success
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    # Teste direto
    import sys
    
    if len(sys.argv) < 2:
        print("Uso: python3 task_navigation_client.py <workstation> [arena]")
        sys.exit(1)
    
    workstation = sys.argv[1]
    arena = sys.argv[2] if len(sys.argv) > 2 else ""
    
    success = navigate_to_ws(workstation, arena, wait=True)
    
    if success:
        print(f"✅ Navegação para {workstation} concluída!")
    else:
        print(f"❌ Falha na navegação para {workstation}")
        sys.exit(1)
