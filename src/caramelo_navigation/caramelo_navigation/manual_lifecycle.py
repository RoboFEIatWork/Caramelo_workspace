#!/usr/bin/env python3

import time

import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from rclpy.node import Node


class LifecycleManagerManual(Node):
    def __init__(self):
        super().__init__('lifecycle_manager_manual')
        self.get_logger().info('🔧 Lifecycle Manager Manual iniciado!')
        
        # Lista de nós para ativar
        self.nodes_to_activate = [
            'planner_server',
            'controller_server', 
            'smoother_server',
            'behavior_server',
            'bt_navigator',
            'velocity_smoother',
            'waypoint_follower',
            'collision_monitor',
            'docking_server'
        ]
        
        # Aguardar os serviços ficarem disponíveis
        time.sleep(5)
        
        # Configurar todos os nós
        self.configure_all_nodes()
        
        # Ativar todos os nós
        self.activate_all_nodes()
        
        self.get_logger().info('✅ Todos os nós Nav2 foram ativados manualmente!')

    def configure_all_nodes(self):
        """Configura todos os nós do Nav2"""
        self.get_logger().info('🔄 Configurando nós Nav2...')
        
        for node_name in self.nodes_to_activate:
            try:
                self.change_state(node_name, Transition.TRANSITION_CONFIGURE)
                self.get_logger().info(f'✅ {node_name} configurado')
                time.sleep(1)
            except Exception as e:
                self.get_logger().error(f'❌ Erro ao configurar {node_name}: {str(e)}')

    def activate_all_nodes(self):
        """Ativa todos os nós do Nav2"""
        self.get_logger().info('🚀 Ativando nós Nav2...')
        
        for node_name in self.nodes_to_activate:
            try:
                self.change_state(node_name, Transition.TRANSITION_ACTIVATE)
                self.get_logger().info(f'✅ {node_name} ativado')
                time.sleep(1)
            except Exception as e:
                self.get_logger().error(f'❌ Erro ao ativar {node_name}: {str(e)}')

    def change_state(self, node_name, transition):
        """Muda o estado de um nó específico"""
        service_name = f'/{node_name}/change_state'
        
        # Esperar o serviço ficar disponível
        client = self.create_client(ChangeState, service_name)
        
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f'Serviço {service_name} não disponível')
            return False
        
        # Fazer a requisição
        request = ChangeState.Request()
        request.transition = transition
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            result = future.result()
            if result.success:
                return True
            else:
                self.get_logger().error(f'Falha ao mudar estado de {node_name}')
                return False
        else:
            self.get_logger().error(f'Timeout ao mudar estado de {node_name}')
            return False

def main(args=None):
    rclpy.init(args=args)
    
    try:
        lifecycle_manager = LifecycleManagerManual()
        # Não fazer spin, apenas executar e sair
        lifecycle_manager.destroy_node()
    except Exception as e:
        print(f'Erro: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
