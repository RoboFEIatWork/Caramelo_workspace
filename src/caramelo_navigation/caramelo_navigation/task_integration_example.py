#!/usr/bin/env python3
"""
EXEMPLO DE INTEGRAÇÃO: CARAMELO_TASKS + NAVIGATION SERVER
========================================================

Este exemplo mostra como integrar o novo sistema de navegação por workstations
no pacote caramelo_tasks. 

O exemplo implementa um orquestrador básico que:
1. Lê um arquivo task.yaml
2. Planeja a sequência de movimentos necessários  
3. Usa o servidor de navegação para executar os movimentos
4. Coordena com um sistema de manipulação (simulado)

Uso:
    ros2 run caramelo_navigation task_integration_example --ros-args -p arena:=arena_fei -p task:=BMT

Autor: GitHub Copilot
Data: 2025-01-18
"""

import os
import time
from typing import Dict, List, Tuple

import rclpy
import yaml
from rclpy.node import Node

# Importar o cliente de navegação
try:
    from caramelo_navigation.task_navigation_client import TaskNavigationClient
except ImportError:
    # Fallback se ainda não compilado
    TaskNavigationClient = None


class TaskIntegrationExample(Node):
    """
    Exemplo de orquestrador de tarefas usando o servidor de navegação por workstations.
    """
    
    def __init__(self):
        super().__init__('task_integration_example')
        
        # Parâmetros
        self.declare_parameter('arena', 'arena_fei')
        self.declare_parameter('task', 'BMT')
        self.declare_parameter('tasks_base_path', '/home/work/Caramelo_workspace/src/caramelo_tasks')
        
        self.arena = self.get_parameter('arena').get_parameter_value().string_value
        self.task = self.get_parameter('task').get_parameter_value().string_value
        self.tasks_base_path = self.get_parameter('tasks_base_path').get_parameter_value().string_value
        
        # Estado
        self.task_data = {}
        self.movement_plan = []
        self.current_step = 0
        self.robot_inventory = []  # Objetos carregados pelo robô
        
        # Cliente de navegação
        if TaskNavigationClient:
            self.nav_client = TaskNavigationClient(node=self, arena=self.arena)
        else:
            self.nav_client = None
            self.get_logger().error("❌ TaskNavigationClient não disponível!")
        
        self.get_logger().info(f"🎯 Task Integration Example iniciado")
        self.get_logger().info(f"   Arena: {self.arena}")
        self.get_logger().info(f"   Task: {self.task}")
        
        # Carregar e executar task
        if self._load_task():
            self._plan_movements()
            self._execute_task()
        else:
            self.get_logger().error("❌ Falha ao carregar task!")
    
    def _load_task(self) -> bool:
        """Carrega o arquivo task.yaml da task especificada."""
        task_file = os.path.join(self.tasks_base_path, self.task, 'task.yaml')
        
        if not os.path.exists(task_file):
            self.get_logger().error(f"❌ Arquivo de task não encontrado: {task_file}")
            return False
        
        try:
            with open(task_file, 'r') as f:
                self.task_data = yaml.safe_load(f)
            
            self.get_logger().info(f"✅ Task carregada: {task_file}")
            self.get_logger().info(f"   Workstations iniciais: {len(self.task_data.get('start', []))}")
            self.get_logger().info(f"   Workstations finais: {len(self.task_data.get('target', []))}")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Erro ao carregar task: {e}")
            return False
    
    def _plan_movements(self):
        """Planeja a sequência de movimentos necessários."""
        start_state = {ws['ws']: ws['objects'] for ws in self.task_data.get('start', [])}
        target_state = {ws['ws']: ws['objects'] for ws in self.task_data.get('target', [])}
        
        self.get_logger().info("🧠 Planejando movimentos...")
        
        # Encontrar objetos que precisam ser movidos
        movements_needed = []
        
        for ws_name, target_objects in target_state.items():
            for obj in target_objects:
                if not obj.get('decoy', True):  # Apenas objetos reais
                    # Encontrar onde está atualmente
                    current_location = None
                    for start_ws, start_objects in start_state.items():
                        for start_obj in start_objects:
                            if start_obj['id'] == obj['id'] and not start_obj.get('decoy', True):
                                current_location = start_ws
                                break
                        if current_location:
                            break
                    
                    if current_location and current_location != ws_name:
                        movements_needed.append({
                            'object_id': obj['id'],
                            'from_ws': current_location,
                            'to_ws': ws_name
                        })
        
        self.get_logger().info(f"📋 Movimentos necessários: {len(movements_needed)}")
        for movement in movements_needed:
            self.get_logger().info(f"   Objeto {movement['object_id']}: {movement['from_ws']} → {movement['to_ws']}")
        
        # Criar plano simples (um objeto por vez para este exemplo)
        self.movement_plan = []
        for movement in movements_needed:
            self.movement_plan.append({
                'action': 'goto_pickup',
                'workstation': movement['from_ws'],
                'object_id': movement['object_id']
            })
            self.movement_plan.append({
                'action': 'pickup',
                'workstation': movement['from_ws'],
                'object_id': movement['object_id']
            })
            self.movement_plan.append({
                'action': 'goto_delivery',
                'workstation': movement['to_ws'],
                'object_id': movement['object_id']
            })
            self.movement_plan.append({
                'action': 'delivery',
                'workstation': movement['to_ws'],
                'object_id': movement['object_id']
            })
        
        # Adicionar retorno ao FINISH
        self.movement_plan.append({
            'action': 'goto_finish',
            'workstation': 'FINISH',
            'object_id': None
        })
        
        self.get_logger().info(f"📝 Plano criado com {len(self.movement_plan)} etapas")
    
    def _execute_task(self):
        """Executa o plano de movimentos."""
        if not self.nav_client:
            self.get_logger().error("❌ Cliente de navegação não disponível!")
            return
        
        self.get_logger().info("🚀 Iniciando execução da task...")
        
        for i, step in enumerate(self.movement_plan):
            self.current_step = i
            self.get_logger().info(f"\n📍 Etapa {i+1}/{len(self.movement_plan)}: {step['action']}")
            
            if step['action'] in ['goto_pickup', 'goto_delivery', 'goto_finish']:
                # Navegação
                success = self._navigate_to_workstation(step['workstation'])
                if not success:
                    self.get_logger().error(f"❌ Falha na navegação para {step['workstation']}!")
                    break
            
            elif step['action'] == 'pickup':
                # Ação de manipulação (simulada)
                success = self._simulate_pickup(step['workstation'], step['object_id'])
                if not success:
                    self.get_logger().error(f"❌ Falha no pickup do objeto {step['object_id']}!")
                    break
            
            elif step['action'] == 'delivery':
                # Ação de entrega (simulada)
                success = self._simulate_delivery(step['workstation'], step['object_id'])
                if not success:
                    self.get_logger().error(f"❌ Falha na entrega do objeto {step['object_id']}!")
                    break
        
        self.get_logger().info("🎉 Task concluída com sucesso!")
        self.get_logger().info(f"   Objetos transportados: {len([s for s in self.movement_plan if s['action'] == 'delivery'])}")
    
    def _navigate_to_workstation(self, workstation: str) -> bool:
        """Navega para uma workstation usando o servidor de navegação."""
        self.get_logger().info(f"🚶 Navegando para {workstation}...")
        
        if not self.nav_client:
            return False
        
        success = self.nav_client.navigate_to_workstation(
            workstation_name=workstation,
            wait=True,  # Aguardar conclusão
            timeout=120.0
        )
        
        if success:
            self.get_logger().info(f"✅ Chegou em {workstation}")
        else:
            self.get_logger().error(f"❌ Falha na navegação para {workstation}")
        
        return success
    
    def _simulate_pickup(self, workstation: str, object_id: int) -> bool:
        """Simula ação de pickup."""
        self.get_logger().info(f"🤏 Pegando objeto {object_id} em {workstation}...")
        
        # Simular tempo de manipulação
        time.sleep(2.0)
        
        # Adicionar ao inventário
        self.robot_inventory.append(object_id)
        
        self.get_logger().info(f"✅ Objeto {object_id} coletado! Inventário: {self.robot_inventory}")
        return True
    
    def _simulate_delivery(self, workstation: str, object_id: int) -> bool:
        """Simula ação de entrega."""
        self.get_logger().info(f"📦 Entregando objeto {object_id} em {workstation}...")
        
        # Simular tempo de manipulação
        time.sleep(2.0)
        
        # Remover do inventário
        if object_id in self.robot_inventory:
            self.robot_inventory.remove(object_id)
        
        self.get_logger().info(f"✅ Objeto {object_id} entregue! Inventário: {self.robot_inventory}")
        return True


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TaskIntegrationExample()
        
        # Manter o nó vivo para logs
        rclpy.spin_once(node, timeout_sec=1.0)
        
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
