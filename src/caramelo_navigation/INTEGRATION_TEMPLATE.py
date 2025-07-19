#!/usr/bin/env python3
"""
TEMPLATE PARA INTEGRAÇÃO NO CARAMELO_TASKS
==========================================

Este é um template que mostra como integrar o novo sistema de navegação
por workstations no pacote caramelo_tasks.

Copie este código para o seu orquestrador de tarefas no caramelo_tasks.

Autor: GitHub Copilot
Data: 2025-01-18
"""

import os
from typing import Dict, List

import rclpy
import yaml
from rclpy.node import Node

# Importar o cliente de navegação (adicionar ao caramelo_tasks)
try:
    from caramelo_navigation.task_navigation_client import TaskNavigationClient
except ImportError:
    TaskNavigationClient = None


class CarameloTaskOrchestrator(Node):
    """
    Template de orquestrador de tarefas usando o servidor de navegação.
    
    PARA USAR NO CARAMELO_TASKS:
    1. Copie este código para caramelo_tasks/caramelo_tasks/
    2. Adapte conforme sua arquitetura existente
    3. Integre com seu sistema de manipulação
    """
    
    def __init__(self):
        super().__init__('caramelo_task_orchestrator')
        
        # Parâmetros
        self.declare_parameter('arena', 'arena_fei')
        self.declare_parameter('task', 'BMT')
        
        self.arena = self.get_parameter('arena').get_parameter_value().string_value
        self.task = self.get_parameter('task').get_parameter_value().string_value
        
        # Cliente de navegação
        if TaskNavigationClient:
            self.nav_client = TaskNavigationClient(node=self, arena=self.arena)
        else:
            self.nav_client = None
            self.get_logger().error("❌ TaskNavigationClient não disponível!")
        
        # Estado da tarefa
        self.current_task_data = {}
        self.robot_inventory = []
        self.task_completed = False
        
        self.get_logger().info(f"🎯 Caramelo Task Orchestrator iniciado")
        self.get_logger().info(f"   Arena: {self.arena}")
        self.get_logger().info(f"   Task: {self.task}")
        
        # Iniciar execução
        self.execute_task()
    
    def execute_task(self):
        """
        Pipeline principal de execução de tarefas.
        Adapte esta função conforme sua necessidade.
        """
        try:
            # 1. Ler e interpretar tarefas
            if not self._load_task():
                return
            
            # 2. Planejar sequência de movimentos
            movements = self._plan_movements()
            
            # 3. Executar sequência
            self._execute_movements(movements)
            
            # 4. Finalizar na posição de FINISH
            self._go_to_finish()
            
            self.task_completed = True
            self.get_logger().info("🎉 Tarefa concluída com sucesso!")
            
        except Exception as e:
            self.get_logger().error(f"❌ Erro na execução da tarefa: {e}")
    
    def _load_task(self) -> bool:
        """Carrega dados da tarefa."""
        # ADAPTE: Caminho para seus arquivos de task
        task_file = f"/home/work/Caramelo_workspace/src/caramelo_tasks/{self.task}/task.yaml"
        
        if not os.path.exists(task_file):
            self.get_logger().error(f"❌ Task file não encontrado: {task_file}")
            return False
        
        try:
            with open(task_file, 'r') as f:
                self.current_task_data = yaml.safe_load(f)
            
            self.get_logger().info(f"✅ Task carregada: {self.task}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Erro ao carregar task: {e}")
            return False
    
    def _plan_movements(self) -> List[Dict]:
        """
        Planeja movimentos necessários.
        ADAPTE: Implementar sua lógica de planejamento aqui.
        """
        start_state = {ws['ws']: ws['objects'] for ws in self.current_task_data.get('start', [])}
        target_state = {ws['ws']: ws['objects'] for ws in self.current_task_data.get('target', [])}
        
        movements = []
        
        # Exemplo simples: mover objetos um por vez
        for target_ws, target_objects in target_state.items():
            for obj in target_objects:
                if not obj.get('decoy', True):  # Apenas objetos reais
                    # Encontrar localização atual
                    for start_ws, start_objects in start_state.items():
                        for start_obj in start_objects:
                            if start_obj['id'] == obj['id'] and start_ws != target_ws:
                                movements.append({
                                    'object_id': obj['id'],
                                    'from_ws': start_ws,
                                    'to_ws': target_ws
                                })
        
        self.get_logger().info(f"📋 Movimentos planejados: {len(movements)}")
        return movements
    
    def _execute_movements(self, movements: List[Dict]):
        """Executa a sequência de movimentos."""
        for i, movement in enumerate(movements):
            self.get_logger().info(f"\n🔄 Movimento {i+1}/{len(movements)}")
            self.get_logger().info(f"   Objeto {movement['object_id']}: {movement['from_ws']} → {movement['to_ws']}")
            
            # 1. Navegar para origem
            if not self._navigate_to_workstation(movement['from_ws']):
                self.get_logger().error(f"❌ Falha na navegação para {movement['from_ws']}")
                return
            
            # 2. Pegar objeto
            if not self._pickup_object(movement['from_ws'], movement['object_id']):
                self.get_logger().error(f"❌ Falha no pickup do objeto {movement['object_id']}")
                return
            
            # 3. Navegar para destino  
            if not self._navigate_to_workstation(movement['to_ws']):
                self.get_logger().error(f"❌ Falha na navegação para {movement['to_ws']}")
                return
            
            # 4. Entregar objeto
            if not self._deliver_object(movement['to_ws'], movement['object_id']):
                self.get_logger().error(f"❌ Falha na entrega do objeto {movement['object_id']}")
                return
            
            self.get_logger().info(f"✅ Movimento {i+1} concluído!")
    
    def _navigate_to_workstation(self, workstation: str) -> bool:
        """
        Navega para uma workstation.
        Esta é a principal integração com o novo sistema!
        """
        if not self.nav_client:
            self.get_logger().error("❌ Cliente de navegação não disponível!")
            return False
        
        self.get_logger().info(f"🚶 Navegando para {workstation}...")
        
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
    
    def _pickup_object(self, workstation: str, object_id: int) -> bool:
        """
        Pega um objeto na workstation.
        ADAPTE: Integrar com seu sistema de manipulação.
        """
        self.get_logger().info(f"🤏 Pegando objeto {object_id} em {workstation}...")
        
        # AQUI: Chamar seu serviço/ação de manipulação
        # Exemplo:
        # manipulation_success = self.manipulation_client.pickup(object_id)
        
        # Por enquanto, simular
        manipulation_success = True  # Substituir pela chamada real
        
        if manipulation_success:
            self.robot_inventory.append(object_id)
            self.get_logger().info(f"✅ Objeto {object_id} coletado!")
            return True
        else:
            self.get_logger().error(f"❌ Falha no pickup do objeto {object_id}")
            return False
    
    def _deliver_object(self, workstation: str, object_id: int) -> bool:
        """
        Entrega um objeto na workstation.
        ADAPTE: Integrar com seu sistema de manipulação.
        """
        self.get_logger().info(f"📦 Entregando objeto {object_id} em {workstation}...")
        
        # AQUI: Chamar seu serviço/ação de manipulação
        # Exemplo:
        # manipulation_success = self.manipulation_client.place(object_id)
        
        # Por enquanto, simular
        manipulation_success = True  # Substituir pela chamada real
        
        if manipulation_success:
            if object_id in self.robot_inventory:
                self.robot_inventory.remove(object_id)
            self.get_logger().info(f"✅ Objeto {object_id} entregue!")
            return True
        else:
            self.get_logger().error(f"❌ Falha na entrega do objeto {object_id}")
            return False
    
    def _go_to_finish(self) -> bool:
        """Navega para a posição final."""
        self.get_logger().info("🏁 Indo para posição final...")
        return self._navigate_to_workstation("FINISH")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        orchestrator = CarameloTaskOrchestrator()
        
        # Manter ativo até task completar
        while rclpy.ok() and not orchestrator.task_completed:
            rclpy.spin_once(orchestrator, timeout_sec=1.0)
            
    except KeyboardInterrupt:
        pass
    finally:
        try:
            orchestrator.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# ==============================================================================
# INSTRUÇÕES PARA INTEGRAÇÃO NO CARAMELO_TASKS:
# ==============================================================================

"""
PASSO 1: Copiar arquivos necessários
-------------------------------------
1. Copie task_navigation_client.py para caramelo_tasks/caramelo_tasks/
2. Adapte este template conforme sua arquitetura

PASSO 2: Atualizar setup.py do caramelo_tasks
---------------------------------------------
Adicione a dependência no setup.py:

install_requires=[
    'setuptools',
    'caramelo_navigation',  # <- Adicionar esta linha
],

PASSO 3: Atualizar package.xml do caramelo_tasks
-----------------------------------------------
Adicione a dependência no package.xml:

<depend>caramelo_navigation</depend>

PASSO 4: Adaptar o código
------------------------
1. Substitua _pickup_object() pela chamada real ao seu sistema de manipulação
2. Substitua _deliver_object() pela chamada real ao seu sistema de manipulação  
3. Adapte _plan_movements() conforme sua estratégia de planejamento
4. Adapte caminhos de arquivos conforme sua estrutura

PASSO 5: Compilar e testar
-------------------------
1. colcon build --packages-select caramelo_tasks
2. source install/setup.bash
3. ros2 launch caramelo_navigation basic_navigation.launch.py arena:=arena_fei
4. ros2 run caramelo_tasks caramelo_task_orchestrator --ros-args -p task:=BMT

VANTAGENS:
----------
✅ Navegação testável independentemente
✅ Interface simples e limpa
✅ Reutilizável para qualquer arena/task
✅ Logs claros para depuração
✅ Gestão de estado robusta
"""
