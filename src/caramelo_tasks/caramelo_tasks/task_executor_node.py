#!/usr/bin/env python3

import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class TaskExecutor(Node):
    def __init__(self):
        super().__init__('task_executor')
        
        # Action client para navegação
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Parâmetros
        self.declare_parameter('task_file', 'tasks.yaml')
        
        # Variáveis
        self.tasks = []
        self.workstations = {}
        self.current_task_index = 0
        
        self.get_logger().info('🤖 Task Executor iniciado!')
        
        # Carrega arquivo de tarefas
        self.load_tasks()
        
        # Aguarda nav2 ficar disponível
        self.wait_for_nav2()
        
        # Inicia execução das tarefas
        self.execute_tasks()
        
    def load_tasks(self):
        """Carrega arquivo YAML com tarefas"""
        try:
            task_file = self.get_parameter('task_file').value
            
            # Tenta primeiro no diretório do pacote
            try:
                package_share = get_package_share_directory('caramelo_tasks')
                full_path = os.path.join(package_share, 'config', task_file)
            except:
                # Se não encontrar, usa caminho absoluto ou relativo
                full_path = task_file
                
            with open(full_path, 'r') as file:
                data = yaml.safe_load(file)
                
            self.tasks = data.get('task_list', [])
            self.workstations = data.get('workstations', {})
            
            self.get_logger().info(f'📋 Carregadas {len(self.tasks)} tarefas')
            self.get_logger().info(f'🏭 Encontradas {len(self.workstations)} estações')
            
            for i, task in enumerate(self.tasks):
                self.get_logger().info(f'  Tarefa {i+1}: {task["object"]} | {task["pick_from"]} → {task["place_to"]}')
                
        except Exception as e:
            self.get_logger().error(f'❌ Erro ao carregar tarefas: {e}')
            self.get_logger().info('📝 Criando arquivo de exemplo...')
            self.create_example_tasks()
            
    def create_example_tasks(self):
        """Cria arquivo de exemplo se não existir"""
        example = {
            'task_list': [
                {'object': 'R20', 'pick_from': 'WS3', 'place_to': 'WS5'},
                {'object': 'M20', 'pick_from': 'WS1', 'place_to': 'WS2'},
                {'object': 'F20_20_B', 'pick_from': 'WS4', 'place_to': 'WS6'}
            ],
            'workstations': {
                'WS1': {'position': [1.0, 1.0], 'orientation': [0.0, 0.0, 0.0, 1.0]},
                'WS2': {'position': [2.0, 1.0], 'orientation': [0.0, 0.0, 0.0, 1.0]},
                'WS3': {'position': [1.0, 2.0], 'orientation': [0.0, 0.0, 0.0, 1.0]},
                'WS4': {'position': [2.0, 2.0], 'orientation': [0.0, 0.0, 0.0, 1.0]},
                'WS5': {'position': [3.0, 1.0], 'orientation': [0.0, 0.0, 0.0, 1.0]},
                'WS6': {'position': [3.0, 2.0], 'orientation': [0.0, 0.0, 0.0, 1.0]}
            }
        }
        
        try:
            package_share = get_package_share_directory('caramelo_tasks')
            config_dir = os.path.join(package_share, 'config')
            os.makedirs(config_dir, exist_ok=True)
            
            example_file = os.path.join(config_dir, 'tasks.yaml')
            with open(example_file, 'w') as file:
                yaml.dump(example, file, default_flow_style=False)
                
            self.get_logger().info(f'📝 Arquivo de exemplo criado: {example_file}')
            
        except Exception as e:
            self.get_logger().error(f'❌ Erro ao criar exemplo: {e}')
            
    def wait_for_nav2(self):
        """Aguarda Nav2 ficar disponível"""
        self.get_logger().info('⏳ Aguardando Nav2...')
        
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('⏳ Nav2 não disponível, aguardando...')
            
        self.get_logger().info('✅ Nav2 disponível!')
        
    def execute_tasks(self):
        """Executa todas as tarefas sequencialmente"""
        if not self.tasks:
            self.get_logger().warn('⚠️ Nenhuma tarefa para executar!')
            return
            
        self.get_logger().info(f'🚀 Iniciando execução de {len(self.tasks)} tarefas')
        self.execute_next_task()
        
    def execute_next_task(self):
        """Executa a próxima tarefa"""
        if self.current_task_index >= len(self.tasks):
            self.get_logger().info('🎉 Todas as tarefas concluídas!')
            return
            
        task = self.tasks[self.current_task_index]
        self.get_logger().info(f'📋 Tarefa {self.current_task_index + 1}/{len(self.tasks)}: {task}')
        
        # Executa pickup
        self.execute_pickup(task)
        
    def execute_pickup(self, task):
        """Executa pickup do objeto"""
        pick_station = task['pick_from']
        obj = task['object']
        
        self.get_logger().info(f'📦 Indo buscar {obj} na {pick_station}')
        
        if pick_station not in self.workstations:
            self.get_logger().error(f'❌ Estação {pick_station} não encontrada!')
            self.handle_task_failure(task)
            return
            
        # Navega para estação de pickup
        position = self.workstations[pick_station]['position']
        orientation = self.workstations[pick_station].get('orientation', [0.0, 0.0, 0.0, 1.0])
        
        self.navigate_to_position(position, orientation, 
                                lambda: self.attempt_manipulation(task, 'pickup'))
        
    def execute_delivery(self, task):
        """Executa entrega do objeto"""
        place_station = task['place_to']
        obj = task['object']
        
        self.get_logger().info(f'📦 Entregando {obj} na {place_station}')
        
        if place_station not in self.workstations:
            self.get_logger().error(f'❌ Estação {place_station} não encontrada!')
            self.handle_task_failure(task)
            return
            
        # Navega para estação de entrega
        position = self.workstations[place_station]['position']
        orientation = self.workstations[place_station].get('orientation', [0.0, 0.0, 0.0, 1.0])
        
        self.navigate_to_position(position, orientation, 
                                lambda: self.attempt_manipulation(task, 'place'))
        
    def navigate_to_position(self, position, orientation, callback):
        """Navega para uma posição específica"""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = position[0]
        goal.pose.pose.position.y = position[1]
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.x = orientation[0]
        goal.pose.pose.orientation.y = orientation[1]
        goal.pose.pose.orientation.z = orientation[2]
        goal.pose.pose.orientation.w = orientation[3]
        
        self.get_logger().info(f'🧭 Navegando para: ({position[0]:.2f}, {position[1]:.2f})')
        
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self.handle_nav_response(f, callback))
        
    def handle_nav_response(self, future, callback):
        """Lida com resposta da navegação"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejeitado!')
            self.handle_task_failure(self.tasks[self.current_task_index])
            return
            
        self.get_logger().info('✅ Goal aceito, navegando...')
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.handle_nav_result(f, callback))
        
    def handle_nav_result(self, future, callback):
        """Lida com resultado da navegação"""
        result = future.result().result
        
        if result:
            self.get_logger().info('✅ Navegação concluída!')
            callback()
        else:
            self.get_logger().error('❌ Falha na navegação!')
            self.handle_task_failure(self.tasks[self.current_task_index])
            
    def attempt_manipulation(self, task, action, attempt=1):
        """Tenta manipulação com até 3 tentativas"""
        obj = task['object']
        max_attempts = 3
        
        self.get_logger().info(f'🤏 Tentativa {attempt}/{max_attempts}: {action} {obj}')
        
        # Simula detecção e manipulação (será substituído por código real)
        success = self.simulate_manipulation(obj, action)
        
        if success:
            self.get_logger().info(f'✅ {action} de {obj} bem-sucedido!')
            
            if action == 'pickup':
                # Sucesso no pickup, vai para entrega
                self.execute_delivery(task)
            else:
                # Sucesso na entrega, vai para próxima tarefa
                self.get_logger().info(f'🎉 Tarefa {self.current_task_index + 1} concluída!')
                self.current_task_index += 1
                self.execute_next_task()
        else:
            self.get_logger().warn(f'⚠️ {action} de {obj} falhou!')
            
            if attempt < max_attempts:
                # Tenta novamente
                self.attempt_manipulation(task, action, attempt + 1)
            else:
                # Máximo de tentativas atingido
                self.get_logger().error(f'❌ {action} de {obj} falhou após {max_attempts} tentativas!')
                self.handle_task_failure(task)
                
    def simulate_manipulation(self, obj, action):
        """Simula manipulação (mock) - será substituído por código real"""
        import random

        # Detecção: 70% de chance de sucesso
        detection_success = random.random() < 0.7
        
        if not detection_success:
            self.get_logger().warn(f'👁️ Objeto {obj} não detectado!')
            return False
            
        self.get_logger().info(f'👁️ Objeto {obj} detectado!')
        
        # Manipulação: 60% de chance de sucesso
        manipulation_success = random.random() < 0.6
        
        if not manipulation_success:
            self.get_logger().warn(f'🤏 Falha na manipulação de {obj}!')
            return False
            
        return True
        
    def handle_task_failure(self, task):
        """Lida com falha na tarefa"""
        self.get_logger().error(f'❌ Tarefa falhou: {task}')
        self.get_logger().info('⏭️ Passando para próxima tarefa...')
        
        self.current_task_index += 1
        self.execute_next_task()


def main(args=None):
    rclpy.init(args=args)
    
    executor = TaskExecutor()
    
    try:
        rclpy.spin(executor)
    except KeyboardInterrupt:
        executor.get_logger().info('🛑 Task Executor interrompido')
    
    executor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
