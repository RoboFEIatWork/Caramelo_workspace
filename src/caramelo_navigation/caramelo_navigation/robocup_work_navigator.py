#!/usr/bin/env python3
"""
RoboCup@Work Task Navigator - Executa tarefas de pickup/delivery entre workstations
"""

import json
import math
import os
import time
from enum import Enum
from typing import List, Optional

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class TaskState(Enum):
    IDLE = 0
    NAVIGATING_TO_PICKUP = 1
    PERFORMING_PICKUP = 2
    NAVIGATING_TO_DELIVERY = 3
    PERFORMING_DELIVERY = 4
    TASK_COMPLETED = 5
    FAILED = 6


class RoboCupWorkNavigator(Node):
    def __init__(self):
        super().__init__('robocup_work_navigator')
        
        # Declarar parâmetros
        self.declare_parameter('waypoints_file', '/home/work/Caramelo_workspace/src/caramelo_navigation/config/waypoints.json')
        self.declare_parameter('tasks_file', '/home/work/Caramelo_workspace/src/caramelo_tasks/config/tasks.yaml')
        self.declare_parameter('map_file', '/home/work/Caramelo_workspace/mapa_20250704_145039.yaml')
        
        # Configurações
        self.approach_distance = 0.25
        self.pickup_wait_time = 5.0  # 5 segundos para simular pickup
        self.delivery_wait_time = 3.0  # 3 segundos para simular delivery
        
        # Estado da navegação
        self.state = TaskState.IDLE
        self.current_task_index = 0
        self.current_goal_handle = None
        self.operation_start_time = 0.0
        
        # Dados carregados
        self.workstations = {}
        self.tasks = []
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Action client para Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer para controle principal
        self.control_timer = self.create_timer(0.5, self.control_loop)
        
        # Carregar dados
        self.load_waypoints()
        self.load_tasks()
        
        self.get_logger().info("🏭 RoboCup@Work Navigator iniciado!")
        self.get_logger().info(f"🏭 Workstations carregadas: {len(self.workstations)}")
        self.get_logger().info(f"📋 Tarefas carregadas: {len(self.tasks)}")
        self.get_logger().info("⚠️ ATENÇÃO: Certifique-se que PWM e Encoder bringup estão rodando!")
        
    def load_waypoints(self):
        """Carrega waypoints e converte para workstations"""
        try:
            waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
            
            if not os.path.exists(waypoints_file):
                self.get_logger().error(f"❌ Arquivo de waypoints não encontrado: {waypoints_file}")
                return
            
            with open(waypoints_file, 'r') as f:
                data = json.load(f)
                
            # Converter waypoints WS para workstations
            for wp in data.get('waypoints', []):
                if wp.get('name', '').startswith('WS'):
                    # Converter WS01 -> WS1, WS02 -> WS2, etc.
                    ws_name = wp['name'].replace('WS0', 'WS').replace('WS', 'WS')
                    if wp['name'] == 'WS01':
                        ws_name = 'WS1'
                    elif wp['name'] == 'WS02':
                        ws_name = 'WS2'
                    elif wp['name'] == 'WS03':
                        ws_name = 'WS3'
                    elif wp['name'] == 'WS04':
                        ws_name = 'WS4'
                    elif wp['name'] == 'WS05':
                        ws_name = 'WS5'
                    elif wp['name'] == 'WS06':
                        ws_name = 'WS6'
                    
                    self.workstations[ws_name] = {
                        'position': [
                            wp['position']['x'],
                            wp['position']['y']
                        ],
                        'orientation': [
                            wp['orientation']['x'],
                            wp['orientation']['y'], 
                            wp['orientation']['z'],
                            wp['orientation']['w']
                        ]
                    }
            
            self.get_logger().info(f"🏭 Workstations: {list(self.workstations.keys())}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Erro ao carregar waypoints: {e}")
    
    def load_tasks(self):
        """Carrega tarefas do arquivo YAML"""
        try:
            tasks_file = self.get_parameter('tasks_file').get_parameter_value().string_value
            
            if not os.path.exists(tasks_file):
                self.get_logger().warn(f"⚠️ Arquivo de tarefas não encontrado: {tasks_file}")
                self.create_default_tasks()
                return
            
            with open(tasks_file, 'r') as f:
                data = yaml.safe_load(f)
                
            self.tasks = data.get('task_list', [])
            
            # Log das tarefas
            for i, task in enumerate(self.tasks):
                self.get_logger().info(f"📋 Tarefa {i+1}: {task['object']} | {task['pick_from']} → {task['place_to']}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Erro ao carregar tarefas: {e}")
            self.create_default_tasks()
    
    def create_default_tasks(self):
        """Cria tarefas padrão se não existir arquivo"""
        self.tasks = [
            {'object': 'R20', 'pick_from': 'WS1', 'place_to': 'WS3'},
            {'object': 'M20', 'pick_from': 'WS2', 'place_to': 'WS4'},
            {'object': 'F20_20_B', 'pick_from': 'WS5', 'place_to': 'WS6'}
        ]
        self.get_logger().info("📋 Usando tarefas padrão")
    
    def get_robot_pose(self) -> Optional[PoseStamped]:
        """Obter pose atual do robô"""
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            
            return pose
            
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None
    
    def navigate_to_workstation(self, ws_name: str) -> bool:
        """Navegar para uma workstation"""
        if ws_name not in self.workstations:
            self.get_logger().error(f"❌ Workstation {ws_name} não encontrada!")
            return False
        
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("❌ Nav2 server não disponível!")
            return False
        
        # Criar goal
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        ws = self.workstations[ws_name]
        goal_pose.pose.position.x = float(ws['position'][0])
        goal_pose.pose.position.y = float(ws['position'][1])
        goal_pose.pose.position.z = 0.0
        
        goal_pose.pose.orientation.x = float(ws['orientation'][0])
        goal_pose.pose.orientation.y = float(ws['orientation'][1])
        goal_pose.pose.orientation.z = float(ws['orientation'][2])
        goal_pose.pose.orientation.w = float(ws['orientation'][3])
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.get_logger().info(f"🎯 Navegando para {ws_name}")
        self.get_logger().info(f"📍 Posição: ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})")
        
        # Enviar goal
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        
        return True
    
    def goal_response_callback(self, future):
        """Callback quando o goal é aceito/rejeitado"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejeitado!")
            self.state = TaskState.FAILED
            return
        
        self.get_logger().info("✅ Goal aceito!")
        self.current_goal_handle = goal_handle
        
        # Aguardar resultado
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """Callback quando a navegação termina"""
        result = future.result()
        
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("✅ Chegou na workstation!")
            # Próximo estado depende do contexto
            if self.state == TaskState.NAVIGATING_TO_PICKUP:
                self.state = TaskState.PERFORMING_PICKUP
                self.operation_start_time = time.time()
            elif self.state == TaskState.NAVIGATING_TO_DELIVERY:
                self.state = TaskState.PERFORMING_DELIVERY
                self.operation_start_time = time.time()
        else:
            self.get_logger().warn(f"⚠️ Navegação falhou: {result.status}")
            self.state = TaskState.FAILED
    
    def simulate_pickup(self, task) -> bool:
        """Simula operação de pickup (espera 5 segundos)"""
        elapsed = time.time() - self.operation_start_time
        
        if elapsed < self.pickup_wait_time:
            if int(elapsed) != int(elapsed - 0.5):  # Log a cada segundo
                remaining = self.pickup_wait_time - elapsed
                self.get_logger().info(f"🤏 Pegando {task['object']} em {task['pick_from']}... ({remaining:.1f}s)")
            return False
        else:
            self.get_logger().info(f"✅ {task['object']} coletado de {task['pick_from']}!")
            return True
    
    def simulate_delivery(self, task) -> bool:
        """Simula operação de delivery (espera 3 segundos)"""
        elapsed = time.time() - self.operation_start_time
        
        if elapsed < self.delivery_wait_time:
            if int(elapsed) != int(elapsed - 0.5):  # Log a cada segundo
                remaining = self.delivery_wait_time - elapsed
                self.get_logger().info(f"📦 Entregando {task['object']} em {task['place_to']}... ({remaining:.1f}s)")
            return False
        else:
            self.get_logger().info(f"✅ {task['object']} entregue em {task['place_to']}!")
            return True
    
    def control_loop(self):
        """Loop principal de controle"""
        if not self.tasks:
            return
        
        if self.current_task_index >= len(self.tasks):
            self.get_logger().info("🎉 Todas as tarefas foram concluídas!")
            return
        
        current_task = self.tasks[self.current_task_index]
        
        if self.state == TaskState.IDLE:
            # Iniciar nova tarefa
            self.get_logger().info(f"🚀 Iniciando tarefa {self.current_task_index + 1}/{len(self.tasks)}")
            self.get_logger().info(f"📋 {current_task['object']}: {current_task['pick_from']} → {current_task['place_to']}")
            
            # Navegar para pickup
            if self.navigate_to_workstation(current_task['pick_from']):
                self.state = TaskState.NAVIGATING_TO_PICKUP
        
        elif self.state == TaskState.PERFORMING_PICKUP:
            # Executar pickup
            if self.simulate_pickup(current_task):
                # Pickup concluído, navegar para delivery
                if self.navigate_to_workstation(current_task['place_to']):
                    self.state = TaskState.NAVIGATING_TO_DELIVERY
        
        elif self.state == TaskState.PERFORMING_DELIVERY:
            # Executar delivery
            if self.simulate_delivery(current_task):
                # Tarefa concluída
                self.state = TaskState.TASK_COMPLETED
        
        elif self.state == TaskState.TASK_COMPLETED:
            # Ir para próxima tarefa
            self.current_task_index += 1
            self.state = TaskState.IDLE
            self.get_logger().info(f"✅ Tarefa {self.current_task_index}/{len(self.tasks)} concluída!")
        
        elif self.state == TaskState.FAILED:
            # Falha na tarefa, tentar próxima
            self.get_logger().warn(f"❌ Tarefa {self.current_task_index + 1} falhou!")
            self.current_task_index += 1
            self.state = TaskState.IDLE
    
    def start_tasks(self):
        """Iniciar execução das tarefas"""
        if not self.tasks:
            self.get_logger().error("❌ Nenhuma tarefa carregada!")
            return
        
        if not self.workstations:
            self.get_logger().error("❌ Nenhuma workstation carregada!")
            return
        
        self.current_task_index = 0
        self.state = TaskState.IDLE
        self.get_logger().info("🏭 Iniciando execução das tarefas RoboCup@Work!")


def main(args=None):
    rclpy.init(args=args)
    
    navigator = RoboCupWorkNavigator()
    
    try:
        # Aguardar TF se estabilizar
        time.sleep(3)
        
        # Iniciar tarefas
        navigator.start_tasks()
        
        # Spin
        rclpy.spin(navigator)
        
    except KeyboardInterrupt:
        pass
    finally:
        # Parar o robô
        cmd = Twist()
        navigator.cmd_vel_pub.publish(cmd)
        
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
