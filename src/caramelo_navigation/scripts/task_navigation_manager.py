#!/usr/bin/env python3
"""
CARAMELO TASK NAVIGATION MANAGER
Lê arquivos task.yaml e gera sequência de navegação automática
Compatível com BMT, BTT1, BTT2, ATT1, ATT2
"""

import json
import os
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class TaskNavigationManager(Node):
    def __init__(self):
        super().__init__('task_navigation_manager')
        
        # Parâmetros
        self.declare_parameter('arena', 'teste_robocup25')
        self.declare_parameter('task_type', 'BMT')
        
        self.arena = self.get_parameter('arena').value
        self.task_type = self.get_parameter('task_type').value
        
        # Caminhos
        self.workspace_path = '/home/work/Caramelo_workspace'
        self.maps_path = f'{self.workspace_path}/maps/{self.arena}'
        self.task_path = f'{self.workspace_path}/src/caramelo_tasks/{self.task_type}'
        
        # Action client para navegação
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publisher para pose inicial (localização)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            'initialpose', 
            10
        )
        
        # Dados carregados
        self.waypoints = {}
        self.task_data = {}
        self.navigation_sequence = []
        
        self.get_logger().info(f"🤖 TASK NAVIGATION MANAGER inicializado!")
        self.get_logger().info(f"🏁 Arena: {self.arena}")
        self.get_logger().info(f"📋 Task Type: {self.task_type}")
        
        # Aguardar Nav2 e carregar dados
        self.create_timer(2.0, self.initialize_system)
    
    def initialize_system(self):
        """Inicializa o sistema completo"""
        try:
            # 1. Carregar waypoints
            self.load_waypoints()
            
            # 2. Carregar task
            self.load_task()
            
            # 3. Gerar sequência de navegação
            self.generate_navigation_sequence()
            
            # 4. Aguardar Nav2 e iniciar
            self.create_timer(5.0, self.start_navigation)
            
        except Exception as e:
            self.get_logger().error(f"❌ Erro na inicialização: {e}")
    
    def load_waypoints(self):
        """Carrega waypoints da arena"""
        waypoints_file = f'{self.maps_path}/waypoints.json'
        
        if not os.path.exists(waypoints_file):
            raise FileNotFoundError(f"Arquivo waypoints não encontrado: {waypoints_file}")
        
        with open(waypoints_file, 'r') as f:
            data = json.load(f)
        
        # Converter para dicionário por nome
        for wp in data['waypoints']:
            self.waypoints[wp['name']] = {
                'position': wp['position'],
                'orientation': wp['orientation']
            }
        
        self.get_logger().info(f"📍 Waypoints carregados: {list(self.waypoints.keys())}")
    
    def load_task(self):
        """Carrega task.yaml"""
        task_file = f'{self.task_path}/task.yaml'
        
        if not os.path.exists(task_file):
            raise FileNotFoundError(f"Arquivo task não encontrado: {task_file}")
        
        with open(task_file, 'r') as f:
            self.task_data = yaml.safe_load(f)
        
        self.get_logger().info(f"📋 Task carregado: {task_file}")
        self.get_logger().info(f"   Start state: {self.task_data['start']}")
        self.get_logger().info(f"   Target state: {self.task_data['target']}")
    
    def generate_navigation_sequence(self):
        """Gera sequência de navegação baseada no task.yaml"""
        sequence = []
        
        # Começar no START
        sequence.append({
            'waypoint': 'START',
            'action': 'start',
            'description': 'Posição inicial - localização automática'
        })
        
        # Analisar task para encontrar movimentos necessários
        start_state = {ws['ws']: ws['objects'] for ws in self.task_data['start']}
        target_state = {ws['ws']: ws['objects'] for ws in self.task_data['target']}
        
        # Encontrar workstations com objetos para pegar
        for ws_name, objects in start_state.items():
            if objects:  # Se tem objetos no estado inicial
                # Verificar se esses objetos devem ser movidos
                target_objects = target_state.get(ws_name, [])
                if len(objects) > len(target_objects):
                    # Objetos devem ser removidos desta WS
                    sequence.append({
                        'waypoint': ws_name,
                        'action': 'pickup',
                        'objects': [obj for obj in objects if obj not in target_objects],
                        'description': f'Pegar objetos da {ws_name}'
                    })
        
        # Encontrar workstations que devem receber objetos
        for ws_name, objects in target_state.items():
            if objects:  # Se deve ter objetos no estado final
                start_objects = start_state.get(ws_name, [])
                if len(objects) > len(start_objects):
                    # Objetos devem ser adicionados a esta WS
                    sequence.append({
                        'waypoint': ws_name,
                        'action': 'place',
                        'objects': [obj for obj in objects if obj not in start_objects],
                        'description': f'Colocar objetos na {ws_name}'
                    })
        
        # Finalizar no FINISH
        if 'FINISH' in self.waypoints:
            sequence.append({
                'waypoint': 'FINISH',
                'action': 'finish',
                'description': 'Posição final - missão completa'
            })
        
        self.navigation_sequence = sequence
        
        self.get_logger().info("🗺️  SEQUÊNCIA DE NAVEGAÇÃO GERADA:")
        for i, step in enumerate(sequence):
            self.get_logger().info(f"   {i+1}. {step['waypoint']}: {step['description']}")
    
    def start_navigation(self):
        """Inicia navegação automática"""
        if not self.navigation_sequence:
            self.get_logger().error("❌ Nenhuma sequência de navegação gerada!")
            return
        
        self.get_logger().info("🚀 Iniciando navegação automática...")
        
        # Publicar pose inicial para localização
        self.publish_initial_pose()
        
        # Aguardar localização e iniciar sequência
        self.create_timer(3.0, self.execute_navigation_sequence)
    
    def publish_initial_pose(self):
        """Publica pose inicial para AMCL"""
        if 'START' not in self.waypoints:
            self.get_logger().warn("⚠️  Waypoint START não encontrado!")
            return
        
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        start_wp = self.waypoints['START']
        pose_msg.pose.pose.position.x = start_wp['position']['x']
        pose_msg.pose.pose.position.y = start_wp['position']['y']
        pose_msg.pose.pose.position.z = start_wp['position']['z']
        
        pose_msg.pose.pose.orientation.x = start_wp['orientation']['x']
        pose_msg.pose.pose.orientation.y = start_wp['orientation']['y']
        pose_msg.pose.pose.orientation.z = start_wp['orientation']['z']
        pose_msg.pose.pose.orientation.w = start_wp['orientation']['w']
        
        # Covariância baixa (confiança alta)
        pose_msg.pose.covariance = [0.1] * 36
        
        self.initial_pose_pub.publish(pose_msg)
        self.get_logger().info("📍 Pose inicial publicada para localização automática")
    
    def execute_navigation_sequence(self):
        """Executa sequência de navegação"""
        if not self.navigation_sequence:
            self.get_logger().info("✅ Navegação completa!")
            return
        
        current_step = self.navigation_sequence.pop(0)
        waypoint_name = current_step['waypoint']
        
        if waypoint_name not in self.waypoints:
            self.get_logger().error(f"❌ Waypoint {waypoint_name} não encontrado!")
            self.execute_navigation_sequence()  # Continuar com próximo
            return
        
        self.get_logger().info(f"🎯 Navegando para: {waypoint_name}")
        self.get_logger().info(f"   Ação: {current_step['description']}")
        
        # Criar goal de navegação
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        wp = self.waypoints[waypoint_name]
        goal_msg.pose.pose.position.x = wp['position']['x']
        goal_msg.pose.pose.position.y = wp['position']['y']
        goal_msg.pose.pose.position.z = wp['position']['z']
        
        goal_msg.pose.pose.orientation.x = wp['orientation']['x']
        goal_msg.pose.pose.orientation.y = wp['orientation']['y']
        goal_msg.pose.pose.orientation.z = wp['orientation']['z']
        goal_msg.pose.pose.orientation.w = wp['orientation']['w']
        
        # Enviar goal
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("❌ Nav2 action server não disponível!")
            return
        
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.navigation_callback)
    
    def navigation_callback(self, future):
        """Callback para resultado da navegação"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejeitado pelo Nav2!")
            self.execute_navigation_sequence()  # Continuar
            return
        
        self.get_logger().info("✅ Goal aceito! Aguardando conclusão...")
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)
    
    def navigation_result_callback(self, future):
        """Callback para resultado final da navegação"""
        result = future.result()
        
        if result.result:
            self.get_logger().info("✅ Navegação bem-sucedida!")
        else:
            self.get_logger().warn("⚠️  Navegação falhou, mas continuando...")
        
        # Aguardar um pouco e continuar sequência
        self.create_timer(2.0, self.execute_navigation_sequence)


def main():
    rclpy.init()
    
    try:
        manager = TaskNavigationManager()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
