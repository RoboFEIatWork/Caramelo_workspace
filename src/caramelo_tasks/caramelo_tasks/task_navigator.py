#!/usr/bin/env python3
"""
CARAMELO TASK NAVIGATOR
Sistema de navegação baseado em arquivos task.yaml
Responsável por:
1. Ler arquivo task.yaml
2. Comparar estado inicial vs final
3. Gerar sequência de navegação
4. Executar missão completa
5. Finalizar em FINISH
"""

import json
import os
import time
from typing import Dict, List, Tuple

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


class TaskNavigator(Node):
    def __init__(self):
        super().__init__('task_navigator')
        
        # Declarar parâmetros
        self.declare_parameter('task_file', '')
        self.declare_parameter('arena', 'teste_robocup25')
        self.declare_parameter('waypoints_file', '')
        
        # Action client para navegação
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publisher para status
        self.status_pub = self.create_publisher(String, '/task_status', 10)
        
        # Variáveis da missão
        self.waypoints = {}
        self.task_data = {}
        self.mission_sequence = []
        
        self.get_logger().info("🚀 CARAMELO TASK NAVIGATOR iniciado!")
        
        # Carregar configurações
        self.load_configurations()
        
        # Aguardar Nav2 estar disponível
        self.wait_for_nav2()
        
        # Processar task e iniciar missão
        self.process_task()
        
    def load_configurations(self):
        """Carrega waypoints e task data"""
        try:
            # Carregar waypoints
            arena = self.get_parameter('arena').get_parameter_value().string_value
            waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
            
            if not waypoints_file:
                waypoints_file = f"/home/work/Caramelo_workspace/maps/{arena}/waypoints.json"
            
            with open(waypoints_file, 'r') as f:
                waypoints_data = json.load(f)
                for wp in waypoints_data['waypoints']:
                    self.waypoints[wp['name']] = wp
            
            self.get_logger().info(f"✅ Waypoints carregados: {list(self.waypoints.keys())}")
            
            # Carregar task
            task_file = self.get_parameter('task_file').get_parameter_value().string_value
            if task_file:
                with open(task_file, 'r') as f:
                    self.task_data = yaml.safe_load(f)
                
                self.get_logger().info(f"✅ Task carregado: {task_file}")
                self.get_logger().info(f"   Estado inicial: {len(self.task_data.get('start', []))} workstations")
                self.get_logger().info(f"   Estado final: {len(self.task_data.get('target', []))} workstations")
            
        except Exception as e:
            self.get_logger().error(f"❌ Erro ao carregar configurações: {e}")
    
    def wait_for_nav2(self):
        """Aguarda Nav2 estar disponível"""
        self.get_logger().info("⏳ Aguardando Nav2 estar disponível...")
        
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("   Aguardando action server...")
        
        self.get_logger().info("✅ Nav2 disponível!")
    
    def process_task(self):
        """Processa o arquivo task.yaml e gera sequência de navegação"""
        if not self.task_data:
            self.get_logger().error("❌ Nenhum task carregado!")
            return
        
        try:
            # Analisar diferenças entre start e target
            start_state = {ws['ws']: ws for ws in self.task_data.get('start', [])}
            target_state = {ws['ws']: ws for ws in self.task_data.get('target', [])}
            
            self.get_logger().info("🔍 Analisando task...")
            
            # Encontrar objetos que precisam ser movidos
            moves_needed = []
            
            for ws_name in set(list(start_state.keys()) + list(target_state.keys())):
                start_ws = start_state.get(ws_name, {'objects': []})
                target_ws = target_state.get(ws_name, {'objects': []})
                
                start_objects = {obj['id'] for obj in start_ws.get('objects', [])}
                target_objects = {obj['id'] for obj in target_ws.get('objects', [])}
                
                # Objetos que saem desta workstation
                objects_to_remove = start_objects - target_objects
                # Objetos que chegam nesta workstation
                objects_to_add = target_objects - start_objects
                
                if objects_to_remove:
                    self.get_logger().info(f"   📤 {ws_name}: remover objetos {objects_to_remove}")
                
                if objects_to_add:
                    self.get_logger().info(f"   📥 {ws_name}: adicionar objetos {objects_to_add}")
                
                # Para BMT (teste básico), vamos só navegar para workstations com mudanças
                if objects_to_remove or objects_to_add:
                    moves_needed.append(ws_name)
            
            # Gerar sequência de navegação
            self.mission_sequence = []
            
            # Adicionar workstations que precisam de atenção
            for ws_name in moves_needed:
                if ws_name in self.waypoints:
                    self.mission_sequence.append(ws_name)
                else:
                    self.get_logger().warn(f"⚠️ Waypoint {ws_name} não encontrado!")
            
            # Adicionar FINISH no final
            if 'FINISH' in self.waypoints:
                self.mission_sequence.append('FINISH')
            
            self.get_logger().info(f"🎯 Sequência de navegação: {self.mission_sequence}")
            
            # Iniciar missão
            self.execute_mission()
            
        except Exception as e:
            self.get_logger().error(f"❌ Erro ao processar task: {e}")
    
    def execute_mission(self):
        """Executa a sequência de navegação"""
        if not self.mission_sequence:
            self.get_logger().warn("⚠️ Nenhuma missão para executar!")
            return
        
        self.get_logger().info("🚀 Iniciando execução da missão!")
        self.publish_status("MISSION_STARTED")
        
        # Executar cada waypoint da sequência
        for i, waypoint_name in enumerate(self.mission_sequence):
            self.get_logger().info(f"🎯 Navegando para {waypoint_name} ({i+1}/{len(self.mission_sequence)})")
            
            success = self.navigate_to_waypoint(waypoint_name)
            
            if success:
                self.get_logger().info(f"✅ Chegou em {waypoint_name}")
                self.publish_status(f"REACHED_{waypoint_name}")
                
                # Pausa para simulação de manipulação
                if waypoint_name != 'FINISH':
                    self.get_logger().info("🤖 Simulando manipulação de objetos...")
                    time.sleep(3.0)  # 3 segundos de "manipulação"
                
            else:
                self.get_logger().error(f"❌ Falha ao navegar para {waypoint_name}")
                self.publish_status(f"FAILED_{waypoint_name}")
                break
        
        # Missão completa
        self.get_logger().info("🏁 Missão completa!")
        self.publish_status("MISSION_COMPLETED")
    
    def navigate_to_waypoint(self, waypoint_name: str) -> bool:
        """Navega para um waypoint específico"""
        if waypoint_name not in self.waypoints:
            self.get_logger().error(f"❌ Waypoint {waypoint_name} não encontrado!")
            return False
        
        waypoint = self.waypoints[waypoint_name]
        
        # Criar goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Posição
        goal_msg.pose.pose.position.x = waypoint['position']['x']
        goal_msg.pose.pose.position.y = waypoint['position']['y']
        goal_msg.pose.pose.position.z = waypoint['position']['z']
        
        # Orientação
        goal_msg.pose.pose.orientation.x = waypoint['orientation']['x']
        goal_msg.pose.pose.orientation.y = waypoint['orientation']['y']
        goal_msg.pose.pose.orientation.z = waypoint['orientation']['z']
        goal_msg.pose.pose.orientation.w = waypoint['orientation']['w']
        
        # Enviar goal
        future = self.nav_client.send_goal_async(goal_msg)
        
        # Aguardar resposta
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f"❌ Goal rejeitado para {waypoint_name}")
            return False
        
        # Aguardar resultado
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)  # 60s timeout
        
        result = result_future.result()
        if result and result.status == 4:  # STATUS_SUCCEEDED
            return True
        else:
            return False
    
    def publish_status(self, status: str):
        """Publica status da missão"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    navigator = TaskNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info("🛑 Task Navigator interrompido pelo usuário")
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
