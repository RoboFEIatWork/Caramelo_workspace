#!/usr/bin/env python3
"""
CARAMELO BMT WAYPOINT NAVIGATION - Sistema Completo Integrado
Baseado no sistema funcional do commit 7e8a309 + integração BMT

Sistema que combina:
1. Navegação automática por waypoints (formato JSON)
2. Integração com sistema BMT                            self.get_logger().info(f"🔍 ANÁLISE BMT:")
        self.get_logger().info(f"   📦 Objetos para mover: {len(object_sources)}")
        self.get_logger().info(f"   🎯 Destinos únicos: {len(delivery_groups)}")
        
        # CRIAR TASKS DE NAVEGAÇÃO REAL COM COORDENADAS
        task_id_counter = 1           delivery_groups[target_ws]['sources'].add(source_ws)
        
        for target_ws, group_data in delivery_groups.items():       for target_ws, group_data in delivery_groups.items():   delivery_groups[target_ws]['sources'].add(source_ws)
        
        self.get_logger().info(f"🔍 ANÁLISE OTIMIZADA:")
        self.get_logger().info(f"   📦 Objetos para mover: {len(object_sources)}")
        self.get_logger().info(f"   🎯 Destinos únicos: {len(delivery_groups)}")
        
        # CRIAR TASKS SUPER OTIMIZADAS
        task_id_counter = 1self.get_logger().info(f"🔍 ANÁLISE OTIMIZADA:")
        self.get_logger().info(f"   📦 Objetos para mover: {len(object_sources)}")
        self.get_logger().info(f"   🎯 Destinos únicos: {len(delivery_groups)}")   self.get_logger().info(f"🔍 ANÁLISE OTIMIZADA:")
        self.get_logger().info(f"   📦 Objetos para mover: {len(object_sources)}")
        self.get_logger().info(f"   🎯 Destinos únicos: {len(delivery_groups)}")lf.get_logger().info(f"🔍 ANÁLISE OTIMIZADA:")
        self.get_logger().info(f"   📦 Objetos para mover: {len(object_sources)}")
        self.get_logger().info(f"   🎯 Destinos únicos: {len(delivery_groups)}")
        
        # CRIAR TASKS SUPER OTIMIZADAS
        task_id_counter = 1
        
        for target_ws, group_data in delivery_groups.items():k.yaml
3. Localização automática inteligente
4. Execução de tarefas em waypoints

Autor: GitHub Copilot + Sistema Funcional 7e8a309
Data: 2025-07-18
"""

import json
import math
import os
import time
from typing import Any, Dict, List, Optional

import rclpy
import tf2_ros
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String


class CarameloBMTWaypointNav(Node):
    def __init__(self):
        super().__init__('caramelo_bmt_waypoint_nav')
        
        # === PARÂMETROS ===
        self.declare_parameter('arena', 'arena_robocup25')  # Nome da arena
        self.declare_parameter('task_type', 'BMT')  # BMT, BTT1, BTT2
        self.declare_parameter('mission_mode', 'bmt')  # 'bmt' ou 'simple'
        self.declare_parameter('auto_start', True)
        self.declare_parameter('initial_pose_x', 0.0)
        self.declare_parameter('initial_pose_y', 0.0)
        self.declare_parameter('initial_pose_yaw', 0.0)
        
        self.arena = self.get_parameter('arena').get_parameter_value().string_value
        self.task_type = self.get_parameter('task_type').get_parameter_value().string_value
        self.mission_mode = self.get_parameter('mission_mode').get_parameter_value().string_value
        self.auto_start = self.get_parameter('auto_start').get_parameter_value().bool_value
        self.initial_x = self.get_parameter('initial_pose_x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('initial_pose_y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('initial_pose_yaw').get_parameter_value().double_value
        
        # Configurar caminhos baseados na arena e task_type
        self.waypoints_file = f'/home/work/Caramelo_workspace/maps/{self.arena}/waypoints.json'
        self.task_file = f'/home/work/Caramelo_workspace/src/caramelo_tasks/{self.task_type}/task.yaml'
        
        self.get_logger().info(f"🎯 Arena: {self.arena}")
        self.get_logger().info(f"📋 Task Type: {self.task_type}")
        self.get_logger().info(f"� Waypoints: {self.waypoints_file}")
        self.get_logger().info(f"📄 Task File: {self.task_file}")
        
        # === PUBLISHERS E SUBSCRIBERS ===
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )
        
        self.pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        
        # BMT Task Status
        self.task_status_pub = self.create_publisher(
            String,
            '/bmt/task_status',
            10
        )
        
        # === ACTION CLIENTS ===
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # === TF ===
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # === ESTADO DO SISTEMA ===
        self.waypoints_db: Dict[str, Dict] = {}  # Database de waypoints por nome
        self.mission_waypoints: List[Dict] = []  # Lista de waypoints da missão
        self.bmt_tasks: List[Dict] = []          # Lista de tarefas BMT
        self.current_task_index = 0
        self.current_waypoint_index = 0
        self.mission_active = False
        self.localization_ready = False
        self.initial_pose_verified = False
        
        # === STATUS FLAGS ===
        self.navigation_in_progress = False
        self.task_execution_in_progress = False
        
        self.get_logger().info("🤖 CARAMELO BMT WAYPOINT NAVIGATION - Sistema Integrado Iniciado!")
        self.get_logger().info(f"📋 Modo: {self.mission_mode.upper()}")
        
        # === INICIALIZAÇÃO ===
        self.load_waypoints_database()
        self.load_bmt_tasks() if self.mission_mode in ['bmt', 'robocup'] else self.create_simple_mission()
        
        # Timer para inicialização escalonada (baseado no sistema funcional)
        self.init_timer = self.create_timer(2.0, self.initialize_navigation)
    
    def load_waypoints_database(self):
        """Carrega database de waypoints do arquivo JSON"""
        try:
            if os.path.exists(self.waypoints_file):
                with open(self.waypoints_file, 'r') as f:
                    waypoints_data = json.load(f)
                    
                waypoints_list = waypoints_data.get('waypoints', [])
                
                # Cria database por nome
                for wp in waypoints_list:
                    name = wp.get('name')
                    if name:
                        self.waypoints_db[name] = wp
                        
                self.get_logger().info(f"📍 Database carregado: {len(self.waypoints_db)} waypoints")
                for name in self.waypoints_db.keys():
                    self.get_logger().info(f"   - {name}")
                        
            else:
                self.get_logger().error(f"❌ Arquivo waypoints não encontrado: {self.waypoints_file}")
                self.create_default_waypoints()
                
        except Exception as e:
            self.get_logger().error(f"❌ Erro ao carregar waypoints: {e}")
            self.create_default_waypoints()
    
    def load_bmt_tasks(self):
        """Carrega tarefas BMT do arquivo task.yaml (formato RoboCup real)"""
        try:
            if os.path.exists(self.task_file):
                with open(self.task_file, 'r') as f:
                    task_data = yaml.safe_load(f)
                
                self.get_logger().info(f"📋 Carregando tasks do formato RoboCup: {self.task_type}")
                
                # FORMATO ROBOCUP: start e target com workstations e objetos
                if 'start' in task_data and 'target' in task_data:
                    self.bmt_tasks = self.convert_robocup_to_navigation_tasks(task_data)
                    
                # Formato legado (se existir)
                elif 'tasks' in task_data:
                    self.bmt_tasks = task_data['tasks']
                elif 'mission' in task_data and 'tasks' in task_data['mission']:
                    self.bmt_tasks = task_data['mission']['tasks']
                elif 'waypoint_sequence' in task_data:
                    # Converte sequência simples para formato BMT
                    self.bmt_tasks = []
                    for wp_name in task_data['waypoint_sequence']:
                        if wp_name.upper() != 'START':  # Pula START
                            self.bmt_tasks.append({
                                'task_id': f'nav_to_{wp_name}',
                                'type': 'navigation',
                                'waypoint': wp_name,
                                'action': 'navigate_and_wait',
                                'wait_time': 5.0
                            })
                else:
                    self.get_logger().error("❌ Formato de task.yaml não reconhecido!")
                    self.create_default_tasks()
                    return
                
                self.get_logger().info(f"✅ {len(self.bmt_tasks)} tarefas de navegação criadas")
                for i, task in enumerate(self.bmt_tasks):
                    task_id = task.get('task_id', f'task_{i}')
                    waypoint = task.get('waypoint', 'N/A')
                    action = task.get('action_type', 'navigate')
                    self.get_logger().info(f"   {i+1}. {task_id}: {action} -> {waypoint}")
                    
            else:
                self.get_logger().warn(f"⚠️ Arquivo task.yaml não encontrado: {self.task_file}")
                self.create_default_tasks()
                
        except Exception as e:
            self.get_logger().error(f"❌ Erro ao carregar tasks RoboCup: {e}")
            self.create_default_tasks()
    
    def convert_robocup_to_navigation_tasks(self, robocup_data):
        """Converte formato RoboCup (start/target) para tasks de navegação OTIMIZADAS"""
        tasks = []
        
        # ANÁLISE DOS OBJETOS: onde estão e para onde devem ir
        start_config = robocup_data.get('start', [])
        target_config = robocup_data.get('target', [])
        
        # Mapear objetos: origem → destino
        object_sources = {}  # objeto_id → ws_origem
        object_targets = {}  # objeto_id → ws_destino
        
        # Processar configuração inicial (onde estão os objetos)
        for ws_config in start_config:
            ws_name = ws_config.get('ws')
            objects = ws_config.get('objects', [])
            
            if isinstance(objects, list):
                for obj in objects:
                    obj_id = obj.get('id')
                    if obj_id:
                        object_sources[obj_id] = ws_name
            elif isinstance(objects, dict) and 'id' in objects:
                # Objeto único no formato direto
                obj_id = objects['id']
                object_sources[obj_id] = ws_name
        
        # Processar configuração final (para onde devem ir)
        for ws_config in target_config:
            ws_name = ws_config.get('ws')
            objects = ws_config.get('objects', [])
            
            if isinstance(objects, list):
                for obj in objects:
                    obj_id = obj.get('id')
                    if obj_id:
                        object_targets[obj_id] = ws_name
            elif isinstance(objects, dict) and 'id' in objects:
                obj_id = objects['id']
                object_targets[obj_id] = ws_name
        
        # AGRUPAR POR DESTINO (chave da otimização!)
        delivery_groups = {}  # target_ws → {'objects': [obj_info], 'sources': set()}
        
        for obj_id in object_sources:
            if obj_id in object_targets:
                source_ws = object_sources[obj_id]
                target_ws = object_targets[obj_id]
                
                if source_ws != target_ws:  # Só move se origem ≠ destino
                    if target_ws not in delivery_groups:
                        delivery_groups[target_ws] = {
                            'objects': [],
                            'sources': set()
                        }
                    
                    delivery_groups[target_ws]['objects'].append({
                        'id': obj_id,
                        'source': source_ws
                    })
                    delivery_groups[target_ws]['sources'].add(source_ws)
        
        self.get_logger().info(f"� Análise de objetos:")
        self.get_logger().info(f"   📦 Objetos encontrados: {len(object_sources)}")
        #self.get_logger().info(f"   🎯 Movimentos necessários: {len(object_moves)} workstations")
        
        # CRIAR TASKS SUPER OTIMIZADAS
        task_id_counter = 1
        
        for target_ws, group_data in delivery_groups.items():
            if target_ws not in self.waypoints_db:
                self.get_logger().warn(f"⚠️ Waypoint '{target_ws}' não encontrado no waypoints.json!")
                continue
                
            objects = group_data['objects']
            source_workstations = list(group_data['sources'])
            
            # Verificar se todas as sources existem no waypoints_db
            valid_sources = [ws for ws in source_workstations if ws in self.waypoints_db]
            
            if not valid_sources:
                self.get_logger().warn(f"⚠️ Nenhuma source válida para {target_ws}")
                continue
            
            # LÓGICA BMT: Coletar de sources → Entregar em target
            obj_ids = [obj['id'] for obj in objects]
            obj_list = ', '.join(map(str, obj_ids))
            sources_list = ' → '.join(valid_sources)
            
            self.get_logger().info(f"📦 Task {task_id_counter}: Coletar objetos {obj_list}")
            self.get_logger().info(f"   📍 DE: {sources_list}")
            self.get_logger().info(f"   📍 PARA: {target_ws}")
            
            # FASE 1: Ir para cada source e coletar objetos
            for source_ws in valid_sources:
                source_objects = [obj for obj in objects if obj['source'] == source_ws]
                if source_objects:
                    source_obj_ids = [obj['id'] for obj in source_objects]
                    source_obj_list = ', '.join(map(str, source_obj_ids))
                    
                    # USAR COORDENADAS REAIS DO WAYPOINTS.JSON
                    source_waypoint = self.waypoints_db[source_ws]
                    
                    collect_task = {
                        'task_id': f'collect_from_{source_ws}_task_{task_id_counter}',
                        'type': 'navigation',
                        'task_type': 'BMT_COLLECT',
                        'waypoint': source_ws,
                        'position': {
                            'x': source_waypoint['position']['x'],
                            'y': source_waypoint['position']['y'],
                            'z': source_waypoint['position'].get('z', 0.0)
                        },
                        'orientation': {
                            'x': source_waypoint['orientation'].get('x', 0.0),
                            'y': source_waypoint['orientation'].get('y', 0.0),
                            'z': source_waypoint['orientation'].get('z', 0.0),
                            'w': source_waypoint['orientation'].get('w', 1.0)
                        },
                        'action_type': 'navigate_and_collect',
                        'wait_time': 8.0,
                        'timeout': 180.0,
                        'bmt_data': {
                            'source_ws': source_ws,
                            'target_ws': target_ws,
                            'objects_to_collect': source_obj_ids,
                            'phase': 'collect'
                        }
                    }
                    tasks.append(collect_task)
                    self.get_logger().info(f"   ✅ Subtask: Coletar objetos {source_obj_list} em {source_ws}")
                    self.get_logger().info(f"      📍 Coordenadas: x={source_waypoint['position']['x']:.2f}, y={source_waypoint['position']['y']:.2f}")
            
            # FASE 2: Ir para target e entregar todos os objetos
            target_waypoint = self.waypoints_db[target_ws]
            
            deliver_task = {
                'task_id': f'deliver_to_{target_ws}_task_{task_id_counter}',
                'type': 'navigation',
                'task_type': 'BMT_DELIVER',
                'waypoint': target_ws,
                'position': {
                    'x': target_waypoint['position']['x'],
                    'y': target_waypoint['position']['y'],
                    'z': target_waypoint['position'].get('z', 0.0)
                },
                'orientation': {
                    'x': target_waypoint['orientation'].get('x', 0.0),
                    'y': target_waypoint['orientation'].get('y', 0.0),
                    'z': target_waypoint['orientation'].get('z', 0.0),
                    'w': target_waypoint['orientation'].get('w', 1.0)
                },
                'action_type': 'navigate_and_deliver',
                'wait_time': 8.0,
                'timeout': 180.0,
                'bmt_data': {
                    'target_ws': target_ws,
                    'objects_to_deliver': obj_ids,
                    'phase': 'deliver'
                }
            }
            tasks.append(deliver_task)
            self.get_logger().info(f"   ✅ Subtask: Entregar objetos {obj_list} em {target_ws}")
            self.get_logger().info(f"      📍 Coordenadas: x={target_waypoint['position']['x']:.2f}, y={target_waypoint['position']['y']:.2f}")
            
            task_id_counter += 1
        
        # Task de retorno ao FINISH
        if 'FINISH' in self.waypoints_db:
            finish_task = {
                'task_id': 'return_to_finish',
                'type': 'navigation',
                'waypoint': 'FINISH',
                'action_type': 'finish_mission',
                'wait_time': 5.0,
                'timeout': 120.0,
                'robocup_data': {
                    'phase': 'finish'
                }
            }
            tasks.append(finish_task)
            self.get_logger().info(f"🏁 Task {task_id_counter}: Retornar ao FINISH")
        
        self.get_logger().info(f"✅ Total de {len(tasks)} tasks SUPER OTIMIZADAS criadas")
        self.get_logger().info(f"💡 Vantagens: Menos viagens, múltiplas coletas, entregas agrupadas")
        return tasks
    
    def create_simple_mission(self):
        """Cria missão simples visitando todos os waypoints (exceto START)"""
        self.bmt_tasks = []
        
        for name, wp_data in self.waypoints_db.items():
            if name.upper() != 'START':  # Pula START
                self.bmt_tasks.append({
                    'task_id': f'visit_{name}',
                    'type': 'navigation',
                    'waypoint': name,
                    'action': 'navigate_and_wait',
                    'wait_time': 3.0
                })
        
        self.get_logger().info(f"🎯 Missão simples criada: {len(self.bmt_tasks)} waypoints")
    
    def create_default_waypoints(self):
        """Cria waypoints padrão se não houver arquivo"""
        default_waypoints = {
            "frame_id": "map",
            "waypoints": [
                {
                    "name": "START",
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                },
                {
                    "name": "WP1",
                    "position": {"x": 1.0, "y": 0.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                },
                {
                    "name": "WP2", 
                    "position": {"x": 1.0, "y": 1.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.707, "w": 0.707}
                }
            ]
        }
        
        # Salva arquivo padrão
        os.makedirs(os.path.dirname(self.waypoints_file), exist_ok=True)
        with open(self.waypoints_file, 'w') as f:
            json.dump(default_waypoints, f, indent=2)
            
        self.get_logger().info(f"✅ Waypoints padrão criados em: {self.waypoints_file}")
        self.load_waypoints_database()  # Recarrega
    
    def create_default_tasks(self):
        """Cria tarefas BMT padrão"""
        self.bmt_tasks = [
            {
                'task_id': 'nav_to_wp1',
                'type': 'navigation',
                'waypoint': 'WP1',
                'action': 'navigate_and_wait',
                'wait_time': 5.0
            },
            {
                'task_id': 'nav_to_wp2',
                'type': 'navigation', 
                'waypoint': 'WP2',
                'action': 'navigate_and_wait',
                'wait_time': 5.0
            }
        ]
        
        # Salva arquivo padrão
        task_data = {'tasks': self.bmt_tasks}
        os.makedirs(os.path.dirname(self.task_file), exist_ok=True)
        with open(self.task_file, 'w') as f:
            yaml.dump(task_data, f, default_flow_style=False)
            
        self.get_logger().info(f"✅ Tasks BMT padrão criadas em: {self.task_file}")
    
    def initialize_navigation(self):
        """Inicializa navegação com correção automática (baseado no sistema funcional)"""
        self.init_timer.cancel()
        
        # Aguarda Nav2
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("❌ Nav2 não está disponível!")
            return
            
        self.get_logger().info("✅ Nav2 conectado!")
        
        # Sistema de correção automática de localização
        self.get_logger().info("🔍 Iniciando correção automática de localização...")
        self.correct_initial_pose()
        
        # Se auto_start estiver habilitado, sistema iniciará automaticamente
        if not self.auto_start:
            self.get_logger().info("🔄 Auto-start desabilitado. Sistema aguarda comando manual.")
    
    def correct_initial_pose(self):
        """Publica pose inicial para AMCL (baseado no sistema funcional)"""
        self.get_logger().info(f"🔧 Corrigindo pose inicial para ({self.initial_x:.2f}, {self.initial_y:.2f}, {self.initial_yaw:.2f})")
        
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        # Pose inicial configurável
        pose_msg.pose.pose.position.x = self.initial_x
        pose_msg.pose.pose.position.y = self.initial_y
        pose_msg.pose.pose.position.z = 0.0
        
        # Converte yaw para quaternion
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = math.sin(self.initial_yaw / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(self.initial_yaw / 2.0)
        
        # Covariância (confiança moderada)
        covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]
        pose_msg.pose.covariance = covariance
        
        # Publica correção
        self.initial_pose_pub.publish(pose_msg)
        self.get_logger().info("📍 Pose inicial publicada para AMCL")
        
        # Aguarda verificação automática
        self.verification_timer = self.create_timer(3.0, self.verify_localization)
    
    def pose_callback(self, msg):
        """Callback para verificar pose atual (baseado no sistema funcional)"""
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        
        # Tolerância para considerar localização correta
        tolerance = 0.5  # 50cm
        
        target_distance = math.sqrt(
            (current_x - self.initial_x)**2 + (current_y - self.initial_y)**2
        )
        
        if not self.localization_ready:
            if target_distance < tolerance:
                self.get_logger().info(f"✅ Localização verificada: ({current_x:.3f}, {current_y:.3f})")
                self.localization_ready = True
                self.initial_pose_verified = True
                
                # Se auto_start, inicia missão
                if self.auto_start and not self.mission_active:
                    self.start_bmt_mission()
            else:
                self.get_logger().warn(
                    f"⚠️ Pose atual ({current_x:.3f}, {current_y:.3f}) distante do target. "
                    f"Distância: {target_distance:.3f}m (tolerância: {tolerance}m)"
                )
    
    def verify_localization(self):
        """Verifica status da localização"""
        if hasattr(self, 'verification_timer'):
            self.verification_timer.cancel()
            
        if not self.localization_ready:
            self.get_logger().warn("⚠️ Localização ainda não verificada. Aguardando...")
            # Reagenda verificação
            self.verification_timer = self.create_timer(2.0, self.verify_localization)
    
    def start_bmt_mission(self):
        """Inicia missão BMT"""
        if not self.bmt_tasks:
            self.get_logger().warn("⚠️ Nenhuma task BMT para executar!")
            return
            
        if self.mission_active:
            self.get_logger().warn("⚠️ Missão já está ativa!")
            return
            
        self.mission_active = True
        self.current_task_index = 0
        
        self.get_logger().info(f"🚀 Iniciando missão BMT com {len(self.bmt_tasks)} tasks!")
        self.publish_task_status("MISSION_STARTED")
        
        self.execute_next_task()
    
    def execute_next_task(self):
        """Executa próxima task BMT"""
        if self.current_task_index >= len(self.bmt_tasks):
            self.get_logger().info("🏁 MISSÃO BMT COMPLETA!")
            self.publish_task_status("MISSION_COMPLETE")
            self.mission_active = False
            return
            
        current_task = self.bmt_tasks[self.current_task_index]
        task_id = current_task.get('task_id', f'task_{self.current_task_index}')
        task_type = current_task.get('type', 'unknown')
        
        self.get_logger().info(f"📋 Executando Task {self.current_task_index + 1}/{len(self.bmt_tasks)}: {task_id}")
        self.publish_task_status(f"EXECUTING_{task_id}")
        
        if task_type == 'navigation':
            self.execute_navigation_task(current_task)
        elif task_type == 'manipulation':
            self.execute_manipulation_task(current_task)
        elif task_type == 'wait':
            self.execute_wait_task(current_task)
        else:
            self.get_logger().error(f"❌ Tipo de task desconhecido: {task_type}")
            self.task_completed(False)
    
    def execute_navigation_task(self, task):
        """Executa task de navegação (incluindo coleta múltipla otimizada)"""
        action_type = task.get('action_type', 'navigate')
        
        if action_type == 'multi_collect_and_deliver':
            self.execute_multi_transport_task(task)
        elif action_type == 'transport_objects':
            self.execute_transport_task(task)
        else:
            self.execute_simple_navigation(task)
    
    def execute_multi_transport_task(self, task):
        """Executa task de coleta múltipla e entrega otimizada"""
        target_waypoint = task.get('target_waypoint')
        robocup_data = task.get('robocup_data', {})
        objects = robocup_data.get('objects', [])
        collection_sequence = robocup_data.get('collection_sequence', [])
        
        self.get_logger().info(f"🚀 COLETA MÚLTIPLA OTIMIZADA:")
        self.get_logger().info(f"   📦 {len(objects)} objetos para coletar")
        self.get_logger().info(f"   🎯 Destino final: {target_waypoint}")
        self.get_logger().info(f"   🔄 Sequência: {' → '.join(collection_sequence)} → {target_waypoint}")
        
        # Inicializar estado de coleta múltipla
        if not hasattr(self, 'multi_collection_phase'):
            self.multi_collection_phase = 'collecting'
            self.current_multi_task = task
            self.collection_index = 0
            self.collected_objects = []
        
        # Fase 1: Coletar objetos (percorre cada workstation)
        if self.multi_collection_phase == 'collecting':
            if self.collection_index < len(collection_sequence):
                current_source = collection_sequence[self.collection_index]
                
                # Filtrar objetos desta workstation
                objects_from_source = [obj for obj in objects if obj['source'] == current_source]
                obj_ids = [obj['id'] for obj in objects_from_source]
                
                self.get_logger().info(f"📍 Coletando em {current_source}: objetos {', '.join(map(str, obj_ids))}")
                self.navigate_to_waypoint(current_source, 'collect_multiple_objects')
            else:
                # Todas as coletas concluídas, partir para entrega
                self.multi_collection_phase = 'delivering'
                self.get_logger().info(f"✅ Coleta completa! {len(self.collected_objects)} objetos coletados")
                self.get_logger().info(f"🚚 Transportando para {target_waypoint}...")
                self.navigate_to_waypoint(target_waypoint, 'deliver_multiple_objects')
        
        # Fase 2: Entregar todos os objetos
        elif self.multi_collection_phase == 'delivering':
            self.navigate_to_waypoint(target_waypoint, 'deliver_multiple_objects')
    
    def execute_transport_task(self, task):
        """Executa task de transporte: origem → destino"""
        source_waypoint = task.get('waypoint')
        target_waypoint = task.get('target_waypoint')
        robocup_data = task.get('robocup_data', {})
        object_ids = robocup_data.get('object_ids', [])
        
        self.get_logger().info(f"📦 TRANSPORTE: {source_waypoint} → {target_waypoint}")
        self.get_logger().info(f"   Objetos: {', '.join(map(str, object_ids))}")
        
        # Fase 1: Navegar para origem (coletar)
        if not hasattr(self, 'transport_phase') or self.transport_phase == 'collect':
            self.transport_phase = 'collect'
            self.current_transport_task = task
            self.navigate_to_waypoint(source_waypoint, 'collect_objects')
        # Fase 2: Navegar para destino (entregar)
        elif self.transport_phase == 'deliver':
            self.navigate_to_waypoint(target_waypoint, 'deliver_objects')
    
    def execute_simple_navigation(self, task):
        """Executa navegação simples para um waypoint"""
        waypoint_name = task.get('waypoint')
        action_type = task.get('action_type', 'navigate')
        
        # Extrair coordenadas da task se existirem
        task_data = None
        if 'position' in task and 'orientation' in task:
            task_data = {
                'position': task['position'],
                'orientation': task['orientation']
            }
        
        self.navigate_to_waypoint(waypoint_name, action_type, task_data)
    
    def navigate_to_waypoint(self, waypoint_name, action_type='navigate', task_data=None):
        """Navega para um waypoint específico"""
        if waypoint_name not in self.waypoints_db:
            self.get_logger().error(f"❌ Waypoint '{waypoint_name}' não encontrado!")
            self.task_completed(False)
            return
            
        # Se task_data tem coordenadas, usar elas. Senão, usar waypoints_db
        if task_data and 'position' in task_data and 'orientation' in task_data:
            pos = task_data['position']
            ori = task_data['orientation']
            self.get_logger().info(f"🎯 Usando coordenadas da task para {waypoint_name}")
        else:
            waypoint_data = self.waypoints_db[waypoint_name]
            pos = waypoint_data['position']
            ori = waypoint_data['orientation']
            self.get_logger().info(f"🎯 Usando coordenadas do waypoints.json para {waypoint_name}")
        
        # Cria goal de navegação
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_pose.pose.position.x = float(pos['x'])
        goal_pose.pose.position.y = float(pos['y'])
        goal_pose.pose.position.z = float(pos['z'])
        goal_pose.pose.orientation.x = float(ori['x'])
        goal_pose.pose.orientation.y = float(ori['y'])
        goal_pose.pose.orientation.z = float(ori['z'])
        goal_pose.pose.orientation.w = float(ori['w'])
        
        self.get_logger().info(f"📍 Navegando para {waypoint_name}: x={pos['x']:.2f}, y={pos['y']:.2f}")
        
        # Cria action goal
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_pose
        
        action_desc = {
            'collect_objects': f"🔄 Coletando objetos em",
            'deliver_objects': f"📦 Entregando objetos em",
            'collect_multiple_objects': f"🔄 Coletando múltiplos objetos em",
            'deliver_multiple_objects': f"📦 Entregando múltiplos objetos em",
            'navigate': f"🎯 Navegando para",
            'finish_mission': f"🏁 Retornando ao"
        }.get(action_type, f"🎯 Navegando para")
        
        self.get_logger().info(f"{action_desc} '{waypoint_name}': x={pos['x']:.2f}, y={pos['y']:.2f}")
        self.navigation_in_progress = True
        self.current_action_type = action_type
        
        # Armazena task atual para usar nos callbacks
        current_task = self.bmt_tasks[self.current_task_index] if self.current_task_index < len(self.bmt_tasks) else {}
        self.current_navigation_task = current_task
        
        # Se for task de transporte, usar a task de transporte
        if hasattr(self, 'current_transport_task'):
            self.current_navigation_task = self.current_transport_task
        
        # Se for task de coleta múltipla, usar a task múltipla
        if hasattr(self, 'current_multi_task'):
            self.current_navigation_task = self.current_multi_task
        
        self.get_logger().info(f"🔍 Task armazenada: {self.current_navigation_task.get('task_id', 'unknown')}")
        
        # Inicializa variáveis de controle se não existirem
        if not hasattr(self, 'transport_phase'):
            self.transport_phase = None
        
        # Envia goal
        future = self.nav_client.send_goal_async(
            nav_goal,
            feedback_callback=self.navigation_feedback
        )
        future.add_done_callback(self.navigation_response)
    
    def execute_manipulation_task(self, task):
        """Executa task de manipulação - PLACEHOLDER"""
        self.get_logger().info("🦾 Executando task de manipulação...")
        # TODO: Implementar integração com sistema de manipulação
        
        # Simula execução
        wait_time = task.get('duration', 5.0)
        self.get_logger().info(f"⏳ Simulando manipulação por {wait_time}s...")
        self.create_timer(wait_time, lambda: self.task_completed(True))
    
    def execute_wait_task(self, task):
        """Executa task de espera"""
        wait_time = task.get('duration', 5.0)
        self.get_logger().info(f"⏳ Aguardando {wait_time}s...")
        self.create_timer(wait_time, lambda: self.task_completed(True))
    
    def navigation_feedback(self, feedback_msg):
        """Callback de feedback da navegação"""
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        
        if hasattr(self, 'current_navigation_task'):
            waypoint_name = self.current_navigation_task.get('waypoint', 'unknown')
            self.get_logger().info(
                f"🚶 Navegando para '{waypoint_name}': {distance:.2f}m restantes", 
                throttle_duration_sec=3.0
            )
    
    def navigation_response(self, future):
        """Callback de resposta da navegação"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal de navegação rejeitado!")
            self.task_completed(False)
            return
            
        waypoint_name = self.current_navigation_task.get('waypoint', 'unknown')
        self.get_logger().info(f"✅ Goal aceito para '{waypoint_name}'")
        
        # Aguarda resultado
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result)
    
    def navigation_result(self, future):
        """Callback de resultado da navegação (incluindo transporte)"""
        result = future.result().result
        
        self.navigation_in_progress = False
        
        if result:
            # Verifica se é uma task de transporte
            action_type = getattr(self, 'current_action_type', 'navigate')
            
            if action_type == 'collect_multiple_objects' and hasattr(self, 'multi_collection_phase'):
                # Coleta múltipla em progresso
                current_source = self.current_multi_task.get('robocup_data', {}).get('collection_sequence', [])[self.collection_index]
                objects_from_source = [obj for obj in self.current_multi_task.get('robocup_data', {}).get('objects', []) 
                                     if obj['source'] == current_source]
                
                self.get_logger().info(f"✅ Objetos coletados em '{current_source}'!")
                for obj in objects_from_source:
                    self.collected_objects.append(obj)
                    self.get_logger().info(f"   📦 Objeto {obj['id']} armazenado no robô")
                
                # Simula tempo de coleta
                wait_time = self.current_multi_task.get('wait_time', 5.0)
                self.get_logger().info(f"⏳ Coletando objetos por {wait_time}s...")
                
                # Próxima workstation ou entrega
                self.collection_index += 1
                self.create_timer(wait_time, lambda: self.execute_multi_transport_task(self.current_multi_task))
                
            elif action_type == 'deliver_multiple_objects' and hasattr(self, 'multi_collection_phase'):
                # Entrega múltipla concluída
                target_ws = self.current_multi_task.get('target_waypoint')
                total_objects = len(self.collected_objects)
                obj_ids = [obj['id'] for obj in self.collected_objects]
                
                self.get_logger().info(f"✅ ENTREGA MÚLTIPLA COMPLETA em '{target_ws}'!")
                self.get_logger().info(f"   📦 {total_objects} objetos entregues: {', '.join(map(str, obj_ids))}")
                
                # Simula tempo de entrega múltipla
                wait_time = self.current_multi_task.get('wait_time', 8.0)
                self.get_logger().info(f"⏳ Entregando {total_objects} objetos por {wait_time}s...")
                
                # Reset estado de coleta múltipla
                self.multi_collection_phase = None
                self.collection_index = 0
                self.collected_objects = []
                if hasattr(self, 'current_multi_task'):
                    delattr(self, 'current_multi_task')
                    
                self.create_timer(wait_time, lambda: self.task_completed(True))
                
            elif action_type == 'collect_objects' and hasattr(self, 'transport_phase'):
                # Fase 1 concluída: objetos coletados, agora entregar
                source_ws = self.current_transport_task.get('waypoint')
                target_ws = self.current_transport_task.get('target_waypoint')
                robocup_data = self.current_transport_task.get('robocup_data', {})
                object_ids = robocup_data.get('object_ids', [])
                
                self.get_logger().info(f"✅ Objetos coletados em '{source_ws}'!")
                self.get_logger().info(f"📦 Transportando {', '.join(map(str, object_ids))} para '{target_ws}'...")
                
                # Simula tempo de coleta
                wait_time = self.current_transport_task.get('wait_time', 5.0)
                self.get_logger().info(f"⏳ Coletando objetos por {wait_time}s...")
                
                # Muda para fase de entrega após aguardar
                self.transport_phase = 'deliver'
                self.create_timer(wait_time, lambda: self.execute_transport_task(self.current_transport_task))
                
            elif action_type == 'deliver_objects' and hasattr(self, 'transport_phase'):
                # Fase 2 concluída: objetos entregues
                target_ws = self.current_transport_task.get('target_waypoint')
                robocup_data = self.current_transport_task.get('robocup_data', {})
                object_ids = robocup_data.get('object_ids', [])
                
                self.get_logger().info(f"✅ Objetos {', '.join(map(str, object_ids))} entregues em '{target_ws}'!")
                
                # Simula tempo de entrega
                wait_time = self.current_transport_task.get('wait_time', 5.0)
                self.get_logger().info(f"⏳ Entregando objetos por {wait_time}s...")
                
                # Reset fase de transporte e completa task
                self.transport_phase = None
                if hasattr(self, 'current_transport_task'):
                    delattr(self, 'current_transport_task')
                    
                self.create_timer(wait_time, lambda: self.task_completed(True))
                
            else:
                # Navegação simples
                waypoint_name = self.current_navigation_task.get('waypoint', 'unknown')
                self.get_logger().info(f"🎉 Waypoint '{waypoint_name}' alcançado!")
                
                # Executa wait_time se especificado
                wait_time = self.current_navigation_task.get('wait_time', 0.0)
                if wait_time > 0:
                    self.get_logger().info(f"⏳ Aguardando {wait_time}s no waypoint...")
                    self.create_timer(wait_time, lambda: self.task_completed(True))
                else:
                    self.task_completed(True)
        else:
            # Falha na navegação
            waypoint_name = self.current_navigation_task.get('waypoint', 'unknown')
            self.get_logger().error(f"❌ Falha ao alcançar '{waypoint_name}'")
            
            # Reset estados de transporte em caso de falha
            if hasattr(self, 'transport_phase'):
                self.transport_phase = None
            if hasattr(self, 'current_transport_task'):
                delattr(self, 'current_transport_task')
            if hasattr(self, 'multi_collection_phase'):
                self.multi_collection_phase = None
                self.collection_index = 0
                self.collected_objects = []
            if hasattr(self, 'current_multi_task'):
                delattr(self, 'current_multi_task')
                
            self.task_completed(False)
    
    def task_completed(self, success):
        """Callback para task completada"""
        task_id = self.bmt_tasks[self.current_task_index].get('task_id', f'task_{self.current_task_index}')
        
        if success:
            self.get_logger().info(f"✅ Task '{task_id}' completada com sucesso!")
            self.publish_task_status(f"COMPLETED_{task_id}")
            
            # Próxima task
            self.current_task_index += 1
            self.create_timer(2.0, self.execute_next_task)
        else:
            self.get_logger().error(f"❌ Task '{task_id}' falhou!")
            self.publish_task_status(f"FAILED_{task_id}")
            self.mission_active = False
    
    def publish_task_status(self, status):
        """Publica status da task BMT"""
        msg = String()
        msg.data = status
        self.task_status_pub.publish(msg)
        self.get_logger().info(f"📡 Status BMT: {status}")


def main(args=None):
    """Função principal"""
    rclpy.init(args=args)
    
    try:
        node = CarameloBMTWaypointNav()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Erro durante execução: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
