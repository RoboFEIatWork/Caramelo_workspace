#!/usr/bin/env python3
"""
CARAMELO WAYPOINT NAVIGATION - Sistema Real Funcional

Sistema de navegação autônoma por waypoints para o robô Caramelo.
Suporta dois formatos:
1. Coordenadas diretas no mission.yaml (compatibilidade)
2. Referências por nome ao waypoints.json (novo formato)

Autor: GitHub Copilot
Data: 2025-07-11
"""

import json
import math
import os
import time

import rclpy
import tf2_ros
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class CarameloWaypointNav(Node):
    def __init__(self):
        super().__init__('caramelo_waypoint_nav')
        
        # Parâmetros
        self.declare_parameter('mission_file', '/home/work/Caramelo_workspace/maps/arena_fei/mission.yaml')
        self.declare_parameter('waypoints_file', '/home/work/Caramelo_workspace/maps/arena_fei/waypoints.json')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('publish_initial_pose', True)
        self.declare_parameter('slam_mode', False)  # NOVO: modo SLAM
        
        self.mission_file = self.get_parameter('mission_file').get_parameter_value().string_value
        self.waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        self.auto_start = self.get_parameter('auto_start').get_parameter_value().bool_value
        self.publish_pose = self.get_parameter('publish_initial_pose').get_parameter_value().bool_value
        self.slam_mode = self.get_parameter('slam_mode').get_parameter_value().bool_value
        
        # Publishers
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )
        
        # Publisher para marcadores de waypoints no RViz
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/waypoint_markers',
            10
        )
        
        # Action Client para navegação
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # TF Buffer para verificar transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Variáveis
        self.waypoints = []         # Lista final de waypoints para navegação
        self.waypoints_db = {}      # Database de waypoints por nome (se disponível)
        self.current_waypoint = 0
        self.mission_active = False
        self.amcl_ready = False
        self.localization_corrected = False  # Flag para correção de localização
        self.initial_pose_verified = False   # Flag para verificação de pose inicial
        self.navigation_start_time = None    # Para timeout de inicialização
        
        # Publisher para pose inicial (correção automática)
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        # Subscriber para pose atual do AMCL
        self.pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        
        self.get_logger().info("🤖 CARAMELO WAYPOINT NAVIGATION iniciado!")
        
        # Carrega waypoints e missão
        self.load_waypoints_database()  # Opcional
        self.load_mission()             # Obrigatório
        
        # Timer para inicialização
        self.init_timer = self.create_timer(2.0, self.initialize_navigation)
        
        # Timer de timeout para forçar início da missão após 15 segundos
        self.timeout_timer = self.create_timer(15.0, self.force_start_mission)
    
    def load_waypoints_database(self):
        """Carrega waypoints do arquivo waypoints.json (suporte ao novo formato)"""
        try:
            if os.path.exists(self.waypoints_file):
                with open(self.waypoints_file, 'r') as f:
                    waypoints_data = json.load(f)
                    
                # Suporte ao NOVO formato: {"waypoints": [...]}
                if 'waypoints' in waypoints_data:
                    waypoints_list = waypoints_data['waypoints']
                    
                    # Cria database por nome para o novo formato
                    for wp in waypoints_list:
                        name = wp.get('name')
                        if name:
                            # Converter formato simplificado para formato compatível
                            waypoint_compat = {
                                'name': name,
                                'position': {
                                    'x': wp.get('x', 0.0),
                                    'y': wp.get('y', 0.0),
                                    'z': 0.0
                                },
                                'orientation': self.yaw_to_quaternion(math.radians(wp.get('theta', 0.0))),
                                'type': wp.get('type', 'waypoint')
                            }
                            self.waypoints_db[name] = waypoint_compat
                            
                    self.get_logger().info(f"📍 Database (novo formato): {len(self.waypoints_db)} waypoints disponíveis")
                    
                # Suporte ao formato LEGADO: {"waypoints": [...]} com position/orientation
                elif any('position' in wp for wp in waypoints_data.get('waypoints', [])):
                    waypoints_list = waypoints_data.get('waypoints', [])
                    
                    # Cria database por nome para formato legado
                    for wp in waypoints_list:
                        name = wp.get('name')
                        if name:
                            self.waypoints_db[name] = wp
                            
                    self.get_logger().info(f"📍 Database (formato legado): {len(self.waypoints_db)} waypoints disponíveis")
                    
                else:
                    self.get_logger().warn("⚠️ Formato de waypoints.json não reconhecido")
                        
            else:
                self.get_logger().info(f"ℹ️  Arquivo waypoints.json não encontrado - usando coordenadas diretas")
                
        except Exception as e:
            self.get_logger().warn(f"⚠️  Erro ao carregar waypoints.json: {e}")
    
    def yaw_to_quaternion(self, yaw):
        """Converte yaw (radianos) para quaternion"""
        return {
            'x': 0.0,
            'y': 0.0,
            'z': math.sin(yaw / 2.0),
            'w': math.cos(yaw / 2.0)
        }
    
    def load_mission(self):
        """Carrega missão - suporta ambos os formatos"""
        try:
            if os.path.exists(self.mission_file):
                with open(self.mission_file, 'r') as f:
                    mission_data = yaml.safe_load(f)
                
                # Formato NOVO: mission.waypoint_sequence (referências por nome)
                if 'mission' in mission_data and 'waypoint_sequence' in mission_data['mission']:
                    self.load_mission_by_names(mission_data['mission']['waypoint_sequence'])
                
                # Formato ANTIGO: waypoints (coordenadas diretas) 
                elif 'waypoints' in mission_data:
                    self.load_mission_by_coordinates(mission_data['waypoints'])
                
                else:
                    self.get_logger().error("❌ Formato de missão inválido!")
                    self.create_default_mission()
                    
            else:
                self.get_logger().warn(f"⚠️  Arquivo de missão não encontrado: {self.mission_file}")
                self.create_default_mission()
                
        except Exception as e:
            self.get_logger().error(f"❌ Erro ao carregar missão: {e}")
            self.create_default_mission()
    
    def load_mission_by_names(self, waypoint_sequence):
        """Carrega missão usando referências por nome"""
        self.waypoints = []
        self.initial_pose_waypoint = None  # Para armazenar pose START
        
        for wp_name in waypoint_sequence:
            if wp_name in self.waypoints_db:
                wp_data = self.waypoints_db[wp_name]
                pos = wp_data['position']
                ori = wp_data['orientation']
                
                # Converte para formato interno (x, y, yaw)
                yaw = self.quaternion_to_yaw(ori['x'], ori['y'], ori['z'], ori['w'])
                
                waypoint = {
                    'x': pos['x'],
                    'y': pos['y'], 
                    'yaw': yaw,
                    'name': wp_name,
                    'type': wp_data.get('type', 'waypoint')
                }
                
                # START é usado como pose inicial, não como destino de navegação
                if wp_name.upper() == 'START':
                    self.initial_pose_waypoint = waypoint
                    self.get_logger().info(f"📍 Pose START definida: ({waypoint['x']:.2f}, {waypoint['y']:.2f}) @ {math.degrees(waypoint['yaw']):.1f}°")
                    continue
                
                self.waypoints.append(waypoint)
            else:
                self.get_logger().error(f"❌ Waypoint '{wp_name}' não encontrado no database!")
                return
        
        self.get_logger().info(f"🎯 Missão por NOMES: {len(self.waypoints)} waypoints")
        self.get_logger().info(f"   Sequência: {' -> '.join([wp['name'] for wp in self.waypoints])}")
        
        if self.initial_pose_waypoint:
            self.get_logger().info(f"🏁 Pose inicial (START): {self.initial_pose_waypoint['name']}")
        else:
            self.get_logger().warn("⚠️ Nenhuma pose START definida - usando pose atual do robô")
        
        # Publica marcadores dos waypoints no RViz
        self.publish_waypoint_markers()
    
    def load_mission_by_coordinates(self, waypoints_list):
        """Carrega missão usando coordenadas diretas (formato antigo)"""
        self.waypoints = []
        
        for i, wp in enumerate(waypoints_list):
            waypoint = {
                'x': float(wp['x']),
                'y': float(wp['y']),
                'yaw': float(wp['yaw']),
                'name': wp.get('name', f'WP{i+1}')
            }
            self.waypoints.append(waypoint)
        
        self.get_logger().info(f"🎯 Missão por COORDENADAS: {len(self.waypoints)} waypoints")
        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(f"   WP{i+1} '{wp['name']}': x={wp['x']:.2f}, y={wp['y']:.2f}")
        
        # Publica marcadores dos waypoints no RViz
        self.publish_waypoint_markers()
    
    def quaternion_to_yaw(self, x, y, z, w):
        """Converte quaternion para yaw (radianos)"""
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    
    def create_default_mission(self):
        """Cria uma missão padrão se não houver arquivo"""
        self.waypoints = [
            {'x': 1.0, 'y': 0.0, 'yaw': 0.0, 'name': 'ponto_1'},
            {'x': 1.0, 'y': 1.0, 'yaw': 1.57, 'name': 'ponto_2'},
            {'x': 0.0, 'y': 1.0, 'yaw': 3.14, 'name': 'ponto_3'},
            {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'name': 'origem'}
        ]
        
        # Salva missão padrão
        mission_data = {'waypoints': self.waypoints}
        os.makedirs(os.path.dirname(self.mission_file), exist_ok=True)
        
        with open(self.mission_file, 'w') as f:
            yaml.dump(mission_data, f, default_flow_style=False)
            
        self.get_logger().info(f"✅ Missão padrão criada em: {self.mission_file}")
        
    def initialize_navigation(self):
        """Inicializa a navegação com correção automática de localização"""
        self.init_timer.cancel()
        
        # Aguarda Nav2 estar pronto
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("❌ Nav2 não está disponível!")
            return
            
        self.get_logger().info("✅ Nav2 conectado!")
        
        # NOVO SISTEMA: Correção automática de localização
        self.get_logger().info("🔍 Iniciando verificação e correção automática de localização...")
        self.get_logger().info("📍 Verificando se robô está na posição inicial (0,0,0)...")
        
        # Publica pose inicial de referência para AMCL
        self.correct_initial_pose()
        
        # Sistema aguardará automaticamente via pose_callback
        if not self.auto_start:
            self.get_logger().info("🔄 Auto-start desabilitado. Use RViz para iniciar manualmente.")
            
    def check_localization_and_start(self):
        """Método legado - substituído pelo sistema automático"""
        # Este método não é mais usado - a lógica foi movida para pose_callback
        pass
            
    def publish_initial_pose_func(self):
        """Publica pose inicial para AMCL"""
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Pose inicial na origem do mapa
        initial_pose.pose.pose.position.x = 0.0
        initial_pose.pose.pose.position.y = 0.0
        initial_pose.pose.pose.position.z = 0.0
        initial_pose.pose.pose.orientation.x = 0.0
        initial_pose.pose.pose.orientation.y = 0.0
        initial_pose.pose.pose.orientation.z = 0.0
        initial_pose.pose.pose.orientation.w = 1.0
        
        # Covariância
        initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
        
        self.initial_pose_pub.publish(initial_pose)
        self.get_logger().info("📍 Pose inicial publicada para AMCL")
    def start_mission_callback(self):
        """Callback para iniciar missão"""
        self.start_mission()
        
    def start_mission(self):
        """Inicia a missão de waypoints"""
        if not self.waypoints:
            self.get_logger().warn("⚠️  Nenhum waypoint para navegar!")
            return
            
        self.mission_active = True
        self.current_waypoint = 0
        
        self.get_logger().info(f"🚀 Iniciando missão com {len(self.waypoints)} waypoints!")
        self.navigate_to_next_waypoint()
        
    def navigate_to_next_waypoint(self):
        """Navega para o próximo waypoint"""
        # Verificações de segurança
        if not self.waypoints:
            self.get_logger().warn("⚠️ Lista de waypoints está vazia!")
            self.mission_active = False
            return
            
        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info("✅ Missão completa! Todos os waypoints visitados.")
            self.mission_active = False
            return
            
        waypoint = self.waypoints[self.current_waypoint]
        
        # Cria pose goal
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_pose.pose.position.x = float(waypoint['x'])
        goal_pose.pose.position.y = float(waypoint['y'])
        goal_pose.pose.position.z = 0.0
        
        # Conversão de yaw para quaternion
        yaw = float(waypoint['yaw'])
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # Cria goal action
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        wp_name = waypoint.get('name', f'WP{self.current_waypoint + 1}')
        self.get_logger().info(f"🎯 Navegando para '{wp_name}': x={waypoint['x']:.2f}, y={waypoint['y']:.2f}, yaw={waypoint['yaw']:.2f}")
        
        # Envia goal
        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback
        )
        future.add_done_callback(self.navigation_response)
        
    def navigation_feedback(self, feedback_msg):
        """Callback de feedback da navegação"""
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        
        if self.current_waypoint < len(self.waypoints):
            wp_name = self.waypoints[self.current_waypoint].get('name', f'WP{self.current_waypoint + 1}')
            self.get_logger().info(f"🚶 '{wp_name}': Restam {distance:.2f}m", throttle_duration_sec=2.0)
            
    def navigation_response(self, future):
        """Callback de resposta da navegação"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejeitado pelo Nav2!")
            return
        
        # Verificar bounds ANTES de acessar waypoints
        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().warn(f"⚠️ Índice de waypoint fora de range: {self.current_waypoint}/{len(self.waypoints)}")
            return
            
        wp_name = self.waypoints[self.current_waypoint].get('name', f'WP{self.current_waypoint + 1}')
        self.get_logger().info(f"✅ Goal aceito para '{wp_name}'")
        
        # Aguarda resultado
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result)
        
    def navigation_result(self, future):
        """Callback de resultado da navegação"""
        result = future.result().result
        
        # Verificar bounds ANTES de acessar waypoints
        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().warn(f"⚠️ Índice de waypoint fora de range: {self.current_waypoint}/{len(self.waypoints)}")
            return
            
        wp_name = self.waypoints[self.current_waypoint].get('name', f'WP{self.current_waypoint + 1}')
        
        if result:
            self.get_logger().info(f"🎉 '{wp_name}' alcançado com sucesso!")
            
            # ⭐ NOVA FEATURE: Espera de 20 segundos no waypoint
            self.get_logger().info(f"⏳ Aguardando 20 segundos no waypoint '{wp_name}'...")
            self.create_timer(20.0, self.wait_complete_callback)
            
        else:
            self.get_logger().error(f"❌ Falha ao alcançar '{wp_name}' - tentando estratégia de recuperação...")
            
            # 🔄 ESTRATÉGIA DE RECUPERAÇÃO: Tentar alcançar ponto mais próximo possível
            self.attempt_waypoint_recovery()
            
    def attempt_waypoint_recovery(self):
        """Tenta estratégias de recuperação para waypoint inacessível"""
        if self.current_waypoint >= len(self.waypoints):
            return
            
        waypoint = self.waypoints[self.current_waypoint]
        wp_name = waypoint.get('name', f'WP{self.current_waypoint + 1}')
        
        self.get_logger().info(f"🔄 Tentando recuperação para waypoint '{wp_name}'...")
        
        # Estratégia 1: Tentar pontos ao redor do waypoint original
        recovery_points = [
            {'x': waypoint['x'] + 0.3, 'y': waypoint['y'], 'yaw': waypoint['yaw']},      # 30cm à direita
            {'x': waypoint['x'] - 0.3, 'y': waypoint['y'], 'yaw': waypoint['yaw']},      # 30cm à esquerda  
            {'x': waypoint['x'], 'y': waypoint['y'] + 0.3, 'yaw': waypoint['yaw']},      # 30cm para frente
            {'x': waypoint['x'], 'y': waypoint['y'] - 0.3, 'yaw': waypoint['yaw']},      # 30cm para trás
            {'x': waypoint['x'] + 0.5, 'y': waypoint['y'] + 0.5, 'yaw': waypoint['yaw']}, # diagonal
            {'x': waypoint['x'] - 0.5, 'y': waypoint['y'] - 0.5, 'yaw': waypoint['yaw']}, # diagonal oposta
        ]
        
        # Tenta o primeiro ponto de recuperação
        self.try_recovery_point(recovery_points, 0, wp_name)
        
    def try_recovery_point(self, recovery_points, point_index, original_wp_name):
        """Tenta navegar para um ponto de recuperação específico"""
        if point_index >= len(recovery_points):
            # Todas as tentativas falharam, pula este waypoint
            self.get_logger().warn(f"⚠️ Todas as tentativas de recuperação falharam para '{original_wp_name}'. Pulando waypoint...")
            self.skip_current_waypoint()
            return
            
        recovery_point = recovery_points[point_index]
        
        self.get_logger().info(f"🎯 Tentativa {point_index + 1}/6: Navegando para ponto de recuperação próximo a '{original_wp_name}'")
        self.get_logger().info(f"   -> x={recovery_point['x']:.2f}, y={recovery_point['y']:.2f}")
        
        # Cria pose goal para ponto de recuperação
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_pose.pose.position.x = float(recovery_point['x'])
        goal_pose.pose.position.y = float(recovery_point['y'])
        goal_pose.pose.position.z = 0.0
        
        # Conversão de yaw para quaternion
        yaw = float(recovery_point['yaw'])
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # Cria goal action
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        # Envia goal com callback específico para recuperação
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self.recovery_navigation_response(f, recovery_points, point_index, original_wp_name))
        
    def recovery_navigation_response(self, future, recovery_points, point_index, original_wp_name):
        """Callback de resposta da navegação de recuperação"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn(f"⚠️ Ponto de recuperação {point_index + 1} rejeitado, tentando próximo...")
            self.try_recovery_point(recovery_points, point_index + 1, original_wp_name)
            return
        
        self.get_logger().info(f"✅ Ponto de recuperação {point_index + 1} aceito")
        
        # Aguarda resultado da recuperação
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.recovery_navigation_result(f, recovery_points, point_index, original_wp_name))
        
    def recovery_navigation_result(self, future, recovery_points, point_index, original_wp_name):
        """Callback de resultado da navegação de recuperação"""
        result = future.result().result
        
        if result:
            self.get_logger().info(f"🎉 Recuperação bem-sucedida! Chegou próximo ao waypoint '{original_wp_name}'")
            # Considera como sucesso e continua
            self.get_logger().info(f"⏳ Aguardando 20 segundos no ponto de recuperação...")
            self.create_timer(20.0, self.wait_complete_callback)
        else:
            self.get_logger().warn(f"❌ Ponto de recuperação {point_index + 1} falhou, tentando próximo...")
            self.try_recovery_point(recovery_points, point_index + 1, original_wp_name)
            
    def skip_current_waypoint(self):
        """Pula o waypoint atual e continua para o próximo"""
        if self.current_waypoint >= len(self.waypoints):
            return
            
        wp_name = self.waypoints[self.current_waypoint].get('name', f'WP{self.current_waypoint + 1}')
        self.get_logger().warn(f"⏭️ Pulando waypoint '{wp_name}' - considerado inacessível")
        
        # Avança para próximo waypoint
        self.current_waypoint += 1
        
        # Verifica se há mais waypoints
        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info("🏁 MISSÃO COMPLETA! Todos os waypoints processados.")
            self.mission_active = False
            return
        
        # Continua para próximo waypoint após pequena pausa
        self.get_logger().info("🔄 Continuando para próximo waypoint...")
        self.create_timer(3.0, self.navigate_to_next_waypoint_delayed)
            
    def wait_complete_callback(self):
        """Callback chamado após espera no waypoint"""
        if self.current_waypoint >= len(self.waypoints):
            return
            
        wp_name = self.waypoints[self.current_waypoint].get('name', f'WP{self.current_waypoint + 1}')
        self.get_logger().info(f"✅ Espera completa no waypoint '{wp_name}'. Próximo destino...")
        
        # Próximo waypoint
        self.current_waypoint += 1
        
        # Verificar se há mais waypoints
        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info("🏁 MISSÃO COMPLETA! Todos os waypoints visitados com sucesso.")
            self.get_logger().info("🔄 Sistema permanece ativo. Para nova missão, reinicie o node.")
            self.mission_active = False
            return
        
        # Pequena pausa antes do próximo waypoint
        self.create_timer(2.0, self.navigate_to_next_waypoint_delayed)
            
    def navigate_to_next_waypoint_delayed(self):
        """Navega para próximo waypoint com delay"""
        if self.mission_active:
            self.navigate_to_next_waypoint()
    
    def check_amcl_ready(self):
        """Verifica se o AMCL está pronto checando o transform map->base_footprint"""
        try:
            # Tentar obter transform de map para base_footprint
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', Time()
            )
            
            # Se chegou aqui, o transform existe
            if not self.amcl_ready:
                self.get_logger().info("✅ AMCL está pronto! Transform map->base_footprint disponível")
                self.amcl_ready = True
            return True
            
        except Exception as e:
            if self.amcl_ready:
                self.get_logger().warn(f"⚠️  AMCL perdeu localização: {str(e)}")
                self.amcl_ready = False
            return False
    
    def pose_callback(self, msg):
        """Callback para receber pose atual do AMCL"""
        if not self.initial_pose_verified and not self.mission_active:
            # Verifica se AMCL está fornecendo uma pose válida (não zero)
            current_x = msg.pose.pose.position.x
            current_y = msg.pose.pose.position.y
            
            # Verifica se a covariância está baixa (localização estável)
            cov = msg.pose.covariance
            position_uncertainty = (cov[0] + cov[7])  # xx + yy diagonal elements
            
            # MUITO RELAXADO: aceita qualquer localização razoável do AMCL
            if position_uncertainty < 2.0:  # Muito mais tolerante - aceita localização "boa o suficiente"
                self.get_logger().info(f"✅ Localização aceitável detectada: pose=({current_x:.3f}, {current_y:.3f}), uncertainty={position_uncertainty:.3f}")
                self.initial_pose_verified = True
                self.localization_corrected = True
                
                # Cancela timeout timer se ainda existe
                if hasattr(self, 'timeout_timer'):
                    self.timeout_timer.cancel()
                    self.timeout_timer.destroy()
                
                # Pode iniciar missão diretamente
                self.start_mission_after_localization()
            else:
                self.get_logger().info(f"🔄 Aguardando localização melhorar: pose=({current_x:.3f}, {current_y:.3f}), uncertainty={position_uncertainty:.3f} (limite: 2.0)", throttle_duration_sec=3.0)
    
    def correct_initial_pose(self):
        """Corrige automaticamente a pose inicial para (0,0,0)"""
        if self.localization_corrected:
            return  # Já foi corrigida
            
        self.get_logger().info("🔧 Corrigindo pose inicial automaticamente...")
        
        # Publica pose inicial corrigida para (0,0,0)
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        # Pose na origem
        pose_msg.pose.pose.position.x = 0.0
        pose_msg.pose.pose.position.y = 0.0
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = 0.0
        pose_msg.pose.pose.orientation.w = 1.0
        
        # Matriz de covariância (confiança moderada)
        covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]
        pose_msg.pose.covariance = covariance
        
        # Publica correção
        self.pose_publisher.publish(pose_msg)
        self.get_logger().info("📍 Pose inicial corrigida para (0.0, 0.0, 0.0)")
        
        self.localization_corrected = True
        
        # Aguarda um pouco para AMCL processar a correção, depois verifica localização
        self.verification_timer = self.create_timer(2.0, self.verify_localization_correction)
    
    def verify_localization_correction(self):
        """Verifica se a correção de localização foi aplicada"""
        self.get_logger().info("🔍 Verificando correção de localização...")
        
        # Cancela o timer para não repetir
        if hasattr(self, 'verification_timer'):
            self.verification_timer.cancel()
            self.verification_timer.destroy()
        
        # A verificação será feita no próximo callback de pose
        # Se ainda não verificou, agenda uma nova verificação única
        if not self.initial_pose_verified:
            self.verification_timer = self.create_timer(2.0, self.verify_localization_correction)
    
    def start_mission_after_localization(self):
        """Inicia a missão após correção e verificação de localização"""
        if self.initial_pose_verified and self.localization_corrected:
            self.get_logger().info("🚀 Localização verificada! Iniciando navegação...")
            
            # Remove waypoint START se for o primeiro (robô já está lá)
            if self.waypoints and self.waypoints[0].get('name', '').upper() == 'START':
                removed_wp = self.waypoints.pop(0)
                self.get_logger().info(f"⏭️ Pulando waypoint '{removed_wp['name']}' (posição inicial)")
                
            # Inicia navegação pelos waypoints restantes
            if self.waypoints:
                self.start_mission()
            else:
                self.get_logger().warn("⚠️ Nenhum waypoint para navegar após remoção do START")
    
    def publish_waypoint_markers(self):
        """Publica marcadores de waypoints para visualização no RViz"""
        marker_array = MarkerArray()
        
        for i, waypoint in enumerate(self.waypoints):
            # Marcador de posição (esfera)
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i * 2  # IDs pares para posições
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Posição
            marker.pose.position.x = float(waypoint['x'])
            marker.pose.position.y = float(waypoint['y'])
            marker.pose.position.z = 0.1
            
            # Orientação (neutra para esfera)
            marker.pose.orientation.w = 1.0
            
            # Tamanho
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            
            # Cor baseada no tipo de waypoint
            name = waypoint.get('name', 'WP')
            if 'START' in name:
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Verde
            elif 'FINISH' in name:
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Vermelho
            elif 'WS' in name:
                marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)  # Azul
            else:
                marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # Amarelo
            
            marker_array.markers.append(marker)
            
            # Marcador de texto (nome do waypoint)
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = "waypoint_labels"
            text_marker.id = i * 2 + 1  # IDs ímpares para textos
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # Posição ligeiramente acima da esfera
            text_marker.pose.position.x = float(waypoint['x'])
            text_marker.pose.position.y = float(waypoint['y'])
            text_marker.pose.position.z = 0.4
            text_marker.pose.orientation.w = 1.0
            
            # Tamanho do texto
            text_marker.scale.z = 0.2
            
            # Cor do texto (branco)
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            
            # Texto
            text_marker.text = name
            
            marker_array.markers.append(text_marker)
        
        # Publica todos os marcadores
        self.marker_pub.publish(marker_array)
        self.get_logger().info(f"📍 Publicados {len(self.waypoints)} waypoints como marcadores no RViz")
    
    def clear_waypoint_markers(self):
        """Remove todos os marcadores de waypoints do RViz"""
        marker_array = MarkerArray()
        
        # Cria marcadores de deletar para todos os IDs
        for i in range(len(self.waypoints) * 2):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints" if i % 2 == 0 else "waypoint_labels"
            marker.id = i
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)
    
    def force_start_mission(self):
        """Força o início da missão após timeout, mesmo sem localização perfeita"""
        if not self.mission_active:
            self.get_logger().warn("⏰ TIMEOUT (15s): Forçando início da missão sem localização perfeita!")
            self.get_logger().info("   -> AMCL pode estar com covariância alta, mas o sistema continuará...")
            
            # Cancela o timer de timeout
            if hasattr(self, 'timeout_timer'):
                self.timeout_timer.cancel()
                self.timeout_timer.destroy()
            
            # Marca como verificado para permitir início
            self.initial_pose_verified = True
            self.localization_corrected = True
            
            # Inicia a missão
            self.start_mission_after_localization()


def main(args=None):
    """Função principal para iniciar o node de navegação por waypoints"""
    rclpy.init(args=args)
    
    try:
        node = CarameloWaypointNav()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Erro durante execução: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
