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


class CarameloWaypointNav(Node):
    def __init__(self):
        super().__init__('caramelo_waypoint_nav')
        
        # Parâmetros
        self.declare_parameter('mission_file', '/home/work/Caramelo_workspace/maps/arena_fei/mission.yaml')
        self.declare_parameter('waypoints_file', '/home/work/Caramelo_workspace/maps/arena_fei/waypoints.json')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('publish_initial_pose', True)
        
        self.mission_file = self.get_parameter('mission_file').get_parameter_value().string_value
        self.waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        self.auto_start = self.get_parameter('auto_start').get_parameter_value().bool_value
        self.publish_pose = self.get_parameter('publish_initial_pose').get_parameter_value().bool_value
        
        # Publishers
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
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
        
        self.get_logger().info("🤖 CARAMELO WAYPOINT NAVIGATION iniciado!")
        
        # Carrega waypoints e missão
        self.load_waypoints_database()  # Opcional
        self.load_mission()             # Obrigatório
        
        # Timer para inicialização
        self.init_timer = self.create_timer(2.0, self.initialize_navigation)
    
    def load_waypoints_database(self):
        """Carrega waypoints do arquivo waypoints.json (opcional)"""
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
                        
                self.get_logger().info(f"📍 Database: {len(self.waypoints_db)} waypoints disponíveis")
                        
            else:
                self.get_logger().info(f"ℹ️  Arquivo waypoints.json não encontrado - usando coordenadas diretas")
                
        except Exception as e:
            self.get_logger().warn(f"⚠️  Erro ao carregar waypoints.json: {e}")
    
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
        
        for wp_name in waypoint_sequence:
            # ⚠️ PULAR WAYPOINT "START" - é apenas pose inicial
            if wp_name.upper() == 'START':
                self.get_logger().info(f"⏭️  Pulando waypoint '{wp_name}' (pose inicial)")
                continue
                
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
                    'name': wp_name
                }
                self.waypoints.append(waypoint)
            else:
                self.get_logger().error(f"❌ Waypoint '{wp_name}' não encontrado no database!")
                return
        
        self.get_logger().info(f"🎯 Missão por NOMES: {len(self.waypoints)} waypoints")
        self.get_logger().info(f"   Sequência: {' -> '.join([wp['name'] for wp in self.waypoints])}")
    
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
        """Inicializa a navegação - publica pose inicial"""
        self.init_timer.cancel()
        
        # Aguarda Nav2 estar pronto
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("❌ Nav2 não está disponível!")
            return
            
        self.get_logger().info("✅ Nav2 conectado!")
        
        # Publica pose inicial (se habilitado)
        if self.publish_pose:
            self.publish_initial_pose_func()
        
        # Aguarda um pouco e inicia missão
        if self.auto_start and self.waypoints:
            self.create_timer(3.0, self.start_mission_callback)
            
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
            
        wp_name = self.waypoints[self.current_waypoint].get('name', f'WP{self.current_waypoint + 1}')
        self.get_logger().info(f"✅ Goal aceito para '{wp_name}'")
        
        # Aguarda resultado
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result)
        
    def navigation_result(self, future):
        """Callback de resultado da navegação"""
        result = future.result().result
        
        wp_name = self.waypoints[self.current_waypoint].get('name', f'WP{self.current_waypoint + 1}')
        
        if result:
            self.get_logger().info(f"🎉 '{wp_name}' alcançado com sucesso!")
            
            # Próximo waypoint
            self.current_waypoint += 1
            
            # Pequena pausa entre waypoints
            self.create_timer(2.0, self.navigate_to_next_waypoint_delayed)
            
        else:
            self.get_logger().error(f"❌ Falha ao alcançar '{wp_name}'")
            self.mission_active = False
            
    def navigate_to_next_waypoint_delayed(self):
        """Navega para próximo waypoint com delay"""
        if self.mission_active:
            self.navigate_to_next_waypoint()
    
    def check_amcl_ready(self):
        """Verifica se o AMCL está pronto checando o transform map->base_footprint"""
        try:
            # Tentar obter transform de map para base_footprint
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
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
    
def main(args=None):
    rclpy.init(args=args)
    
    navigator = CarameloWaypointNav()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info("🛑 Navegação interrompida pelo usuário")
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
