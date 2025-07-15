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
        self.localization_corrected = False  # Flag para correção de localização
        self.initial_pose_verified = False   # Flag para verificação de pose inicial
        
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
            self.get_logger().error(f"❌ Falha ao alcançar '{wp_name}'")
            self.mission_active = False
            
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
            # Verifica se a pose está próxima da origem (START)
            current_x = msg.pose.pose.position.x
            current_y = msg.pose.pose.position.y
            
            # Tolerância para considerar que está na origem
            tolerance = 0.3  # 30cm
            
            if abs(current_x) < tolerance and abs(current_y) < tolerance:
                self.get_logger().info(f"✅ Pose inicial verificada: ({current_x:.3f}, {current_y:.3f})")
                self.initial_pose_verified = True
                
                # Se já fez correção de localização, pode iniciar missão
                if self.localization_corrected:
                    self.start_mission_after_localization()
            else:
                self.get_logger().warn(f"⚠️ Pose atual ({current_x:.3f}, {current_y:.3f}) não está na origem!")
                # Automaticamente corrige a pose para (0,0,0)
                self.correct_initial_pose()
    
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
