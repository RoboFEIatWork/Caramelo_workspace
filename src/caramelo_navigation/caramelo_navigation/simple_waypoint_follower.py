#!/usr/bin/env python3

import json
import os
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


class SimpleWaypointFollower(Node):
    """
    Navegador simples por waypoints baseado no Nav2 oficial.
    Inicia automaticamente assim que Nav2 estiver pronto.
    """

    def __init__(self):
        super().__init__('simple_waypoint_follower')
        
        # Parâmetros
        self.declare_parameter('arena', 'arena_fei')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('loop_mission', False)
        
        self.arena = self.get_parameter('arena').get_parameter_value().string_value
        self.auto_start = self.get_parameter('auto_start').get_parameter_value().bool_value
        self.loop_mission = self.get_parameter('loop_mission').get_parameter_value().bool_value
        
        # Caminhos
        self.maps_path = '/home/work/Caramelo_workspace/maps'
        self.waypoints_file = os.path.join(self.maps_path, self.arena, 'waypoints.json')
        
        # Action client para navegação
        self._action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        
        # Publisher para pose inicial
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        
        # Publisher para status
        self.status_pub = self.create_publisher(String, '/navigation_status', 10)
        
        # Variáveis de controle
        self.goal_handle = None
        self.waypoints_loaded = False
        self.poses = []
        
        self.get_logger().info(f'🤖 Simple Waypoint Follower iniciado')
        self.get_logger().info(f'📁 Arena: {self.arena}')
        self.get_logger().info(f'🗂️ Waypoints: {self.waypoints_file}')
        
        # Timer para inicialização automática
        if self.auto_start:
            self.init_timer = self.create_timer(5.0, self.auto_initialize)
        
    def auto_initialize(self):
        """Inicialização automática."""
        try:
            self.get_logger().info('🔄 Iniciando automaticamente...')
            
            # Cancelar timer
            self.init_timer.cancel()
            
            # Carregar waypoints
            if self.load_waypoints():
                # Definir pose inicial
                self.set_initial_pose()
                
                # Aguardar um pouco e iniciar navegação
                time.sleep(5.0)  # Aguarda mais tempo para AMCL se estabelecer
                self.start_navigation()
            else:
                self.get_logger().error('❌ Falha ao carregar waypoints')
                
        except Exception as e:
            self.get_logger().error(f'❌ Erro na inicialização: {str(e)}')
    
    def load_waypoints(self):
        """Carrega waypoints do arquivo JSON."""
        if not os.path.exists(self.waypoints_file):
            self.get_logger().error(f'❌ Arquivo não encontrado: {self.waypoints_file}')
            return False
            
        try:
            with open(self.waypoints_file, 'r') as f:
                data = json.load(f)
            
            self.poses = []
            waypoints = data.get('waypoints', [])
            
            for i, waypoint in enumerate(waypoints):
                pose = PoseStamped()
                pose.header.frame_id = data.get('frame_id', 'map')
                pose.header.stamp = self.get_clock().now().to_msg()
                
                # Posição
                pose.pose.position.x = float(waypoint['position']['x'])
                pose.pose.position.y = float(waypoint['position']['y'])
                pose.pose.position.z = float(waypoint['position']['z'])
                
                # Orientação
                pose.pose.orientation.x = float(waypoint['orientation']['x'])
                pose.pose.orientation.y = float(waypoint['orientation']['y'])
                pose.pose.orientation.z = float(waypoint['orientation']['z'])
                pose.pose.orientation.w = float(waypoint['orientation']['w'])
                
                self.poses.append(pose)
                
                name = waypoint.get('name', f'WP{i+1}')
                self.get_logger().info(
                    f'📌 {name}: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})')
            
            self.get_logger().info(f'✅ Carregados {len(self.poses)} waypoints')
            self.waypoints_loaded = True
            return True
            
        except Exception as e:
            self.get_logger().error(f'❌ Erro ao carregar waypoints: {str(e)}')
            return False
    
    def set_initial_pose(self):
        """Define pose inicial no origem (0,0) para AMCL."""
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Posição inicial (0,0)
        initial_pose.pose.pose.position.x = 0.0
        initial_pose.pose.pose.position.y = 0.0
        initial_pose.pose.pose.position.z = 0.0
        
        # Orientação inicial (facing forward)
        initial_pose.pose.pose.orientation.x = 0.0
        initial_pose.pose.pose.orientation.y = 0.0
        initial_pose.pose.pose.orientation.z = 0.0
        initial_pose.pose.pose.orientation.w = 1.0
        
        # Covariância (incerteza inicial)
        initial_pose.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        
        self.get_logger().info('📍 Definindo pose inicial em (0.0, 0.0)')
        self.initial_pose_pub.publish(initial_pose)
        
        # Publicar status
        status = String()
        status.data = "INITIALIZING"
        self.status_pub.publish(status)
    
    def start_navigation(self):
        """Inicia navegação pelos waypoints."""
        if not self.waypoints_loaded or len(self.poses) == 0:
            self.get_logger().error('❌ Nenhum waypoint carregado!')
            return
            
        self.get_logger().info(f'🚀 Iniciando navegação por {len(self.poses)} waypoints...')
        
        # Aguardar action server
        self.get_logger().info('⏳ Aguardando action server...')
        if not self._action_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('❌ Action server não disponível!')
            return
            
        # Criar goal
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.poses
        
        # Publicar status
        status = String()
        status.data = "NAVIGATING"
        self.status_pub.publish(status)
        
        # Enviar goal
        self.get_logger().info('📤 Enviando waypoints para Nav2...')
        self.goal_handle = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self.goal_handle.add_done_callback(self.goal_response_callback)
    
    def feedback_callback(self, feedback_msg):
        """Callback de feedback da navegação."""
        feedback = feedback_msg.feedback
        current_wp = feedback.current_waypoint + 1
        total_wp = len(self.poses)
        
        self.get_logger().info(f'🚶 Navegando para waypoint: {current_wp}/{total_wp}')
        
        # Publicar status detalhado
        status = String()
        status.data = f"NAVIGATING_WP_{current_wp}_OF_{total_wp}"
        self.status_pub.publish(status)
    
    def goal_response_callback(self, future):
        """Callback de resposta do goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejeitado!')
            return
            
        self.get_logger().info('✅ Goal aceito! Navegação iniciada.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Callback de resultado final."""
        result = future.result().result
        
        if len(result.missed_waypoints) == 0:
            self.get_logger().info('🎉 === TODOS OS WAYPOINTS ALCANÇADOS! ===')
            status = String()
            status.data = "COMPLETED_SUCCESS"
            self.status_pub.publish(status)
        else:
            missed = len(result.missed_waypoints)
            self.get_logger().warn(f'⚠️ {missed} waypoints perdidos!')
            status = String()
            status.data = f"COMPLETED_WITH_{missed}_MISSED"
            self.status_pub.publish(status)
        
        # Loop se solicitado
        if self.loop_mission:
            self.get_logger().info('🔄 Reiniciando em loop em 5 segundos...')
            time.sleep(5.0)
            self.start_navigation()
        else:
            self.get_logger().info('✅ Missão completa!')
    
    def cancel_navigation(self):
        """Cancela a navegação atual."""
        if self.goal_handle:
            self.get_logger().info('⏹️ Cancelando navegação...')
            
            status = String()
            status.data = "CANCELLED"
            self.status_pub.publish(status)

def main(args=None):
    rclpy.init(args=args)
    
    navigator = SimpleWaypointFollower()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('🛑 Interrompido pelo usuário')
        navigator.cancel_navigation()
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
