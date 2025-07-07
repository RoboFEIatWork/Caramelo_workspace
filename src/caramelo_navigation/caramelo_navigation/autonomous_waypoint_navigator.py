#!/usr/bin/env python3
"""
Navegador Autônomo de Waypoints - Navega pelos waypoints usando Nav2
"""

import json
import math
import os
import time
from enum import Enum
from typing import List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener


class NavigationState(Enum):
    IDLE = 0
    NAVIGATING = 1
    APPROACHING = 2
    POSITIONING = 3
    COMPLETED = 4
    FAILED = 5


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Declarar parâmetros
        self.declare_parameter('waypoints_file', '/home/work/Caramelo_workspace/src/caramelo_navigation/config/waypoints.json')
        self.declare_parameter('map_file', '/home/work/Caramelo_workspace/mapa_20250704_145039.yaml')
        self.declare_parameter('use_sim_time', False)
        
        # Configurações ajustáveis
        self.approach_distance = 0.25  # Distância para considerar "próximo" ao waypoint
        self.orientation_tolerance = 0.15  # Tolerância de orientação em radianos
        self.max_approach_time = 45.0  # Tempo máximo para tentar se aproximar
        self.obstacle_distance = 0.6  # Distância mínima para obstáculos
        self.obstacle_check_angle = 60.0  # Ângulo para verificar obstáculos (graus)
        self.recovery_distance = 0.5  # Distância para recuar quando encontrar obstáculo
        
        # Estado da navegação
        self.state = NavigationState.IDLE
        self.waypoints = []
        self.current_waypoint_index = 0
        self.current_goal_handle = None
        self.navigation_start_time = 0.0
        self.last_obstacle_detection = 0.0
        
        # Controle de obstáculos dinâmicos
        self.obstacle_detected = False
        self.obstacle_persistent_time = 0.0
        self.obstacle_wait_time = 3.0  # Tempo para aguardar obstáculo sair
        self.recovery_mode = False
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Action client para Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            qos_profile
        )
        
        # Dados dos sensores
        self.laser_data = None
        self.obstacles_detected = False
        
        # Timer para controle principal
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Carregar waypoints do parâmetro
        self.load_waypoints()
        
        self.get_logger().info("🤖 Waypoint Navigator para Robô Real iniciado!")
        self.get_logger().info(f"📍 Carregados {len(self.waypoints)} waypoints")
        self.get_logger().info(f"🗺️ Mapa: {self.get_parameter('map_file').get_parameter_value().string_value}")
        self.get_logger().info(f"📋 Waypoints: {self.get_parameter('waypoints_file').get_parameter_value().string_value}")
        self.get_logger().info("⚠️ ATENÇÃO: Certifique-se que PWM e Encoder bringup estão rodando!")
        
    def load_waypoints(self):
        """Carrega waypoints do arquivo especificado no parâmetro"""
        try:
            waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
            
            if not os.path.exists(waypoints_file):
                self.get_logger().error(f"❌ Arquivo de waypoints não encontrado: {waypoints_file}")
                self.waypoints = []
                return
            
            with open(waypoints_file, 'r') as f:
                data = json.load(f)
                
            # Filtrar apenas waypoints WS (excluir START e FINISH por enquanto)
            self.waypoints = [wp for wp in data.get('waypoints', []) 
                            if wp.get('name', '').startswith('WS')]
            
            # Ordenar waypoints por nome (WS01, WS02, etc.)
            self.waypoints.sort(key=lambda x: x.get('name', ''))
            
            self.get_logger().info(f"📋 Waypoints carregados: {[wp['name'] for wp in self.waypoints]}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Erro ao carregar waypoints: {e}")
            self.waypoints = []
    
    def laser_callback(self, msg):
        """Callback do laser para detecção de obstáculos dinâmicos"""
        self.laser_data = msg
        
        # Verificar obstáculos próximos na frente do robô
        if len(msg.ranges) > 0:
            # Calcular índices para verificar apenas a frente do robô
            total_points = len(msg.ranges)
            angle_per_point = (msg.angle_max - msg.angle_min) / total_points
            
            # Verificar apenas um cone na frente (±30 graus)
            front_angle_rad = math.radians(self.obstacle_check_angle / 2)
            
            # Encontrar índices para o cone frontal
            center_index = total_points // 2
            angle_range = int(front_angle_rad / angle_per_point)
            
            start_index = max(0, center_index - angle_range)
            end_index = min(total_points, center_index + angle_range)
            
            # Verificar distâncias no cone frontal
            front_distances = []
            for i in range(start_index, end_index):
                if msg.ranges[i] > 0.1 and msg.ranges[i] < 10.0:  # Filtrar leituras válidas
                    front_distances.append(msg.ranges[i])
            
            # Detectar obstáculo se houver algo próximo na frente
            if front_distances:
                min_distance = min(front_distances)
                obstacle_detected = min_distance < self.obstacle_distance
                
                if obstacle_detected:
                    if not self.obstacle_detected:
                        self.obstacle_persistent_time = time.time()
                        self.get_logger().warn(f"🚨 Obstáculo detectado a {min_distance:.2f}m!")
                    self.obstacle_detected = True
                    self.last_obstacle_detection = time.time()
                else:
                    # Obstáculo não detectado, mas aguardar um pouco antes de limpar
                    if self.obstacle_detected and (time.time() - self.last_obstacle_detection) > 1.0:
                        self.obstacle_detected = False
                        self.obstacle_persistent_time = 0.0
                        self.get_logger().info("✅ Caminho livre!")
            else:
                self.obstacle_detected = False
    
    def get_robot_pose(self) -> Optional[PoseStamped]:
        """Obter pose atual do robô"""
        try:
            # Obter transformação de map para base_link
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', Time()
            )
            
            # Criar PoseStamped
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
    
    def distance_to_waypoint(self, waypoint) -> float:
        """Calcular distância até o waypoint"""
        robot_pose = self.get_robot_pose()
        if not robot_pose:
            return float('inf')
        
        dx = waypoint['position']['x'] - robot_pose.pose.position.x
        dy = waypoint['position']['y'] - robot_pose.pose.position.y
        return math.sqrt(dx*dx + dy*dy)
    
    def orientation_difference(self, waypoint) -> float:
        """Calcular diferença de orientação até o waypoint"""
        robot_pose = self.get_robot_pose()
        if not robot_pose:
            return float('inf')
        
        # Converter quaternions para yaw
        def quat_to_yaw(q):
            return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        robot_yaw = quat_to_yaw(robot_pose.pose.orientation)
        
        # Yaw do waypoint
        wp_q = waypoint['orientation']
        wp_yaw = math.atan2(2.0 * (wp_q['w'] * wp_q['z'] + wp_q['x'] * wp_q['y']),
                           1.0 - 2.0 * (wp_q['y'] * wp_q['y'] + wp_q['z'] * wp_q['z']))
        
        # Diferença angular normalizada
        diff = wp_yaw - robot_yaw
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        
        return abs(diff)
    
    def create_goal_pose(self, waypoint) -> PoseStamped:
        """Criar PoseStamped para o waypoint"""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Posição
        goal_pose.pose.position.x = waypoint['position']['x']
        goal_pose.pose.position.y = waypoint['position']['y']
        goal_pose.pose.position.z = waypoint['position']['z']
        
        # Orientação
        goal_pose.pose.orientation.x = waypoint['orientation']['x']
        goal_pose.pose.orientation.y = waypoint['orientation']['y']
        goal_pose.pose.orientation.z = waypoint['orientation']['z']
        goal_pose.pose.orientation.w = waypoint['orientation']['w']
        
        return goal_pose
    
    def send_nav_goal(self, waypoint):
        """Enviar goal para o Nav2"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("❌ Nav2 server não disponível!")
            return False
        
        goal_pose = self.create_goal_pose(waypoint)
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.get_logger().info(f"🎯 Enviando goal para {waypoint['name']}")
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
            self.state = NavigationState.FAILED
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
            self.get_logger().info("✅ Navegação concluída com sucesso!")
            self.state = NavigationState.APPROACHING
        else:
            self.get_logger().warn(f"⚠️ Navegação falhou: {result.status}")
            self.state = NavigationState.APPROACHING  # Tentar se aproximar mesmo assim
    
    def fine_position_control(self, waypoint):
        """Controle fino para posicionamento preciso"""
        robot_pose = self.get_robot_pose()
        if not robot_pose:
            return
        
        # Calcular erro de posição
        dx = waypoint['position']['x'] - robot_pose.pose.position.x
        dy = waypoint['position']['y'] - robot_pose.pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Calcular erro de orientação
        orientation_error = self.orientation_difference(waypoint)
        
        # Controle de velocidade
        cmd = Twist()
        
        # Velocidade linear (proporcional à distância)
        if distance > 0.05:  # 5cm
            cmd.linear.x = min(0.2, distance * 0.5)  # Velocidade baixa
        
        # Velocidade angular (proporcional ao erro de orientação)
        if orientation_error > 0.05:  # ~3 graus
            wp_q = waypoint['orientation']
            target_yaw = math.atan2(2.0 * (wp_q['w'] * wp_q['z'] + wp_q['x'] * wp_q['y']),
                                  1.0 - 2.0 * (wp_q['y'] * wp_q['y'] + wp_q['z'] * wp_q['z']))
            
            robot_yaw = math.atan2(2.0 * (robot_pose.pose.orientation.w * robot_pose.pose.orientation.z + 
                                        robot_pose.pose.orientation.x * robot_pose.pose.orientation.y),
                                 1.0 - 2.0 * (robot_pose.pose.orientation.y * robot_pose.pose.orientation.y + 
                                            robot_pose.pose.orientation.z * robot_pose.pose.orientation.z))
            
            yaw_error = target_yaw - robot_yaw
            while yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            while yaw_error < -math.pi:
                yaw_error += 2 * math.pi
            
            cmd.angular.z = min(0.3, abs(yaw_error) * 0.8) * (1 if yaw_error > 0 else -1)
        
        # Verificar obstáculos dinâmicos
        if self.obstacle_detected:
            # Se obstáculo persistir por muito tempo, tentar contornar
            if (time.time() - self.obstacle_persistent_time) > self.obstacle_wait_time:
                if not self.recovery_mode:
                    self.get_logger().warn("🔄 Obstáculo persistente - tentando contornar")
                    self.recovery_mode = True
                    # Cancelar navegação atual e tentar novamente
                    if self.current_goal_handle:
                        self.current_goal_handle.cancel_goal_async()
                        self.current_goal_handle = None
                    self.state = NavigationState.IDLE
                    return False
            
            # Parar movimento se obstáculo muito próximo
            if cmd.linear.x > 0:
                cmd.linear.x = 0.0
                self.get_logger().warn("⏹️ Parado - aguardando obstáculo sair")
        else:
            # Obstáculo não detectado, pode continuar
            if self.recovery_mode:
                self.get_logger().info("✅ Caminho livre - saindo do modo recovery")
                self.recovery_mode = False
        
        # Publicar comando
        self.cmd_vel_pub.publish(cmd)
        
        # Verificar se chegou na posição
        if distance < 0.1 and orientation_error < self.orientation_tolerance:
            cmd = Twist()  # Parar
            self.cmd_vel_pub.publish(cmd)
            return True
        
        return False
    
    def control_loop(self):
        """Loop principal de controle"""
        if not self.waypoints:
            return
        
        current_waypoint = self.waypoints[self.current_waypoint_index]
        
        if self.state == NavigationState.IDLE:
            # Iniciar navegação
            if self.send_nav_goal(current_waypoint):
                self.state = NavigationState.NAVIGATING
                self.navigation_start_time = time.time()
        
        elif self.state == NavigationState.NAVIGATING:
            # Aguardando navegação...
            if time.time() - self.navigation_start_time > self.max_approach_time:
                self.get_logger().warn("⏰ Timeout na navegação - tentando aproximação manual")
                self.state = NavigationState.APPROACHING
        
        elif self.state == NavigationState.APPROACHING:
            # Tentar se aproximar do waypoint
            distance = self.distance_to_waypoint(current_waypoint)
            
            if distance > self.approach_distance:
                # Tentar navegar novamente ou usar controle manual
                if time.time() - self.navigation_start_time < self.max_approach_time:
                    self.send_nav_goal(current_waypoint)
                    self.state = NavigationState.NAVIGATING
                else:
                    self.get_logger().warn(f"⚠️ Não foi possível se aproximar de {current_waypoint['name']}")
                    self.state = NavigationState.POSITIONING
            else:
                self.state = NavigationState.POSITIONING
        
        elif self.state == NavigationState.POSITIONING:
            # Posicionamento fino
            if self.fine_position_control(current_waypoint):
                self.get_logger().info(f"✅ Chegou em {current_waypoint['name']}!")
                self.state = NavigationState.COMPLETED
            
            # Timeout para posicionamento
            if time.time() - self.navigation_start_time > self.max_approach_time + 10:
                self.get_logger().warn(f"⏰ Timeout no posicionamento de {current_waypoint['name']}")
                self.state = NavigationState.COMPLETED
        
        elif self.state == NavigationState.COMPLETED or self.state == NavigationState.FAILED:
            # Ir para próximo waypoint
            self.current_waypoint_index += 1
            
            if self.current_waypoint_index >= len(self.waypoints):
                self.get_logger().info("🏁 Todos os waypoints foram visitados!")
                self.current_waypoint_index = 0  # Reiniciar
                
                # Parar o robô
                cmd = Twist()
                self.cmd_vel_pub.publish(cmd)
                
                # Aguardar um pouco antes de reiniciar
                time.sleep(5)
            
            self.state = NavigationState.IDLE
    
    def start_navigation(self):
        """Iniciar navegação pelos waypoints"""
        if not self.waypoints:
            self.get_logger().error("❌ Nenhum waypoint carregado!")
            return
        
        self.current_waypoint_index = 0
        self.state = NavigationState.IDLE
        self.get_logger().info("🚀 Iniciando navegação pelos waypoints!")


def main(args=None):
    rclpy.init(args=args)
    
    navigator = WaypointNavigator()
    
    try:
        # Aguardar um pouco para TF se estabilizar
        time.sleep(2)
        
        # Iniciar navegação
        navigator.start_navigation()
        
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
