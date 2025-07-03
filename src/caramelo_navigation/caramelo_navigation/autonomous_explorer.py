#!/usr/bin/env python3

import math
import random

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class AutonomousExplorer(Node):
    def __init__(self):
        super().__init__('autonomous_explorer')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)  # Usar filtro de segurança
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.status_pub = self.create_publisher(String, '/exploration_status', 10)
        
        # Subscribers
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.nav_status_sub = self.create_subscription(String, '/navigation_status', self.nav_status_callback, 10)
        
        # Estado atual
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # Dados do mapa
        self.map_data = None
        self.map_info = None
        
        # Dados do lidar
        self.current_scan = None
        
        # Estado da exploração
        self.exploration_state = "STARTING"  # STARTING, EXPLORING, MOVING_TO_FRONTIER, ROTATING, FINISHED
        self.current_goal = None
        self.last_goal_time = self.get_clock().now()
        self.stuck_counter = 0
        self.visited_areas = set()
        
        # Parâmetros de exploração
        self.frontier_distance_threshold = 2.0  # Distância mínima para fronteiras
        self.max_exploration_distance = 10.0    # Distância máxima de exploração
        self.rotation_speed = 0.3               # Velocidade de rotação para varredura
        self.exploration_timeout = 30.0         # Timeout para mudar estratégia
        self.min_frontier_size = 10             # Tamanho mínimo de fronteira
        
        # Timer para exploração
        self.exploration_timer = self.create_timer(2.0, self.exploration_loop)
        
        self.get_logger().info('🔍 Explorador Autônomo iniciado!')
        self.get_logger().info('Aguardando dados do mapa para começar exploração...')
        
    def odom_callback(self, msg):
        """Atualiza posição atual"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Converter quaternion para yaw
        quat = msg.pose.pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)
        
        # Marcar área atual como visitada
        grid_x, grid_y = self.world_to_grid(self.current_x, self.current_y)
        if grid_x is not None and grid_y is not None:
            self.visited_areas.add((grid_x, grid_y))
        
    def map_callback(self, msg):
        """Atualiza dados do mapa"""
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info
        
    def scan_callback(self, msg):
        """Atualiza dados do lidar"""
        self.current_scan = msg
        
    def nav_status_callback(self, msg):
        """Recebe status do navegador"""
        if "GOAL_REACHED" in msg.data or "POSITION_REACHED" in msg.data:
            self.get_logger().info('✅ Objetivo de exploração alcançado!')
            self.exploration_state = "EXPLORING"
            self.current_goal = None
        elif "NAVIGATING" in msg.data:
            self.exploration_state = "MOVING_TO_FRONTIER"
            
    def exploration_loop(self):
        """Loop principal de exploração"""
        if self.map_data is None:
            return
            
        current_time = self.get_clock().now()
        
        # Publicar status
        status_msg = String()
        status_msg.data = f"STATE_{self.exploration_state}_X_{self.current_x:.1f}_Y_{self.current_y:.1f}"
        self.status_pub.publish(status_msg)
        
        if self.exploration_state == "STARTING":
            self.start_exploration()
            
        elif self.exploration_state == "EXPLORING":
            self.find_and_go_to_frontier()
            
        elif self.exploration_state == "MOVING_TO_FRONTIER":
            # Verificar se ficou preso
            if (current_time - self.last_goal_time).nanoseconds / 1e9 > self.exploration_timeout:
                self.get_logger().warn('⚠️ Timeout na navegação, mudando estratégia...')
                self.exploration_state = "ROTATING"
                self.stuck_counter += 1
                
        elif self.exploration_state == "ROTATING":
            self.rotate_and_scan()
            
        elif self.exploration_state == "FINISHED":
            self.get_logger().info('🏁 Exploração finalizada!')
            
    def start_exploration(self):
        """Inicia processo de exploração"""
        self.get_logger().info('🚀 Iniciando exploração autônoma...')
        
        # Fazer uma rotação inicial para varredura
        self.exploration_state = "ROTATING"
        
    def find_and_go_to_frontier(self):
        """Encontra fronteiras e navega para elas"""
        frontiers = self.find_frontiers()
        
        if not frontiers:
            self.get_logger().info('🔍 Nenhuma fronteira encontrada, fazendo varredura...')
            self.exploration_state = "ROTATING"
            return
            
        # Escolher melhor fronteira
        best_frontier = self.select_best_frontier(frontiers)
        
        if best_frontier is None:
            self.get_logger().info('🔍 Nenhuma fronteira viável, tentando área aleatória...')
            self.explore_random_area()
            return
            
        # Navegar para a fronteira
        self.navigate_to_point(best_frontier[0], best_frontier[1])
        self.exploration_state = "MOVING_TO_FRONTIER"
        self.last_goal_time = self.get_clock().now()
        
        self.get_logger().info(f'🎯 Navegando para fronteira: ({best_frontier[0]:.2f}, {best_frontier[1]:.2f})')
        
    def find_frontiers(self):
        """Encontra células de fronteira no mapa"""
        if self.map_data is None:
            return []
            
        frontiers = []
        height, width = self.map_data.shape
        
        # Procurar por células livres adjacentes a células desconhecidas
        for y in range(1, height-1):
            for x in range(1, width-1):
                if self.map_data[y, x] == 0:  # Célula livre
                    # Verificar se tem células desconhecidas ao redor
                    has_unknown_neighbor = False
                    for dy in [-1, 0, 1]:
                        for dx in [-1, 0, 1]:
                            if self.map_data[y+dy, x+dx] == -1:  # Desconhecido
                                has_unknown_neighbor = True
                                break
                        if has_unknown_neighbor:
                            break
                            
                    if has_unknown_neighbor:
                        # Converter para coordenadas do mundo
                        world_x, world_y = self.grid_to_world(x, y)
                        if world_x is not None and world_y is not None:
                            frontiers.append((world_x, world_y))
                            
        return frontiers
        
    def select_best_frontier(self, frontiers):
        """Seleciona a melhor fronteira para explorar"""
        if not frontiers:
            return None
            
        best_frontier = None
        best_score = -1
        
        for frontier in frontiers:
            fx, fy = frontier
            
            # Calcular distância
            distance = math.sqrt((fx - self.current_x)**2 + (fy - self.current_y)**2)
            
            # Pular fronteiras muito próximas ou muito distantes
            if distance < self.frontier_distance_threshold or distance > self.max_exploration_distance:
                continue
                
            # Verificar se não visitamos essa área recentemente
            grid_x, grid_y = self.world_to_grid(fx, fy)
            if grid_x is not None and grid_y is not None:
                if (grid_x, grid_y) in self.visited_areas:
                    continue
                    
            # Calcular score (favor fronteiras a distância média)
            optimal_distance = self.max_exploration_distance * 0.4
            distance_score = 1.0 - abs(distance - optimal_distance) / optimal_distance
            
            # Adicionar aleatoriedade para exploração diversificada
            random_bonus = random.random() * 0.3
            
            score = distance_score + random_bonus
            
            if score > best_score:
                best_score = score
                best_frontier = frontier
                
        return best_frontier
        
    def explore_random_area(self):
        """Explora área aleatória quando não há fronteiras"""
        # Gerar ponto aleatório em área segura
        angle = random.random() * 2 * math.pi
        distance = random.uniform(2.0, 5.0)
        
        target_x = self.current_x + distance * math.cos(angle)
        target_y = self.current_y + distance * math.sin(angle)
        
        self.navigate_to_point(target_x, target_y)
        self.exploration_state = "MOVING_TO_FRONTIER"
        self.last_goal_time = self.get_clock().now()
        
        self.get_logger().info(f'🎲 Explorando área aleatória: ({target_x:.2f}, {target_y:.2f})')
        
    def rotate_and_scan(self):
        """Rotaciona para fazer varredura do ambiente"""
        cmd = Twist()
        cmd.angular.z = self.rotation_speed
        self.cmd_vel_pub.publish(cmd)
        
        # Após algum tempo, parar e voltar a explorar
        self.rotation_counter = getattr(self, 'rotation_counter', 0) + 1
        
        if self.rotation_counter > 15:  # ~3 segundos de rotação
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.rotation_counter = 0
            self.exploration_state = "EXPLORING"
            self.get_logger().info('🔄 Varredura completa, retomando exploração...')
            
    def navigate_to_point(self, x, y):
        """Envia comando de navegação para o waypoint navigator"""
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0
        
        # Orientação padrão
        goal_msg.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal_msg)
        self.current_goal = (x, y)
        
    def world_to_grid(self, x, y):
        """Converte coordenadas do mundo para grid"""
        if self.map_info is None:
            return None, None
            
        grid_x = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        grid_y = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        
        if 0 <= grid_x < self.map_info.width and 0 <= grid_y < self.map_info.height:
            return grid_x, grid_y
        return None, None
        
    def grid_to_world(self, grid_x, grid_y):
        """Converte coordenadas do grid para mundo"""
        if self.map_info is None:
            return None, None
            
        x = grid_x * self.map_info.resolution + self.map_info.origin.position.x
        y = grid_y * self.map_info.resolution + self.map_info.origin.position.y
        
        return x, y

def main(args=None):
    rclpy.init(args=args)
    explorer = AutonomousExplorer()
    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        pass
    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
