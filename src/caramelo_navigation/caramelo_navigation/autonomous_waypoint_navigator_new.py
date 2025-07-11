#!/usr/bin/env python3

import json
import os
import time
from typing import List

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node


class AutonomousWaypointNavigator(Node):
    """
    Navegador autônomo que lê waypoints de um arquivo JSON e navega sequencialmente
    usando Nav2 Simple Commander API.
    """

    def __init__(self):
        super().__init__('autonomous_waypoint_navigator')
        
        # Declarar parâmetros
        self.declare_parameter('map_folder', 'arena_fei')
        self.declare_parameter('loop_mission', False)
        
        # Pegar parâmetros
        self.map_folder = self.get_parameter('map_folder').get_parameter_value().string_value
        self.loop_mission = self.get_parameter('loop_mission').get_parameter_value().bool_value
        
        # Inicializar o navegador básico
        self.navigator = BasicNavigator()
        
        # Lista de waypoints carregados
        self.waypoints: List[PoseStamped] = []
        
        self.get_logger().info(f'Navegador iniciado para pasta: {self.map_folder}')
        
        # Timer para aguardar Nav2 estar ativo e iniciar navegação
        self.initialization_timer = self.create_timer(2.0, self.initialize_navigation)
        
    def initialize_navigation(self):
        """Inicializa a navegação após aguardar Nav2 estar pronto."""
        try:
            self.get_logger().info('Aguardando Nav2 ficar ativo...')
            
            # Aguardar Nav2 estar completamente ativo
            self.navigator.waitUntilNav2Active()
            
            self.get_logger().info('Nav2 está ativo! Carregando waypoints...')
            self.initialization_timer.cancel()
            
            # Carregar waypoints e iniciar navegação
            if self.load_waypoints():
                self.start_navigation()
            else:
                self.get_logger().error('Falha ao carregar waypoints')
                
        except Exception as e:
            self.get_logger().error(f'Erro na inicialização: {str(e)}')
    
    def load_waypoints(self) -> bool:
        """Carrega waypoints do arquivo JSON na pasta do mapa."""
        waypoints_file = f'/home/work/Caramelo_workspace/maps/{self.map_folder}/waypoints_simple.json'
        
        if not os.path.exists(waypoints_file):
            self.get_logger().error(f'Arquivo de waypoints não encontrado: {waypoints_file}')
            return False
        
        try:
            with open(waypoints_file, 'r') as f:
                waypoints_data = json.load(f)
            
            self.waypoints = []
            
            # Converter waypoints do JSON para PoseStamped
            for waypoint_data in waypoints_data.get('waypoints', []):
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.navigator.get_clock().now().to_msg()
                
                # Posição
                pose.pose.position.x = float(waypoint_data['position']['x'])
                pose.pose.position.y = float(waypoint_data['position']['y'])
                pose.pose.position.z = float(waypoint_data['position']['z'])
                
                # Orientação
                pose.pose.orientation.x = float(waypoint_data['orientation']['x'])
                pose.pose.orientation.y = float(waypoint_data['orientation']['y'])
                pose.pose.orientation.z = float(waypoint_data['orientation']['z'])
                pose.pose.orientation.w = float(waypoint_data['orientation']['w'])
                
                self.waypoints.append(pose)
                
            self.get_logger().info(f'Carregados {len(self.waypoints)} waypoints')
            return len(self.waypoints) > 0
                
        except Exception as e:
            self.get_logger().error(f'Erro ao carregar waypoints: {str(e)}')
            return False
    
    def start_navigation(self):
        """Inicia a navegação pelos waypoints usando Nav2 Simple Commander."""
        if not self.waypoints:
            self.get_logger().warn('Nenhum waypoint para navegar')
            return
            
        self.get_logger().info(f'Iniciando navegação por {len(self.waypoints)} waypoints...')
        
        # Definir pose inicial (posição atual do robô)
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0  # Robô inicia na origem
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.position.z = 0.0
        initial_pose.pose.orientation.x = 0.0
        initial_pose.pose.orientation.y = 0.0
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 1.0
        
        # Definir pose inicial no AMCL
        self.navigator.setInitialPose(initial_pose)
        
        # Iniciar navegação sequencial pelos waypoints
        self.navigate_waypoints()
    
    def navigate_waypoints(self):
        """Navega pelos waypoints sequencialmente."""
        while rclpy.ok():
            try:
                self.get_logger().info('=== INICIANDO NAVEGAÇÃO POR WAYPOINTS ===')
                
                # Usar followWaypoints para navegação sequencial
                nav_start = self.navigator.get_clock().now()
                self.navigator.followWaypoints(self.waypoints)
                
                i = 0
                while not self.navigator.isTaskComplete():
                    # Processar feedback
                    i += 1
                    feedback = self.navigator.getFeedback()
                    
                    if feedback and i % 5 == 0:
                        current_wp = feedback.current_waypoint + 1
                        total_wp = len(self.waypoints)
                        self.get_logger().info(f'Executando waypoint: {current_wp}/{total_wp}')
                    
                    # Timeout de segurança (10 minutos)
                    now = self.navigator.get_clock().now()
                    duration_ns = (now - nav_start).nanoseconds
                    if duration_ns > 600_000_000_000:  # 10 minutos em nanosegundos
                        self.get_logger().warn('Timeout de navegação! Cancelando...')
                        self.navigator.cancelTask()
                        break
                    
                    # Pequena pausa para não sobrecarregar
                    time.sleep(0.1)
                
                # Verificar resultado
                result = self.navigator.getResult()
                
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info('=== TODOS OS WAYPOINTS ALCANÇADOS COM SUCESSO! ===')
                elif result == TaskResult.CANCELED:
                    self.get_logger().warn('Navegação cancelada!')
                elif result == TaskResult.FAILED:
                    self.get_logger().error('Navegação falhou!')
                else:
                    self.get_logger().error('Navegação retornou status inválido!')
                
                # Verificar se deve repetir em loop
                if self.loop_mission:
                    self.get_logger().info('Reiniciando missão em loop em 5 segundos...')
                    time.sleep(5.0)
                else:
                    self.get_logger().info('Missão completa! Finalizando navegador.')
                    break
                    
            except KeyboardInterrupt:
                self.get_logger().info('Navegação interrompida pelo usuário')
                self.navigator.cancelTask()
                break
            except Exception as e:
                self.get_logger().error(f'Erro durante navegação: {str(e)}')
                time.sleep(5.0)  # Aguardar antes de tentar novamente
    
    def shutdown(self):
        """Finaliza o navegador."""
        self.navigator.lifecycleShutdown()


def main(args=None):
    rclpy.init(args=args)
    navigator = None
    
    try:
        navigator = AutonomousWaypointNavigator()
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        print('\nNavegação interrompida pelo usuário')
    finally:
        if navigator is not None:
            navigator.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
