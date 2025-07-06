#!/usr/bin/env python3
"""
Sistema de Navegação por Checkpoints para o Robô Caramelo
Implementação baseada no action FollowWaypoints do Nav2
"""

import json
import math
import os
import time
from typing import Dict, List, Optional, Tuple

import rclpy
import yaml
from geometry_msgs.msg import (PoseStamped, PoseWithCovarianceStamped,
                               Quaternion, Twist)
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger


class CarameloCheckpointNavigator(Node):
    """
    Navegador de checkpoints para o robô Caramelo
    Baseado no sistema FollowWaypoints do Nav2
    """
    
    def __init__(self):
        super().__init__('caramelo_checkpoint_navigator')
        
        # Configuração QoS
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        
        # Subscribers
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_profile
        )
        
        self.laser_subscriber = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile
        )
        
        # Action clients
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Serviços
        self.srv_start = self.create_service(
            Trigger, '~/start_checkpoints', self.start_checkpoints_callback
        )
        self.srv_cancel = self.create_service(
            Trigger, '~/cancel_checkpoints', self.cancel_checkpoints_callback
        )
        
        # Parâmetros
        self.declare_parameter('checkpoints_file', '')
        self.declare_parameter('workstations_file', '')
        self.declare_parameter('frame_id', 'map')
        
        # Variáveis de estado
        self.current_pose = None
        self.laser_data = None
        self.workstations = {}
        self.goal_handle = None
        self.navigation_active = False
        
        # Configurações
        self.position_tolerance = 0.3  # metros
        self.orientation_tolerance = 0.1  # radianos
        self.max_navigation_time = 60.0  # segundos
        
        self.get_logger().info("Caramelo Checkpoint Navigator inicializado!")
        
        # Carrega workstations se especificado
        workstations_file = self.get_parameter('workstations_file').get_parameter_value().string_value
        if workstations_file:
            self.load_workstations_from_yaml(workstations_file)
    
    def odom_callback(self, msg):
        """Callback para receber odometria do robô"""
        self.current_pose = msg.pose.pose
    
    def laser_callback(self, msg):
        """Callback para receber dados do LiDAR"""
        self.laser_data = msg.ranges
    
    def euler_from_quaternion(self, quaternion):
        """Converte quaternion para ângulos de Euler"""
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        
        # Cálculo do yaw (rotação em Z)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw
    
    def quaternion_from_euler(self, yaw):
        """Converte ângulo yaw para quaternion"""
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat
    
    def load_workstations_from_yaml(self, yaml_file: str) -> bool:
        """Carrega configurações de workstations de um arquivo YAML"""
        try:
            # Verifica se o arquivo existe
            if not os.path.exists(yaml_file):
                self.get_logger().error(f"Arquivo não encontrado: {yaml_file}")
                return False
            
            with open(yaml_file, 'r') as file:
                data = yaml.safe_load(file)
                
            if 'workstations' in data:
                self.workstations = data['workstations']
                self.get_logger().info(f"Carregadas {len(self.workstations)} workstations do arquivo {yaml_file}")
                return True
            else:
                self.get_logger().error(f"Arquivo {yaml_file} não contém seção 'workstations'")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Erro ao carregar arquivo {yaml_file}: {str(e)}")
            return False
    
    def workstations_to_poses(self, workstation_names: List[str]) -> List[PoseStamped]:
        """Converte lista de workstations para lista de PoseStamped"""
        poses = []
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        for name in workstation_names:
            if name not in self.workstations:
                self.get_logger().error(f"Workstation {name} não encontrada!")
                continue
            
            station = self.workstations[name]
            x, y = station['position']
            yaw = station['orientation']
            
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation = self.quaternion_from_euler(yaw)
            
            poses.append(pose)
        
        return poses
    
    def follow_waypoints_navigation(self, poses: List[PoseStamped]) -> bool:
        """Executa navegação por waypoints usando FollowWaypoints"""
        if not poses:
            self.get_logger().error("Lista de poses vazia!")
            return False
        
        self.get_logger().info(f"Iniciando navegação por {len(poses)} waypoints")
        
        # Prepara mensagem de goal
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses
        
        # Aguarda servidor estar disponível
        if not self.follow_waypoints_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Servidor FollowWaypoints não disponível!")
            return False
        
        # Envia goal
        self.navigation_active = True
        send_goal_future = self.follow_waypoints_client.send_goal_async(goal_msg)
        
        # Aguarda resultado
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal rejeitado pelo servidor!")
            self.navigation_active = False
            return False
        
        self.get_logger().info("Goal aceito, aguardando resultado...")
        self.goal_handle = goal_handle
        
        # Aguarda conclusão
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        self.navigation_active = False
        
        if result.result.result == 1:  # SUCCEEDED
            self.get_logger().info("Navegação por waypoints concluída com sucesso!")
            return True
        else:
            self.get_logger().error(f"Falha na navegação por waypoints! Resultado: {result.result.result}")
            return False
    
    def navigate_to_single_workstation(self, workstation_name: str) -> bool:
        """Navega para uma única workstation"""
        if workstation_name not in self.workstations:
            self.get_logger().error(f"Workstation {workstation_name} não encontrada!")
            return False
        
        station = self.workstations[workstation_name]
        x, y = station['position']
        yaw = station['orientation']
        
        self.get_logger().info(f"Navegando para workstation {workstation_name} em ({x:.2f}, {y:.2f})")
        
        # Cria goal para navegação
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = self.quaternion_from_euler(yaw)
        
        # Envia goal
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Servidor NavigateToPose não disponível!")
            return False
        
        self.navigation_active = True
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        
        # Aguarda resultado
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal rejeitado pelo navegador!")
            self.navigation_active = False
            return False
        
        self.get_logger().info("Goal aceito, aguardando resultado...")
        self.goal_handle = goal_handle
        
        # Aguarda conclusão
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        self.navigation_active = False
        
        if result.result.result == 1:  # SUCCEEDED
            self.get_logger().info(f"Navegação para {workstation_name} concluída com sucesso!")
            return True
        else:
            self.get_logger().error(f"Falha na navegação para {workstation_name}!")
            return False
    
    def start_checkpoints_callback(self, request, response):
        """Callback para o serviço de iniciar checkpoints"""
        try:
            checkpoints_file = self.get_parameter('checkpoints_file').get_parameter_value().string_value
            
            if not checkpoints_file:
                response.success = False
                response.message = "Parâmetro 'checkpoints_file' não especificado"
                return response
            
            if not os.path.exists(checkpoints_file):
                response.success = False
                response.message = f"Arquivo de checkpoints não encontrado: {checkpoints_file}"
                return response
            
            # Carrega checkpoints do arquivo JSON
            with open(checkpoints_file, 'r') as f:
                data = json.load(f)
            
            poses = []
            for checkpoint in data['poses']:
                pose = PoseStamped()
                pose.header.frame_id = data['frame_id']
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = checkpoint['position']['x']
                pose.pose.position.y = checkpoint['position']['y']
                pose.pose.position.z = checkpoint['position']['z']
                pose.pose.orientation.x = checkpoint['orientation']['x']
                pose.pose.orientation.y = checkpoint['orientation']['y']
                pose.pose.orientation.z = checkpoint['orientation']['z']
                pose.pose.orientation.w = checkpoint['orientation']['w']
                poses.append(pose)
            
            # Executa navegação por waypoints
            if self.follow_waypoints_navigation(poses):
                response.success = True
                response.message = "Navegação por checkpoints iniciada com sucesso"
            else:
                response.success = False
                response.message = "Falha ao iniciar navegação por checkpoints"
                
        except FileNotFoundError as e:
            self.get_logger().error(f"Arquivo de checkpoints não encontrado: {e}")
            response.success = False
            response.message = f"Arquivo de checkpoints não encontrado: {e}"
        except Exception as e:
            self.get_logger().error(f"Erro ao processar checkpoints: {e}")
            response.success = False
            response.message = f"Erro ao processar checkpoints: {e}"
            
        return response
    
    def cancel_checkpoints_callback(self, request, response):
        """Callback para o serviço de cancelar checkpoints"""
        try:
            if self.goal_handle is not None and self.navigation_active:
                # Cancela o goal atual
                cancel_future = self.goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                
                self.navigation_active = False
                self.goal_handle = None
                
                response.success = True
                response.message = "Navegação por checkpoints cancelada com sucesso"
            else:
                response.success = False
                response.message = "Nenhuma navegação ativa para cancelar"
                
        except Exception as e:
            self.get_logger().error(f"Erro ao cancelar navegação: {e}")
            response.success = False
            response.message = f"Erro ao cancelar navegação: {e}"
            
        return response
    
    def execute_workstation_sequence(self, workstation_names: List[str]) -> bool:
        """Executa uma sequência de workstations"""
        poses = self.workstations_to_poses(workstation_names)
        if not poses:
            self.get_logger().error("Nenhuma pose válida para executar!")
            return False
        
        return self.follow_waypoints_navigation(poses)
    
    def list_workstations(self):
        """Lista todas as workstations configuradas"""
        if not self.workstations:
            self.get_logger().info("Nenhuma workstation configurada.")
            return
        
        self.get_logger().info("Workstations configuradas:")
        for name, data in self.workstations.items():
            x, y = data['position']
            yaw = data['orientation']
            desc = data.get('description', '')
            self.get_logger().info(f"  {name}: ({x:.2f}, {y:.2f}) @ {yaw:.2f} rad - {desc}")
    
    def get_current_position_info(self):
        """Retorna informações da posição atual"""
        if self.current_pose is None:
            return "Posição atual não disponível"
        
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        yaw = self.euler_from_quaternion(self.current_pose.orientation)
        
        return f"Posição atual: ({x:.2f}, {y:.2f}) @ {yaw:.2f} rad"

def main(args=None):
    """Função principal"""
    rclpy.init(args=args)
    
    navigator = CarameloCheckpointNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info("Navegador interrompido pelo usuário")
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
