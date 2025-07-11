#!/usr/bin/env python3
"""
Helper node para inicialização fail-proof dos lifecycle nodes do Nav2.

Este node garante que todos os lifecycle nodes sejam configurados e ativados
automaticamente, e envia uma pose inicial para o AMCL para estabelecer
a transformação map -> odom.

Funciona como fail-safe para casos de reboot, troca de mapa ou configurações perdidas.
"""

import json
import os
import time
from threading import Thread

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node


class LifecycleStartupHelper(Node):
    def __init__(self):
        super().__init__('lifecycle_startup_helper')
        
        # Parâmetros
        self.declare_parameter('timeout_seconds', 30.0)
        self.declare_parameter('arena', 'arena_fei')
        
        self.timeout_seconds = self.get_parameter('timeout_seconds').value
        self.arena = self.get_parameter('arena').value
        
        # Publisher para pose inicial
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )
        
        # Lista de nodes de localização e navegação
        self.localization_nodes = ['map_server', 'amcl']
        self.navigation_nodes = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'waypoint_follower', 'smoother_server']
        
        self.get_logger().info("🚀 Lifecycle Startup Helper iniciado!")
        self.get_logger().info(f"Arena: {self.arena}")
        self.get_logger().info(f"Timeout: {self.timeout_seconds}s")
        
        # Inicia processo de inicialização em thread separada
        self.startup_thread = Thread(target=self.startup_sequence)
        self.startup_thread.daemon = True
        self.startup_thread.start()

    def startup_sequence(self):
        """Sequência de inicialização fail-proof."""
        try:
            self.get_logger().info("⏳ Aguardando 5 segundos para Nav2 carregar...")
            time.sleep(5.0)
            
            # 1. Força inicialização dos lifecycle managers
            self.get_logger().info("🔧 Iniciando lifecycle managers...")
            self.activate_lifecycle_managers()
            
            # 2. Verifica e ativa nodes individuais se necessário
            self.get_logger().info("🔍 Verificando e ativando nodes de localização...")
            self.ensure_nodes_active(self.localization_nodes)
            
            self.get_logger().info("🔍 Verificando e ativando nodes de navegação...")
            self.ensure_nodes_active(self.navigation_nodes)
            
            # 3. Aguarda estabilização
            self.get_logger().info("⏳ Aguardando estabilização do sistema...")
            time.sleep(3.0)
            
            # 4. Envia pose inicial
            self.get_logger().info("📍 Enviando pose inicial para AMCL...")
            self.send_initial_pose()
            
            # 5. Aguarda estabelecimento do TF
            self.get_logger().info("🗺️ Aguardando estabelecimento do TF map->odom...")
            time.sleep(2.0)
            
            self.get_logger().info("✅ Inicialização fail-proof concluída com sucesso!")
            
        except Exception as e:
            self.get_logger().error(f"❌ Erro na inicialização: {e}")

    def activate_lifecycle_managers(self):
        """Ativa os lifecycle managers."""
        managers = [
            '/lifecycle_manager_localization/manage_nodes',
            '/lifecycle_manager_navigation/manage_nodes'
        ]
        
        for manager in managers:
            try:
                client = self.create_client(ManageLifecycleNodes, manager)
                if client.wait_for_service(timeout_sec=5.0):
                    request = ManageLifecycleNodes.Request()
                    request.command = ManageLifecycleNodes.Request.STARTUP
                    
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
                    
                    if future.result():
                        self.get_logger().info(f"✅ {manager} ativado com sucesso")
                    else:
                        self.get_logger().warn(f"⚠️ {manager} não respondeu")
                else:
                    self.get_logger().warn(f"⚠️ Serviço {manager} não disponível")
            except Exception as e:
                self.get_logger().warn(f"⚠️ Erro ao ativar {manager}: {e}")

    def ensure_nodes_active(self, node_names):
        """Garante que todos os nodes estejam ativos."""
        for node_name in node_names:
            try:
                self.get_logger().info(f"🔧 Verificando node: {node_name}")
                
                # Cliente para verificar estado
                state_client = self.create_client(GetState, f'/{node_name}/get_state')
                if not state_client.wait_for_service(timeout_sec=5.0):
                    self.get_logger().warn(f"⚠️ Serviço de estado não encontrado para {node_name}")
                    continue
                
                # Verifica estado atual
                request = GetState.Request()
                future = state_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if not future.result():
                    self.get_logger().warn(f"⚠️ Não foi possível obter estado de {node_name}")
                    continue
                
                result = future.result()
                current_state = result.current_state.id
                self.get_logger().info(f"📊 {node_name} está em estado: {current_state}")
                
                # Se não estiver ativo, força ativação
                if current_state != State.PRIMARY_STATE_ACTIVE:
                    self.get_logger().info(f"🔧 Ativando {node_name}...")
                    self.activate_node(node_name, current_state)
                else:
                    self.get_logger().info(f"✅ {node_name} já está ativo")
                    
            except Exception as e:
                self.get_logger().warn(f"⚠️ Erro ao verificar {node_name}: {e}")

    def activate_node(self, node_name, current_state):
        """Ativa um node específico."""
        try:
            change_client = self.create_client(ChangeState, f'/{node_name}/change_state')
            if not change_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn(f"⚠️ Serviço de mudança de estado não encontrado para {node_name}")
                return
            
            # Sequência de ativação baseada no estado atual
            transitions = []
            
            if current_state == State.PRIMARY_STATE_UNKNOWN:
                transitions = [Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE]
            elif current_state == State.PRIMARY_STATE_UNCONFIGURED:
                transitions = [Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE]
            elif current_state == State.PRIMARY_STATE_INACTIVE:
                transitions = [Transition.TRANSITION_ACTIVATE]
            
            # Executa transições
            for transition in transitions:
                request = ChangeState.Request()
                request.transition.id = transition
                
                future = change_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
                
                result = future.result()
                if result and result.success:
                    self.get_logger().info(f"✅ Transição {transition} para {node_name} bem-sucedida")
                    time.sleep(1.0)  # Aguarda estabilização
                else:
                    self.get_logger().warn(f"⚠️ Falha na transição {transition} para {node_name}")
                    break
                    
        except Exception as e:
            self.get_logger().error(f"❌ Erro ao ativar {node_name}: {e}")

    def send_initial_pose(self):
        """Envia pose inicial para AMCL."""
        try:
            # Carrega pose inicial do arquivo de waypoints
            maps_path = f'/home/work/Caramelo_workspace/maps/{self.arena}'
            waypoints_file = os.path.join(maps_path, 'waypoints.json')
            
            initial_x, initial_y, initial_yaw = 0.0, 0.0, 0.0
            
            if os.path.exists(waypoints_file):
                try:
                    with open(waypoints_file, 'r') as f:
                        data = json.load(f)
                        if 'initial_pose' in data:
                            initial_x = data['initial_pose'].get('x', 0.0)
                            initial_y = data['initial_pose'].get('y', 0.0)
                            initial_yaw = data['initial_pose'].get('yaw', 0.0)
                            self.get_logger().info(f"📍 Pose inicial carregada: x={initial_x}, y={initial_y}, yaw={initial_yaw}")
                except Exception as e:
                    self.get_logger().warn(f"⚠️ Erro ao carregar waypoints: {e}. Usando pose padrão.")
            
            # Cria mensagem de pose inicial
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            
            pose_msg.pose.pose.position.x = initial_x
            pose_msg.pose.pose.position.y = initial_y
            pose_msg.pose.pose.position.z = 0.0
            
            # Converte yaw para quaternion
            import math
            pose_msg.pose.pose.orientation.x = 0.0
            pose_msg.pose.pose.orientation.y = 0.0
            pose_msg.pose.pose.orientation.z = math.sin(initial_yaw / 2.0)
            pose_msg.pose.pose.orientation.w = math.cos(initial_yaw / 2.0)
            
            # Covariância padrão
            pose_msg.pose.covariance = [
                0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787
            ]
            
            # Publica pose inicial múltiplas vezes para garantia
            for i in range(5):
                self.initial_pose_pub.publish(pose_msg)
                self.get_logger().info(f"📍 Pose inicial enviada (tentativa {i+1}/5)")
                time.sleep(0.5)
                
        except Exception as e:
            self.get_logger().error(f"❌ Erro ao enviar pose inicial: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    node = LifecycleStartupHelper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
