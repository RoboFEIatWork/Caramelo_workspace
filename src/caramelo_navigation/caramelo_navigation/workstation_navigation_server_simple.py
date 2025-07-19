#!/usr/bin/env python3
"""
WORKSTATION NAVIGATION SERVER - VERSÃO SIMPLIFICADA
===================================================

Servidor ROS2 que navega para workstations usando tópicos em vez de serviços customizados.
Mais simples de implementar e usar.

O servidor:
1. Carrega waypoints automaticamente da arena 
2. Escuta tópico /navigate_to_workstation
3. Utiliza Nav2 para navegação autônoma
4. Publica status no tópico /navigation_status

Uso:
    ros2 run caramelo_navigation workstation_navigation_server_simple --ros-args -p arena:=arena_fei

Para navegar:
    ros2 topic pub --once /navigate_to_workstation std_msgs/String "data: WS01"

Para verificar status:
    ros2 topic echo /navigation_status

Autor: GitHub Copilot
Data: 2025-01-18
"""

import json
import os
import threading
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import Path as NavPath
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


class WorkstationNavigationServerSimple(Node):
    """
    Servidor de navegação para workstations específicas usando tópicos.
    """
    
    def __init__(self):
        super().__init__('workstation_navigation_server_simple')
        
        # Parâmetros
        self.declare_parameter('arena', 'arena_robocup25')
        self.declare_parameter('waypoints_base_path', '/home/work/Caramelo_workspace/maps')
        self.declare_parameter('default_timeout', 120.0)
        
        self.arena = self.get_parameter('arena').get_parameter_value().string_value
        self.waypoints_base_path = self.get_parameter('waypoints_base_path').get_parameter_value().string_value
        self.default_timeout = self.get_parameter('default_timeout').get_parameter_value().double_value
        
        # Estado
        self.waypoints = {}
        self.navigator = None
        self.is_navigating = False
        self.navigation_lock = threading.Lock()
        self.current_goal = ""
        
        # Action Clients para navegação individual
        self.planner_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.controller_client = ActionClient(self, FollowPath, 'follow_path')
        
        # Callback groups para permitir execução paralela
        self.callback_group = ReentrantCallbackGroup()
        
        # Carregar waypoints
        self._load_waypoints()
        
        # Inicializar Nav2 (opcional, vai usar action clients individuais)
        self._initialize_navigator()
        
        # Criar subscriber e publisher
        self.navigation_subscriber = self.create_subscription(
            String,
            'navigate_to_workstation',
            self._navigate_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.status_publisher = self.create_publisher(
            String,
            'navigation_status',
            10
        )
        
        # Timer para publicar status
        self.status_timer = self.create_timer(
            1.0,  # 1 Hz
            self._publish_status,
            callback_group=self.callback_group
        )
        
        self.get_logger().info(f"🗺️  Workstation Navigation Server (Simple) iniciado!")
        self.get_logger().info(f"   Arena: {self.arena}")
        self.get_logger().info(f"   Waypoints carregados: {len(self.waypoints)}")
        self.get_logger().info(f"   Tópicos:")
        self.get_logger().info(f"     Entrada: /navigate_to_workstation (std_msgs/String)")
        self.get_logger().info(f"     Status:  /navigation_status (std_msgs/String)")
        self.get_logger().info(f"   Waypoints disponíveis: {list(self.waypoints.keys())}")
        
        self._publish_status_msg("READY", "Sistema pronto para navegação")
    
    def _load_waypoints(self):
        """Carrega waypoints do arquivo JSON da arena."""
        waypoints_file = os.path.join(self.waypoints_base_path, self.arena, 'waypoints.json')
        
        if not os.path.exists(waypoints_file):
            self.get_logger().error(f"❌ Arquivo de waypoints não encontrado: {waypoints_file}")
            return
        
        try:
            with open(waypoints_file, 'r') as f:
                data = json.load(f)
            
            # Converter para dicionário de fácil acesso
            for waypoint in data.get('waypoints', []):
                name = waypoint['name']
                self.waypoints[name] = {
                    'position': waypoint['position'],
                    'orientation': waypoint['orientation']
                }
            
            self.get_logger().info(f"✅ Waypoints carregados: {list(self.waypoints.keys())}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Erro ao carregar waypoints: {e}")
    
    def _initialize_navigator(self):
        """Inicializa o navegador Nav2."""
        try:
            self.navigator = BasicNavigator()
            # Aguardar o sistema estar pronto sem exigir bt_navigator
            self.get_logger().info("✅ Nav2 BasicNavigator inicializado")
        except Exception as e:
            self.get_logger().warn(f"⚠️  Erro ao inicializar Navigator (pode ser normal se bt_navigator falhou): {e}")
            self.navigator = None
    
    def _navigate_callback(self, msg):
        """Callback para comandos de navegação."""
        workstation_name = msg.data.strip()
        
        self.get_logger().info(f"🎯 Comando de navegação recebido: {workstation_name}")
        
        # Verificar se workstation existe
        if workstation_name not in self.waypoints:
            error_msg = f"Workstation '{workstation_name}' não encontrada. Disponíveis: {list(self.waypoints.keys())}"
            self.get_logger().error(f"❌ {error_msg}")
            self._publish_status_msg("ERROR", error_msg)
            return
        
        # Verificar se navigator está disponível - REMOVIDO: usar action clients sempre
        # if not self.navigator:
        #     error_msg = "Navigator não está inicializado"
        #     self.get_logger().error(f"❌ {error_msg}")
        #     self._publish_status_msg("ERROR", error_msg)
        #     return
        
        # Verificar se já está navegando
        with self.navigation_lock:
            if self.is_navigating:
                error_msg = "Robô já está navegando. Aguarde a conclusão."
                self.get_logger().warn(f"⚠️  {error_msg}")
                self._publish_status_msg("BUSY", error_msg)
                return
            
            self.is_navigating = True
            self.current_goal = workstation_name
        
        # Executar navegação em thread separada
        thread = threading.Thread(target=self._execute_navigation, args=(workstation_name,))
        thread.daemon = True
        thread.start()
    
    def _execute_navigation(self, workstation_name):
        """Executa a navegação usando action clients individuais."""
        try:
            # Criar pose de destino
            goal_pose = self._create_pose_stamped(workstation_name)
            
            if not goal_pose:
                self._publish_status_msg("ERROR", f"Erro ao criar pose para {workstation_name}")
                return
            
            # Iniciar navegação
            start_time = time.time()
            self.get_logger().info(f"🚀 Iniciando navegação para {workstation_name}")
            self._publish_status_msg("NAVIGATING", f"Planejando rota para {workstation_name}")
            
            # Etapa 1: Planejar caminho
            path = self._compute_path(goal_pose)
            if not path:
                self._publish_status_msg("FAILED", f"Falha no planejamento para {workstation_name}")
                return
            
            elapsed = time.time() - start_time
            self.get_logger().info(f"📍 Caminho planejado para {workstation_name} ({elapsed:.1f}s)")
            self._publish_status_msg("NAVIGATING", f"Seguindo caminho para {workstation_name}")
            
            # Etapa 2: Seguir caminho
            success = self._follow_path(path, workstation_name, start_time)
            
            elapsed_time = time.time() - start_time
            
            if success:
                success_msg = f"Navegação para {workstation_name} concluída com sucesso (tempo: {elapsed_time:.2f}s)"
                self._publish_status_msg("SUCCESS", success_msg)
                self.get_logger().info(f"🎉 {success_msg}")
            else:
                error_msg = f"Navegação para {workstation_name} falhou durante execução do caminho"
                self._publish_status_msg("FAILED", error_msg)
                self.get_logger().error(f"❌ {error_msg}")
        
        except Exception as e:
            error_msg = f"Erro durante navegação: {str(e)}"
            self._publish_status_msg("ERROR", error_msg)
            self.get_logger().error(f"❌ {error_msg}")
        
        finally:
            with self.navigation_lock:
                self.is_navigating = False
                self.current_goal = ""
    
    def _compute_path(self, goal_pose):
        """Computa caminho usando o planner server."""
        try:
            # Aguardar planner estar disponível
            if not self.planner_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("❌ Planner server não disponível")
                return None
            
            # Criar goal para ComputePathToPose
            goal_msg = ComputePathToPose.Goal()
            goal_msg.goal = goal_pose
            goal_msg.use_start = False  # Usar posição atual do robô
            
            # Enviar goal
            future = self.planner_client.send_goal_async(goal_msg)
            
            # Aguardar resposta do goal
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if not future.done():
                self.get_logger().error("❌ Timeout ao enviar goal para planner")
                return None
            
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error("❌ Goal rejeitado pelo planner")
                return None
            
            # Aguardar resultado
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
            
            if not result_future.done():
                self.get_logger().error("❌ Timeout ao computar caminho")
                try:
                    goal_handle.cancel_goal_async()
                except:
                    pass
                return None
            
            result = result_future.result()
            
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                path = result.result.path
                self.get_logger().info(f"✅ Caminho computado com {len(path.poses)} poses")
                return path
            else:
                self.get_logger().error(f"❌ Planejamento falhou com status: {result.status}")
                return None
                
        except Exception as e:
            self.get_logger().error(f"❌ Erro ao computar caminho: {e}")
            return None
    
    def _follow_path(self, path, workstation_name, start_time):
        """Segue o caminho usando o controller server."""
        try:
            # Aguardar controller estar disponível
            if not self.controller_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("❌ Controller server não disponível")
                return False
            
            # Criar goal para FollowPath
            goal_msg = FollowPath.Goal()
            goal_msg.path = path
            goal_msg.controller_id = "FollowPath"  # ID do controller configurado
            
            # Enviar goal
            future = self.controller_client.send_goal_async(goal_msg)
            
            # Aguardar resposta do goal
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if not future.done():
                self.get_logger().error("❌ Timeout ao enviar goal para controller")
                return False
            
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error("❌ Goal rejeitado pelo controller")
                return False
            
            # Aguardar resultado com timeout e atualizações de status
            result_future = goal_handle.get_result_async()
            
            while not result_future.done():
                time.sleep(0.1)
                elapsed = time.time() - start_time
                
                if elapsed > self.default_timeout:
                    self.get_logger().error(f"⏰ Timeout após {self.default_timeout}s")
                    try:
                        goal_handle.cancel_goal_async()
                    except:
                        pass
                    return False
                
                # Atualizar status com tempo decorrido
                self._publish_status_msg("NAVIGATING", f"Seguindo caminho para {workstation_name} ({elapsed:.1f}s)")
                
                # Continue spinning to process the future
                rclpy.spin_once(self, timeout_sec=0.1)
            
            result = result_future.result()
            
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f"✅ Caminho seguido com sucesso")
                return True
            else:
                self.get_logger().error(f"❌ Falha ao seguir caminho, status: {result.status}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"❌ Erro ao seguir caminho: {e}")
            return False
    
    def _create_pose_stamped(self, workstation_name):
        """Cria um PoseStamped a partir dos dados do waypoint."""
        try:
            waypoint = self.waypoints[workstation_name]
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
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
            
            return pose
        
        except Exception as e:
            self.get_logger().error(f"❌ Erro ao criar pose para {workstation_name}: {e}")
            return None
    
    def _publish_status_msg(self, status, message):
        """Publica status da navegação."""
        status_data = {
            "status": status,
            "message": message,
            "timestamp": time.time(),
            "current_goal": self.current_goal,
            "is_navigating": self.is_navigating
        }
        
        msg = String()
        msg.data = json.dumps(status_data)
        self.status_publisher.publish(msg)
    
    def _simulate_navigation(self, workstation_name):
        """Simula navegação quando bt_navigator não está disponível."""
        self.get_logger().info(f"🔄 Simulando navegação para {workstation_name}")
        self._publish_status_msg("NAVIGATING", f"Simulando navegação para {workstation_name}")
        
        # Simular tempo de navegação
        simulation_time = 10.0  # segundos
        start_time = time.time()
        
        while time.time() - start_time < simulation_time:
            elapsed = time.time() - start_time
            self._publish_status_msg("NAVIGATING", f"Simulando navegação para {workstation_name} ({elapsed:.1f}s)")
            time.sleep(1.0)
        
        # Simular sucesso
        success_msg = f"Simulação de navegação para {workstation_name} concluída"
        self._publish_status_msg("SUCCESS", success_msg)
        self.get_logger().info(f"✅ {success_msg}")
    
    def _publish_status(self):
        """Timer callback para publicar status periodicamente."""
        if self.is_navigating:
            self._publish_status_msg("NAVIGATING", f"Navegando para {self.current_goal}")
        else:
            self._publish_status_msg("READY", "Sistema pronto para navegação")


def main(args=None):
    rclpy.init(args=args)
    
    # Usar MultiThreadedExecutor para permitir processamento paralelo
    executor = MultiThreadedExecutor(num_threads=4)
    
    node = WorkstationNavigationServerSimple()
    
    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
