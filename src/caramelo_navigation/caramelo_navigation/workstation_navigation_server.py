#!/usr/bin/env python3
"""
WORKSTATION NAVIGATION SERVER
============================

Servidor ROS2 que expõe um serviço para navegação a workstations específicas.
O servidor:
1. Carrega waypoints de uma arena específica 
2. Expõe serviço NavigateToWorkstation
3. Utiliza Nav2 para navegação autônoma
4. Retorna status de sucesso/falha

Este servidor permite que:
- caramelo_tasks envie comandos de navegação
- Desenvolvedores testem navegação manualmente
- Sistema seja modular e reutilizável

Uso:
    ros2 run caramelo_navigation workstation_navigation_server --ros-args -p arena:=arena_fei

Serviço disponível:
    /navigate_to_workstation (caramelo_navigation/srv/NavigateToWorkstation)

Autor: GitHub Copilot
Data: 2025-01-18
"""

import json
import os
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.srv import GetPlan
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.callback_groups import (MutuallyExclusiveCallbackGroup,
                                   ReentrantCallbackGroup)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool

try:
    from caramelo_navigation.srv import NavigateToWorkstation
except ImportError:
    # Placeholder para desenvolvimento até compilarmos o serviço
    class NavigateToWorkstationRequest:
        def __init__(self):
            self.workstation_name = ""
            self.arena_name = ""
            self.timeout = 0.0
            self.wait_for_completion = False

    class NavigateToWorkstationResponse:
        def __init__(self):
            self.success = False
            self.message = ""
            self.time_elapsed = 0.0
            self.distance_traveled = 0.0

    class NavigateToWorkstation:
        Request = NavigateToWorkstationRequest
        Response = NavigateToWorkstationResponse


class WorkstationNavigationServer(Node):
    """
    Servidor de navegação para workstations específicas.
    """
    
    def __init__(self):
        super().__init__('workstation_navigation_server')
        
        # Parâmetros
        self.declare_parameter('arena', 'arena_fei')
        self.declare_parameter('waypoints_base_path', '/home/work/Caramelo_workspace/maps')
        self.declare_parameter('default_timeout', 120.0)  # 2 minutos
        
        self.arena = self.get_parameter('arena').get_parameter_value().string_value
        self.waypoints_base_path = self.get_parameter('waypoints_base_path').get_parameter_value().string_value
        self.default_timeout = self.get_parameter('default_timeout').get_parameter_value().double_value
        
        # Estado
        self.waypoints = {}
        self.navigator = None
        self.is_navigating = False
        self.navigation_lock = threading.Lock()
        
        # Callback groups para permitir execução paralela
        self.service_callback_group = ReentrantCallbackGroup()
        
        # Carregar waypoints
        self._load_waypoints()
        
        # Inicializar Nav2
        self._initialize_navigator()
        
        # Criar serviço
        self.navigation_service = self.create_service(
            NavigateToWorkstation,
            'navigate_to_workstation',
            self._navigate_to_workstation_callback,
            callback_group=self.service_callback_group
        )
        
        self.get_logger().info(f"🗺️  Workstation Navigation Server iniciado!")
        self.get_logger().info(f"   Arena: {self.arena}")
        self.get_logger().info(f"   Waypoints carregados: {len(self.waypoints)}")
        self.get_logger().info(f"   Serviço: /navigate_to_workstation")
        self.get_logger().info(f"   Waypoints disponíveis: {list(self.waypoints.keys())}")
    
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
            self.get_logger().info("✅ Nav2 BasicNavigator inicializado")
        except Exception as e:
            self.get_logger().error(f"❌ Erro ao inicializar Navigator: {e}")
    
    def _navigate_to_workstation_callback(self, request, response):
        """Callback do serviço de navegação."""
        workstation_name = request.workstation_name
        arena_name = request.arena_name if request.arena_name else self.arena
        timeout = request.timeout if request.timeout > 0.0 else self.default_timeout
        wait_for_completion = request.wait_for_completion
        
        self.get_logger().info(f"🎯 Solicitação de navegação recebida:")
        self.get_logger().info(f"   Workstation: {workstation_name}")
        self.get_logger().info(f"   Arena: {arena_name}")
        self.get_logger().info(f"   Timeout: {timeout}s")
        self.get_logger().info(f"   Aguardar conclusão: {wait_for_completion}")
        
        # Verificar se arena mudou
        if arena_name != self.arena:
            self.get_logger().info(f"🔄 Mudando arena de {self.arena} para {arena_name}")
            self.arena = arena_name
            self._load_waypoints()
        
        # Verificar se workstation existe
        if workstation_name not in self.waypoints:
            response.success = False
            response.message = f"Workstation '{workstation_name}' não encontrada. Disponíveis: {list(self.waypoints.keys())}"
            self.get_logger().error(f"❌ {response.message}")
            return response
        
        # Verificar se navegador está disponível
        if not self.navigator:
            response.success = False
            response.message = "Navigator não está inicializado"
            self.get_logger().error(f"❌ {response.message}")
            return response
        
        # Verificar se já está navegando
        with self.navigation_lock:
            if self.is_navigating:
                response.success = False
                response.message = "Robô já está navegando. Aguarde a conclusão."
                self.get_logger().warn(f"⚠️  {response.message}")
                return response
            
            self.is_navigating = True
        
        try:
            # Criar pose de destino
            goal_pose = self._create_pose_stamped(workstation_name)
            
            if not goal_pose:
                response.success = False
                response.message = f"Erro ao criar pose para {workstation_name}"
                return response
            
            # Iniciar navegação
            start_time = time.time()
            self.get_logger().info(f"🚀 Iniciando navegação para {workstation_name}")
            
            self.navigator.goToPose(goal_pose)
            
            if not wait_for_completion:
                # Retornar imediatamente (modo assíncrono)
                response.success = True
                response.message = f"Navegação para {workstation_name} iniciada com sucesso"
                response.time_elapsed = 0.0
                response.distance_traveled = 0.0
                self.get_logger().info(f"✅ {response.message}")
                return response
            
            # Aguardar conclusão (modo síncrono)
            self.get_logger().info(f"⏳ Aguardando conclusão da navegação...")
            
            while not self.navigator.isTaskComplete():
                time.sleep(0.1)
                elapsed = time.time() - start_time
                
                if elapsed > timeout:
                    self.navigator.cancelTask()
                    response.success = False
                    response.message = f"Timeout após {timeout}s"
                    response.time_elapsed = elapsed
                    self.get_logger().error(f"⏰ {response.message}")
                    return response
            
            # Verificar resultado
            result = self.navigator.getResult()
            elapsed_time = time.time() - start_time
            
            if result == 3:  # TaskResult.SUCCEEDED
                response.success = True
                response.message = f"Navegação para {workstation_name} concluída com sucesso"
                response.time_elapsed = elapsed_time
                response.distance_traveled = 0.0  # TODO: calcular distância real
                self.get_logger().info(f"🎉 {response.message} (tempo: {elapsed_time:.2f}s)")
            else:
                response.success = False
                response.message = f"Navegação falhou com resultado: {result}"
                response.time_elapsed = elapsed_time
                self.get_logger().error(f"❌ {response.message}")
        
        except Exception as e:
            response.success = False
            response.message = f"Erro durante navegação: {str(e)}"
            self.get_logger().error(f"❌ {response.message}")
        
        finally:
            with self.navigation_lock:
                self.is_navigating = False
        
        return response
    
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


def main(args=None):
    rclpy.init(args=args)
    
    # Usar MultiThreadedExecutor para permitir processamento paralelo
    executor = MultiThreadedExecutor(num_threads=4)
    
    node = WorkstationNavigationServer()
    
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
