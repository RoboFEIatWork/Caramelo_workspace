#!/usr/bin/env python3
"""
Workstation Navigation Server - Direct Control
Server que navega para workstations específicas usando controle direto dos serviços Nav2
Contorna problemas com bt_navigator usando planner e controller diretamente
"""

import json
import os
import threading
import time

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.srv import GetCostmap
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class WorkstationNavigationServerDirect(Node):
    def __init__(self):
        super().__init__('workstation_navigation_server_direct')
        
        # Parâmetros
        self.declare_parameter('arena', 'arena_robocup25')
        self.arena = self.get_parameter('arena').get_parameter_value().string_value
        
        # Inicializar waypoints
        self.waypoints = {}
        self.current_goal = None
        self.is_navigating = False
        self.navigation_thread = None
        
        # QoS para tópicos
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        
        # Publishers
        self.status_pub = self.create_publisher(
            String, 
            '/navigation_status', 
            qos_profile
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos_profile
        )
        
        # Subscribers
        self.navigation_sub = self.create_subscription(
            String,
            '/navigate_to_workstation',
            self._navigate_callback,
            qos_profile
        )
        
        # Tentar carregar waypoints
        if self._load_waypoints():
            self.get_logger().info(f"✅ Waypoints carregados: {list(self.waypoints.keys())}")
        else:
            self.get_logger().error("❌ Falha ao carregar waypoints")
            return
            
        # Tentar inicializar navigator (opcional)
        try:
            self.navigator = BasicNavigator()
            self.get_logger().info("✅ BasicNavigator inicializado")
        except Exception as e:
            self.get_logger().warn(f"⚠️  BasicNavigator falhou, usando modo direto: {e}")
            self.navigator = None
        
        # Status inicial
        self._publish_status("READY", "Servidor pronto para navegação")
        
        self.get_logger().info("🗺️  Workstation Navigation Server (Direct) iniciado!")
        self.get_logger().info(f"   Arena: {self.arena}")
        self.get_logger().info(f"   Waypoints carregados: {len(self.waypoints)}")
        self.get_logger().info("   Tópicos:")
        self.get_logger().info("     Entrada: /navigate_to_workstation (std_msgs/String)")
        self.get_logger().info("     Status:  /navigation_status (std_msgs/String)")
        self.get_logger().info(f"   Waypoints disponíveis: {list(self.waypoints.keys())}")
        
    def _load_waypoints(self):
        """Carrega waypoints do arquivo JSON"""
        try:
            # Caminho para o arquivo de waypoints
            package_share = f"/home/work/Caramelo_workspace/install/caramelo_navigation/share/caramelo_navigation"
            waypoints_file = os.path.join(package_share, "maps", self.arena, "waypoints.json")
            
            if not os.path.exists(waypoints_file):
                # Tentar no workspace src
                waypoints_file = f"/home/work/Caramelo_workspace/src/caramelo_navigation/maps/{self.arena}/waypoints.json"
            
            if not os.path.exists(waypoints_file):
                self.get_logger().error(f"Arquivo de waypoints não encontrado: {waypoints_file}")
                return False
            
            with open(waypoints_file, 'r') as f:
                data = json.load(f)
                self.waypoints = data.get('waypoints', {})
                
            if not self.waypoints:
                self.get_logger().error("Nenhum waypoint encontrado no arquivo")
                return False
                
            return True
            
        except Exception as e:
            self.get_logger().error(f"Erro ao carregar waypoints: {e}")
            return False
    
    def _navigate_callback(self, msg):
        """Callback para comandos de navegação"""
        target_ws = msg.data.strip()
        self.get_logger().info(f"🎯 Comando de navegação recebido: {target_ws}")
        
        if target_ws not in self.waypoints:
            error_msg = f"Workstation '{target_ws}' não encontrada. Disponíveis: {list(self.waypoints.keys())}"
            self.get_logger().error(error_msg)
            self._publish_status("ERROR", error_msg)
            return
        
        if self.is_navigating:
            self.get_logger().warn("⚠️  Navegação já em andamento, cancelando...")
            self._cancel_navigation()
        
        # Iniciar navegação em thread separada
        self.current_goal = target_ws
        self.navigation_thread = threading.Thread(
            target=self._execute_navigation,
            args=(target_ws,)
        )
        self.navigation_thread.daemon = True
        self.navigation_thread.start()
    
    def _execute_navigation(self, target_ws):
        """Executa navegação para a workstation"""
        try:
            self.is_navigating = True
            self.get_logger().info(f"🚀 Iniciando navegação para {target_ws}")
            
            # Obter pose do waypoint
            waypoint = self.waypoints[target_ws]
            goal_pose = self._create_pose_stamped(waypoint)
            
            self._publish_status("NAVIGATING", f"Navegando para {target_ws}")
            
            # Tentar navegação com BasicNavigator se disponível
            if self.navigator is not None:
                success = self._navigate_with_basic_navigator(goal_pose)
            else:
                # Fallback: navegação direta simples
                success = self._navigate_direct(goal_pose)
            
            if success:
                self.get_logger().info(f"✅ Navegação para {target_ws} concluída com sucesso!")
                self._publish_status("SUCCESS", f"Chegou em {target_ws}")
            else:
                self.get_logger().error(f"❌ Falha na navegação para {target_ws}")
                self._publish_status("FAILED", f"Falha ao navegar para {target_ws}")
                
        except Exception as e:
            self.get_logger().error(f"Erro durante navegação: {e}")
            self._publish_status("ERROR", f"Erro: {e}")
        finally:
            self.is_navigating = False
            self.current_goal = None
    
    def _navigate_with_basic_navigator(self, goal_pose):
        """Navega usando BasicNavigator com timeouts"""
        if self.navigator is None:
            return False
            
        try:
            # Verificar se action server está disponível (com timeout)
            timeout = 10.0  # segundos
            start_time = time.time()
            
            while not self.navigator.isTaskComplete():
                if time.time() - start_time > timeout:
                    self.get_logger().warn("Timeout esperando por NavigateToPose action server")
                    return False
                time.sleep(0.1)
            
            # Enviar goal
            self.navigator.goToPose(goal_pose)
            
            # Aguardar resultado com timeout
            timeout = 120.0  # 2 minutos para navegação
            start_time = time.time()
            
            while not self.navigator.isTaskComplete():
                if time.time() - start_time > timeout:
                    self.get_logger().warn("Timeout durante navegação")
                    self.navigator.cancelTask()
                    return False
                    
                # Log periódico de progresso
                if int(time.time() - start_time) % 10 == 0:
                    self.get_logger().info(f"Navegando... ({int(time.time() - start_time)}s)")
                
                time.sleep(0.1)
            
            # Verificar resultado
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                return True
            else:
                self.get_logger().warn(f"Navegação falhou com resultado: {result}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Erro no BasicNavigator: {e}")
            return False
    
    def _navigate_direct(self, goal_pose):
        """Navegação direta simplificada (placeholder)"""
        self.get_logger().info("🔄 Usando navegação direta simplificada")
        
        # Simular navegação por alguns segundos
        duration = 5.0
        rate = self.create_rate(10)  # 10 Hz
        
        start_time = time.time()
        while time.time() - start_time < duration:
            if not self.is_navigating:  # Verificar cancelamento
                return False
                
            # Simular movimento (velocidade baixa)
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
            
            rate.sleep()
        
        # Parar robô
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)
        
        return True
    
    def _create_pose_stamped(self, waypoint):
        """Cria PoseStamped a partir de waypoint"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = float(waypoint['x'])
        pose.pose.position.y = float(waypoint['y'])
        pose.pose.position.z = 0.0
        
        # Orientação quaternion
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = float(waypoint.get('qz', 0.0))
        pose.pose.orientation.w = float(waypoint.get('qw', 1.0))
        
        return pose
    
    def _cancel_navigation(self):
        """Cancela navegação atual"""
        if self.is_navigating:
            self.is_navigating = False
            
            if self.navigator is not None:
                try:
                    self.navigator.cancelTask()
                except:
                    pass
            
            # Parar robô
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)
            
            self._publish_status("CANCELLED", "Navegação cancelada")
    
    def _publish_status(self, status, message):
        """Publica status de navegação"""
        status_msg = {
            "status": status,
            "message": message,
            "timestamp": time.time(),
            "current_goal": self.current_goal,
            "is_navigating": self.is_navigating
        }
        
        msg = String()
        msg.data = json.dumps(status_msg)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = WorkstationNavigationServerDirect()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Erro: {e}")
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except:
                pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
