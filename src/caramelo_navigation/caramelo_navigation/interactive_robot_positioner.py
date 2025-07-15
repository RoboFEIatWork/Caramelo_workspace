#!/usr/bin/env python3
"""
Interactive Robot Positioner - Posiciona robô virtual no mapa para criar waypoints
"""

import json
import math
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import (PointStamped, PoseStamped,
                               PoseWithCovarianceStamped, TransformStamped)
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


class InteractiveRobotPositioner(Node):
    def __init__(self):
        super().__init__('interactive_robot_positioner')
        
        # Transform broadcaster para posicionar o robô
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Posição atual do robô virtual
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # Contador de waypoints
        self.waypoint_count = 0
        
        # Declarar parâmetro para o arquivo de mapa  
        self.declare_parameter('map_file', '')
        self.declare_parameter('map_folder', '')
        
        map_file_path = self.get_parameter('map_file').get_parameter_value().string_value
        map_folder_path = self.get_parameter('map_folder').get_parameter_value().string_value
        
        if map_file_path:
            # Extrair diretório do mapa e definir arquivo de workstations
            map_dir = os.path.dirname(map_file_path)
            self.waypoints_file = os.path.join(map_dir, 'workstations.json')
            self.arena_name = os.path.basename(map_dir)
            self.get_logger().info(f"🗺️ Arena: {self.arena_name}")
            self.get_logger().info(f"📁 Workstations serão salvas em: {self.waypoints_file}")
        elif map_folder_path:
            # Usar map_folder diretamente
            self.waypoints_file = os.path.join(map_folder_path, 'workstations.json')
            self.arena_name = os.path.basename(map_folder_path.rstrip('/'))
            self.get_logger().info(f"�️ Arena: {self.arena_name}")
            self.get_logger().info(f"�📁 Workstations serão salvas em: {self.waypoints_file}")
        else:
            # Fallback para pasta source padrão
            self.source_config_dir = '/home/work/Caramelo_workspace/src/caramelo_navigation/config'
            self.waypoints_file = os.path.join(self.source_config_dir, 'workstations.json')
            self.arena_name = "default"
            self.get_logger().warn(f"⚠️ Parâmetros map_file/map_folder não fornecidos. Usando: {self.waypoints_file}")
            
        self.get_logger().info(f"📁 Waypoints salvos em: {self.waypoints_file}")
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_callback,
            10
        )
        
        # Subscriber para goal pose (2D Nav Goal) - salva waypoint
        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10
        )
        
        # Subscriber para clicked point (Publish Point) - salva waypoint simples
        self.clicked_point_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10
        )
        
        # Publisher para comandos de teclado (opcional)
        self.command_pub = self.create_publisher(String, '/robot_command', 10)
        
        # Timer para publicar TF do robô
        self.tf_timer = self.create_timer(0.1, self.publish_robot_tf)
        
        # Carregar waypoints existentes
        self.load_existing_waypoints()
        
        self.get_logger().info("🤖 Interactive Robot Positioner iniciado!")
        self.get_logger().info("📍 COMO USAR PARA WORKSTATIONS:")
        self.get_logger().info("   1. Use '2D Pose Estimate' para POSICIONAR o robô na pose de docking")
        self.get_logger().info("   2. Use '2D Nav Goal' para SALVAR waypoint de workstation (com orientação)")
        self.get_logger().info("   3. Ou use 'Publish Point' para SALVAR waypoint (orientação padrão 0°)")
        self.get_logger().info("   4. Ajuste a posição/orientação até ficar correta para docking")
        self.get_logger().info("   5. Salve a workstation!")
        self.get_logger().info(f"🏭 Arena: {self.arena_name}")
        self.get_logger().info(f"📁 Workstations: {self.waypoints_file}")
        self.get_logger().info("🎯 Formato otimizado para competição RoboCup@Work")
        
    def initial_pose_callback(self, msg):
        """Move o robô virtual para a posição clicada (2D Pose Estimate)"""
        # Atualizar posição do robô
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Calcular yaw da orientação
        quat = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2.0 * (quat.w * quat.z + quat.x * quat.y),
            1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        )
        
        self.get_logger().info(f"🤖 Robô movido para: ({self.robot_x:.2f}, {self.robot_y:.2f}) @ {self.robot_yaw:.2f} rad")
        self.get_logger().info("💡 Ajuste a posição se necessário, depois use '2D Nav Goal' para salvar!")
        
    def goal_pose_callback(self, msg):
        """Salva waypoint na posição atual do robô (2D Nav Goal)"""
        # Usar nomenclatura WS01, WS02, etc.
        waypoint_name = f"WS{self.waypoint_count + 1:02d}"
        
        # Criar waypoint SIMPLIFICADO para workstation docking
        # Formato otimizado para competição RoboCup@Work
        waypoint = {
            "name": waypoint_name,
            "x": round(self.robot_x, 3),
            "y": round(self.robot_y, 3), 
            "theta": round(math.degrees(self.robot_yaw), 1),
            "type": "workstation_docking"
        }
        
        # Salvar waypoint
        self.save_workstation_waypoint(waypoint)
        self.waypoint_count += 1
        
        self.get_logger().info(f"✅ Waypoint {waypoint_name} salvo!")
        self.get_logger().info(f"📍 Posição de docking: ({self.robot_x:.3f}, {self.robot_y:.3f}) @ {math.degrees(self.robot_yaw):.1f}°")
        self.get_logger().info(f"📊 Total de workstations: {self.count_ws_waypoints()}")
        self.get_logger().info(f"🎯 Formato otimizado para competição RoboCup@Work")
        
    def clicked_point_callback(self, msg):
        """Salva waypoint onde foi clicado (Publish Point) - sem orientação específica"""
        # Usar nomenclatura WS01, WS02, etc.
        waypoint_name = f"WS{self.waypoint_count + 1:02d}"
        
        # Usar a posição clicada
        x = msg.point.x
        y = msg.point.y
        
        # Orientação padrão (pode ser editada depois)
        yaw_degrees = 0.0  # Orientação padrão em graus
        
        # Criar waypoint SIMPLIFICADO para workstation docking
        waypoint = {
            "name": waypoint_name,
            "x": round(x, 3),
            "y": round(y, 3),
            "theta": yaw_degrees,
            "type": "workstation_docking"
        }
        
        # Salvar waypoint
        self.save_workstation_waypoint(waypoint)
        self.waypoint_count += 1
        
        self.get_logger().info(f"✅ Waypoint {waypoint_name} salvo via Publish Point!")
        self.get_logger().info(f"📍 Posição de docking: ({x:.3f}, {y:.3f}) @ {yaw_degrees:.1f}° (orientação padrão)")
        self.get_logger().info(f"📊 Total de workstations: {self.count_ws_waypoints()}")
        
        # Opcional: mover o robô virtual para esta posição
        self.robot_x = x
        self.robot_y = y
        self.robot_yaw = math.radians(yaw_degrees)
        
    def publish_robot_tf(self):
        """Publica transform do robô virtual"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # Posição
        t.transform.translation.x = self.robot_x
        t.transform.translation.y = self.robot_y
        t.transform.translation.z = 0.0
        
        # Orientação (quaternion do yaw)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.robot_yaw / 2.0)
        t.transform.rotation.w = math.cos(self.robot_yaw / 2.0)
        
        self.tf_broadcaster.sendTransform(t)
        
    def save_workstation_waypoint(self, waypoint):
        """Salva waypoint de workstation no formato simplificado para competição"""
        try:
            # Tentar carregar arquivo existente
            if os.path.exists(self.waypoints_file):
                with open(self.waypoints_file, 'r') as f:
                    data = json.load(f)
            else:
                # Criar estrutura inicial
                data = {"workstations": []}
            
            # Garantir que existe a seção workstations
            if "workstations" not in data:
                data["workstations"] = []
                
            # Adicionar nova workstation
            data["workstations"].append(waypoint)
            
            # Salvar arquivo na pasta do mapa/arena específica
            os.makedirs(os.path.dirname(self.waypoints_file), exist_ok=True)
            with open(self.waypoints_file, 'w') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            
            self.get_logger().info(f"💾 Workstation '{waypoint['name']}' salva em {self.waypoints_file}")
            self.get_logger().info(f"🎯 Formato otimizado para RoboCup@Work: x={waypoint['x']}, y={waypoint['y']}, θ={waypoint['theta']}°")
            
        except Exception as e:
            self.get_logger().error(f"❌ Erro ao salvar workstation: {e}")
    
    def save_waypoint(self, waypoint):
        """MÉTODO LEGADO - mantido para compatibilidade se necessário"""
        try:
            with open(self.waypoints_file, 'r') as f:
                data = json.load(f)
        except:
            data = {"frame_id": "map", "waypoints": []}
        
        if "waypoints" not in data:
            data["waypoints"] = []
            
        data["waypoints"].append(waypoint)
        
        # Salvar arquivo
        os.makedirs(os.path.dirname(self.waypoints_file), exist_ok=True)
        with open(self.waypoints_file, 'w') as f:
            json.dump(data, f, indent=2)
        
        self.get_logger().info(f"💾 Waypoint '{waypoint['name']}' salvo em {self.waypoints_file}")
            
    def count_ws_waypoints(self):
        """Conta quantos waypoints WS existem no arquivo (formato novo e legado)"""
        if os.path.exists(self.waypoints_file):
            try:
                with open(self.waypoints_file, 'r') as f:
                    data = json.load(f)
                    
                ws_count = 0
                
                # Formato novo: workstations
                if "workstations" in data:
                    for ws in data["workstations"]:
                        if ws.get("name", "").startswith("WS"):
                            ws_count += 1
                
                # Formato legado: waypoints 
                elif "waypoints" in data:
                    for wp in data["waypoints"]:
                        if wp.get("name", "").startswith("WS"):
                            ws_count += 1
                            
                return ws_count
            except:
                pass
        return 0
            
    def load_existing_waypoints(self):
        """Carrega waypoints existentes e conta apenas os WS (formato novo e legado)"""
        if os.path.exists(self.waypoints_file):
            try:
                with open(self.waypoints_file, 'r') as f:
                    data = json.load(f)
                    
                ws_count = 0
                total_count = 0
                
                # Formato novo: workstations
                if "workstations" in data:
                    total_count = len(data["workstations"])
                    for ws in data["workstations"]:
                        if ws.get("name", "").startswith("WS"):
                            ws_count += 1
                    self.get_logger().info(f"📍 Carregadas {total_count} workstations ({ws_count} WS) - formato novo")
                
                # Formato legado: waypoints 
                elif "waypoints" in data:
                    total_count = len(data["waypoints"])
                    for wp in data["waypoints"]:
                        if wp.get("name", "").startswith("WS"):
                            ws_count += 1
                    self.get_logger().info(f"📍 Carregados {total_count} waypoints ({ws_count} WS) - formato legado")
                
                self.waypoint_count = ws_count
                
            except Exception as e:
                self.get_logger().warn(f"⚠️ Erro ao carregar waypoints existentes: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveRobotPositioner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
