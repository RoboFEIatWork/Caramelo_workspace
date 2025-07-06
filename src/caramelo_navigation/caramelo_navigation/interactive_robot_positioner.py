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
        
        # Arquivo de waypoints - sempre salvar na pasta source
        try:
            # Tenta encontrar o diretório do pacote instalado
            package_dir = get_package_share_directory('caramelo_navigation')
            self.config_dir = os.path.join(package_dir, 'config')
        except:
            # Se não encontrar, usar a pasta src diretamente
            self.config_dir = '/home/work/Caramelo_workspace/src/caramelo_navigation/config'
        
        # Sempre salvar na pasta source para persistir as alterações
        self.source_config_dir = '/home/work/Caramelo_workspace/src/caramelo_navigation/config'
        self.waypoints_file = os.path.join(self.source_config_dir, 'waypoints.json')
        
        # Subscriber para initial pose (2D Pose Estimate)
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
        self.get_logger().info("📍 COMO USAR:")
        self.get_logger().info("   1. Use '2D Pose Estimate' para MOVER o robô")
        self.get_logger().info("   2. Use '2D Nav Goal' para SALVAR waypoint (com orientação)")
        self.get_logger().info("   3. Ou use 'Publish Point' para SALVAR waypoint (orientação padrão)")
        self.get_logger().info("   4. Ajuste a posição até ficar satisfeito")
        self.get_logger().info("   5. Salve o waypoint!")
        self.get_logger().info(f"📁 Waypoints salvos em: {self.waypoints_file}")
        
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
        
        # Usar a posição atual do robô virtual
        waypoint = {
            "name": waypoint_name,
            "position": {
                "x": self.robot_x,
                "y": self.robot_y,
                "z": 0.0
            },
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": math.sin(self.robot_yaw / 2.0),
                "w": math.cos(self.robot_yaw / 2.0)
            }
        }
        
        # Salvar waypoint
        self.save_waypoint(waypoint)
        self.waypoint_count += 1
        
        self.get_logger().info(f"✅ Waypoint {waypoint_name} salvo!")
        self.get_logger().info(f"📍 Posição: ({self.robot_x:.2f}, {self.robot_y:.2f}) @ {self.robot_yaw:.2f} rad")
        self.get_logger().info(f"📊 Total de waypoints: {self.count_ws_waypoints()}")
        
    def clicked_point_callback(self, msg):
        """Salva waypoint onde foi clicado (Publish Point) - sem orientação específica"""
        # Usar nomenclatura WS01, WS02, etc.
        waypoint_name = f"WS{self.waypoint_count + 1:02d}"
        
        # Usar a posição clicada
        x = msg.point.x
        y = msg.point.y
        
        # Orientação padrão (pode ser editada depois)
        yaw = 0.0  # Orientação padrão
        
        waypoint = {
            "name": waypoint_name,
            "position": {
                "x": x,
                "y": y,
                "z": 0.0
            },
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": math.sin(yaw / 2.0),
                "w": math.cos(yaw / 2.0)
            }
        }
        
        # Salvar waypoint
        self.save_waypoint(waypoint)
        self.waypoint_count += 1
        
        self.get_logger().info(f"✅ Waypoint {waypoint_name} salvo via Publish Point!")
        self.get_logger().info(f"📍 Posição: ({x:.2f}, {y:.2f}) @ {yaw:.2f} rad (orientação padrão)")
        self.get_logger().info(f"📊 Total de waypoints: {self.count_ws_waypoints()}")
        
        # Opcional: mover o robô virtual para esta posição
        self.robot_x = x
        self.robot_y = y
        self.robot_yaw = yaw
        
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
        
    def save_waypoint(self, waypoint):
        """Salva waypoint no arquivo JSON"""
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
            
        # Também salvar na pasta install se existir
        try:
            install_waypoints_file = os.path.join(self.config_dir, 'waypoints.json')
            if install_waypoints_file != self.waypoints_file:
                os.makedirs(os.path.dirname(install_waypoints_file), exist_ok=True)
                with open(install_waypoints_file, 'w') as f:
                    json.dump(data, f, indent=2)
        except:
            pass
            
    def count_ws_waypoints(self):
        """Conta quantos waypoints WS existem no arquivo"""
        if os.path.exists(self.waypoints_file):
            try:
                with open(self.waypoints_file, 'r') as f:
                    data = json.load(f)
                    if "waypoints" in data:
                        ws_count = 0
                        for wp in data["waypoints"]:
                            if wp.get("name", "").startswith("WS"):
                                ws_count += 1
                        return ws_count
            except:
                pass
        return 0
            
    def load_existing_waypoints(self):
        """Carrega waypoints existentes e conta apenas os WS"""
        if os.path.exists(self.waypoints_file):
            try:
                with open(self.waypoints_file, 'r') as f:
                    data = json.load(f)
                    if "waypoints" in data:
                        # Contar apenas waypoints WS para determinar o próximo número
                        ws_count = 0
                        for wp in data["waypoints"]:
                            if wp.get("name", "").startswith("WS"):
                                ws_count += 1
                        self.waypoint_count = ws_count
                        total_waypoints = len(data["waypoints"])
                        self.get_logger().info(f"📍 Carregados {total_waypoints} waypoints totais ({ws_count} WS)")
            except:
                pass


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
