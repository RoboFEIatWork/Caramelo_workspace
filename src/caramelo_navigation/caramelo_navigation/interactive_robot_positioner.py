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
from std_msgs.msg import ColorRGBA, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


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
        
        # Publisher para mostrar robot radius e inflation no RViz
        self.robot_viz_pub = self.create_publisher(MarkerArray, '/robot_visualization', 10)
        
        # Timer para publicar TF do robô
        self.tf_timer = self.create_timer(0.1, self.publish_robot_tf)
        
        # Timer para publicar visualização do robô
        self.viz_timer = self.create_timer(0.5, self.publish_robot_visualization)
        
        # Carregar waypoints existentes
        self.load_existing_waypoints()
        
        self.get_logger().info("🤖 Interactive Robot Positioner iniciado!")
        self.get_logger().info("📍 COMO USAR PARA WAYPOINTS:")
        self.get_logger().info("   1. Use '2D Pose Estimate' para POSICIONAR o robô na pose desejada")
        self.get_logger().info("   2. Use '2D Nav Goal' para SALVAR waypoint (sequência automática)")
        self.get_logger().info("   3. Ou use 'Publish Point' para SALVAR waypoint (orientação padrão 0°)")
        self.get_logger().info("   4. SEQUÊNCIA AUTOMÁTICA:")
        self.get_logger().info("      - 1º waypoint → START (ponto de início)")
        self.get_logger().info("      - 2º waypoint → FINISH (ponto final)")
        self.get_logger().info("      - 3º+ waypoints → WS01, WS02, WS03... (workstations)")
        self.get_logger().info("   5. Os waypoints são nomeados automaticamente!")
        self.get_logger().info(f"🏭 Arena: {self.arena_name}")
        self.get_logger().info(f"📁 Waypoints: {self.waypoints_file}")
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
        # Determinar tipo de waypoint baseado na sequência
        waypoint_name, waypoint_type = self.get_next_waypoint_info()
        
        # Criar waypoint otimizado baseado no tipo
        waypoint = {
            "name": waypoint_name,
            "x": round(self.robot_x, 3),
            "y": round(self.robot_y, 3), 
            "theta": round(math.degrees(self.robot_yaw), 1),
            "type": waypoint_type
        }
        
        # Salvar waypoint
        self.save_universal_waypoint(waypoint)
        
        self.get_logger().info(f"✅ Waypoint {waypoint_name} ({waypoint_type}) salvo!")
        self.get_logger().info(f"📍 Posição: ({self.robot_x:.3f}, {self.robot_y:.3f}) @ {math.degrees(self.robot_yaw):.1f}°")
        self.get_logger().info(f"📊 Total: {self.count_total_waypoints()} waypoints")
        
        # Atualizar contador se for WS
        if waypoint_name.startswith("WS"):
            self.waypoint_count += 1
        
    def clicked_point_callback(self, msg):
        """Salva waypoint onde foi clicado (Publish Point) - sem orientação específica"""
        # Determinar tipo de waypoint baseado na sequência
        waypoint_name, waypoint_type = self.get_next_waypoint_info()
        
        # Usar a posição clicada
        x = msg.point.x
        y = msg.point.y
        
        # Orientação padrão (pode ser editada depois)
        yaw_degrees = 0.0  # Orientação padrão em graus
        
        # Criar waypoint otimizado baseado no tipo
        waypoint = {
            "name": waypoint_name,
            "x": round(x, 3),
            "y": round(y, 3),
            "theta": yaw_degrees,
            "type": waypoint_type
        }
        
        # Salvar waypoint
        self.save_universal_waypoint(waypoint)
        
        self.get_logger().info(f"✅ Waypoint {waypoint_name} ({waypoint_type}) salvo via Publish Point!")
        self.get_logger().info(f"📍 Posição: ({x:.3f}, {y:.3f}) @ {yaw_degrees:.1f}° (orientação padrão)")
        self.get_logger().info(f"📊 Total: {self.count_total_waypoints()} waypoints")
        
        # Atualizar contador se for WS
        if waypoint_name.startswith("WS"):
            self.waypoint_count += 1
        
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
        
    def publish_robot_visualization(self):
        """Publica marcadores para mostrar robot radius e inflation no RViz"""
        marker_array = MarkerArray()
        
        # Marcador 1: Robot radius (círculo sólido vermelho)
        robot_marker = Marker()
        robot_marker.header.frame_id = "map"
        robot_marker.header.stamp = self.get_clock().now().to_msg()
        robot_marker.ns = "robot_footprint"
        robot_marker.id = 0
        robot_marker.type = Marker.CYLINDER
        robot_marker.action = Marker.ADD
        robot_marker.pose.position.x = self.robot_x
        robot_marker.pose.position.y = self.robot_y
        robot_marker.pose.position.z = 0.05  # Ligeiramente acima do chão
        robot_marker.pose.orientation.w = 1.0
        robot_marker.scale.x = 0.74  # Diâmetro = 2 * 0.37m radius
        robot_marker.scale.y = 0.74
        robot_marker.scale.z = 0.1   # Altura do cilindro
        robot_marker.color.r = 1.0   # Vermelho
        robot_marker.color.g = 0.0
        robot_marker.color.b = 0.0
        robot_marker.color.a = 0.7   # Semi-transparente
        robot_marker.lifetime.sec = 1
        
        # Marcador 2: Inflation layer (círculo transparente azul)
        inflation_marker = Marker()
        inflation_marker.header.frame_id = "map"
        inflation_marker.header.stamp = self.get_clock().now().to_msg()
        inflation_marker.ns = "inflation_layer"
        inflation_marker.id = 1
        inflation_marker.type = Marker.CYLINDER
        inflation_marker.action = Marker.ADD
        inflation_marker.pose.position.x = self.robot_x
        inflation_marker.pose.position.y = self.robot_y
        inflation_marker.pose.position.z = 0.02  # Abaixo do robot
        inflation_marker.pose.orientation.w = 1.0
        inflation_marker.scale.x = 0.84  # Robot radius + inflation (0.37 + 0.05) * 2
        inflation_marker.scale.y = 0.84
        inflation_marker.scale.z = 0.05
        inflation_marker.color.r = 0.0
        inflation_marker.color.g = 0.5  # Azul escuro
        inflation_marker.color.b = 1.0
        inflation_marker.color.a = 0.3   # Muito transparente
        inflation_marker.lifetime.sec = 1
        
        # Marcador 3: Seta indicando orientação
        arrow_marker = Marker()
        arrow_marker.header.frame_id = "map"
        arrow_marker.header.stamp = self.get_clock().now().to_msg()
        arrow_marker.ns = "robot_orientation"
        arrow_marker.id = 2
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD
        arrow_marker.pose.position.x = self.robot_x
        arrow_marker.pose.position.y = self.robot_y
        arrow_marker.pose.position.z = 0.1
        arrow_marker.pose.orientation.x = 0.0
        arrow_marker.pose.orientation.y = 0.0
        arrow_marker.pose.orientation.z = math.sin(self.robot_yaw / 2.0)
        arrow_marker.pose.orientation.w = math.cos(self.robot_yaw / 2.0)
        arrow_marker.scale.x = 0.5   # Comprimento da seta
        arrow_marker.scale.y = 0.05  # Largura da seta
        arrow_marker.scale.z = 0.05  # Altura da seta
        arrow_marker.color.r = 0.0
        arrow_marker.color.g = 1.0   # Verde
        arrow_marker.color.b = 0.0
        arrow_marker.color.a = 0.9
        arrow_marker.lifetime.sec = 1
        
        # Adicionar marcadores ao array
        marker_array.markers = [robot_marker, inflation_marker, arrow_marker]
        
        self.robot_viz_pub.publish(marker_array)
        
    def save_workstation_waypoint(self, waypoint):
        """Salva waypoint de workstation no formato simplificado para competição"""
        try:
            # Tentar carregar arquivo existente
            data = {"workstations": []}
            if os.path.exists(self.waypoints_file):
                try:
                    with open(self.waypoints_file, 'r') as f:
                        content = f.read().strip()
                        if content:  # Se arquivo não está vazio
                            data = json.loads(content)
                        else:
                            self.get_logger().warn(f"📄 Arquivo {self.waypoints_file} estava vazio - criando novo")
                except json.JSONDecodeError as json_error:
                    self.get_logger().warn(f"🔧 Arquivo JSON corrompido, criando novo: {json_error}")
                    data = {"workstations": []}
            else:
                self.get_logger().info(f"📁 Criando novo arquivo: {self.waypoints_file}")
            
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
            import traceback
            self.get_logger().error(f"📋 Traceback: {traceback.format_exc()}")
    
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
                    content = f.read().strip()
                    if not content:  # Arquivo vazio
                        self.get_logger().info(f"📄 Arquivo {self.waypoints_file} está vazio - começando do zero")
                        self.waypoint_count = 0
                        return
                    
                    data = json.loads(content)
                    
                ws_count = 0
                total_count = 0
                special_count = 0  # START, FINISH
                
                # Formato universal: waypoints
                if "waypoints" in data:
                    total_count = len(data["waypoints"])
                    for wp in data["waypoints"]:
                        name = wp.get("name", "")
                        if name.startswith("WS"):
                            ws_count += 1
                        elif name in ["START", "FINISH"]:
                            special_count += 1
                    self.get_logger().info(f"📍 Carregados {total_count} waypoints ({ws_count} WS + {special_count} especiais)")
                
                # Formato legado: workstations (manter compatibilidade)
                elif "workstations" in data:
                    total_count = len(data["workstations"])
                    for ws in data["workstations"]:
                        name = ws.get("name", "")
                        if name.startswith("WS"):
                            ws_count += 1
                        elif name in ["START", "FINISH"]:
                            special_count += 1
                    self.get_logger().info(f"📍 Carregadas {total_count} workstations ({ws_count} WS + {special_count} especiais) - formato legado")
                
                self.waypoint_count = ws_count
                
            except json.JSONDecodeError as e:
                self.get_logger().warn(f"🔧 Arquivo JSON corrompido, será recriado: {e}")
                self.waypoint_count = 0
            except Exception as e:
                self.get_logger().warn(f"⚠️ Erro ao carregar waypoints existentes: {e}")
                self.waypoint_count = 0

    def get_next_waypoint_info(self):
        """Determina o próximo nome e tipo de waypoint baseado na sequência lógica"""
        existing_waypoints = self.get_existing_waypoint_names()
        
        # Nova sequência lógica: START -> FINISH -> WS01, WS02, WS03...
        if "START" not in existing_waypoints:
            return "START", "mission_start"
        elif "FINISH" not in existing_waypoints:
            # Segundo waypoint sempre é FINISH
            return "FINISH", "mission_finish"
        else:
            # A partir do terceiro waypoint, criar workstations
            next_ws = f"WS{self.waypoint_count + 1:02d}"
            return next_ws, "workstation_docking"
    
    def get_existing_waypoint_names(self):
        """Retorna lista de nomes de waypoints já existentes"""
        names = []
        if os.path.exists(self.waypoints_file):
            try:
                with open(self.waypoints_file, 'r') as f:
                    content = f.read().strip()
                    if content:
                        data = json.loads(content)
                        if "workstations" in data:
                            names = [wp.get("name", "") for wp in data["workstations"]]
                        elif "waypoints" in data:
                            names = [wp.get("name", "") for wp in data["waypoints"]]
            except:
                pass
        return names
    
    def save_universal_waypoint(self, waypoint):
        """Salva waypoint universal (START, FINISH, WS) no formato otimizado"""
        try:
            # Tentar carregar arquivo existente
            data = {"waypoints": []}  # Usar formato universal
            if os.path.exists(self.waypoints_file):
                try:
                    with open(self.waypoints_file, 'r') as f:
                        content = f.read().strip()
                        if content:
                            data = json.loads(content)
                        else:
                            self.get_logger().warn(f"📄 Arquivo {self.waypoints_file} estava vazio - criando novo")
                except json.JSONDecodeError as json_error:
                    self.get_logger().warn(f"🔧 Arquivo JSON corrompido, criando novo: {json_error}")
                    data = {"waypoints": []}
            else:
                self.get_logger().info(f"📁 Criando novo arquivo: {self.waypoints_file}")
            
            # Garantir que existe a seção waypoints
            if "waypoints" not in data:
                data["waypoints"] = []
                
            # Adicionar novo waypoint
            data["waypoints"].append(waypoint)
            
            # Salvar arquivo na pasta do mapa/arena específica
            os.makedirs(os.path.dirname(self.waypoints_file), exist_ok=True)
            with open(self.waypoints_file, 'w') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            
            self.get_logger().info(f"💾 Waypoint '{waypoint['name']}' salvo em {self.waypoints_file}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Erro ao salvar waypoint: {e}")
            import traceback
            self.get_logger().error(f"📋 Traceback: {traceback.format_exc()}")
    
    def count_total_waypoints(self):
        """Conta todos os waypoints no arquivo"""
        if os.path.exists(self.waypoints_file):
            try:
                with open(self.waypoints_file, 'r') as f:
                    content = f.read().strip()
                    if content:
                        data = json.loads(content)
                        if "waypoints" in data:
                            return len(data["waypoints"])
                        elif "workstations" in data:
                            return len(data["workstations"])
            except:
                pass
        return 0


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
