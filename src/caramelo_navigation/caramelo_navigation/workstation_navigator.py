#!/usr/bin/env python3
"""
WORKSTATION NAVIGATOR - Navegação precisa para competição RoboCup@Work

Sistema especializado para navegação até workstations (WS) com:
- Posicionamento preciso para manipulação
- Orientação correta em direção à mesa
- Múltiplas tentativas de aproximação
- Validação de posição final

Autor: GitHub Copilot  
Data: 2025-07-14
"""

import json
import math
import os
import time
from enum import Enum
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class WSNavigationState(Enum):
    """Estados da navegação para workstation"""
    IDLE = "idle"
    NAVIGATING_TO_WS = "navigating_to_ws"
    FINE_POSITIONING = "fine_positioning"
    VALIDATING_POSITION = "validating_position"
    WS_REACHED = "ws_reached"
    WS_FAILED = "ws_failed"


class WorkstationNavigator(Node):
    """
    Navegador especializado para workstations da competição RoboCup@Work
    """
    
    def __init__(self):
        super().__init__('workstation_navigator')
        
        # Configurações
        self.ws_tolerance_xy = 0.08      # 8cm de tolerância posicional
        self.ws_tolerance_yaw = 0.087    # 5° de tolerância angular  
        self.approach_distance = 0.30    # Distância de aproximação inicial (30cm)
        self.fine_position_distance = 0.15  # Distância final de posicionamento (15cm)
        self.max_attempts = 3            # Máximo 3 tentativas por WS
        self.validation_time = 2.0       # 2s para validar posição estável
        
        # Estado
        self.state = WSNavigationState.IDLE
        self.current_ws = None
        self.current_attempt = 0
        self.ws_data = {}
        self.validation_start_time = None
        
        # Nav2
        self.navigator = BasicNavigator()
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/ws_navigation_status', 10)
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
            
        # Estado atual
        self.current_pose = None
        self.current_scan = None
        
        self.get_logger().info("Workstation Navigator iniciado!")
        
    def load_workstations(self, ws_file_path: str) -> bool:
        """
        Carrega workstations do arquivo JSON (suporta formato novo e legado)
        
        Formato NOVO (lista):
        {
          "workstations": [
            {"name": "WS01", "x": 2.5, "y": 1.2, "theta": 0.0, "type": "workstation_docking"},
            {"name": "WS02", "x": 4.8, "y": 2.4, "theta": 90, "type": "workstation_docking"}
          ]
        }
        
        Formato LEGADO (dicionário):
        {
          "workstations": {
            "WS01": {"x": 2.5, "y": 1.2, "theta": 0.0, "approach_angle": 180},
            "WS02": {"x": 4.8, "y": 2.4, "theta": 90, "approach_angle": 270}
          }
        }
        """
        try:
            if not os.path.exists(ws_file_path):
                self.get_logger().error(f"❌ Arquivo de workstations não encontrado: {ws_file_path}")
                return False
                
            with open(ws_file_path, 'r') as f:
                data = json.load(f)
                
            workstations_raw = data.get('workstations', {})
            
            # Detectar formato e processar
            if isinstance(workstations_raw, list):
                # Formato NOVO - lista de workstations
                self.ws_data = {}
                for ws in workstations_raw:
                    if 'name' in ws and 'x' in ws and 'y' in ws and 'theta' in ws:
                        ws_name = ws['name']
                        self.ws_data[ws_name] = {
                            'x': ws['x'],
                            'y': ws['y'], 
                            'theta': ws['theta'],
                            'approach_angle': ws.get('approach_angle', ws['theta'] + 180)  # Default: oposto
                        }
                self.get_logger().info(f"✅ Formato NOVO detectado - lista de workstations")
                
            elif isinstance(workstations_raw, dict):
                # Formato LEGADO - dicionário de workstations
                self.ws_data = workstations_raw
                self.get_logger().info(f"✅ Formato LEGADO detectado - dicionário de workstations")
                
            else:
                self.get_logger().error("❌ Formato de workstations inválido")
                return False
            
            if not self.ws_data:
                self.get_logger().error("❌ Nenhuma workstation encontrada no arquivo")
                return False
                
            self.get_logger().info(f"✅ Carregadas {len(self.ws_data)} workstations:")
            for ws_name, ws_info in self.ws_data.items():
                self.get_logger().info(f"  📍 {ws_name}: ({ws_info['x']:.2f}, {ws_info['y']:.2f}, {ws_info['theta']:.1f}°)")
                
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Erro ao carregar workstations: {e}")
            return False
    
    def navigate_to_workstation(self, ws_name: str) -> bool:
        """
        Inicia navegação para workstation específica
        """
        if ws_name not in self.ws_data:
            self.get_logger().error(f"❌ Workstation {ws_name} não encontrada!")
            return False
            
        if self.state != WSNavigationState.IDLE:
            self.get_logger().warn(f"⚠️ Navegação em andamento (estado: {self.state.value})")
            return False
            
        self.current_ws = ws_name
        self.current_attempt = 1
        self.state = WSNavigationState.NAVIGATING_TO_WS
        
        # Calcular posição de aproximação
        ws_info = self.ws_data[ws_name]
        approach_pose = self._calculate_approach_pose(ws_info)
        
        self.get_logger().info(f"🎯 Navegando para WS {ws_name} (tentativa {self.current_attempt}/{self.max_attempts})")
        self.get_logger().info(f"📍 Aproximação: ({approach_pose.pose.position.x:.2f}, {approach_pose.pose.position.y:.2f})")
        
        # Navegar usando Nav2
        self.navigator.goToPose(approach_pose)
        
        # Publicar status
        self._publish_status(f"NAVIGATING_TO_WS: {ws_name} (tentativa {self.current_attempt})")
        
        return True
    
    def _calculate_approach_pose(self, ws_info: Dict) -> PoseStamped:
        """
        Calcula pose de aproximação baseada na workstation
        """
        ws_x = ws_info['x']
        ws_y = ws_info['y'] 
        ws_theta = math.radians(ws_info['theta'])
        approach_angle = math.radians(ws_info.get('approach_angle', ws_info['theta'] + 180))
        
        # Calcular posição de aproximação (recuar na direção de approach)
        approach_x = ws_x + self.approach_distance * math.cos(approach_angle)
        approach_y = ws_y + self.approach_distance * math.sin(approach_angle)
        
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = approach_x
        pose.pose.position.y = approach_y
        pose.pose.position.z = 0.0
        
        # Orientação voltada para a WS
        pose.pose.orientation.z = math.sin(ws_theta / 2.0)
        pose.pose.orientation.w = math.cos(ws_theta / 2.0)
        
        return pose
    
    def _calculate_final_pose(self, ws_info: Dict) -> PoseStamped:
        """
        Calcula pose final precisa na workstation
        """
        ws_x = ws_info['x']
        ws_y = ws_info['y'] 
        ws_theta = math.radians(ws_info['theta'])
        approach_angle = math.radians(ws_info.get('approach_angle', ws_info['theta'] + 180))
        
        # Posição final mais próxima da mesa
        final_x = ws_x + self.fine_position_distance * math.cos(approach_angle)
        final_y = ws_y + self.fine_position_distance * math.sin(approach_angle)
        
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = final_x
        pose.pose.position.y = final_y
        pose.pose.position.z = 0.0
        
        # Orientação precisa voltada para a WS
        pose.pose.orientation.z = math.sin(ws_theta / 2.0)
        pose.pose.orientation.w = math.cos(ws_theta / 2.0)
        
        return pose
    
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Callback da pose atual do robô"""
        self.current_pose = msg
        
        # Processar estados da navegação
        if self.state == WSNavigationState.NAVIGATING_TO_WS:
            self._check_navigation_progress()
        elif self.state == WSNavigationState.VALIDATING_POSITION:
            self._validate_final_position()
    
    def scan_callback(self, msg: LaserScan):
        """Callback do scan do LiDAR"""
        self.current_scan = msg
    
    def _check_navigation_progress(self):
        """
        Verifica progresso da navegação Nav2
        """
        if not self.navigator.isTaskComplete():
            return
            
        result = self.navigator.getResult()
        
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"✅ Aproximação à WS {self.current_ws} concluída!")
            self._start_fine_positioning()
        else:
            self.get_logger().warn(f"⚠️ Falha na aproximação à WS {self.current_ws}: {result}")
            self._retry_or_fail()
    
    def _start_fine_positioning(self):
        """
        Inicia posicionamento fino na workstation
        """
        self.state = WSNavigationState.FINE_POSITIONING
        
        ws_info = self.ws_data[self.current_ws]
        final_pose = self._calculate_final_pose(ws_info)
        
        self.get_logger().info(f"🎯 Posicionamento fino na WS {self.current_ws}")
        self.get_logger().info(f"📍 Posição final: ({final_pose.pose.position.x:.2f}, {final_pose.pose.position.y:.2f})")
        
        self.navigator.goToPose(final_pose)
        self._publish_status(f"FINE_POSITIONING: {self.current_ws}")
    
    def _validate_final_position(self):
        """
        Valida se posição final está adequada
        """
        if not self.current_pose:
            return
            
        ws_info = self.ws_data[self.current_ws]
        
        # Calcular erro de posição
        dx = self.current_pose.pose.pose.position.x - ws_info['x']
        dy = self.current_pose.pose.pose.position.y - ws_info['y']
        distance_error = math.sqrt(dx*dx + dy*dy)
        
        # Calcular erro de orientação
        current_yaw = self._get_yaw_from_pose(self.current_pose.pose.pose)
        target_yaw = math.radians(ws_info['theta'])
        yaw_error = abs(self._normalize_angle(current_yaw - target_yaw))
        
        # Verificar tolerâncias
        position_ok = distance_error <= self.ws_tolerance_xy
        orientation_ok = yaw_error <= self.ws_tolerance_yaw
        
        if position_ok and orientation_ok:
            if self.validation_start_time is None:
                self.validation_start_time = time.time()
                self.get_logger().info(f"⏱️ Validando posição na WS {self.current_ws}...")
            elif time.time() - self.validation_start_time >= self.validation_time:
                self._ws_reached_successfully()
        else:
            self.validation_start_time = None
            if not position_ok:
                self.get_logger().warn(f"⚠️ Erro de posição: {distance_error*100:.1f}cm (máx: {self.ws_tolerance_xy*100:.1f}cm)")
            if not orientation_ok:
                self.get_logger().warn(f"⚠️ Erro de orientação: {math.degrees(yaw_error):.1f}° (máx: {math.degrees(self.ws_tolerance_yaw):.1f}°)")
    
    def _ws_reached_successfully(self):
        """
        Workstation alcançada com sucesso
        """
        self.state = WSNavigationState.WS_REACHED
        
        ws_info = self.ws_data[self.current_ws]
        self.get_logger().info(f"🎉 WS {self.current_ws} alcançada com sucesso!")
        self.get_logger().info(f"📍 Posição final: ({ws_info['x']:.2f}, {ws_info['y']:.2f}, {ws_info['theta']:.1f}°)")
        
        self._publish_status(f"WS_REACHED: {self.current_ws} em {self.current_attempt} tentativa(s)")
        
        # Reset para próxima navegação
        self.current_ws = None
        self.current_attempt = 0
        self.validation_start_time = None
        self.state = WSNavigationState.IDLE
    
    def _retry_or_fail(self):
        """
        Tenta novamente ou falha definitivamente
        """
        if self.current_attempt < self.max_attempts:
            self.current_attempt += 1
            self.get_logger().warn(f"🔄 Tentativa {self.current_attempt}/{self.max_attempts} para WS {self.current_ws}")
            
            # Tentar aproximação novamente
            ws_info = self.ws_data[self.current_ws]
            approach_pose = self._calculate_approach_pose(ws_info)
            self.navigator.goToPose(approach_pose)
            
            self.state = WSNavigationState.NAVIGATING_TO_WS
            self._publish_status(f"RETRY_NAVIGATING_TO_WS: {self.current_ws} (tentativa {self.current_attempt})")
        else:
            self.get_logger().error(f"❌ Falha definitiva na WS {self.current_ws} após {self.max_attempts} tentativas")
            
            self.state = WSNavigationState.WS_FAILED
            self._publish_status(f"WS_FAILED: {self.current_ws} após {self.max_attempts} tentativas")
            
            # Reset
            self.current_ws = None
            self.current_attempt = 0
            self.validation_start_time = None
            self.state = WSNavigationState.IDLE
    
    def _get_yaw_from_pose(self, pose) -> float:
        """Extrai yaw da quaternion"""
        return 2 * math.atan2(pose.orientation.z, pose.orientation.w)
    
    def _normalize_angle(self, angle: float) -> float:
        """Normaliza ângulo para [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def _publish_status(self, status: str):
        """Publica status da navegação"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f"📊 Status: {status}")


def main():
    rclpy.init()
    
    navigator = WorkstationNavigator()
    
    # Aguardar Nav2 estar pronto
    navigator.get_logger().info("⏳ Aguardando Nav2 estar pronto...")
    navigator.navigator.waitUntilNav2Active()
    navigator.get_logger().info("✅ Nav2 ativo!")
    
    # Carregar workstations
    ws_file = '/home/work/Caramelo_workspace/maps/arena_fei/workstations.json'
    if navigator.load_workstations(ws_file):
        navigator.get_logger().info("🏭 Sistema de Workstations pronto!")
    else:
        navigator.get_logger().error("❌ Falha ao carregar workstations!")
        return
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info("🛑 Workstation Navigator finalizado pelo usuário")
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
