#!/usr/bin/env python3

import json
import os
import time
from enum import Enum
from typing import Any, Dict, List

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty


class MissionState(Enum):
    """Estados da missão autônoma"""
    IDLE = "IDLE"
    LOADING = "LOADING"
    NAVIGATING = "NAVIGATING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    EMERGENCY_STOP = "EMERGENCY_STOP"


class AutonomousWaypointNavigator(Node):
    """
    Navegador autônomo que usa Nav2 para navegar por waypoints em mapa existente.
    
    - Carrega mapa .pgm/.yaml existente
    - Lê waypoints reais do arquivo .json
    - Usa Nav2 para navegação (igual ao goalpose)
    - LIDAR para evitar obstáculos dinâmicos
    - Navegação automática sequencial
    """
    
    def __init__(self):
        super().__init__('autonomous_waypoint_navigator')
        
        # Parâmetros
        self.declare_parameter('map_folder', 'arena_fei')
        self.declare_parameter('loop_mission', False)
        
        self.map_folder = self.get_parameter('map_folder').get_parameter_value().string_value
        self.loop_mission = self.get_parameter('loop_mission').get_parameter_value().bool_value
        
        # Estado da missão
        self.state = MissionState.IDLE
        self.waypoints: List[Dict[str, Any]] = []
        self.current_waypoint_index = 0
        self.mission_start_time = None
        
        # Navegador Nav2 (mesmo sistema do goalpose)
        self.navigator = BasicNavigator()
        
        # Cliente de serviço para localização global
        self.global_localization_client = self.create_client(
            Empty, '/global_localization'
        )
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/mission_status', 10)
        
        # Subscribers para segurança
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        # Estado de segurança LIDAR - parâmetros otimizados para persistência
        self.obstacle_detected = False
        self.min_safe_distance = 0.3  # 30cm para alertas
        self.emergency_stop_distance = 0.10  # REDUZIDO: apenas 10cm para emergência real
        
        # Parâmetros de navegação agressiva
        self.max_retries_per_waypoint = 3  # Máximo 3 tentativas por waypoint
        self.navigation_timeout = 180.0  # 3 minutos por waypoint (aumentado)
        self.retry_delay = 3.0  # Pausa entre tentativas
        
        # Timer será criado apenas quando necessário
        self.mission_timer = None
        
        self.get_logger().info('🤖 Navegador Autônomo iniciado!')
        self.get_logger().info(f'📁 Pasta do mapa: {self.map_folder}')
        self.get_logger().info(f'🔄 Loop da missão: {self.loop_mission}')
        
        # Aguardar Nav2 ficar ativo antes de carregar missão
        self.initialization_timer = self.create_timer(15.0, self.wait_for_nav2_and_load_mission)
        
    def perform_global_localization(self):
        """Executa localização global usando scan matching"""
        self.get_logger().info('🎯 Iniciando localização global com scan matching...')
        
        try:
            # Usar comando do Nav2 para localização global
            self.navigator.clearAllCostmaps()
            time.sleep(2.0)
            
            # Trigger global localization via AMCL service
            self.get_logger().info('🔍 Executando scan matching para encontrar posição do robô...')
            
            # Chamar serviço de localização global
            if self.global_localization_client.wait_for_service(timeout_sec=10.0):
                request = Empty.Request()
                future = self.global_localization_client.call_async(request)
                
                # Aguardar resultado
                rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
                
                if future.result():
                    self.get_logger().info('✅ Serviço de localização global executado!')
                else:
                    self.get_logger().warn('⚠️ Serviço de localização global falhou')
            else:
                self.get_logger().warn('⚠️ Serviço de localização global não disponível')
            
            # Aguardar convergência da localização
            localization_timeout = 30.0
            start_time = time.time()
            
            self.get_logger().info('⏳ Aguardando scan matching convergir (até 30s)...')
            
            # Aguardar um tempo para o AMCL processar
            time.sleep(10.0)
            
            self.get_logger().info('✅ Localização global completa!')
            return True
            
        except Exception as e:
            self.get_logger().error(f'❌ Erro na localização global: {str(e)}')
            return False
        
    def scan_callback(self, msg: LaserScan):
        """Monitora LIDAR para detecção de obstáculos - APENAS setor 90° a 270°"""
        if not msg.ranges:
            return
            
        # Usar apenas setor frontal: 90° a 270° (180° total)
        # 0° = frente, 90° = esquerda, 180° = trás, 270° = direita
        total_points = len(msg.ranges)
        
        # Calcular índices para 90° a 270°
        start_angle = 90   # Graus
        end_angle = 270    # Graus
        
        start_idx = int((start_angle / 360.0) * total_points)
        end_idx = int((end_angle / 360.0) * total_points)
        
        # Extrair apenas o setor desejado (90° a 270°)
        sector_ranges = msg.ranges[start_idx:end_idx]
        
        # Filtrar valores válidos no setor 90°-270°
        valid_ranges = [r for r in sector_ranges if msg.range_min <= r <= msg.range_max]
        
        if valid_ranges:
            min_distance = min(valid_ranges)
            
            # APENAS parada de emergência em casos extremos (muito próximo)
            if min_distance < self.emergency_stop_distance:
                if self.state == MissionState.NAVIGATING and not self.obstacle_detected:
                    self.get_logger().warn(f'� Obstáculo muito próximo a {min_distance:.2f}m - Nav2 desviará')
                    # NÃO cancelar navegação - deixar Nav2 replanejar automaticamente
                    self.publish_status("OBSTACLE_DETECTED", f"Obstáculo próximo: {min_distance:.2f}m - Replanejando rota")
                self.obstacle_detected = True
                    
            # Detecção de obstáculo (apenas alerta, Nav2 continua)
            elif min_distance < self.min_safe_distance:
                if not self.obstacle_detected:
                    self.get_logger().info(f'⚠️ Obstáculo detectado a {min_distance:.2f}m - Nav2 encontrando caminho alternativo')
                self.obstacle_detected = True
            else:
                if self.obstacle_detected:
                    self.get_logger().info('✅ Caminho livre - navegação normal')
                self.obstacle_detected = False
        
    def wait_for_nav2_and_load_mission(self):
        """Aguarda Nav2 ficar ativo antes de carregar missão"""
        # Destruir o timer de inicialização
        if self.initialization_timer:
            self.initialization_timer.destroy()
            self.initialization_timer = None
            
        self.get_logger().info('⏳ Aguardando Nav2 ficar ativo...')
        # Carregar missão após um delay maior para garantir que Nav2 esteja ativo
        self.load_timer = self.create_timer(20.0, self.load_mission)
        
    def load_mission(self):
        """Carrega waypoints do arquivo JSON"""
        # Destruir o timer de carregamento
        if hasattr(self, 'load_timer') and self.load_timer:
            self.load_timer.destroy()
            self.load_timer = None
            
        self.state = MissionState.LOADING
        self.publish_status("LOADING", "Carregando waypoints...")
        
        waypoints_file = f'/home/work/Caramelo_workspace/maps/{self.map_folder}/waypoints_simple.json'
        
        if not os.path.exists(waypoints_file):
            self.get_logger().error(f'❌ Arquivo de waypoints não encontrado: {waypoints_file}')
            self.state = MissionState.FAILED
            self.publish_status("FAILED", f"Arquivo não encontrado: {waypoints_file}")
            return
            
        try:
            with open(waypoints_file, 'r') as f:
                data = json.load(f)
                
            self.waypoints = data.get('waypoints', [])
            
            if not self.waypoints:
                self.get_logger().error('❌ Nenhum waypoint encontrado no arquivo')
                self.state = MissionState.FAILED
                self.publish_status("FAILED", "Nenhum waypoint no arquivo")
                return
                
            self.get_logger().info(f'✅ Carregados {len(self.waypoints)} waypoints:')
            for i, wp in enumerate(self.waypoints):
                name = wp.get('name', f'Waypoint_{i}')
                pos = wp.get('position', {})
                self.get_logger().info(f'   {i}: {name} -> ({pos.get("x", 0):.2f}, {pos.get("y", 0):.2f})')
                
            # Pular START (assumir que robô já está na posição inicial)
            # Começar pelo primeiro waypoint de trabalho (WS01)
            start_index = 0
            for i, wp in enumerate(self.waypoints):
                name = wp.get('name', '').upper()
                if name != 'START':  # Pular START
                    start_index = i
                    break
                    
            self.current_waypoint_index = start_index
            self.state = MissionState.NAVIGATING
            self.mission_start_time = time.time()
            self.publish_status("LOADED", f"Waypoints carregados: {len(self.waypoints)} (início: {start_index})")
            
            # NOVA ETAPA: Localização automática antes de navegar
            self.get_logger().info('🎯 Executando localização automática...')
            if self.perform_global_localization():
                self.get_logger().info('✅ Robô localizado no mapa!')
            else:
                self.get_logger().warn('⚠️ Localização pode não estar perfeita, mas continuando...')
            
            # Aguardar estabilização
            time.sleep(5.0)
            
            # Iniciar navegação com um único timer
            if not self.mission_timer:
                self.mission_timer = self.create_timer(2.0, self.mission_loop)
            
        except Exception as e:
            self.get_logger().error(f'❌ Erro ao carregar waypoints: {str(e)}')
            self.state = MissionState.FAILED
            self.publish_status("FAILED", f"Erro ao carregar: {str(e)}")
    
    def mission_loop(self):
        """Loop principal da missão - executa apenas quando necessário"""
        if self.state == MissionState.NAVIGATING:
            # Executar navegação apenas uma vez por waypoint
            if self.mission_timer:
                self.mission_timer.destroy()
                self.mission_timer = None
            self.navigate_to_next_waypoint()
        elif self.state == MissionState.EMERGENCY_STOP:
            # Verificar se o obstáculo foi removido
            if not self.obstacle_detected:
                self.get_logger().info('🔄 Retomando navegação - obstáculo removido')
                self.state = MissionState.NAVIGATING
        elif self.state in [MissionState.COMPLETED, MissionState.FAILED]:
            # Parar timer quando missão terminar
            if self.mission_timer:
                self.mission_timer.destroy()
                self.mission_timer = None
                
    def navigate_to_next_waypoint(self):
        """Navega para o próximo waypoint usando Nav2 com persistência"""
        if self.current_waypoint_index >= len(self.waypoints):
            # Missão completa
            if self.loop_mission:
                self.get_logger().info('🔄 Reiniciando missão em loop...')
                self.current_waypoint_index = 0
                # Reiniciar timer para próximo ciclo
                if not self.mission_timer:
                    self.mission_timer = self.create_timer(2.0, self.mission_loop)
            else:
                elapsed_time = time.time() - (self.mission_start_time or 0)
                self.get_logger().info(f'🎉 Missão completa! Tempo total: {elapsed_time:.1f}s')
                self.state = MissionState.COMPLETED
                self.publish_status("COMPLETED", f"Missão completa em {elapsed_time:.1f}s")
                return
        
        # Obter waypoint atual
        waypoint = self.waypoints[self.current_waypoint_index]
        wp_name = waypoint.get('name', f'Waypoint_{self.current_waypoint_index}')
        
        # Criar pose do waypoint
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        pos = waypoint.get('position', {})
        ori = waypoint.get('orientation', {})
        
        goal_pose.pose.position.x = float(pos.get('x', 0.0))
        goal_pose.pose.position.y = float(pos.get('y', 0.0))
        goal_pose.pose.position.z = float(pos.get('z', 0.0))
        
        goal_pose.pose.orientation.x = float(ori.get('x', 0.0))
        goal_pose.pose.orientation.y = float(ori.get('y', 0.0))
        goal_pose.pose.orientation.z = float(ori.get('z', 0.0))
        goal_pose.pose.orientation.w = float(ori.get('w', 1.0))
        
        self.get_logger().info(f'🎯 Navegando para waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: {wp_name}')
        self.get_logger().info(f'   Posição: ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})')
        
        # Enviar goal para Nav2 (mesmo sistema do goalpose)
        self.navigator.goToPose(goal_pose)
        
        # Controle de tentativas e timeout - usar parâmetros da classe
        max_retries = self.max_retries_per_waypoint
        current_retry = 0
        navigation_timeout = self.navigation_timeout
        start_time = time.time()
        
        # Aguardar resultado com maior persistência
        while not self.navigator.isTaskComplete():
            current_time = time.time()
            elapsed_time = current_time - start_time
            
            # Verificar timeout
            if elapsed_time > navigation_timeout:
                self.get_logger().warn(f'⏰ Timeout na navegação para {wp_name} ({elapsed_time:.1f}s)')
                if current_retry < max_retries:
                    current_retry += 1
                    self.get_logger().info(f'🔄 Tentativa {current_retry}/{max_retries} para {wp_name}')
                    self.navigator.cancelTask()
                    time.sleep(self.retry_delay)  # Pausa configurável antes de tentar novamente
                    self.navigator.goToPose(goal_pose)  # Tentar novamente
                    start_time = time.time()  # Resetar timer
                else:
                    self.get_logger().error(f'❌ Máximo de tentativas atingido para {wp_name}')
                    self.navigator.cancelTask()
                    break
            
            # Mostrar progresso apenas se não houver obstáculos críticos
            feedback = self.navigator.getFeedback()
            if feedback and elapsed_time % 5.0 < 1.0:  # A cada 5 segundos
                remaining_distance = feedback.distance_remaining
                status_msg = f"Indo para {wp_name}: {remaining_distance:.2f}m restantes"
                if self.obstacle_detected:
                    status_msg += " (contornando obstáculo)"
                self.publish_status("NAVIGATING", status_msg)
            
            time.sleep(0.1)
        
        # Verificar resultado final
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            elapsed_time = time.time() - start_time
            self.get_logger().info(f'✅ Chegou ao waypoint: {wp_name} em {elapsed_time:.1f}s')
            self.current_waypoint_index += 1
            self.publish_status("WAYPOINT_REACHED", f"Waypoint {wp_name} alcançado em {elapsed_time:.1f}s")
            
            # Pausa entre waypoints e continuar próximo waypoint
            time.sleep(2.0)
            
            # Reiniciar timer para próximo waypoint
            if not self.mission_timer:
                self.mission_timer = self.create_timer(2.0, self.mission_loop)
            
        elif result == TaskResult.CANCELED:
            if current_retry < max_retries:
                self.get_logger().warn(f'⏹️ Navegação cancelada para {wp_name} - tentando novamente')
                # Não avançar waypoint, tentar novamente no próximo loop
            else:
                self.get_logger().warn(f'⏹️ Desistindo do waypoint {wp_name} após {max_retries} tentativas')
                self.current_waypoint_index += 1  # Pular para próximo waypoint
                self.publish_status("WAYPOINT_SKIPPED", f"Waypoint {wp_name} pulado após tentativas")
            
        elif result == TaskResult.FAILED:
            if current_retry < max_retries:
                self.get_logger().error(f'❌ Falha ao navegar para {wp_name} - tentativa {current_retry + 1}/{max_retries}')
                # Não avançar waypoint, tentar novamente no próximo loop
            else:
                self.get_logger().error(f'❌ Falha definitiva para {wp_name} após {max_retries} tentativas')
                self.current_waypoint_index += 1  # Pular para próximo waypoint
                self.publish_status("WAYPOINT_FAILED", f"Waypoint {wp_name} falhou após tentativas")
        
        else:
            # Estado desconhecido - avançar para próximo waypoint
            self.get_logger().warn(f'⚠️ Estado desconhecido para {wp_name} - avançando para próximo')
            self.current_waypoint_index += 1
            
    def publish_status(self, status: str, details: str = ""):
        """Publica status da missão"""
        msg = String()
        msg.data = f"{status}: {details}" if details else status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    navigator = AutonomousWaypointNavigator()
    
    # Usar executor multi-thread para Nav2
    executor = MultiThreadedExecutor()
    executor.add_node(navigator)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        navigator.get_logger().info('🛑 Navegação autônoma finalizada')
    finally:
        navigator.navigator.lifecycleShutdown()
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
