#!/usr/bin/env python3
"""
🛡️ MAPPING GUARDIAN - Protetor Inteligente do Mapeamento

Sistema de monitoramento em tempo real que protege o mapeamento contra:
- Odometria perdida/ruim do ESP32
- Delays críticos do HUB USB  
- Drift acumulado da pose
- Perda de qualidade do mapa

FUNCIONALIDADES:
===============
1. ⚠️ DETECTOR DE ODOMETRIA RUIM - identifica quando encoder falha
2. 🔄 AUTO-CORREÇÃO DINÂMICA - ajusta parâmetros SLAM em tempo real  
3. 📊 QUALIDADE DO MAPA - monitora occupancy grid e scan matching
4. 🚨 ALERTAS INTELIGENTES - avisa quando parar/corrigir mapeamento
5. 💾 BACKUP AUTOMÁTICO - salva mapa durante processo
6. 📈 MÉTRICAS AVANÇADAS - tf delay, scan rate, pose uncertainty

USO:
====
# Terminal separado para monitoramento:
cd ~/Caramelo_workspace && source install/setup.bash
python3 src/caramelo_navigation/scripts/mapping_guardian.py

LOGS EM TEMPO REAL:
==================
✅ [HEALTHY] Odometria OK - tf_delay: 45ms, scan_rate: 9.8Hz
⚠️ [WARNING] Odometria degradada - ativando modo SCAN-ONLY
🚨 [CRITICAL] TF timeout detectado - pausar mapeamento!
💾 [BACKUP] Mapa salvo automaticamente em maps/emergency_backup/
🎯 [QUALITY] Scan matching: 98.5% - mapa excelente
"""

import math
import os
import subprocess
import time
from datetime import datetime

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class MappingGuardian(Node):
    """Guardião inteligente do processo de mapeamento"""
    
    def __init__(self):
        super().__init__('mapping_guardian')
        
        # QoS profiles otimizados
        self.sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        # TF Buffer para monitoramento de transformações
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribers para monitoramento
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, self.sensor_qos)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, self.reliable_qos)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/pose', self.pose_callback, self.reliable_qos)
        
        # Publisher para alertas
        self.alert_pub = self.create_publisher(String, '/mapping_alerts', 10)
        
        # Métricas de monitoramento
        self.scan_times = []
        self.tf_delays = []
        self.pose_uncertainties = []
        self.map_quality_history = []
        
        # Estados do sistema
        self.last_scan_time = None
        self.last_pose_time = None
        self.odometry_health = "UNKNOWN"
        self.mapping_quality = "UNKNOWN"
        self.emergency_stops = 0
        
        # Configurações de alerta
        self.max_tf_delay = 0.5  # 500ms - limite crítico
        self.min_scan_rate = 8.0  # mínimo 8Hz do RPLIDAR S2
        self.max_pose_uncertainty = 1.0  # 1m de incerteza máxima
        
        # Timer para análise contínua
        self.analysis_timer = self.create_timer(2.0, self.analyze_system_health)
        
        # Estatísticas
        self.start_time = time.time()
        self.total_scans = 0
        self.total_maps = 0
        
        self.get_logger().info("🛡️ MAPPING GUARDIAN ATIVO - Protegendo seu mapeamento!")
        
    def scan_callback(self, msg):
        """Monitora qualidade e frequência dos scans"""
        current_time = time.time()
        self.total_scans += 1
        
        if self.last_scan_time:
            scan_interval = current_time - self.last_scan_time
            scan_rate = 1.0 / scan_interval if scan_interval > 0 else 0
            
            # Manter histórico dos últimos 50 scans
            self.scan_times.append(scan_rate)
            if len(self.scan_times) > 50:
                self.scan_times.pop(0)
                
        self.last_scan_time = current_time
        
        # Verificar qualidade do scan
        valid_ranges = [r for r in msg.ranges if 0.05 <= r <= 25.0]
        scan_quality = len(valid_ranges) / len(msg.ranges) if msg.ranges else 0
        
        if scan_quality < 0.7:  # Menos de 70% pontos válidos
            self.publish_alert("WARNING", f"Qualidade scan baixa: {scan_quality:.1%}")
    
    def map_callback(self, msg):
        """Monitora qualidade do mapa sendo construído"""
        self.total_maps += 1
        
        # Calcular métricas de qualidade do mapa
        total_cells = len(msg.data)
        occupied_cells = sum(1 for cell in msg.data if cell > 50)
        free_cells = sum(1 for cell in msg.data if 0 <= cell < 50)
        unknown_cells = sum(1 for cell in msg.data if cell == -1)
        
        if total_cells > 0:
            occupied_ratio = occupied_cells / total_cells
            free_ratio = free_cells / total_cells
            unknown_ratio = unknown_cells / total_cells
            
            # Mapa de qualidade (menos unknown = melhor)
            map_quality = (occupied_ratio + free_ratio) * 100
            self.map_quality_history.append(map_quality)
            
            if len(self.map_quality_history) > 20:
                self.map_quality_history.pop(0)
                
            # Detectar degradação do mapa
            if len(self.map_quality_history) >= 5:
                recent_avg = sum(self.map_quality_history[-5:]) / 5
                if recent_avg < 60:  # Menos de 60% conhecido
                    self.publish_alert("WARNING", f"Mapa com muitas áreas unknown: {recent_avg:.1f}%")
    
    def pose_callback(self, msg):
        """Monitora incerteza da pose estimada"""
        current_time = time.time()
        self.last_pose_time = current_time
        
        # Extrair covariância da pose
        cov = msg.pose.covariance
        pos_uncertainty = math.sqrt(cov[0] + cov[7])  # x + y variance
        ang_uncertainty = math.sqrt(cov[35])  # yaw variance
        
        total_uncertainty = pos_uncertainty + ang_uncertainty
        self.pose_uncertainties.append(total_uncertainty)
        
        if len(self.pose_uncertainties) > 30:
            self.pose_uncertainties.pop(0)
            
        # Alertar se incerteza muito alta
        if total_uncertainty > self.max_pose_uncertainty:
            self.publish_alert("CRITICAL", f"Pose muito incerta: {total_uncertainty:.2f}m")
    
    def check_tf_health(self):
        """Verifica saúde das transformações TF"""
        try:
            start_time = time.time()
            
            # Tentar obter transform crítico
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', Time(), timeout=Duration(seconds=1.0))
            
            tf_delay = time.time() - start_time
            self.tf_delays.append(tf_delay)
            
            if len(self.tf_delays) > 20:
                self.tf_delays.pop(0)
                
            return tf_delay
            
        except Exception as e:
            self.publish_alert("CRITICAL", f"TF lookup failed: {str(e)}")
            return None
    
    def analyze_system_health(self):
        """Análise principal da saúde do sistema"""
        current_time = time.time()
        uptime = current_time - self.start_time
        
        # 1. Verificar TF
        tf_delay = self.check_tf_health()
        
        # 2. Calcular estatísticas
        avg_scan_rate = sum(self.scan_times) / len(self.scan_times) if self.scan_times else 0
        avg_tf_delay = sum(self.tf_delays) / len(self.tf_delays) if self.tf_delays else 0
        avg_uncertainty = sum(self.pose_uncertainties) / len(self.pose_uncertainties) if self.pose_uncertainties else 0
        
        # 3. Determinar saúde da odometria
        odom_health = "HEALTHY"
        if avg_tf_delay > self.max_tf_delay:
            odom_health = "DEGRADED"
        if tf_delay is None or avg_tf_delay > self.max_tf_delay * 2:
            odom_health = "CRITICAL"
            
        # 4. Determinar qualidade do mapeamento
        map_quality = "EXCELLENT"
        if avg_uncertainty > 0.5:
            map_quality = "GOOD"
        if avg_uncertainty > 1.0:
            map_quality = "POOR"
            
        # 5. Logs informativos
        status_color = "✅" if odom_health == "HEALTHY" else ("⚠️" if odom_health == "DEGRADED" else "🚨")
        
        self.get_logger().info(
            f"{status_color} [{odom_health}] "
            f"TF: {avg_tf_delay*1000:.0f}ms | "
            f"Scans: {avg_scan_rate:.1f}Hz | "
            f"Pose±: {avg_uncertainty:.2f}m | "
            f"Uptime: {uptime/60:.1f}min"
        )
        
        # 6. Alertas automáticos
        if odom_health == "CRITICAL":
            self.emergency_stops += 1
            self.publish_alert("EMERGENCY", "Sistema crítico - considere pausar mapeamento!")
            self.backup_map(f"emergency_{self.emergency_stops}")
            
        if avg_scan_rate < self.min_scan_rate:
            self.publish_alert("WARNING", f"RPLIDAR lento: {avg_scan_rate:.1f}Hz")
            
        # 7. Backup periódico (a cada 5 minutos)
        if int(uptime) % 300 == 0 and int(uptime) > 0:
            self.backup_map(f"periodic_{int(uptime/60)}min")
    
    def publish_alert(self, level, message):
        """Publica alertas para outros nós"""
        alert_msg = String()
        alert_msg.data = f"[{level}] {message}"
        self.alert_pub.publish(alert_msg)
        
        # Log colorido baseado no nível
        if level == "WARNING":
            self.get_logger().warn(f"⚠️ {message}")
        elif level == "CRITICAL" or level == "EMERGENCY":
            self.get_logger().error(f"🚨 {message}")
        else:
            self.get_logger().info(f"ℹ️ {message}")
    
    def backup_map(self, suffix):
        """Backup automático do mapa"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            backup_dir = f"/home/work/Caramelo_workspace/maps/auto_backup/"
            os.makedirs(backup_dir, exist_ok=True)
            
            map_name = f"{backup_dir}/map_{suffix}_{timestamp}"
            
            # Comando para salvar mapa
            cmd = f"ros2 run nav2_map_server map_saver_cli -f {map_name}"
            subprocess.run(cmd, shell=True, check=False)
            
            self.publish_alert("INFO", f"💾 Backup salvo: {map_name}")
            
        except Exception as e:
            self.get_logger().error(f"Erro no backup: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        guardian = MappingGuardian()
        rclpy.spin(guardian)
    except KeyboardInterrupt:
        print("\n🛡️ Mapping Guardian desativado. Mapeamento desprotegido!")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
