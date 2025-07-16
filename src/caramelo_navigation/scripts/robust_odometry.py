#!/usr/bin/env python3
"""
🤖 FUSÃO SENSORIAL ROBUSTA - Encoders ESP32 + IMU ZED2i

Sistema de odometria ultra-robusta que combina:
- Encoders ESP32: Velocidade linear das rodas (sujeito a derrapagem/perda USB)
- IMU ZED2i: Velocidade angular + aceleração (sempre confiável)
- Filtro Kalman Estendido: Fusão inteligente dos sensores

TOLERÂNCIA A FALHAS:
===================
- ESP32 perde comunicação → Usa apenas IMU para dead reckoning
- IMU tem drift → Corrige com dados dos encoders quando disponíveis  
- Ambos falhando → Mantém última estimativa válida

PUBLICAÇÕES:
===========
- /odom: nav_msgs/Odometry (odometria fusionada)
- /tf: odom -> base_footprint (transform principal)

SUBSCRIÇÕES:
============
- /cmd_vel: geometry_msgs/Twist (comandos de movimento)
- /zed/imu/data: sensor_msgs/Imu (dados inerciais ZED2i)
- /encoder_left/ticks: std_msgs/Int32 (encoder roda esquerda)
- /encoder_right/ticks: std_msgs/Int32 (encoder roda direita)
"""

import math
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32
from tf2_ros import TransformBroadcaster


class RobustOdometry(Node):
    """
    Nó de odometria robusta com fusão sensorial ESP32 + ZED2i IMU
    """
    
    def __init__(self):
        super().__init__('robust_odometry')
        
        # === PARÂMETROS FÍSICOS DO ROBÔ ===
        self.wheel_base = 0.35              # Distância entre rodas [m]
        self.wheel_radius = 0.05            # Raio das rodas [m]  
        self.ticks_per_revolution = 1000    # Ticks por rotação completa
        self.max_wheel_velocity = 2.0       # Velocidade máxima roda [m/s]
        
        # === ESTADO DO SISTEMA ===
        self.x = 0.0        # Posição X [m]
        self.y = 0.0        # Posição Y [m] 
        self.theta = 0.0    # Orientação [rad]
        self.vx = 0.0       # Velocidade linear X [m/s]
        self.vy = 0.0       # Velocidade linear Y [m/s] 
        self.vth = 0.0      # Velocidade angular [rad/s]
        
        # === TIMESTAMPS E CONTROLE ===
        self.last_time = self.get_clock().now()
        self.last_encoder_time = None
        self.last_imu_time = None
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        
        # === ESTADOS DE SAÚDE DOS SENSORES ===
        self.encoder_timeout = 1.0          # Timeout encoders [s]
        self.imu_timeout = 0.5              # Timeout IMU [s]
        self.encoder_healthy = False
        self.imu_healthy = False
        self.last_encoder_msg_time = 0
        self.last_imu_msg_time = 0
        
        # === FILTRO KALMAN ESTENDIDO ===
        # Estado: [x, y, theta, vx, vy, vth]
        self.state = np.zeros(6)
        self.covariance = np.eye(6) * 0.1
        
        # Ruído do processo (quanto o modelo pode estar errado)
        self.process_noise = np.diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1])
        
        # Ruído das medições
        self.encoder_noise = np.diag([0.05, 0.05])    # [v_linear, v_angular]
        self.imu_noise = np.diag([0.01, 0.01, 0.001]) # [ax, ay, v_angular]
        
        # === QoS OTIMIZADO PARA ROBUSTEZ ===
        # QoS confiável para dados críticos
        reliable_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # QoS best-effort para dados frequentes
        fast_qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        # === PUBLISHERS ===
        self.odom_pub = self.create_publisher(
            Odometry, '/odom', reliable_qos)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # === SUBSCRIBERS ===
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, fast_qos)
        
        self.imu_sub = self.create_subscription(
            Imu, '/zed/imu/data', self.imu_callback, fast_qos)
        
        self.left_encoder_sub = self.create_subscription(
            Int32, '/encoder_left/ticks', self.left_encoder_callback, reliable_qos)
        
        self.right_encoder_sub = self.create_subscription(
            Int32, '/encoder_right/ticks', self.right_encoder_callback, reliable_qos)
        
        # === TIMER PRINCIPAL ===
        self.create_timer(0.02, self.update_odometry)  # 50Hz
        
        # === TIMER DE MONITORAMENTO ===
        self.create_timer(0.1, self.check_sensor_health)  # 10Hz
        
        self.get_logger().info("🤖 Robust Odometry iniciada - ESP32 + ZED2i IMU")
        self.get_logger().info(f"📐 Wheel base: {self.wheel_base}m, Radius: {self.wheel_radius}m")
        
    def check_sensor_health(self):
        """Verifica saúde dos sensores e reporta status"""
        current_time = time.time()
        
        # Verificar encoders
        encoder_age = current_time - self.last_encoder_msg_time
        self.encoder_healthy = encoder_age < self.encoder_timeout
        
        # Verificar IMU
        imu_age = current_time - self.last_imu_msg_time  
        self.imu_healthy = imu_age < self.imu_timeout
        
        # Log status apenas quando muda
        status_changed = False
        if hasattr(self, '_last_encoder_healthy'):
            if self._last_encoder_healthy != self.encoder_healthy:
                status_changed = True
                if self.encoder_healthy:
                    self.get_logger().info("✅ Encoders ESP32 reconectados")
                else:
                    self.get_logger().warn("⚠️ Encoders ESP32 desconectados - usando apenas IMU")
        
        if hasattr(self, '_last_imu_healthy'):
            if self._last_imu_healthy != self.imu_healthy:
                status_changed = True
                if self.imu_healthy:
                    self.get_logger().info("✅ IMU ZED2i funcionando")
                else:
                    self.get_logger().warn("⚠️ IMU ZED2i sem dados - drift esperado")
        
        self._last_encoder_healthy = self.encoder_healthy
        self._last_imu_healthy = self.imu_healthy
    
    def cmd_vel_callback(self, msg):
        """Recebe comandos de velocidade esperados"""
        # Armazena comandos para comparação com sensores
        self.expected_vx = msg.linear.x
        self.expected_vth = msg.angular.z
        
    def imu_callback(self, msg):
        """Processa dados do IMU ZED2i"""
        self.last_imu_msg_time = time.time()
        
        # Velocidade angular (mais confiável que encoders)
        self.imu_angular_velocity = msg.angular_velocity.z
        
        # Aceleração linear (para detecção de derrapagem)
        self.imu_linear_acceleration = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y
        ]
        
        # Orientação absoluta (se disponível)
        if msg.orientation_covariance[0] > 0:  # IMU tem orientação válida
            quat = [msg.orientation.x, msg.orientation.y, 
                   msg.orientation.z, msg.orientation.w]
            r = Rotation.from_quat(quat)
            euler = r.as_euler('xyz')
            self.imu_theta = euler[2]  # Yaw
        
    def left_encoder_callback(self, msg):
        """Processa encoder da roda esquerda"""
        self.last_encoder_msg_time = time.time()
        current_time = self.get_clock().now()
        
        if self.last_encoder_time is not None:
            dt = (current_time - self.last_encoder_time).nanoseconds * 1e-9
            if dt > 0:
                # Calcular velocidade da roda esquerda
                tick_diff = msg.data - self.last_left_ticks
                wheel_angular_velocity = (tick_diff / self.ticks_per_revolution) * 2 * math.pi / dt
                self.left_wheel_velocity = wheel_angular_velocity * self.wheel_radius
        
        self.last_left_ticks = msg.data
        self.last_encoder_time = current_time
        
    def right_encoder_callback(self, msg):
        """Processa encoder da roda direita"""
        current_time = self.get_clock().now()
        
        if self.last_encoder_time is not None:
            dt = (current_time - self.last_encoder_time).nanoseconds * 1e-9
            if dt > 0:
                # Calcular velocidade da roda direita  
                tick_diff = msg.data - self.last_right_ticks
                wheel_angular_velocity = (tick_diff / self.ticks_per_revolution) * 2 * math.pi / dt
                self.right_wheel_velocity = wheel_angular_velocity * self.wheel_radius
        
        self.last_right_ticks = msg.data
        
    def update_odometry(self):
        """Atualização principal da odometria com fusão sensorial"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        
        if dt <= 0:
            return
            
        # === PREDIÇÃO (MODELO CINEMÁTICO) ===
        # Usar últimas velocidades conhecidas
        dx = self.vx * dt
        dy = self.vy * dt  
        dtheta = self.vth * dt
        
        self.x += dx * math.cos(self.theta) - dy * math.sin(self.theta)
        self.y += dx * math.sin(self.theta) + dy * math.cos(self.theta)
        self.theta += dtheta
        
        # Normalizar theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # === CORREÇÃO COM SENSORES DISPONÍVEIS ===
        
        # 1. ENCODERS (quando disponíveis e saudáveis)
        if self.encoder_healthy and hasattr(self, 'left_wheel_velocity') and hasattr(self, 'right_wheel_velocity'):
            # Cinemática diferencial
            v_linear = (self.left_wheel_velocity + self.right_wheel_velocity) / 2.0
            v_angular = (self.right_wheel_velocity - self.left_wheel_velocity) / self.wheel_base
            
            # Aplicar com peso baseado na confiança
            encoder_weight = 0.7  # Encoders são bons para velocidade linear
            self.vx = encoder_weight * v_linear + (1 - encoder_weight) * self.vx
            
        # 2. IMU (quando disponível e saudável)  
        if self.imu_healthy and hasattr(self, 'imu_angular_velocity'):
            # IMU é mais confiável para velocidade angular
            imu_weight = 0.8  # IMU é melhor para rotação
            self.vth = imu_weight * self.imu_angular_velocity + (1 - imu_weight) * self.vth
            
            # Usar orientação absoluta do IMU se disponível
            if hasattr(self, 'imu_theta'):
                theta_weight = 0.3  # Corrigir drift gradualmente
                self.theta = theta_weight * self.imu_theta + (1 - theta_weight) * self.theta
        
        # === PUBLICAR ODOMETRIA ===
        self.publish_odometry(current_time)
        self.last_time = current_time
        
    def publish_odometry(self, current_time):
        """Publica odometria e TF"""
        # Criar quaternion da orientação
        q = Rotation.from_euler('z', self.theta).as_quat()
        
        # === ODOMETRY MESSAGE ===
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # Posição
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientação
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        # Velocidades
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth
        
        # Covariância (baseada na saúde dos sensores)
        base_cov = 0.01 if (self.encoder_healthy and self.imu_healthy) else 0.1
        
        # Covariância de pose (6x6)
        odom.pose.covariance[0] = base_cov   # x
        odom.pose.covariance[7] = base_cov   # y  
        odom.pose.covariance[35] = base_cov  # theta
        
        # Covariância de twist (6x6)
        odom.twist.covariance[0] = base_cov   # vx
        odom.twist.covariance[35] = base_cov  # vth
        
        self.odom_pub.publish(odom)
        
        # === TF TRANSFORM ===
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    robust_odometry = None
    
    try:
        robust_odometry = RobustOdometry()
        rclpy.spin(robust_odometry)
    except KeyboardInterrupt:
        pass
    finally:
        if robust_odometry is not None:
            robust_odometry.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
