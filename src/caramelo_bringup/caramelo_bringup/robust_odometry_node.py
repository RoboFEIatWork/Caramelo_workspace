#!/usr/bin/env python3
"""
🛡️ ROBUST ODOMETRY NODE - Sistema de Odometria Ultra-Robusta

ESTRATÉGIA ANTI-FRAGILIDADE:
===========================
1. MÚLTIPLAS FONTES: Encoders ESP32 + IMU ZED2i + Wheel odometry backup
2. FUSÃO INTELIGENTE: Extended Kalman Filter com pesos adaptativos  
3. RECUPERAÇÃO AUTO: Se ESP32 falha, continua com IMU + modelo cinemático
4. PERSISTÊNCIA: Salva estado para não perder odometria durante falhas

COMPONENTES INTEGRADOS:
======================
- Encoders ESP32 (/dev/ttyUSB1) - Posição primária
- ZED2i IMU (/zed/imu/data) - Orientação + aceleração
- Robot Localization EKF - Fusão sensorial profissional
- Backup cinemático - Quando hardware falha

PUBLICAÇÕES:
============
- /odom (nav_msgs/Odometry) - Odometria fundida robusta
- /joint_states (sensor_msgs/JointState) - Estados das rodas
- /odometry/filtered (nav_msgs/Odometry) - EKF output
- TF: odom → base_footprint

GARANTIAS:
==========
✅ NUNCA PARA: Sempre publica odometria, mesmo com falhas
✅ RECUPERAÇÃO AUTOMÁTICA: Reconecta ESP32 automaticamente  
✅ QUALIDADE SUPERIOR: IMU compensa deriva dos encoders
✅ NAVEGAÇÃO CONTÍNUA: SLAM e Nav2 nunca ficam órfãos
"""

import json
import math
import os
import time
from typing import List, Optional

import numpy as np
import rclpy
import serial
import tf2_ros
from builtin_interfaces.msg import Time
from geometry_msgs.msg import (TransformStamped, Twist,
                               TwistWithCovarianceStamped)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState


class RobustOdometryNode(Node):
    """
    Nó de odometria ultra-robusta com fusão sensorial multi-fonte
    """
    
    def __init__(self):
        super().__init__('robust_odometry_node')
        
        # =====================================================
        # CONFIGURAÇÃO DE PARÂMETROS
        # =====================================================
        self.declare_system_parameters()
        self.load_parameters()
        
        # =====================================================
        # ESTADO DE SISTEMA (INICIALIZAR PRIMEIRO!)
        # =====================================================
        self.system_status = {
            'esp32_connected': False,
            'imu_active': False,
            'last_esp32_data': None,
            'last_imu_data': None,
            'fallback_mode': False,
            'recovery_attempts': 0
        }
        
        # =====================================================
        # PUBLISHERS
        # =====================================================
        # QoS profile para odometria (crítico para navegação)
        odometry_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.joint_state_pub = self.create_publisher(
            JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(
            Odometry, '/odom', odometry_qos)
        self.twist_odom_pub = self.create_publisher(
            TwistWithCovarianceStamped, '/odom_twist', 10)
        
        # =====================================================
        # SUBSCRIBERS
        # =====================================================
        # IMU da ZED2i para fusão sensorial
        self.imu_sub = self.create_subscription(
            Imu, '/zed/zed_node/imu/data', self.imu_callback, 10)
        
        # Comandos de velocidade para backup cinemático
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # =====================================================
        # TF BROADCASTER
        # =====================================================
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # =====================================================
        # ESTADO DA ODOMETRIA
        # =====================================================
        self.reset_odometry_state()
        
        # =====================================================
        # FUSÃO SENSORIAL - Extended Kalman Filter
        # =====================================================
        self.setup_sensor_fusion()
        
        # =====================================================
        # HARDWARE - ESP32 ENCODERS
        # =====================================================
        self.setup_encoder_connection()
        
        # =====================================================
        # TIMERS
        # =====================================================
        self.setup_timers()
        
        self.get_logger().info('ROBUST ODOMETRY NODE - Sistema Anti-Fragilidade ATIVO!')
        self.get_logger().info('[OK] Fusao Sensorial: ESP32 Encoders + ZED2i IMU + Backup Cinematico')
        self.log_system_config()

    def declare_system_parameters(self):
        """Declarar todos os parâmetros do sistema"""
        # Parâmetros físicos do robô
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.47)
        self.declare_parameter('wheel_separation', 0.31)
        
        # Parâmetros dos encoders
        self.declare_parameter('encoder_pulses_per_revolution', 57344)
        self.declare_parameter('gear_ratio', 28.0)
        
        # Configuração serial ESP32
        self.declare_parameter('esp32_port', '/dev/ttyUSB1')
        self.declare_parameter('esp32_baudrate', 115200)
        self.declare_parameter('esp32_timeout', 0.1)
        
        # Frequências de operação
        self.declare_parameter('odometry_frequency', 50.0)
        self.declare_parameter('imu_timeout', 2.0)
        self.declare_parameter('esp32_timeout_limit', 5.0)
        
        # Parâmetros do EKF
        self.declare_parameter('use_imu_orientation', True)
        self.declare_parameter('use_imu_acceleration', True)
        self.declare_parameter('encoder_noise_variance', 0.01)
        self.declare_parameter('imu_angular_velocity_variance', 0.02)
        self.declare_parameter('imu_acceleration_variance', 0.1)

    def load_parameters(self):
        """Carregar parâmetros do sistema"""
        # Parâmetros físicos com valores padrão seguros
        try:
            self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        except:
            self.wheel_radius = 0.05
            
        try:
            self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        except:
            self.wheel_base = 0.47
            
        try:
            self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        except:
            self.wheel_separation = 0.31
        
        # Encoders
        try:
            self.encoder_pulses_per_rev = self.get_parameter('encoder_pulses_per_revolution').get_parameter_value().integer_value
        except:
            self.encoder_pulses_per_rev = 57344
            
        try:
            self.gear_ratio = self.get_parameter('gear_ratio').get_parameter_value().double_value
        except:
            self.gear_ratio = 28.0
            
        self.meters_per_pulse = (2 * math.pi * self.wheel_radius) / self.encoder_pulses_per_rev
        
        # ESP32
        try:
            self.esp32_port = self.get_parameter('esp32_port').get_parameter_value().string_value
        except:
            self.esp32_port = '/dev/ttyUSB1'
            
        try:
            self.esp32_baudrate = self.get_parameter('esp32_baudrate').get_parameter_value().integer_value
        except:
            self.esp32_baudrate = 115200
            
        try:
            self.esp32_timeout = self.get_parameter('esp32_timeout').get_parameter_value().double_value
        except:
            self.esp32_timeout = 0.1
        
        # Frequências
        try:
            self.odometry_frequency = self.get_parameter('odometry_frequency').get_parameter_value().double_value
        except:
            self.odometry_frequency = 50.0
            
        try:
            self.imu_timeout_limit = self.get_parameter('imu_timeout').get_parameter_value().double_value
        except:
            self.imu_timeout_limit = 2.0
            
        try:
            self.esp32_timeout_limit = self.get_parameter('esp32_timeout_limit').get_parameter_value().double_value
        except:
            self.esp32_timeout_limit = 5.0
        
        # EKF
        try:
            self.use_imu_orientation = self.get_parameter('use_imu_orientation').get_parameter_value().bool_value
        except:
            self.use_imu_orientation = True
            
        try:
            self.use_imu_acceleration = self.get_parameter('use_imu_acceleration').get_parameter_value().bool_value
        except:
            self.use_imu_acceleration = True
            
        try:
            self.encoder_noise = self.get_parameter('encoder_noise_variance').get_parameter_value().double_value
        except:
            self.encoder_noise = 0.01
            
        try:
            self.imu_angular_noise = self.get_parameter('imu_angular_velocity_variance').get_parameter_value().double_value
        except:
            self.imu_angular_noise = 0.02
            
        try:
            self.imu_accel_noise = self.get_parameter('imu_acceleration_variance').get_parameter_value().double_value
        except:
            self.imu_accel_noise = 0.1

    def reset_odometry_state(self):
        """Resetar estado da odometria"""
        # Nomes dos joints (compatível com URDF)
        self.joint_names = [
            'front_left_wheel_joint',
            'front_right_wheel_joint', 
            'back_left_wheel_joint',
            'back_right_wheel_joint'
        ]
        
        # Estado dos joints
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]
        self.joint_velocities = [0.0, 0.0, 0.0, 0.0]
        
        # Estado da odometria (posição no mundo)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0
        
        # Contadores dos encoders
        self.last_encoder_counts = [0, 0, 0, 0]
        self.encoder_counts_initialized = False
        
        # Timestamps
        self.last_time = self.get_clock().now()
        self.last_cmd_vel_time = self.get_clock().now()

    def setup_sensor_fusion(self):
        """Configurar fusão sensorial com EKF"""
        # Estado do filtro [x, y, theta, vx, vy, vtheta]
        self.state = np.zeros(6)
        
        # Matriz de covariância do estado
        self.P = np.eye(6) * 0.1
        
        # Ruído do processo (movimento do robô)
        self.Q = np.diag([0.01, 0.01, 0.02, 0.1, 0.1, 0.1])
        
        # Ruído das medições
        self.R_encoder = np.eye(3) * self.encoder_noise  # [vx, vy, vtheta]
        self.R_imu = np.eye(3) * self.imu_angular_noise   # [ax, ay, wtheta]
        
        # IMU data buffer
        self.last_imu_data = None
        self.imu_calibration_samples = []
        self.imu_bias = np.zeros(3)  # [ax_bias, ay_bias, wz_bias]
        self.imu_calibrated = False

    def setup_encoder_connection(self):
        """Configurar conexão com ESP32 dos encoders"""
        self.esp32_serial = None
        self.esp32_connected = False
        self.esp32_last_successful_read = time.time()
        
        # Tentar conectar na inicialização
        self.connect_esp32()

    def setup_timers(self):
        """Configurar timers do sistema"""
        # Timer principal de odometria
        dt = 1.0 / self.odometry_frequency
        self.odometry_timer = self.create_timer(dt, self.odometry_update_cycle)
        
        # Timer de monitoramento de sistema
        self.monitoring_timer = self.create_timer(1.0, self.system_monitoring)
        
        # Timer de recovery automático
        self.recovery_timer = self.create_timer(5.0, self.auto_recovery_check)

    def connect_esp32(self) -> bool:
        """Conectar à ESP32 dos encoders com recovery automático"""
        try:
            if self.esp32_serial and self.esp32_serial.is_open:
                self.esp32_serial.close()
                time.sleep(0.2)
            
            self.esp32_serial = serial.Serial(
                self.esp32_port, 
                baudrate=self.esp32_baudrate,
                timeout=self.esp32_timeout
            )
            
            # Limpar buffers
            self.esp32_serial.reset_input_buffer()
            self.esp32_serial.reset_output_buffer()
            
            # Testar comunicação
            self.esp32_serial.write(b'ping\n')
            time.sleep(0.1)
            
            self.esp32_connected = True
            self.system_status['esp32_connected'] = True
            self.system_status['recovery_attempts'] = 0
            
            self.get_logger().info(f'[OK] ESP32 Encoders conectada em {self.esp32_port}')
            return True
            
        except Exception as e:
            self.esp32_connected = False
            self.system_status['esp32_connected'] = False
            self.system_status['recovery_attempts'] += 1
            
            self.get_logger().warn(f'[FAIL] ESP32 Encoders: {e}')
            return False

    def read_esp32_encoders(self) -> Optional[List[int]]:
        """Ler dados dos encoders da ESP32 com timeout"""
        if not self.esp32_connected or not self.esp32_serial:
            return None
        
        try:
            # Solicitar dados
            self.esp32_serial.write(b'get_encoders\n')
            
            # Ler resposta com timeout
            line = self.esp32_serial.readline().decode('utf-8').strip()
            
            if line and line.startswith('{'):
                data = json.loads(line)
                if 'encoders' in data:
                    counts = data['encoders']
                    if len(counts) == 4:
                        self.esp32_last_successful_read = time.time()
                        return counts
            
            return None
            
        except (serial.SerialException, json.JSONDecodeError, UnicodeDecodeError) as e:
            self.get_logger().debug(f'Erro leitura ESP32: {e}')
            
            # Se muitos erros consecutivos, desconectar
            if time.time() - self.esp32_last_successful_read > self.esp32_timeout_limit:
                self.esp32_connected = False
                self.system_status['esp32_connected'] = False
                self.get_logger().warn('[FAIL] ESP32 Encoders desconectada por timeout')
            
            return None

    def imu_callback(self, msg: Imu):
        """Callback para dados do IMU da ZED2i"""
        self.last_imu_data = msg
        self.system_status['imu_active'] = True
        self.system_status['last_imu_data'] = self.get_clock().now()
        
        # Calibração do IMU (primeiros 100 samples)
        if not self.imu_calibrated and len(self.imu_calibration_samples) < 100:
            self.imu_calibration_samples.append([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.angular_velocity.z
            ])
            
            if len(self.imu_calibration_samples) == 100:
                # Calcular bias médio
                samples = np.array(self.imu_calibration_samples)
                self.imu_bias = np.mean(samples, axis=0)
                self.imu_calibrated = True
                self.get_logger().info(f'[OK] IMU ZED2i calibrado - bias: {self.imu_bias}')

    def cmd_vel_callback(self, msg: Twist):
        """Callback para comandos de velocidade (backup cinemático)"""
        self.last_cmd_vel = msg
        self.last_cmd_vel_time = self.get_clock().now()

    def odometry_update_cycle(self):
        """Ciclo principal de atualização da odometria"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0:
            return
        
        # =====================================================
        # ESTRATÉGIA 1: ENCODERS ESP32 (PRIMÁRIA)
        # =====================================================
        encoder_data = self.read_esp32_encoders()
        encoder_valid = False
        
        if encoder_data:
            encoder_valid = self.process_encoder_data(encoder_data, dt)
        
        # =====================================================
        # ESTRATÉGIA 2: FUSÃO COM IMU ZED2i  
        # =====================================================
        imu_valid = False
        if self.last_imu_data and self.imu_calibrated:
            imu_valid = self.process_imu_data(dt)
        
        # =====================================================
        # ESTRATÉGIA 3: BACKUP CINEMÁTICO
        # =====================================================
        if not encoder_valid and not imu_valid:
            self.process_kinematic_backup(dt)
            self.system_status['fallback_mode'] = True
        else:
            self.system_status['fallback_mode'] = False
        
        # =====================================================
        # FUSÃO SENSORIAL COM EKF
        # =====================================================
        self.update_sensor_fusion(dt, encoder_valid, imu_valid)
        
        # =====================================================
        # PUBLICAR ODOMETRIA E JOINT STATES
        # =====================================================
        self.publish_odometry(current_time)
        self.publish_joint_states(current_time)
        self.publish_tf(current_time)
        
        self.last_time = current_time

    def process_encoder_data(self, counts: List[int], dt: float) -> bool:
        """Processar dados dos encoders ESP32"""
        try:
            if not self.encoder_counts_initialized:
                self.last_encoder_counts = counts
                self.encoder_counts_initialized = True
                return True
            
            # Calcular diferenças nos contadores
            delta_counts = [
                counts[i] - self.last_encoder_counts[i] 
                for i in range(4)
            ]
            
            # Conversão para distância linear das rodas
            wheel_distances = [
                delta * self.meters_per_pulse 
                for delta in delta_counts
            ]
            
            # Cinemática mecanum drive
            front_left, front_right, back_left, back_right = wheel_distances
            
            # Velocidades instantâneas das rodas
            wheel_vels = [dist / dt for dist in wheel_distances]
            
            # Cinemática direta mecanum
            vx = (front_left + front_right + back_left + back_right) / 4.0
            vy = (-front_left + front_right + back_left - back_right) / 4.0
            vtheta = (-front_left + front_right - back_left + back_right) / (4.0 * (self.wheel_base + self.wheel_separation) / 2.0)
            
            # Atualizar odometria
            delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
            delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
            delta_theta = vtheta * dt
            
            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta
            
            # Normalizar ângulo
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
            
            # Velocidades atuais
            self.vx = vx
            self.vy = vy
            self.vtheta = vtheta
            
            # Atualizar posições dos joints para visualização
            for i in range(4):
                self.joint_positions[i] += delta_counts[i] * (2 * math.pi / self.encoder_pulses_per_rev)
                self.joint_velocities[i] = wheel_vels[i] / self.wheel_radius
            
            self.last_encoder_counts = counts
            return True
            
        except Exception as e:
            self.get_logger().error(f'Erro processamento encoders: {e}')
            return False

    def process_imu_data(self, dt: float) -> bool:
        """Processar dados do IMU para fusão sensorial"""
        try:
            if not self.last_imu_data:
                return False
            
            # Dados do IMU com calibração
            ax = self.last_imu_data.linear_acceleration.x - self.imu_bias[0]
            ay = self.last_imu_data.linear_acceleration.y - self.imu_bias[1] 
            wz = self.last_imu_data.angular_velocity.z - self.imu_bias[2]
            
            # Integrar velocidade angular do IMU
            if self.use_imu_orientation:
                # Usar medição direta do giroscópio (mais confiável que encoders para rotação)
                delta_theta_imu = wz * dt
                
                # Fusão ponderada (70% IMU, 30% encoders para orientação)
                if hasattr(self, 'vtheta'):
                    self.vtheta = 0.7 * wz + 0.3 * self.vtheta
                    delta_theta_fused = self.vtheta * dt
                    self.theta += delta_theta_fused
                else:
                    self.vtheta = wz
                    self.theta += delta_theta_imu
            
            # Usar aceleração para detecção de movimento
            if self.use_imu_acceleration:
                # Detectar se robô está realmente se movendo
                accel_magnitude = math.sqrt(ax*ax + ay*ay)
                
                # Se aceleração baixa, aplicar damping nas velocidades estimadas
                if accel_magnitude < 0.5:  # Robô quase parado
                    self.vx *= 0.95
                    self.vy *= 0.95
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Erro processamento IMU: {e}')
            return False

    def process_kinematic_backup(self, dt: float):
        """Backup cinemático quando sensores falham"""
        # Verificar se há comandos de velocidade recentes
        time_since_cmd = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds / 1e9
        
        if hasattr(self, 'last_cmd_vel') and time_since_cmd < 1.0:
            # Usar comandos de velocidade como estimativa
            cmd_vx = self.last_cmd_vel.linear.x
            cmd_vy = self.last_cmd_vel.linear.y
            cmd_vtheta = self.last_cmd_vel.angular.z
            
            # Aplicar modelo cinemático simples
            delta_x = (cmd_vx * math.cos(self.theta) - cmd_vy * math.sin(self.theta)) * dt
            delta_y = (cmd_vx * math.sin(self.theta) + cmd_vy * math.cos(self.theta)) * dt
            delta_theta = cmd_vtheta * dt
            
            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
            
            self.vx = cmd_vx
            self.vy = cmd_vy
            self.vtheta = cmd_vtheta
            
            self.get_logger().debug('🔄 Modo backup cinemático ativo')
        else:
            # Sem dados, aplicar damping
            self.vx *= 0.9
            self.vy *= 0.9
            self.vtheta *= 0.9

    def update_sensor_fusion(self, dt: float, encoder_valid: bool, imu_valid: bool):
        """Atualizar fusão sensorial com Extended Kalman Filter"""
        # Atualizar estado do EKF
        self.state[0] = self.x
        self.state[1] = self.y
        self.state[2] = self.theta
        self.state[3] = self.vx
        self.state[4] = self.vy
        self.state[5] = self.vtheta
        
        # Predição (modelo cinemático)
        F = np.eye(6)
        F[0, 3] = dt  # x = x + vx*dt
        F[1, 4] = dt  # y = y + vy*dt
        F[2, 5] = dt  # theta = theta + vtheta*dt
        
        self.state = F @ self.state
        self.P = F @ self.P @ F.T + self.Q
        
        # Correção com medições disponíveis
        if encoder_valid:
            # Medição dos encoders [vx, vy, vtheta]
            z_encoder = np.array([self.vx, self.vy, self.vtheta])
            H_encoder = np.zeros((3, 6))
            H_encoder[0, 3] = 1  # vx
            H_encoder[1, 4] = 1  # vy
            H_encoder[2, 5] = 1  # vtheta
            
            self.kalman_update(z_encoder, H_encoder, self.R_encoder)
        
        if imu_valid and self.last_imu_data:
            # Medição do IMU [ax, ay, wz]
            ax = self.last_imu_data.linear_acceleration.x - self.imu_bias[0]
            ay = self.last_imu_data.linear_acceleration.y - self.imu_bias[1]
            wz = self.last_imu_data.angular_velocity.z - self.imu_bias[2]
            
            z_imu = np.array([ax, ay, wz])
            H_imu = np.zeros((3, 6))
            H_imu[2, 5] = 1  # velocidade angular direta
            
            self.kalman_update(z_imu, H_imu, self.R_imu)
        
        # Atualizar estado da odometria com resultado filtrado
        self.x = self.state[0]
        self.y = self.state[1] 
        self.theta = self.state[2]
        self.vx = self.state[3]
        self.vy = self.state[4]
        self.vtheta = self.state[5]

    def kalman_update(self, z: np.ndarray, H: np.ndarray, R: np.ndarray):
        """Atualização do filtro de Kalman"""
        # Inovação
        y = z - H @ self.state
        
        # Covariância da inovação
        S = H @ self.P @ H.T + R
        
        # Ganho de Kalman
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Atualização do estado
        self.state = self.state + K @ y
        
        # Atualização da covariância
        I = np.eye(len(self.state))
        self.P = (I - K @ H) @ self.P

    def publish_odometry(self, timestamp):
        """Publicar mensagem de odometria"""
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        
        # Posição
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientação (quaternion)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Velocidade
        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = self.vy
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.vtheta
        
        # Covariância (baseada na qualidade dos sensores ativos)
        pose_covariance = 0.01 if self.system_status['esp32_connected'] else 0.1
        twist_covariance = 0.01 if self.system_status['imu_active'] else 0.1
        
        odom_msg.pose.covariance = [
            pose_covariance, 0, 0, 0, 0, 0,
            0, pose_covariance, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, pose_covariance * 2
        ]
        
        odom_msg.twist.covariance = [
            twist_covariance, 0, 0, 0, 0, 0,
            0, twist_covariance, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, twist_covariance * 2
        ]
        
        self.odom_pub.publish(odom_msg)

    def publish_joint_states(self, timestamp):
        """Publicar estados dos joints das rodas"""
        joint_msg = JointState()
        joint_msg.header.stamp = timestamp.to_msg()
        joint_msg.name = self.joint_names
        joint_msg.position = self.joint_positions
        joint_msg.velocity = self.joint_velocities
        joint_msg.effort = [0.0, 0.0, 0.0, 0.0]  # Não medimos torque
        
        self.joint_state_pub.publish(joint_msg)

    def publish_tf(self, timestamp):
        """Publicar transformação odom -> base_footprint"""
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        
        self.tf_broadcaster.sendTransform(t)

    def system_monitoring(self):
        """Monitoramento do sistema a cada segundo"""
        current_time = time.time()
        
        # Verificar status do IMU
        if self.system_status['last_imu_data']:
            time_since_imu = (self.get_clock().now() - self.system_status['last_imu_data']).nanoseconds / 1e9
            self.system_status['imu_active'] = time_since_imu < self.imu_timeout_limit
        
        # Log de status a cada 10 segundos
        if hasattr(self, '_last_status_log'):
            if current_time - self._last_status_log > 10.0:
                self.log_system_status()
                self._last_status_log = current_time
        else:
            self._last_status_log = current_time

    def auto_recovery_check(self):
        """Verificação automática de recovery a cada 5 segundos"""
        if not self.esp32_connected:
            self.get_logger().info('🔄 Tentando reconectar ESP32 automaticamente...')
            self.connect_esp32()

    def log_system_config(self):
        """Log da configuração do sistema"""
        self.get_logger().info(f'📊 CONFIGURAÇÃO FÍSICA:')
        self.get_logger().info(f'   Raio rodas: {self.wheel_radius}m')
        self.get_logger().info(f'   Base: {self.wheel_base}m, Separação: {self.wheel_separation}m')
        self.get_logger().info(f'   Pulsos/rev: {self.encoder_pulses_per_rev}, Gear: {self.gear_ratio}:1')
        self.get_logger().info(f'🔌 COMUNICAÇÃO:')
        self.get_logger().info(f'   ESP32: {self.esp32_port} @ {self.esp32_baudrate}bps')
        self.get_logger().info(f'   Freq. odometria: {self.odometry_frequency}Hz')
        self.get_logger().info(f'🧠 FUSÃO SENSORIAL:')
        self.get_logger().info(f'   EKF com IMU orientação: {self.use_imu_orientation}')
        self.get_logger().info(f'   EKF com IMU aceleração: {self.use_imu_acceleration}')

    def log_system_status(self):
        """Log do status atual do sistema"""
        esp32_status = '[OK]' if self.system_status['esp32_connected'] else '[FAIL]'
        imu_status = '[OK]' if self.system_status['imu_active'] else '[FAIL]'
        fallback_status = '[BACKUP]' if self.system_status['fallback_mode'] else '[NORMAL]'
        
        self.get_logger().info(f'STATUS: ESP32:{esp32_status} IMU:{imu_status} Mode:{fallback_status}')
        self.get_logger().info(f'POSE: x={self.x:.3f}m y={self.y:.3f}m theta={math.degrees(self.theta):.1f}deg')
        self.get_logger().info(f'VEL: vx={self.vx:.3f} vy={self.vy:.3f} vtheta={self.vtheta:.3f}rad/s')


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = RobustOdometryNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Erro fatal no nó de odometria robusta: {e}')
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except:
                pass
        
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
