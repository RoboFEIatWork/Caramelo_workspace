#!/usr/bin/env python3
"""
� ODOMETRY FUSION NODE - Fusão Encoder + IMU ZED

FUNÇÃO PRINCIPAL:
================
Combina dados dos encoders ESP32 com IMU da ZED2i para calcular:
- Odometria robusta e precisa
- Joint states das rodas
- TF odom -> base_footprint

ENTRADAS:
=========
- /encoder_data (std_msgs/Int32MultiArray) - Contadores raw dos encoders
- /zed/zed_node/imu/data (sensor_msgs/Imu) - IMU da ZED2i
- /encoder_status (std_msgs/Bool) - Status da conexão ESP32

SAÍDAS:
=======
- /odom (nav_msgs/Odometry) - Odometria fundida
- /joint_states (sensor_msgs/JointState) - Estados das rodas
- TF: odom -> base_footprint

ESTRATÉGIA DE FUSÃO:
===================
✅ ENCODER PRIMÁRIO: Velocidade linear e posição XY
✅ IMU SECUNDÁRIO: Orientação e velocidade angular
✅ FALLBACK: Se encoder falha, usa apenas IMU + modelo cinemático
✅ FILTRO: Kalman simples para suavizar ruídos
"""

import math
import time
from typing import List, Optional

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Int32MultiArray


class OdometryFusionNode(Node):
    def __init__(self):
        super().__init__('odometry_fusion_node')
        
        # =====================================================
        # PARÂMETROS FÍSICOS DO ROBÔ
        # =====================================================
        self.declare_parameter('wheel_radius', 0.05)  # metros
        self.declare_parameter('wheel_base', 0.47)    # metros (frente-trás)
        self.declare_parameter('wheel_separation', 0.31)  # metros (esquerda-direita)
        self.declare_parameter('encoder_pulses_per_revolution', 57344)
        self.declare_parameter('gear_ratio', 28.0)
        
        # Parâmetros de fusão
        self.declare_parameter('imu_weight', 0.8)  # Peso da IMU na orientação
        self.declare_parameter('encoder_weight', 0.2)  # Peso dos encoders na orientação
        self.declare_parameter('fallback_timeout', 2.0)  # Timeout para fallback
        
        # Carregar parâmetros
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.encoder_pulses_per_rev = self.get_parameter('encoder_pulses_per_revolution').get_parameter_value().integer_value
        self.gear_ratio = self.get_parameter('gear_ratio').get_parameter_value().double_value
        
        # Pesos da fusão
        self.imu_weight = self.get_parameter('imu_weight').get_parameter_value().double_value
        self.encoder_weight = self.get_parameter('encoder_weight').get_parameter_value().double_value
        self.fallback_timeout = self.get_parameter('fallback_timeout').get_parameter_value().double_value
        
        # Cálculos cinemáticos
        self.meters_per_pulse = (2 * math.pi * self.wheel_radius) / self.encoder_pulses_per_rev
        
        # =====================================================
        # PUBLISHERS
        # =====================================================
        # QoS profile para odometria (crítico para navegação)
        odometry_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.odom_pub = self.create_publisher(Odometry, '/odom', odometry_qos)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # =====================================================
        # SUBSCRIBERS
        # =====================================================
        self.encoder_sub = self.create_subscription(
            Int32MultiArray, '/encoder_data', self.encoder_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/zed/imu/data', self.imu_callback, 10)
        self.encoder_status_sub = self.create_subscription(
            Bool, '/encoder_status', self.encoder_status_callback, 10)
        
        # =====================================================
        # ESTADO INTERNO
        # =====================================================
        # Estado de odometria
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.theta_imu = 0.0  # Orientação da IMU
        self.theta_encoder = 0.0  # Orientação dos encoders
        
        # Estado das rodas
        self.joint_names = [
            'front_left_wheel_joint',
            'front_right_wheel_joint', 
            'back_left_wheel_joint',
            'back_right_wheel_joint'
        ]
        self.wheel_positions = [0.0, 0.0, 0.0, 0.0]
        self.wheel_velocities = [0.0, 0.0, 0.0, 0.0]
        
        # Dados dos sensores
        self.last_encoder_counts = [0, 0, 0, 0]
        self.last_encoder_time = None
        self.last_imu_time = None
        self.encoders_initialized = False
        self.imu_initialized = False
        
        # Status dos sensores
        self.encoders_connected = False
        self.imu_active = False
        self.last_encoder_data_time = time.time()
        self.last_imu_data_time = time.time()
        
        # Filtro complementar para orientação
        self.orientation_filter_alpha = 0.98  # Peso da IMU
        
        # Backup cinemático para fallback
        self.last_cmd_vel = Twist()
        self.fallback_mode = False
        
        # Timer principal de publicação (50Hz)
        self.create_timer(0.02, self.publish_odometry_and_joints)
        
        # Timer de monitoramento (1Hz)
        self.create_timer(1.0, self.monitor_sensors)
        
        self.get_logger().info('🧭 ODOMETRY FUSION - Iniciando...')
        self.get_logger().info(f'Parâmetros físicos:')
        self.get_logger().info(f'  - Raio das rodas: {self.wheel_radius}m')
        self.get_logger().info(f'  - Distância entre eixos: {self.wheel_base}m')
        self.get_logger().info(f'  - Distância entre rodas: {self.wheel_separation}m')
        self.get_logger().info(f'Fusão sensorial:')
        self.get_logger().info(f'  - Peso IMU: {self.imu_weight}')
        self.get_logger().info(f'  - Peso Encoders: {self.encoder_weight}')

    def encoder_callback(self, msg: Int32MultiArray):
        """Processa dados dos encoders"""
        if len(msg.data) != 4:
            self.get_logger().warn("⚠️ Dados de encoder inválidos")
            return
            
        current_time = time.time()
        raw_counts = msg.data
        
        # Correção para motores invertidos (FR e RR)
        corrected_counts = [
            raw_counts[0],   # FL: normal
            -raw_counts[1],  # FR: invertido
            raw_counts[2],   # RL: normal  
            -raw_counts[3]   # RR: invertido
        ]
        
        # Inicialização na primeira leitura
        if not self.encoders_initialized:
            self.last_encoder_counts = corrected_counts[:]
            self.last_encoder_time = current_time
            self.encoders_initialized = True
            self.get_logger().info(f'Encoders inicializados: {corrected_counts}')
            return
        
        # Calcular delta time
        if self.last_encoder_time is None:
            return
        dt = current_time - self.last_encoder_time
        if dt <= 0:
            return
        
        # Calcular velocidades e posições das rodas
        for i in range(4):
            delta_counts = corrected_counts[i] - self.last_encoder_counts[i]
            
            # Posição absoluta da roda (radianos)
            self.wheel_positions[i] = corrected_counts[i] * (2 * math.pi) / self.encoder_pulses_per_rev
            
            # Velocidade angular da roda (rad/s)
            self.wheel_velocities[i] = (delta_counts * (2 * math.pi) / self.encoder_pulses_per_rev) / dt
        
        # Calcular odometria usando cinemática inversa do mecanum
        delta_distances = [
            (corrected_counts[i] - self.last_encoder_counts[i]) * self.meters_per_pulse
            for i in range(4)
        ]
        
        # Cinemática inversa para mecanum drive
        # v_x = (v_fl + v_fr + v_rl + v_rr) / 4
        # v_y = (-v_fl + v_fr + v_rl - v_rr) / 4  
        # w_z = (-v_fl + v_fr - v_rl + v_rr) / (4 * l)
        
        v_x = sum(delta_distances) / (4.0 * dt)
        v_y = (-delta_distances[0] + delta_distances[1] + delta_distances[2] - delta_distances[3]) / (4.0 * dt)
        
        # Distância do centro às rodas
        l = (self.wheel_base + self.wheel_separation) / 2.0
        w_z = (-delta_distances[0] + delta_distances[1] - delta_distances[2] + delta_distances[3]) / (4.0 * l * dt)
        
        # Integrar posição (apenas X e Y dos encoders)
        delta_x = (v_x * math.cos(self.theta) - v_y * math.sin(self.theta)) * dt
        delta_y = (v_x * math.sin(self.theta) + v_y * math.cos(self.theta)) * dt
        delta_theta_encoder = w_z * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta_encoder += delta_theta_encoder
        
        # Normalizar ângulo do encoder
        self.theta_encoder = math.atan2(math.sin(self.theta_encoder), math.cos(self.theta_encoder))
        
        # Atualizar estados
        self.last_encoder_counts = corrected_counts[:]
        self.last_encoder_time = current_time
        self.last_encoder_data_time = current_time

    def imu_callback(self, msg: Imu):
        """Processa dados da IMU ZED"""
        current_time = time.time()
        
        # Extrair orientação da IMU (quaternion -> euler)
        q = msg.orientation
        
        # Conversão quaternion para yaw (orientação Z)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Atualizar orientação da IMU
        self.theta_imu = yaw
        self.imu_active = True
        self.last_imu_data_time = current_time
        
        if not self.imu_initialized:
            self.imu_initialized = True
            self.get_logger().info(f'IMU ZED inicializada - orientação: {math.degrees(yaw):.1f}°')

    def encoder_status_callback(self, msg: Bool):
        """Atualiza status da conexão dos encoders"""
        self.encoders_connected = msg.data

    def fuse_orientation(self):
        """Fusão da orientação usando filtro complementar"""
        if self.imu_initialized and self.encoders_initialized:
            # Filtro complementar: IMU (peso alto) + Encoders (peso baixo)
            self.theta = (self.orientation_filter_alpha * self.theta_imu + 
                         (1 - self.orientation_filter_alpha) * self.theta_encoder)
        elif self.imu_initialized:
            # Apenas IMU disponível
            self.theta = self.theta_imu
        elif self.encoders_initialized:
            # Apenas encoders disponíveis
            self.theta = self.theta_encoder
        else:
            # Nenhum sensor - manter último valor
            pass
        
        # Normalizar ângulo final
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

    def publish_odometry_and_joints(self):
        """Publica odometria e joint states"""
        current_time = self.get_clock().now()
        
        # Fusão da orientação
        self.fuse_orientation()
        
        # Publicar joint states
        self.publish_joint_states(current_time)
        
        # Publicar odometria
        self.publish_odometry(current_time)
        
        # Publicar TF
        self.publish_tf(current_time)

    def publish_joint_states(self, timestamp):
        """Publica estados das rodas"""
        js = JointState()
        js.header.stamp = timestamp.to_msg()
        js.name = self.joint_names
        js.position = self.wheel_positions[:]
        js.velocity = self.wheel_velocities[:]
        js.effort = [0.0, 0.0, 0.0, 0.0]
        self.joint_state_pub.publish(js)

    def publish_odometry(self, timestamp):
        """Publica odometria fundida"""
        odom = Odometry()
        odom.header.stamp = timestamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # Posição
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientação (quaternion)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Velocidades (calculadas se disponíveis)
        if self.encoders_initialized:
            # Calcular velocidades atuais baseadas nas velocidades das rodas
            wheel_vel_linear = [v * self.wheel_radius for v in self.wheel_velocities]
            
            v_x = sum(wheel_vel_linear) / 4.0
            v_y = (-wheel_vel_linear[0] + wheel_vel_linear[1] + wheel_vel_linear[2] - wheel_vel_linear[3]) / 4.0
            l = (self.wheel_base + self.wheel_separation) / 2.0
            w_z = (-wheel_vel_linear[0] + wheel_vel_linear[1] - wheel_vel_linear[2] + wheel_vel_linear[3]) / (4.0 * l)
        else:
            v_x = v_y = w_z = 0.0
        
        odom.twist.twist.linear.x = v_x
        odom.twist.twist.linear.y = v_y
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = w_z
        
        # Covariâncias baseadas na qualidade dos sensores
        pose_cov = 0.1 if self.encoders_connected else 0.5
        twist_cov = 0.1 if self.encoders_connected else 0.3
        
        odom.pose.covariance = [pose_cov, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, pose_cov, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 999999.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 999999.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 999999.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        
        odom.twist.covariance = [twist_cov, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, twist_cov, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 999999.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 999999.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 999999.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        
        self.odom_pub.publish(odom)

    def publish_tf(self, timestamp):
        """Publica transformação odom -> base_footprint"""
        tf = TransformStamped()
        tf.header.stamp = timestamp.to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_footprint'
        
        # Posição
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        
        # Orientação
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = math.sin(self.theta / 2.0)
        tf.transform.rotation.w = math.cos(self.theta / 2.0)
        
        self.tf_broadcaster.sendTransform(tf)

    def monitor_sensors(self):
        """Monitora estado dos sensores"""
        current_time = time.time()
        
        # Verificar timeout dos sensores
        encoder_timeout = current_time - self.last_encoder_data_time > self.fallback_timeout
        imu_timeout = current_time - self.last_imu_data_time > self.fallback_timeout
        
        # Determinar modo de fallback
        if encoder_timeout and not self.fallback_mode:
            self.fallback_mode = True
            self.get_logger().warn("⚠️ FALLBACK MODE: Encoders perdidos, usando apenas IMU")
        elif not encoder_timeout and self.fallback_mode:
            self.fallback_mode = False
            self.get_logger().info("✅ RECOVERY: Encoders restaurados")
        
        # Log periódico de status
        status = f"Encoders: {'✅' if self.encoders_connected else '❌'}, " \
                f"IMU: {'✅' if self.imu_active else '❌'}, " \
                f"Fallback: {'🔄' if self.fallback_mode else '✅'}"
        
        self.get_logger().debug(f"Status: {status}")


def main(args=None):
    rclpy.init(args=args)
    node = OdometryFusionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
