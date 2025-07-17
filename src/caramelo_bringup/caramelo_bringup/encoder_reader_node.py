#!/usr/bin/env python3
"""
🔧 ENCODER READER NODE - Leitor Dedicado de Encoders ESP32

FUNÇÃO ÚNICA:
============
Lê dados JSON seriais dos encoders ESP32 e publica no tópico ROS2
Não faz cálculos de odometria - apenas leitura e publicação raw

RESPONSABILIDADES:
=================
- Conectar com ESP32 dos encoders (/dev/ttyUSB1)
- Ler dados JSON: {"enc_fl": X, "enc_fr": Y, "enc_rl": Z, "enc_rr": W}
- Publicar no tópico /encoder_data
- Gerenciar reconexão automática
- Log de status da conexão

PUBLICAÇÕES:
============
- /encoder_data (std_msgs/Int32MultiArray) - [FL, FR, RL, RR] raw counts
- /encoder_status (std_msgs/Bool) - Status da conexão ESP32

DESIGN:
=======
✅ SIMPLES: Foco único em leitura serial
✅ ROBUSTO: Reconexão automática sem reset
✅ EFICIENTE: Sem cálculos desnecessários
✅ MODULAR: Outros nós usam os dados publicados
"""

import json
import time
from typing import Optional

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray


class EncoderReaderNode(Node):
    def __init__(self):
        super().__init__('encoder_reader_node')
        
        # =====================================================
        # PARÂMETROS CONFIGURÁVEIS
        # =====================================================
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('serial_baudrate', 115200)
        self.declare_parameter('update_frequency', 50.0)
        self.declare_parameter('reconnect_timeout', 5.0)
        
        # Carregar parâmetros
        self.serial_port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.serial_baudrate = self.get_parameter('serial_baudrate').get_parameter_value().integer_value
        self.update_freq = self.get_parameter('update_frequency').get_parameter_value().double_value
        self.reconnect_timeout = self.get_parameter('reconnect_timeout').get_parameter_value().double_value
        
        # =====================================================
        # PUBLISHERS
        # =====================================================
        self.encoder_data_pub = self.create_publisher(
            Int32MultiArray, '/encoder_data', 10)
        self.connection_status_pub = self.create_publisher(
            Bool, '/encoder_status', 10)
        
        # =====================================================
        # ESTADO INTERNO
        # =====================================================
        self.serial_port: Optional[serial.Serial] = None
        self.connection_established = False
        self.last_successful_read = time.time()
        self.total_reads = 0
        self.failed_reads = 0
        
        # =====================================================
        # SETUP INICIAL
        # =====================================================
        self.setup_serial_connection()
        
        # Timer principal de leitura
        self.read_timer = self.create_timer(
            1.0 / self.update_freq, self.read_encoder_data)
        
        # Timer de status (mais lento)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # Timer de reconexão (quando necessário)
        self.reconnect_timer = self.create_timer(
            self.reconnect_timeout, self.check_connection)
        
        self.get_logger().info('🔧 ENCODER READER - Iniciando...')
        self.get_logger().info(f'Porta serial: {self.serial_port_name}')
        self.get_logger().info(f'Baudrate: {self.serial_baudrate}')
        self.get_logger().info(f'Frequência: {self.update_freq} Hz')

    def setup_serial_connection(self):
        """Estabelece conexão inicial com ESP32"""
        try:
            # Fechar conexão anterior se existir
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
                time.sleep(0.5)
            
            # Nova conexão
            self.serial_port = serial.Serial(
                self.serial_port_name, 
                baudrate=self.serial_baudrate, 
                timeout=0.1
            )
            
            self.get_logger().info(f"🔌 ESP32 encoders conectada em {self.serial_port_name}")
            
            # Reset da ESP32 apenas na inicialização
            self.get_logger().info("🔄 Reset inicial da ESP32...")
            self.serial_port.dtr = False
            time.sleep(0.1)
            self.serial_port.dtr = True
            time.sleep(2.0)  # Aguarda ESP32 reiniciar
            
            # Limpar buffers
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            
            self.connection_established = True
            self.last_successful_read = time.time()
            self.get_logger().info("✅ ESP32 encoders pronta!")
            
        except Exception as e:
            self.get_logger().error(f"❌ Falha ao conectar ESP32: {e}")
            self.connection_established = False

    def reconnect_serial(self):
        """Reconecta sem reset (preserva estado da ESP32)"""
        try:
            self.get_logger().warn("🔄 Reconectando ESP32 (sem reset)...")
            
            # Fechar conexão anterior
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
                time.sleep(0.2)
            
            # Reconectar
            self.serial_port = serial.Serial(
                self.serial_port_name, 
                baudrate=self.serial_baudrate, 
                timeout=0.1
            )
            
            # Limpar apenas buffers (sem reset da ESP32)
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            
            self.connection_established = True
            self.last_successful_read = time.time()
            self.get_logger().info("✅ ESP32 reconectada!")
            
        except Exception as e:
            self.get_logger().error(f"❌ Falha na reconexão: {e}")
            self.connection_established = False

    def read_encoder_data(self):
        """Lê dados dos encoders e publica"""
        if not self.connection_established or not self.serial_port:
            return
            
        try:
            # Tentar ler linha da serial
            line = self.serial_port.readline().decode('utf-8').strip()
            if not line:
                return
                
            # Parse do JSON
            data = json.loads(line)
            
            # Extrair contadores (valores raw da ESP32)
            raw_counts = [
                data.get('enc_fl', 0), 
                data.get('enc_fr', 0), 
                data.get('enc_rl', 0), 
                data.get('enc_rr', 0)
            ]
            
            # Publicar dados raw
            msg = Int32MultiArray()
            msg.data = raw_counts
            self.encoder_data_pub.publish(msg)
            
            # Atualizar estatísticas
            self.total_reads += 1
            self.last_successful_read = time.time()
            
            # Log periódico (a cada 100 leituras)
            if self.total_reads % 100 == 0:
                self.get_logger().debug(
                    f"📊 Leituras: {self.total_reads}, "
                    f"Falhas: {self.failed_reads}, "
                    f"Dados: FL={raw_counts[0]}, FR={raw_counts[1]}, "
                    f"RL={raw_counts[2]}, RR={raw_counts[3]}"
                )
                
        except json.JSONDecodeError:
            self.failed_reads += 1
            self.get_logger().debug("⚠️ Erro de JSON - dados incompletos")
            
        except Exception as e:
            self.failed_reads += 1
            self.get_logger().warn(f"⚠️ Erro na leitura: {e}")

    def publish_status(self):
        """Publica status da conexão"""
        status_msg = Bool()
        status_msg.data = self.connection_established
        self.connection_status_pub.publish(status_msg)

    def check_connection(self):
        """Verifica e reconecta se necessário"""
        current_time = time.time()
        time_since_last_read = current_time - self.last_successful_read
        
        # Se passou muito tempo sem leitura, tentar reconectar
        if time_since_last_read > self.reconnect_timeout and self.connection_established:
            self.get_logger().warn(
                f"⏰ Sem dados há {time_since_last_read:.1f}s - reconectando..."
            )
            self.connection_established = False
            self.reconnect_serial()
        
        # Se não conectado, tentar reconectar
        elif not self.connection_established:
            self.reconnect_serial()

    def __del__(self):
        """Cleanup na destruição"""
        if hasattr(self, 'serial_port') and self.serial_port and self.serial_port.is_open:
            self.serial_port.close()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderReaderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
