#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dynamixel_sdk import *
import os
from datetime import datetime

LOG_FILE = os.path.join(
    os.path.expanduser("~"),
    "Caramelo_workspace",
    "src",
    "caramelo_manipulation",
    "registro_movimentos.txt"
)

class DynamixelMotor:
    def __init__(self, motor_id, port_handler, protocol=2.0):
        self.motor_id = motor_id
        self.protocol = protocol
        self.port_handler = port_handler
        self.packet_handler = PacketHandler(protocol)

        if protocol == 2.0:
            self.ADDR_TORQUE_ENABLE = 64
            self.ADDR_GOAL_POSITION = 116
            self.ADDR_PROFILE_VELOCITY = 112
            self.ADDR_PROFILE_ACCELERATION = 108
        else:
            self.ADDR_TORQUE_ENABLE = 24
            self.ADDR_GOAL_POSITION = 30
            self.ADDR_PROFILE_VELOCITY = None
            self.ADDR_PROFILE_ACCELERATION = None

    def enable_torque(self):
        self.packet_handler.write1ByteTxRx(self.port_handler, self.motor_id, self.ADDR_TORQUE_ENABLE, 1)

    def disable_torque(self):
        self.packet_handler.write1ByteTxRx(self.port_handler, self.motor_id, self.ADDR_TORQUE_ENABLE, 0)

    def set_profile(self, acceleration, velocity):
        if self.protocol == 2.0:
            self.packet_handler.write4ByteTxRx(self.port_handler, self.motor_id, self.ADDR_PROFILE_ACCELERATION, acceleration)
            self.packet_handler.write4ByteTxRx(self.port_handler, self.motor_id, self.ADDR_PROFILE_VELOCITY, velocity)

    def set_goal_position(self, position):
        self.packet_handler.write4ByteTxRx(self.port_handler, self.motor_id, self.ADDR_GOAL_POSITION, int(position))


class PlayMotionNode(Node):
    def __init__(self):
        super().__init__('play_motion_node')

        # 🔧 Lê a porta serial a partir do parâmetro ROS 2
        self.declare_parameter("port", "/dev/ttyUSB0")
        port_name = self.get_parameter("port").get_parameter_value().string_value

        self.port_handler = PortHandler(port_name)

        if not self.port_handler.openPort():
            self.get_logger().error(f"❌ Erro ao abrir a porta {port_name}")
            return

        if not self.port_handler.setBaudRate(1000000):
            self.get_logger().error("❌ Erro ao configurar baudrate")
            return

        self.get_logger().info(f"✅ Porta {port_name} aberta com sucesso")
        self.motores = {}

        try:
            with open(LOG_FILE, "r") as f:
                linhas = f.readlines()

            for linha in linhas:
                linha = linha.strip()
                if linha == "":
                    continue

                # Comando para motores 6 e 7 juntos
                if "MOTORES:6&7" in linha:
                    partes = linha.split(";")
                    pos_6 = int(partes[3].split(":")[1])
                    pos_7 = int(partes[4].split(":")[1])

                    for motor_id, posicao in [(6, pos_6), (7, pos_7)]:
                        self._inicializar_motor(motor_id, protocolo=1.0)
                        self.get_logger().info(f"[Espelhado] Motor {motor_id} → Posição {posicao}")
                        self.motores[motor_id].set_goal_position(posicao)

                elif "MOTOR:" in linha:
                    partes = linha.split(";")
                    motor_id = int(partes[1].split(":")[1])
                    posicao = int(partes[2].split(":")[1])
                    protocolo = 1.0 if motor_id in [6, 7] else 2.0

                    self._inicializar_motor(motor_id, protocolo)
                    self.get_logger().info(f"[Normal] Motor {motor_id} → Posição {posicao}")
                    self.motores[motor_id].set_goal_position(posicao)

        except Exception as e:
            self.get_logger().error(f"Erro ao processar movimentos: {e}")

        finally:
            for motor in self.motores.values():
                motor.disable_torque()
            self.port_handler.closePort()
            self.get_logger().info("✅ Porta serial fechada")

    def _inicializar_motor(self, motor_id, protocolo):
        if motor_id not in self.motores:
            motor = DynamixelMotor(motor_id, self.port_handler, protocolo)
            motor.enable_torque()
            motor.set_profile(acceleration=30, velocity=90)
            self.motores[motor_id] = motor


def main(args=None):
    rclpy.init(args=args)
    node = PlayMotionNode()
    rclpy.shutdown()

