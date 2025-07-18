from datetime import datetime
from dynamixel_sdk import PortHandler, PacketHandler
from excute_manip import executar_movimento_seguindo_dh


class DynamixelMotor:
    def __init__(self, motor_id, port_handler, protocol=2.0):
        self.motor_id = motor_id
        self.port_handler = port_handler
        self.protocol = protocol
        self.packet_handler = PacketHandler(protocol)

        if protocol == 2.0:
            self.ADDR_TORQUE_ENABLE = 64
            self.ADDR_GOAL_POSITION = 116
            self.ADDR_PRESENT_POSITION = 132
            self.ADDR_PROFILE_VELOCITY = 112
            self.ADDR_PROFILE_ACCELERATION = 108
        else:
            self.ADDR_TORQUE_ENABLE = 24
            self.ADDR_GOAL_POSITION = 30
            self.ADDR_PRESENT_POSITION = 36
            self.ADDR_PROFILE_VELOCITY = None
            self.ADDR_PROFILE_ACCELERATION = None

        self.goal_position = 0  # Inicializado a zero, atualizado após torque

    def enable_torque(self):
        self.packet_handler.write1ByteTxRx(self.port_handler, self.motor_id, self.ADDR_TORQUE_ENABLE, 1)

    def disable_torque(self):
        self.packet_handler.write1ByteTxRx(self.port_handler, self.motor_id, self.ADDR_TORQUE_ENABLE, 0)

    def set_profile(self, acceleration, velocity):
        if self.protocol == 2.0:
            self.packet_handler.write4ByteTxRx(
                self.port_handler, self.motor_id, self.ADDR_PROFILE_ACCELERATION, acceleration
            )
            self.packet_handler.write4ByteTxRx(
                self.port_handler, self.motor_id, self.ADDR_PROFILE_VELOCITY, velocity
            )

    def set_goal_position(self, position):
        self.goal_position = position
        self.packet_handler.write4ByteTxRx(
            self.port_handler, self.motor_id, self.ADDR_GOAL_POSITION, int(position)
        )
    

    def get_present_position(self):
        position, _, _ = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.motor_id, self.ADDR_PRESENT_POSITION
        )
        return position
    
    def get_present_position_deg(self):
        """Retorna o ângulo atual do motor em graus no intervalo [0°, 360°]"""
        pos = self.get_present_position()  # 0 a 4095
        return (pos * 360.0) / 4096.0

