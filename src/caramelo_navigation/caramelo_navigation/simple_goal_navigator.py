#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node


class SimpleGoalNavigator(Node):
    """
    Navegador simples que leva o robô até um ponto específico no mapa.
    """

    def __init__(self):
        super().__init__('simple_goal_navigator')
        
        # Declarar parâmetros para o ponto alvo
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 1.0)
        self.declare_parameter('goal_yaw', 0.0)
        
        # Pegar valores dos parâmetros
        self.goal_x = self.get_parameter('goal_x').get_parameter_value().double_value
        self.goal_y = self.get_parameter('goal_y').get_parameter_value().double_value
        self.goal_yaw = self.get_parameter('goal_yaw').get_parameter_value().double_value
        
        # Inicializar o navegador básico
        self.navigator = BasicNavigator()
        
        self.get_logger().info(f'Navegador simples iniciado. Objetivo: ({self.goal_x}, {self.goal_y})')
        
        # Timer para aguardar Nav2 estar ativo e iniciar navegação
        self.initialization_timer = self.create_timer(3.0, self.start_navigation)
        
    def start_navigation(self):
        """Inicia a navegação até o ponto objetivo."""
        try:
            self.get_logger().info('Aguardando Nav2 ficar ativo...')
            
            # Aguardar Nav2 estar completamente ativo
            self.navigator.waitUntilNav2Active()
            
            self.get_logger().info('Nav2 ativo! Definindo pose inicial...')
            self.initialization_timer.cancel()
            
            # Definir pose inicial (origem do mapa)
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            initial_pose.pose.position.x = 0.0
            initial_pose.pose.position.y = 0.0
            initial_pose.pose.position.z = 0.0
            initial_pose.pose.orientation.x = 0.0
            initial_pose.pose.orientation.y = 0.0
            initial_pose.pose.orientation.z = 0.0
            initial_pose.pose.orientation.w = 1.0
            
            self.navigator.setInitialPose(initial_pose)
            self.get_logger().info('Pose inicial definida')
            
            # Aguardar um momento para AMCL processar
            time.sleep(2.0)
            
            # Navegar até o objetivo
            self.navigate_to_goal()
            
        except Exception as e:
            self.get_logger().error(f'Erro na inicialização: {str(e)}')
    
    def navigate_to_goal(self):
        """Navega até o ponto objetivo definido nos parâmetros."""
        # Criar o objetivo
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = self.goal_x
        goal_pose.pose.position.y = self.goal_y
        goal_pose.pose.position.z = 0.0
        
        # Converter yaw para quaternion (rotação apenas em Z)
        import math
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = math.sin(self.goal_yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(self.goal_yaw / 2.0)
        
        self.get_logger().info(f'Navegando para objetivo: ({self.goal_x}, {self.goal_y}, {self.goal_yaw})')
        
        # Enviar o objetivo para o Nav2
        self.navigator.goToPose(goal_pose)
        
        # Monitorar o progresso
        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            
            # Log de progresso a cada 5 segundos
            if i % 50 == 0:  # 50 iterações * 0.1s = 5s
                self.get_logger().info('Navegando para o objetivo...')
            
            # Verificar se o ROS ainda está rodando
            if not rclpy.ok():
                break
                
            # Pequena pausa
            time.sleep(0.1)
        
        # Verificar resultado
        result = self.navigator.getResult()
        
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('🎯 OBJETIVO ALCANÇADO COM SUCESSO!')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('⚠️ Navegação cancelada')
        elif result == TaskResult.FAILED:
            self.get_logger().error('❌ Navegação falhou')
        else:
            self.get_logger().error(f'Status desconhecido: {result}')
    
    def shutdown(self):
        """Finaliza o navegador."""
        self.navigator.lifecycleShutdown()


def main(args=None):
    rclpy.init(args=args)
    navigator = None
    
    try:
        navigator = SimpleGoalNavigator()
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        print('\nNavegação interrompida pelo usuário')
    finally:
        if navigator is not None:
            navigator.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
