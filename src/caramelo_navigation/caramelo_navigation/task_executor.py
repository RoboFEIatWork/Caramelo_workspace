#!/usr/bin/env python3
"""
CARAMELO TASK EXECUTOR
=====================

Nó ROS2 que executa tasks automaticamente baseado em arquivos task.yaml.
O robô navega para cada destino, para 10 segundos, e vai para o próximo.
Cada mesa só é visitada uma vez.

Uso:
    ros2 run caramelo_navigation task_executor --ros-args -p task:=BTT1 -p arena:=arena_robocup25

Parâmetros:
    - task: Nome da task (BTT1, BTT2, BMT, etc.) - pasta em src/caramelo_tasks/
    - arena: Nome da arena (padrão: arena_robocup25)
    - wait_time: Tempo de espera em cada WS em segundos (padrão: 10)
    - auto_start: Iniciar automaticamente (padrão: false)

Autor: GitHub Copilot
Data: 2025-07-19
"""

import json
import os
import time
from typing import Dict, List, Optional

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String


class TaskExecutor(Node):
    """
    Executa tasks automaticamente baseado em arquivos task.yaml.
    """

    def __init__(self):
        super().__init__('task_executor')
        
        # Parâmetros
        self.declare_parameter('task', '')
        self.declare_parameter('arena', 'arena_robocup25')
        self.declare_parameter('wait_time', 10.0)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('navigation_timeout', 60.0)  # Timeout para navegação
        self.declare_parameter('go_to_finish', True)  # Ir para FINISH no final
        
        self.task_name = self.get_parameter('task').get_parameter_value().string_value
        self.arena = self.get_parameter('arena').get_parameter_value().string_value
        self.wait_time = self.get_parameter('wait_time').get_parameter_value().double_value
        self.auto_start = self.get_parameter('auto_start').get_parameter_value().bool_value
        self.navigation_timeout = self.get_parameter('navigation_timeout').get_parameter_value().double_value
        self.go_to_finish = self.get_parameter('go_to_finish').get_parameter_value().bool_value
        
        # Validar parâmetros obrigatórios
        if not self.task_name:
            self.get_logger().error("❌ Parâmetro 'task' é obrigatório!")
            self.get_logger().error("   Uso: ros2 run caramelo_navigation task_executor --ros-args -p task:=BTT1")
            exit(1)
        
        # Estado da execução
        self.task_data = {}
        self.destinations = []
        self.visited_workstations = set()
        self.current_destination_index = 0
        self.is_executing = False
        self.navigation_completed = False
        self.navigation_timeout_timer = None
        self.current_navigation_start_time = None
        
        # Publishers e Subscribers
        self.navigation_publisher = self.create_publisher(
            String,
            'navigate_to_workstation',
            10
        )
        
        self.status_subscriber = self.create_subscription(
            String,
            'navigation_status',
            self._navigation_status_callback,
            10
        )
        
        # Timer para verificar status
        self.status_timer = self.create_timer(1.0, self._check_status)
        
        self.get_logger().info(f"🤖 Task Executor iniciado!")
        self.get_logger().info(f"   Task: {self.task_name}")
        self.get_logger().info(f"   Arena: {self.arena}")
        self.get_logger().info(f"   Tempo de espera: {self.wait_time}s")
        
        # Carregar task
        if self._load_task():
            self.get_logger().info(f"✅ Task '{self.task_name}' carregada com {len(self.destinations)} destinos")
            self._display_task_info()
            
            if self.auto_start:
                self.get_logger().info("🚀 Iniciando execução automática...")
                self._start_execution()
            else:
                self.get_logger().info("⏳ Task carregada. Use 'ros2 topic pub /task_executor/start std_msgs/String \"data: start\"' para iniciar")
                # Subscriber para comando de início manual
                self.start_subscriber = self.create_subscription(
                    String,
                    'task_executor/start',
                    self._start_command_callback,
                    10
                )
        else:
            self.get_logger().error("❌ Falha ao carregar task. Saindo...")
            exit(1)

    def _load_task(self) -> bool:
        """Carrega o arquivo task.yaml da pasta especificada."""
        try:
            # Caminho para o arquivo task.yaml
            task_file_path = f"/home/work/Caramelo_workspace/src/caramelo_tasks/{self.task_name}/task.yaml"
            
            if not os.path.exists(task_file_path):
                self.get_logger().error(f"❌ Arquivo task.yaml não encontrado: {task_file_path}")
                self.get_logger().error(f"   Certifique-se de que a pasta 'src/caramelo_tasks/{self.task_name}' existe")
                return False
            
            # Carregar YAML
            with open(task_file_path, 'r') as f:
                self.task_data = yaml.safe_load(f)
            
            # Extrair destinos (assumindo estrutura padrão do caramelo_tasks)
            self._parse_destinations()
            
            return len(self.destinations) > 0
            
        except Exception as e:
            self.get_logger().error(f"❌ Erro ao carregar task: {e}")
            return False

    def _parse_destinations(self):
        """Extrai lista de destinos únicos do task.yaml."""
        self.destinations = []
        
        # Tentar diferentes formatos de task.yaml
        if 'movements' in self.task_data:
            # Formato: movements com from_ws e to_ws
            for movement in self.task_data['movements']:
                if 'from_ws' in movement:
                    self.destinations.append(movement['from_ws'])
                if 'to_ws' in movement:
                    self.destinations.append(movement['to_ws'])
                    
        elif 'destinations' in self.task_data:
            # Formato: lista direta de destinos
            self.destinations = self.task_data['destinations']
            
        elif 'workstations' in self.task_data:
            # Formato: lista de workstations
            self.destinations = self.task_data['workstations']
            
        else:
            # Tentar extrair de qualquer campo que contenha "WS"
            self._extract_workstations_from_any_field()
        
        # Remover duplicatas mantendo ordem
        unique_destinations = []
        for dest in self.destinations:
            if dest not in unique_destinations and isinstance(dest, str) and ('WS' in dest or dest in ['START', 'FINISH']):
                unique_destinations.append(dest)
        
        self.destinations = unique_destinations
        
        if not self.destinations:
            self.get_logger().warn("⚠️  Nenhum destino encontrado no task.yaml")
            self.get_logger().warn("   Formatos suportados: 'movements', 'destinations', 'workstations'")

    def _extract_workstations_from_any_field(self):
        """Extrai workstations de qualquer campo do YAML."""
        import re
        
        def extract_ws_recursive(obj):
            workstations = []
            if isinstance(obj, dict):
                for value in obj.values():
                    workstations.extend(extract_ws_recursive(value))
            elif isinstance(obj, list):
                for item in obj:
                    workstations.extend(extract_ws_recursive(item))
            elif isinstance(obj, str):
                # Buscar padrões WS01, WS02, etc. ou START, FINISH
                ws_matches = re.findall(r'\b(WS\d+|START|FINISH)\b', obj)
                workstations.extend(ws_matches)
            return workstations
        
        self.destinations = extract_ws_recursive(self.task_data)

    def _display_task_info(self):
        """Exibe informações da task carregada."""
        self.get_logger().info("📋 INFORMAÇÕES DA TASK:")
        self.get_logger().info(f"   📁 Arquivo: src/caramelo_tasks/{self.task_name}/task.yaml")
        self.get_logger().info(f"   🎯 Destinos encontrados: {len(self.destinations)}")
        
        for i, dest in enumerate(self.destinations, 1):
            self.get_logger().info(f"      {i}. {dest}")
        
        total_time = len(self.destinations) * self.wait_time
        self.get_logger().info(f"   ⏱️  Tempo estimado: ~{total_time:.0f}s (sem contar navegação)")
        self.get_logger().info(f"   🚫 Timeout navegação: {self.navigation_timeout}s")
        self.get_logger().info(f"   🏁 Ir para FINISH: {'Sim' if self.go_to_finish else 'Não'}")

    def _start_command_callback(self, msg):
        """Callback para comando de início manual."""
        if msg.data.lower() in ['start', 'begin', 'go']:
            self.get_logger().info("🚀 Comando de início recebido!")
            self._start_execution()

    def _start_execution(self):
        """Inicia a execução da task."""
        if self.is_executing:
            self.get_logger().warn("⚠️  Task já está em execução!")
            return
        
        if not self.destinations:
            self.get_logger().error("❌ Nenhum destino para executar!")
            return
        
        self.is_executing = True
        self.current_destination_index = 0
        self.visited_workstations.clear()
        
        self.get_logger().info("🏁 INICIANDO EXECUÇÃO DA TASK!")
        self._execute_next_destination()

    def _execute_next_destination(self):
        """Executa navegação para o próximo destino."""
        if self.current_destination_index >= len(self.destinations):
            # Se configurado, ir para FINISH antes de completar
            if self.go_to_finish and 'FINISH' not in self.visited_workstations:
                self.get_logger().info("🏁 Indo para FINISH...")
                self._navigate_to_finish()
                return
            
            self._complete_task()
            return
        
        current_dest = self.destinations[self.current_destination_index]
        
        # Verificar se já visitamos esta workstation
        if current_dest in self.visited_workstations:
            self.get_logger().info(f"⏭️  Pulando {current_dest} (já visitada)")
            self.current_destination_index += 1
            self._execute_next_destination()
            return
        
        # Navegar para destino
        self.get_logger().info(f"🎯 Navegando para {current_dest} ({self.current_destination_index + 1}/{len(self.destinations)})")
        
        msg = String()
        msg.data = current_dest
        self.navigation_publisher.publish(msg)
        
        self.navigation_completed = False
        self.visited_workstations.add(current_dest)
        self.current_navigation_start_time = time.time()
        
        # Iniciar timer de timeout para navegação
        self._start_navigation_timeout()

    def _navigation_status_callback(self, msg):
        """Callback para status da navegação."""
        try:
            status_data = json.loads(msg.data)
            status = status_data.get('status', '')
            message = status_data.get('message', '')
            
            if status == "SUCCESS":
                self.get_logger().info(f"✅ Chegou ao destino! {message}")
                self.navigation_completed = True
                self._cancel_navigation_timeout()  # Cancelar timeout
                
                # Verificar se chegamos ao FINISH
                if 'FINISH' in self.visited_workstations and self.current_destination_index >= len(self.destinations):
                    self._complete_task()
                else:
                    self._wait_at_destination()
                
            elif status == "FAILED" or status == "ERROR":
                self.get_logger().error(f"❌ Falha na navegação: {message}")
                self._handle_navigation_failure()
                
            elif status == "NAVIGATING":
                self.get_logger().info(f"🚶 Navegando... {message}")
                
        except json.JSONDecodeError:
            pass  # Ignorar mensagens mal formatadas

    def _wait_at_destination(self):
        """Aguarda na workstation por tempo especificado."""
        current_dest = self.destinations[self.current_destination_index]
        self.get_logger().info(f"⏸️  Aguardando {self.wait_time}s em {current_dest}...")
        
        # Usar timer para aguardar (não bloqueia o nó)
        self.wait_timer = self.create_timer(
            self.wait_time,
            self._finish_waiting
        )

    def _finish_waiting(self):
        """Termina a espera e vai para próximo destino."""
        # Cancelar timer
        if hasattr(self, 'wait_timer'):
            self.wait_timer.cancel()
            self.destroy_timer(self.wait_timer)
        
        current_dest = self.destinations[self.current_destination_index]
        self.get_logger().info(f"✅ Tempo de espera em {current_dest} concluído!")
        
        # Ir para próximo destino
        self.current_destination_index += 1
        self._execute_next_destination()

    def _handle_navigation_failure(self):
        """Lida com falhas de navegação."""
        current_dest = self.destinations[self.current_destination_index]
        self.get_logger().error(f"❌ Falha ao navegar para {current_dest}")
        
        # Cancelar timer de timeout se existir
        self._cancel_navigation_timeout()
        
        # Pular destino e continuar
        self.get_logger().info("⏭️  Pulando destino falho e continuando...")
        self.current_destination_index += 1
        self._execute_next_destination()

    def _start_navigation_timeout(self):
        """Inicia timer de timeout para navegação."""
        self._cancel_navigation_timeout()  # Cancelar timeout anterior se existir
        
        self.navigation_timeout_timer = self.create_timer(
            self.navigation_timeout,
            self._handle_navigation_timeout
        )
        
        self.get_logger().info(f"⏱️  Timeout de navegação: {self.navigation_timeout}s")

    def _cancel_navigation_timeout(self):
        """Cancela timer de timeout de navegação."""
        if self.navigation_timeout_timer:
            self.navigation_timeout_timer.cancel()
            self.destroy_timer(self.navigation_timeout_timer)
            self.navigation_timeout_timer = None

    def _handle_navigation_timeout(self):
        """Lida com timeout de navegação."""
        current_dest = self.destinations[self.current_destination_index]
        elapsed_time = time.time() - self.current_navigation_start_time if self.current_navigation_start_time else 0
        
        self.get_logger().error(f"⏰ TIMEOUT! Navegação para {current_dest} demorou {elapsed_time:.1f}s")
        self.get_logger().error(f"   Limite: {self.navigation_timeout}s")
        
        # Cancelar navegação atual e ir para próximo destino
        self._cancel_navigation_timeout()
        self.get_logger().info("🚫 Cancelando navegação atual e passando para próximo destino...")
        
        # Enviar comando de cancelamento (se necessário)
        # msg = String()
        # msg.data = "CANCEL"
        # self.navigation_publisher.publish(msg)
        
        # Ir para próximo destino
        self.current_destination_index += 1
        self._execute_next_destination()

    def _navigate_to_finish(self):
        """Navega para FINISH no final da task."""
        self.get_logger().info("🏁 Navegando para FINISH...")
        
        msg = String()
        msg.data = "FINISH"
        self.navigation_publisher.publish(msg)
        
        self.navigation_completed = False
        self.visited_workstations.add("FINISH")
        self.current_navigation_start_time = time.time()
        
        # Iniciar timeout para navegação para FINISH
        self._start_navigation_timeout()

    def _complete_task(self):
        """Finaliza a execução da task."""
        self.is_executing = False
        
        self.get_logger().info("🎉 TASK CONCLUÍDA COM SUCESSO!")
        self.get_logger().info(f"   ✅ Workstations visitadas: {len(self.visited_workstations)}")
        
        for ws in sorted(self.visited_workstations):
            self.get_logger().info(f"      • {ws}")
        
        # Opcionalmente, podemos finalizar o nó ou aguardar nova task
        self.get_logger().info("🔄 Pronto para nova task!")

    def _check_status(self):
        """Timer callback para verificar status geral."""
        # Este timer pode ser usado para verificações periódicas se necessário
        pass


def main(args=None):
    rclpy.init(args=args)
    
    try:
        executor = TaskExecutor()
        rclpy.spin(executor)
    except KeyboardInterrupt:
        print("\n⚠️  Execução interrompida pelo usuário")
    except Exception as e:
        print(f"❌ Erro durante execução: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
