# Robo Caramelo - Sistema ROS 2 (Jazzy) para Navegacao e Manipulacao @Work

Este repositorio contem a arquitetura de software do robo Caramelo, uma plataforma movel omnidirecional com quatro rodas mecanum, desenvolvida para a RoboCup@Work. O sistema utiliza ROS 2 Jazzy e foi projetado para controle embarcado real, com base em tutoriais do site Automatic Addison, mas com estrutura propria adaptada para o robo fisico.

---

## Estrutura de pacotes (local: ~/Caramelo_workspace/src)

### caramelo_description
Tipo: simulacao (Gazebo Classic ou Ignition)  
Estado: implementado

Contem os arquivos Xacro e URDF para descrever o robo no ambiente de simulacao. Este pacote e exclusivo para simulacao no Gazebo, com sensores e atuadores virtuais.  
Nao deve ser utilizado no robo real.

---

### caramelo_bringup
Tipo: execucao no robo real  
Estado: funcional

Contem os arquivos launch e scripts para inicializacao dos modulos do robo real. Possui seu proprio URDF com plugins reais (ros2_control, joint_state_broadcaster, mecanum_drive_controller).

Arquivos principais:
- encoder_bringup.launch.py: leitura de encoders via ESP32
- pwm_bringup.launch.py: controle de motores reais via PWM
- lidar_bringup.launch.py: ativacao do LiDAR RPLidar
- teleop_keyboard.launch.py: controle por teclado
- visualization.launch.py: inicializacao do RViz2 com configuracao do robo real

Observacoes:
- Todos os modulos devem ser executados com `ros2 launch`, sem uso de arquivos .sh
- encoder_bringup e pwm_bringup devem ser executados manualmente em terminais separados por causa da arquitetura com ESP32

---

### caramelo_navigation
Tipo: navegacao autonoma com Nav2 e mapeamento  
Estado: em desenvolvimento

Responsavel pelo SLAM, criacao e uso de mapas e configuracao do stack Nav2.

Arquivos:
- map.yaml + map.pgm: mapa gerado com SLAM
- nav2_params.yaml: parametros do Nav2 (amcl, planner, controller, etc)
- teleop_mapping.launch.py: mapeamento manual usando teclado
- goalpose_mapping.launch.py: mapeamento via objetivo do RViz2
- waypoint_creation.launch.py: geracao de waypoints nomeados em arquivo JSON

Melhorias planejadas:
- Criar navigation.launch.py com Nav2 completo
- Adicionar amcl com correcao de odometria
- Criar pasta separada para mapas e waypoints
- Adicionar execucao automatica baseada em waypoints

---

### caramelo_tasks
Tipo: execucao de tarefas definidas por YAML  
Estado: em desenvolvimento

Executa sequencias de tarefas de transporte com base em um arquivo YAML que define o objeto, origem e destino. Utiliza a navegacao para ir ate as estacoes.

Funcoes planejadas:
- Cliente de acao Nav2 com timeout
- Maquina de estados com tolerancia a falhas
- Integracao com o pacote de manipulacao
- Logs estruturados em niveis (info, warn, error)

---

### caramelo_manipulation
Tipo: manipulacao de objetos  
Estado: ainda nao implementado

Pacote para controle do manipulador do robo. Inicialmente com funcoes simuladas (mock), e futuramente com integracao real via ROS 2 Control ou MQTT.

Planejado:
- detect_object() e manipulate_object() simuladas
- Controle real por comandos seriais ou ros2_control
- Integracao com camera ZED para deteccao visual

---

### mecanum_drive_controller
Tipo: plugin ROS 2 Control  
Estado: funcional

Controlador personalizado para rodas mecanum. Converte comandos Twist em velocidades individuais para cada roda.

Parametros principais:
- wheel_radius: raio da roda
- lx e ly: distancia do centro do robo aos eixos X e Y
- feedback de velocidade por encoder

---

## Pacotes externos utilizados

| Pacote         | Funcao                                | Observacoes                            |
|----------------|----------------------------------------|-----------------------------------------|
| rplidar_ros    | Driver do LiDAR Slamtec               | Publica o topico /scan                  |
| zed_ros2       | SDK ROS 2 da ZED                      | Para visao stereo e SLAM no futuro      |
| zed_wrapper, zed_interfaces, zed_components | Complementos do SDK da ZED         | Estao presentes mas ainda nao usados    |

---

## Estado atual do sistema

| Componente                | Estado         |
|---------------------------|----------------|
| Simulacao com Gazebo      | implementado   |
| URDF e controle real      | funcional      |
| SLAM e mapeamento         | em progresso   |
| Navegacao autonoma        | em progresso   |
| Execucao de tarefas       | em progresso   |
| Manipulador               | nao iniciado   |
| Visao computacional       | nao iniciado   |

---

## Meta final

Desenvolver um sistema ROS 2 completo e modular para o robo Caramelo que permita:
- Navegacao autonoma com base em mapas e waypoints
- Execucao de tarefas do tipo pick and place com tolerancia a falhas
- Manipulacao de objetos com ou sem visao computacional
- Expansibilidade para IA, SLAM visual e operacao continua

---

## Referencias

Repositorio de exemplo (base conceitual):  
https://github.com/automaticaddison/yahboom_rosmaster

Tutoriais utilizados:  
https://automaticaddison.com/autonomous-navigation-for-a-mobile-robot-using-ros-2-jazzy/  
https://automaticaddison.com/building-a-map-of-the-environment-using-slam-ros-2-jazzy/
