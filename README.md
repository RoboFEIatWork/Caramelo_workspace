# 🤖 Robô Caramelo - Sistema Completo RoboCup@Work 2024

## 📋 **GUIA COMPLETO DE OPERAÇÃO**

### 🚀 **1. SYSTEM.LAUNCH - Sistema Completo de Competição**

**O que faz:**
- Inicia Navigation Stack (Nav2, AMCL, Map Server)
- Inicia Task Executor (executa tarefas YAML)
- Inicia Robot Description (URDF)
- Inicia RViz (opcional)

**Pré-requisitos (OBRIGATÓRIOS):**
```bash
# Terminal 1 - PWM Bringup
cd /home/work/Caramelo_workspace
source install/setup.bash
ros2 launch caramelo_bringup pwm_bringup.launch.py

# Terminal 2 - Encoder Bringup  
cd /home/work/Caramelo_workspace
source install/setup.bash
ros2 launch caramelo_bringup encoder_bringup.launch.py
```

**Uso:**
```bash
# Terminal 3 - Sistema Completo
cd /home/work/Caramelo_workspace
source install/setup.bash

# Competição padrão
ros2 launch caramelo_bringup system.launch.py

# Com parâmetros customizados
ros2 launch caramelo_bringup system.launch.py \
  map_file:=/home/work/Caramelo_workspace/meu_mapa.yaml \
  task_file:=competicao_tasks.yaml \
  initial_pose_x:=0.5 \
  initial_pose_y:=0.5 \
  initial_pose_yaw:=1.57 \
  use_rviz:=true
```

---

### 🎯 **2. WAYPOINT NAVIGATION - Navegação por Waypoints**

**O que faz:**
- Navega automaticamente por waypoints pré-definidos
- Evita obstáculos dinâmicos
- Posicionamento preciso em cada waypoint

**Pré-requisitos:**
```bash
# Terminal 1 - PWM Bringup
ros2 launch caramelo_bringup pwm_bringup.launch.py

# Terminal 2 - Encoder Bringup
ros2 launch caramelo_bringup encoder_bringup.launch.py
```

**Uso:**
```bash
# Terminal 3 - Waypoint Navigation
cd /home/work/Caramelo_workspace
source install/setup.bash

# Navegação padrão
ros2 launch caramelo_navigation autonomous_navigation.launch.py

# Com arquivos customizados
ros2 launch caramelo_navigation autonomous_navigation.launch.py \
  map_file:=/home/work/Caramelo_workspace/meu_mapa.yaml \
  waypoints_file:=/home/work/Caramelo_workspace/src/caramelo_navigation/config/meus_waypoints.json
```

---

### 🗺️ **3. NAVIGATION - Navegação Manual/Goals**

**O que faz:**
- Permite enviar goals manualmente via RViz
- Sistema de navegação completo Nav2
- Localização AMCL

**Pré-requisitos:**
```bash
# Terminal 1 - PWM Bringup
ros2 launch caramelo_bringup pwm_bringup.launch.py

# Terminal 2 - Encoder Bringup
ros2 launch caramelo_bringup encoder_bringup.launch.py
```

**Uso:**
```bash
# Terminal 3 - Navigation
cd /home/work/Caramelo_workspace
source install/setup.bash

# Navegação padrão
ros2 launch caramelo_navigation navigation_launch.py

# Com mapa customizado
ros2 launch caramelo_navigation navigation_launch.py \
  map_file:=/home/work/Caramelo_workspace/meu_mapa.yaml \
  use_rviz:=true
```

**Como usar:**
1. Abrir RViz
2. Usar "2D Nav Goal" para enviar goals
3. Robô navega automaticamente

---

### �️ **4. TELEOP - Controle Manual**

**O que faz:**
- Controle manual do robô via teclado
- Permite movimentação omnidirecional
- Útil para testes e posicionamento

**Pré-requisitos:**
```bash
# Terminal 1 - PWM Bringup
ros2 launch caramelo_bringup pwm_bringup.launch.py

# Terminal 2 - Encoder Bringup
ros2 launch caramelo_bringup encoder_bringup.launch.py
```

**Uso:**
```bash
# Terminal 3 - Teleop
cd /home/work/Caramelo_workspace
source install/setup.bash

# Controle por teclado
ros2 launch caramelo_bringup teleop_keyboard.launch.py
```

---

### 🗺️ **5. MAPPING - Criação de Mapas**

**O que faz:**
- Cria mapas do ambiente usando SLAM
- Salva mapas em formato .pgm e .yaml
- Permite exploração do ambiente

**Pré-requisitos:**
```bash
# Terminal 1 - PWM Bringup
ros2 launch caramelo_bringup pwm_bringup.launch.py

# Terminal 2 - Encoder Bringup
ros2 launch caramelo_bringup encoder_bringup.launch.py

# Terminal 3 - LiDAR
ros2 launch caramelo_bringup lidar_bringup.launch.py
```

**Uso:**
```bash
# Terminal 4 - SLAM
cd /home/work/Caramelo_workspace
source install/setup.bash

# Iniciar SLAM
ros2 launch caramelo_navigation slam_launch.py

# Terminal 5 - Teleop para explorar
ros2 launch caramelo_bringup teleop_keyboard.launch.py
```

**Como usar:**
1. Mover o robô pelo ambiente usando teleop
2. Observar mapa sendo criado no RViz
3. Salvar mapa quando completo:
```bash
ros2 run nav2_map_server map_saver_cli -f /home/work/Caramelo_workspace/novo_mapa
```

---

### 📍 **6. GOAL POSE MAPPING - Criação de Waypoints**

**O que faz:**
- Permite marcar waypoints interativamente
- Cria arquivos JSON com coordenadas
- Interface visual no RViz

**Pré-requisitos:**
```bash
# Terminal 1 - PWM Bringup
ros2 launch caramelo_bringup pwm_bringup.launch.py

# Terminal 2 - Encoder Bringup
ros2 launch caramelo_bringup encoder_bringup.launch.py
```

**Uso:**
```bash
# Terminal 3 - Waypoint Creator
cd /home/work/Caramelo_workspace
source install/setup.bash

# Criar waypoints interativamente
ros2 launch caramelo_navigation interactive_waypoint_creator.launch.py

# Com mapa customizado
ros2 launch caramelo_navigation interactive_waypoint_creator.launch.py \
  map_file:=/home/work/Caramelo_workspace/meu_mapa.yaml
```

**Como usar:**
1. Abrir RViz
2. Usar "2D Nav Goal" para marcar waypoints
3. Nomear waypoints (WS01, WS02, etc.)
4. Salvar quando terminar

### 4. Controle do Robô
```bash
# Terminal separado: Teleoperação
ros2 launch caramelo_bringup teleop_keyboard.launch.py
```

### 4. Visualização
- **RViz**: Já incluído no launch completo
- **Fixed Frame**: `odom` ou `base_link`
- **Tópicos**: `/scan`, `/odom`, modelo URDF

## 🔧 Arquitetura do Sistema

### Mapeamento de Hardware
| Dispositivo | Porta | Função |
|-------------|-------|--------|
| ESP32 PWM | `/dev/ttyUSB0` | Controle dos motores |
| ESP32 Encoders | `/dev/ttyUSB1` | Odometria das rodas |
| RPLidar S2 | `/dev/ttyUSB2` | Sensor LIDAR |
| ZED 2i | `/dev/video*` | Câmera + IMU |
| RealSense | `/dev/video*` | Câmera de profundidade |

### Arquitetura de Software
```
ROS2 Node Graph:
├── mecanum_drive_controller (PWM + Encoders)
├── rplidar_node (LiDAR)
├── zed_wrapper (IMU + Câmera)
├── ekf_filter_node (Fusão sensorial)
├── slam_toolbox (Mapeamento)
└── robot_state_publisher (TF tree)
```

### Fusão Sensorial
- **Encoders**: Velocidades das rodas (vx, vy, vyaw)
- **ZED IMU**: Orientação e velocidade angular
- **EKF**: Fusão encoder + IMU para odometria robusta
- **LiDAR**: SLAM e detecção de obstáculos

## 📁 Estrutura do Projeto

### Pacotes Principais
- **`caramelo_bringup`**: Launches e configurações
- **`caramelo_navigation`**: SLAM, Nav2 e fusão sensorial
- **`caramelo_description`**: URDF e modelo do robô
- **`caramelo_controller`**: Controladores customizados

### Launches de Bringup (Sensores)
- **`pwm_bringup.launch.py`**: Controlador PWM dos motores
- **`encoder_bringup.launch.py`**: Controlador de encoders
- **`lidar_bringup.launch.py`**: Sensor LiDAR RPLidar S2
- **`zed_imu_bringup.launch.py`**: ZED com foco no IMU
- **`visualization_bringup.launch.py`**: RViz para visualização
- **`teleop_keyboard.launch.py`**: Controle por teclado

### Launches de Navegação
- **`slam_launch.py`**: SLAM para mapeamento
- **`caramelo_navigation_launch.py`**: Navegação com Nav2
- **`ekf_encoder_imu_launch.py`**: Fusão sensorial EKF

## 🛠️ Uso Avançado

### Inicialização Modular Completa
```bash
# Terminal 1: Controlador PWM
ros2 launch caramelo_bringup pwm_bringup.launch.py

# Terminal 2: Controlador de Encoders  
ros2 launch caramelo_bringup encoder_bringup.launch.py

# Terminal 3: LiDAR
ros2 launch caramelo_bringup lidar_bringup.launch.py

# Terminal 4: ZED IMU (quando disponível)
ros2 launch caramelo_bringup zed_imu_bringup.launch.py

# Terminal 5: SLAM ou Navegação
ros2 launch caramelo_navigation slam_launch.py
# OU
ros2 launch caramelo_navigation caramelo_navigation_launch.py

# Terminal 6: Visualização
ros2 launch caramelo_bringup visualization_bringup.launch.py

# Terminal 7: Controle
ros2 launch caramelo_bringup teleop_keyboard.launch.py
```

### SLAM e Mapeamento
```bash
# Iniciar SLAM
ros2 launch caramelo_navigation slam_launch.py

# Salvar mapa
ros2 run nav2_map_server map_saver_cli -f meu_mapa
```

### Navegação Autônoma
```bash
# Carregar mapa e navegar
ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=false \
    map:=/path/to/meu_mapa.yaml
```

## 🔍 Monitoramento e Diagnósticos

### Verificar Status do Sistema
```bash
# Tópicos ativos
ros2 topic list
ros2 topic echo /odom        # Odometria
ros2 topic echo /scan        # LiDAR
ros2 topic echo /zed/imu/data # IMU

# Árvore TF
ros2 run tf2_tools view_frames

# Performance
ros2 topic hz /odom          # Frequência odometria
ros2 topic hz /scan          # Frequência LiDAR
```

### Dispositivos USB
```bash
# Verificar dispositivos conectados
ls -la /dev/ttyUSB*
ls -la /dev/video*

# Recarregar regras udev se necessário
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 🐛 Solução de Problemas

### Problemas Comuns

**1. Dispositivos USB não encontrados**
```bash
# Verificar conexões físicas
lsusb
dmesg | tail

# Verificar permissões
sudo chmod 777 /dev/ttyUSB*
```

**2. Build do workspace falha**
```bash
# Limpar e rebuildar
rm -rf build/ install/ log/
colcon build
source install/setup.bash
```

**3. ZED não conecta**
```bash
# Verificar SDK instalado
/usr/local/zed/tools/ZED_Explorer

# Verificar processo
ps aux | grep zed
```

**4. TF não funciona**
```bash
# Verificar sincronização de tempo
ros2 param set /ekf_filter_node use_sim_time false

# Debug TF
ros2 run tf2_ros tf2_echo base_link laser_frame
```

---

## 🔧 **VERIFICAÇÕES DO SISTEMA**

### Verificar se tudo está funcionando:
```bash
# Verificar tópicos ativos
ros2 topic list

# Verificar nós rodando
ros2 node list

# Verificar transformações
ros2 run tf2_tools view_frames

# Verificar se robô está publicando odometria
ros2 topic echo /odom

# Verificar se robô está recebendo comandos
ros2 topic echo /cmd_vel

# Verificar dados do LiDAR
ros2 topic echo /scan
```

### Tópicos Importantes:
- `/cmd_vel` - Comandos para os motores
- `/odom` - Odometria dos encoders
- `/scan` - Dados do LiDAR
- `/amcl_pose` - Posição estimada do robô
- `/goal_pose` - Goals de navegação
- `/tf` - Transformações entre frames

---

## 🚨 **TROUBLESHOOTING**

### PWM não responde:
1. Verificar se terminal PWM está ativo
2. `ros2 topic echo /cmd_vel` deve mostrar comandos
3. Reiniciar PWM bringup se necessário

### Encoder não funciona:
1. Verificar se terminal Encoder está ativo
2. `ros2 topic echo /odom` deve mostrar odometria
3. Reiniciar Encoder bringup se necessário

### Navegação falha:
1. Verificar se mapa está correto
2. Verificar se AMCL está localizando (`ros2 topic echo /amcl_pose`)
3. Verificar se há obstáculos no caminho

### LiDAR não detecta:
1. Verificar se LiDAR está conectado
2. `ros2 topic echo /scan` deve mostrar dados
3. Reiniciar LiDAR se necessário

---

## 📋 **ARQUIVOS DE CONFIGURAÇÃO**

### Mapas:
- `/home/work/Caramelo_workspace/mapa_20250704_145039.yaml` - Mapa principal
- `/home/work/Caramelo_workspace/mapa_20250704_145039.pgm` - Imagem do mapa

### Waypoints:
- `/home/work/Caramelo_workspace/src/caramelo_navigation/config/waypoints.json` - Waypoints padrão

### Tarefas:
- `/home/work/Caramelo_workspace/src/caramelo_tasks/config/tasks.yaml` - Tarefas da competição

### Configurações:
- `/home/work/Caramelo_workspace/src/caramelo_navigation/config/nav2_params.yaml` - Parâmetros Nav2
- `/home/work/Caramelo_workspace/src/caramelo_navigation/config/ekf_params.yaml` - Filtro EKF

---

## 🏆 **MODO COMPETIÇÃO**

### Para RoboCup@Work:
1. ✅ **PWM e Encoder** rodando em terminais separados
2. ✅ **Mapa** do ambiente carregado
3. ✅ **Tarefas** configuradas no YAML
4. ✅ **Sistema** iniciado com `system.launch.py`
5. ✅ **Robô** opera autonomamente

### Comando único para competição:
```bash
# Após PWM e Encoder estarem rodando
ros2 launch caramelo_bringup system.launch.py map_file:=/home/work/Caramelo_workspace/mapa_20250704_145039.yaml task_file:=tasks.yaml
```

**O robô executará todas as tarefas automaticamente!** 🚀

**Status**: ✅ Sistema completo, integrado e operacional  
**Versão ROS2**: Jazzy Jalisco  
**Última atualização**: Junho 2025  

**O robô Caramelo está pronto para operação, pesquisa e desenvolvimento!**
