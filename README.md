# 🤖 Robô Caramelo - ROS 2 Jazzy Navigation & Manipulation System

![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-green.svg)
![Platform](https://img.shields.io/badge/Platform-Real%20Robot-blue.svg)
![Status](https://img.shields.io/badge/Status-In%20Development-yellow.svg)

Sistema ROS 2 completo e modular para o robô Caramelo, uma plataforma móvel omnidirecional com quatro rodas mecanum, desenvolvida para a **RoboCup@Work**. O sistema utiliza ROS 2 Jazzy e foi projetado para controle embarcado real.

---

## 📋 Índice

1. [🏗️ Arquitetura do Sistema](#-arquitetura-do-sistema)
2. [⚙️ Instalação e Configuração](#-instalação-e-configuração)
3. [❗ IMPORTANTE: Entendendo o `$PWD`](#-importante-entendendo-o-pwd)
4. [🎮 Tutorial 1: Teleoperação Simples](#-tutorial-1-teleoperação-simples)
5. [🔧 Tutorial 2: Ativação Completa do Hardware](#-tutorial-2-ativação-completa-do-hardware)
6. [🗺️ Tutorial 3: Mapeamento com Teleoperação](#-tutorial-3-mapeamento-com-teleoperação)
7. [🎯 Tutorial 4: Mapeamento com Goal Pose](#-tutorial-4-mapeamento-com-goal-pose)
8. [📍 Tutorial 5: Criação de Waypoints](#-tutorial-5-criação-de-waypoints)
9. [🧭 Tutorial 6: Navegação Autônoma](#-tutorial-6-navegação-autônoma)
10. [📁 Estrutura de Mapas e Waypoints](#-estrutura-de-mapas-e-waypoints)
11. [🔍 Solução de Problemas](#-solução-de-problemas)

---

## 🏗️ Arquitetura do Sistema

### Pacotes Principais

| Pacote | Função | Status |
|--------|--------|--------|
| **`caramelo_bringup`** | Hardware do robô real (encoders, PWM, LiDAR) | ✅ Funcional |
| **`caramelo_navigation`** | Sistema de navegação Nav2 + SLAM | ⚠️ Em progresso |
| **`caramelo_tasks`** | Execução de tarefas por YAML | ⚠️ Em desenvolvimento |
| **`caramelo_manipulation`** | Controle do manipulador | ❌ Não iniciado |
| **`caramelo_description`** | URDF para simulação Gazebo | ✅ Implementado |

### Hardware Suportado
- **Rodas:** 4x Mecanum wheels com encoders
- **Sensores:** RPLidar S2, ZED 2i (futuro)
- **Controle:** ESP32 para encoders, PWM para motores
- **Manipulador:** Planejado para versões futuras

---

## ⚙️ Instalação e Configuração

### Pré-requisitos
```bash
# ROS 2 Jazzy instalado
# Nav2 stack
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup

# SLAM Toolbox
sudo apt install ros-jazzy-slam-toolbox

# RPLidar driver
sudo apt install ros-jazzy-rplidar-ros
```

### Build do Workspace
```bash
cd ~/Caramelo_workspace
colcon build
source install/setup.bash
```

---

## 🎮 Tutorial 1: Teleoperação Simples

**Objetivo:** Controlar o robô usando apenas o teclado, sem navegação.

### Passo 1: Preparar Workspace e Ativar Hardware Básico
```bash
# Terminal 1: Encoders + Robot Description
cd ~/Caramelo_workspace && source install/setup.bash
ros2 launch caramelo_bringup encoder_bringup.launch.py

# Terminal 2: Controle de Motores
cd ~/Caramelo_workspace && source install/setup.bash
ros2 launch caramelo_bringup pwm_bringup.launch.py
```

### Passo 2: Teleoperação
```bash
# Terminal 3: Controle por Teclado
cd ~/Caramelo_workspace && source install/setup.bash
ros2 launch caramelo_bringup teleop_keyboard.launch.py
```

✅ **Sucesso:** O robô deve se mover conforme os comandos do teclado.

---

## 🔧 Tutorial 2: Ativação Completa do Hardware

**Objetivo:** Ativar todos os sensores e visualizar no RViz.

### Passo 1: Hardware Completo
```bash
# Terminal 1: Encoders + Robot Description
cd ~/Caramelo_workspace && source install/setup.bash
ros2 launch caramelo_bringup encoder_bringup.launch.py

# Terminal 2: Controle de Motores
cd ~/Caramelo_workspace && source install/setup.bash
ros2 launch caramelo_bringup pwm_bringup.launch.py

# Terminal 3: LiDAR
cd ~/Caramelo_workspace && source install/setup.bash
ros2 launch caramelo_bringup lidar_bringup.launch.py
```

### Passo 2: Visualização
```bash
# Terminal 4: RViz Configurado
cd ~/Caramelo_workspace && source install/setup.bash
ros2 launch caramelo_bringup visualization_bringup.launch.py
```

**📝 NOTA:** O RViz será aberto com configuração básica que inclui:
- **Fixed Frame:** `odom` (para visualização sem mapeamento)
- **RobotModel:** Visualização 3D do robô Caramelo
- **LaserScan:** Dados do LiDAR em tempo real (tópico `/scan`)
- **TF:** Árvore de transformações (`base_link` → `odom`)
- **Odometry:** Dados de odometria
- **Grid:** Grade de referência
- Se o arquivo de configuração não carregar automaticamente, adicione manualmente:
  - **Add → By display type → RobotModel** (tópico: `/robot_description`)
  - **Add → By display type → LaserScan** (tópico: `/scan`)

### Passo 3: Teleoperação (Opcional)
```bash
# Terminal 5: Controle por Teclado
cd ~/Caramelo_workspace && source install/setup.bash
ros2 launch caramelo_bringup teleop_keyboard.launch.py
```

✅ **Sucesso:** 
- RViz mostra o robô, TF tree e dados do LiDAR
- Robô responde aos comandos de movimento
- Scan do LiDAR aparece em tempo real

---

## 🔧 Tutorial 3: Mapeamento com Teleoperação

**Objetivo:** Criar um mapa do ambiente usando controle manual do robô.

### Passo 1: Ativar Hardware Básico
```bash
# Terminal 1: Encoders + Robot Description
cd ~/Caramelo_workspace && source install/setup.bash
ros2 launch caramelo_bringup encoder_bringup.launch.py

# Terminal 2: Controle de Motores
cd ~/Caramelo_workspace && source install/setup.bash
ros2 launch caramelo_bringup pwm_bringup.launch.py
```

### Passo 2: Iniciar SLAM para Mapeamento
```bash
# Terminal 3: Mapeamento SLAM + Teleop + RViz + LiDAR
cd ~/Caramelo_workspace && source install/setup.bash
ros2 launch caramelo_navigation teleop_mapping.launch.py
```

### Passo 3: Mapear o Ambiente com Controle Manual
1. **Aguarde** o RViz abrir completamente (pode levar alguns segundos)
2. **Use as teclas do teclado** para controlar o robô.
3. **Dicas importantes:**
   - **Movimente lentamente** para mapa mais preciso
   - **Explore todos os cantos** do ambiente
   - **Observe o mapa** sendo construído em tempo real no RViz
   - **Evite movimentos bruscos** que podem confundir o SLAM
4. **No RViz você verá:**
   - **Laser scan (vermelho):** Dados do LiDAR em tempo real
   - **Mapa (cinza/preto):** Área mapeada
   - **Robô (modelo 3D):** Posição atual do robô

### Passo 4: Salvar o Mapa
```bash
# Terminal 4: Criar pasta do ambiente e salvar mapa
mkdir -p ~/Caramelo_workspace/maps/ambiente_escritorio
cd ~/Caramelo_workspace/maps/ambiente_escritorio
ros2 run nav2_map_server map_saver_cli -f map
```

✅ **Sucesso:** 
- Pasta `maps/ambiente_escritorio/` criada
- Arquivos `map.yaml` e `map.pgm` salvos na pasta do ambiente

---

## 🎯 Tutorial 4: Mapeamento com Goal Pose

**Objetivo:** Criar mapa usando navegação autônoma por objetivos no RViz.

### Passo 1: Ativar Hardware Básico
```bash
# Terminal 1: Encoders + Robot Description
cd ~/Caramelo_workspace && source install/setup.bash
ros2 launch caramelo_bringup encoder_bringup.launch.py

# Terminal 2: Controle de Motores
cd ~/Caramelo_workspace && source install/setup.bash
ros2 launch caramelo_bringup pwm_bringup.launch.py
```

### Passo 2: Mapeamento com Goals
```bash
# Terminal 3: Mapeamento por Goal Pose + SLAM + RViz + LiDAR
cd ~/Caramelo_workspace && source install/setup.bash
ros2 launch caramelo_navigation goalpose_mapping.launch.py
```

### Passo 3: Navegar por Goals no RViz
1. **Aguarde** o RViz abrir completamente (pode levar alguns segundos)
2. **Localize o botão "2D Nav Goal"** na barra superior do RViz
3. **Processo de navegação:**
   - Clique em **"2D Nav Goal"**
   - **Clique no mapa** onde quer que o robô vá
   - **Arraste** para definir a orientação (direção que o robô deve ficar)
   - **Solte** o mouse
4. **O robô navegará automaticamente:**
   - Planeja a rota evitando obstáculos
   - Move-se suavemente até o destino
   - Para automaticamente ao chegar
5. **Repita o processo** para explorar todo o ambiente
6. **Vantagem sobre teleoperação manual:**
   - Navegação mais suave e precisa
   - Evita obstáculos automaticamente
   - Cria mapas de melhor qualidade

### Passo 4: Salvar o Mapa
```bash
# Terminal 4: Criar pasta do ambiente e salvar mapa
mkdir -p ~/Caramelo_workspace/maps/arena_fei
cd ~/Caramelo_workspace/maps/arena_fei
ros2 run nav2_map_server map_saver_cli -f map
```

✅ **Sucesso:** Mapa criado com navegação autônoma durante o SLAM.

---

## � Tutorial 5: Criação de Waypoints

**Objetivo:** Criar waypoints nomeados a partir de um mapa existente.

### Passo 1: Ir para o Workspace
```bash
# Certifique-se de estar na pasta do workspace
cd ~/Caramelo_workspace
source install/setup.bash
```

### Passo 2: Waypoint Creator
```bash
# Terminal único: Criador interativo de waypoints
# SUBSTITUA "arena_fei" pelo nome da sua pasta de mapa
ros2 launch caramelo_navigation waypoint_creation.launch.py map_file:=$PWD/maps/arena_fei/map.yaml
```

**📝 IMPORTANTE - Explicação do `$PWD`:**
- **`$PWD`** = **P**rint **W**orking **D**irectory = pasta atual do terminal
- Se você está em `~/Caramelo_workspace`, então `$PWD` = `/home/work/Caramelo_workspace`
- **❌ NÃO substitua** `$PWD` por nada! O terminal faz isso automaticamente
- **✅ Substitua APENAS** `ambiente_escritorio` pelo nome da pasta do seu mapa
- **Exemplos válidos:**
  - `map_file:=$PWD/maps/competicao_robocup/map.yaml`
  - `map_file:=$PWD/maps/arena_fei/map.yaml`

### Passo 3: Criar Waypoints no RViz
1. **Aguarde** o RViz abrir completamente
2. **2D Pose Estimate:** Posicione o robô virtual no mapa
3. **2D Nav Goal:** Clique para criar um waypoint
4. **Digite o nome:** Ex: "mesa_1", "porta_entrada", "area_trabalho"
5. **Repita** para todos os pontos importantes
6. **Ctrl+C** para finalizar e salvar

### Passo 4: Verificar Waypoints
```bash
# Os waypoints são salvos automaticamente na pasta do mapa
cat ~/Caramelo_workspace/maps/arena_fei/waypoints.json
```

✅ **Sucesso:** Arquivo JSON com waypoints nomeados criado na pasta do mapa.

---

## 🧭 Tutorial 6: Navegação Autônoma

**Objetivo:** Navegar automaticamente seguindo waypoints pré-definidos.

### Passo 1: Ativar Hardware Completo
```bash
# Terminal 1: Encoders + Robot Description
cd ~/Caramelo_workspace && source install/setup.bash
ros2 launch caramelo_bringup encoder_bringup.launch.py

# Terminal 2: Controle de Motores
cd ~/Caramelo_workspace && source install/setup.bash
ros2 launch caramelo_bringup pwm_bringup.launch.py

# Terminal 3: LiDAR
cd ~/Caramelo_workspace && source install/setup.bash
ros2 launch caramelo_bringup lidar_bringup.launch.py
```

### Passo 2: Ir para o Workspace
```bash
# Certifique-se de estar na pasta do workspace
cd ~/Caramelo_workspace
source install/setup.bash
```

### Passo 3: Navegação Nav2
```bash
# Terminal 4: Sistema completo de navegação (OBRIGATÓRIO especificar mapa)
# SUBSTITUA "ambiente_escritorio" pelo nome da sua pasta de mapa
cd ~/Caramelo_workspace && source install/setup.bash
ros2 launch caramelo_navigation navigation_launch.py map_file:=$PWD/maps/ambiente_escritorio/map.yaml
```

**📝 IMPORTANTE - Explicação do `$PWD`:**
- **`$PWD`** = pasta atual (`~/Caramelo_workspace`)
- **❌ NÃO substitua** `$PWD` por nada! O terminal faz isso automaticamente
- **✅ Substitua APENAS** `ambiente_escritorio` pelo nome da pasta do seu mapa
- **Exemplos válidos:**
  - `map_file:=$PWD/maps/competicao_robocup/map.yaml`
  - `map_file:=$PWD/maps/arena_fei/map.yaml`

### Passo 4: Definir Pose Inicial
1. **Aguarde** o RViz abrir completamente
2. No RViz, clique **"2D Pose Estimate"**
3. **Clique** na posição atual do robô no mapa
4. **Ajuste** a orientação (seta) para combinar com o robô real

### Passo 5: Navegação por Goals
1. **Manual:** Use "2D Nav Goal" para enviar objetivos
2. **Automática:** Use o waypoint follower:

```bash
# Terminal 5: Seguir waypoints automaticamente
ros2 action send_goal /follow_waypoints nav2_msgs/action/FollowWaypoints "
poses:
- header:
    frame_id: map
  pose:
    position: {x: 2.0, y: 1.0, z: 0.0}
    orientation: {w: 1.0}
- header:
    frame_id: map  
  pose:
    position: {x: 4.0, y: 3.0, z: 0.0}
    orientation: {w: 1.0}
"
```

✅ **Sucesso:** Robô navega autonomamente evitando obstáculos.

---

## 📁 Estrutura de Mapas e Waypoints

```
~/Caramelo_workspace/
├── maps/                                    # 🗺️ Mapas organizados por ambiente
│   ├── ambiente_escritorio/                 # Pasta do ambiente escritório
│   │   ├── map.yaml                        # Configuração do mapa
│   │   ├── map.pgm                         # Dados do mapa (imagem)
│   │   └── waypoints.json                  # Waypoints específicos
│   │
│   ├── laboratorio_goalpose/               # Pasta do laboratório
│   │   ├── map.yaml
│   │   ├── map.pgm
│   │   └── waypoints.json
│   │
│   ├── competicao_robocup/                 # Pasta da competição
│   │   ├── map.yaml
│   │   ├── map.pgm
│   │   └── waypoints.json
│   │
│   └── arena_fei/                          # Pasta da arena FEI
│       ├── map.yaml
│       ├── map.pgm
│       └── waypoints.json
│
└── src/                                    # 📦 Código fonte ROS 2
    ├── caramelo_bringup/
    ├── caramelo_navigation/
    └── ...
```

### 📋 Convenção de Nomes:
- **Pasta:** `nome_ambiente/` (ex: `ambiente_escritorio/`)
- **Mapa:** Sempre `map.yaml` e `map.pgm`
- **Waypoints:** Sempre `waypoints.json`
- **Documentação:** TODAS as instruções estão neste README.md principal

### 🚀 Como criar novo ambiente:
```bash
# 1. Criar nova pasta
mkdir -p ~/Caramelo_workspace/maps/novo_ambiente

# 2. Ativar hardware básico (Terminais 1-2)
cd ~/Caramelo_workspace && source install/setup.bash
ros2 launch caramelo_bringup encoder_bringup.launch.py  # Terminal 1

cd ~/Caramelo_workspace && source install/setup.bash
ros2 launch caramelo_bringup pwm_bringup.launch.py      # Terminal 2  

# 3. Mapear o ambiente (Terminal 3 - já inclui LiDAR)
cd ~/Caramelo_workspace && source install/setup.bash
ros2 launch caramelo_navigation teleop_mapping.launch.py

# 4. Salvar mapa (Terminal 4)
cd ~/Caramelo_workspace/maps/novo_ambiente
ros2 run nav2_map_server map_saver_cli -f map

# 5. Criar waypoints
cd ~/Caramelo_workspace && source install/setup.bash
ros2 launch caramelo_navigation waypoint_creation.launch.py map_file:=$PWD/maps/novo_ambiente/map.yaml
```

---

## Solução de Problemas

### Problema: Robô não se move
**Solução:**
```bash
# Verificar se PWM está ativo
ros2 topic echo /cmd_vel_unstamped

# Verificar transformações
ros2 run tf2_tools view_frames
```

### Problema: LiDAR sem dados
**Solução:**
```bash
# Verificar scan topic
ros2 topic echo /scan

# Reiniciar LiDAR
sudo chmod 666 /dev/ttyUSB0
```

### Problema: Navegação imprecisa
**Solução:**
- Melhorar mapa com movimentos mais lentos
- Ajustar parâmetros AMCL em `nav2_params.yaml`
- Verificar odometria dos encoders

### Problema: RViz não conecta
**Solução:**
```bash
# Verificar se ROS está rodando
ros2 node list

# Reiniciar RViz
pkill rviz2
ros2 launch caramelo_bringup visualization_bringup.launch.py
```

### ❌ Problema Comum: "No map_file argument provided"
**Causa:** Esqueceu de especificar o arquivo do mapa nos launch files de navegação  
**Solução:**
```bash
# ERRADO - sem map_file
ros2 launch caramelo_navigation navigation_launch.py

# CORRETO - sempre especificar map_file
cd ~/Caramelo_workspace
ros2 launch caramelo_navigation navigation_launch.py map_file:=$PWD/maps/ambiente_escritorio/map.yaml
```

### ❌ Problema Comum: Erro "$PWD not found" ou caminho inválido
**Causa:** Não está na pasta correta do workspace  
**Solução:**
```bash
# 1. Sempre ir para o workspace primeiro
cd ~/Caramelo_workspace

# 2. Verificar se está na pasta correta
pwd  # Deve mostrar: /home/work/Caramelo_workspace

# 3. Verificar se pasta do mapa existe
ls maps/  # Deve listar suas pastas de ambiente

# 4. Usar o comando correto (NUNCA substitua $PWD)
ros2 launch caramelo_navigation navigation_launch.py map_file:=$PWD/maps/ambiente_escritorio/map.yaml
```

### ❌ Problema Comum: RViz não mostra dados do LiDAR
**Causa:** Configuração incorreta ou LiDAR desconectado  
**Solução:**
```bash
# 1. Verificar se LiDAR está publicando dados
ros2 topic echo /scan

# 2. Se não houver dados, verificar conexão USB
ls /dev/ttyUSB*

# 3. Dar permissão para a porta USB
sudo chmod 666 /dev/ttyUSB0

# 4. Reiniciar o LiDAR
ros2 launch caramelo_bringup lidar_bringup.launch.py
```

---

## 🚀 Próximos Passos

- [ ] **Tarefas Automáticas:** Implementar `caramelo_tasks` para execução de missões
- [ ] **Manipulação:** Adicionar controle do braço robótico
- [ ] **Visão:** Integrar ZED 2i para detecção de objetos
- [ ] **IA:** Adicionar algoritmos de decisão inteligente

---

**Desenvolvido para RoboCup@Work | ROS 2 Jazzy | 2025**