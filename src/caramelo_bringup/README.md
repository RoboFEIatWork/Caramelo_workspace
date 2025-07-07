# 🤖 Sistema Caramelo - RoboCup@Work 2024

Sistema completo e integrado para o robô Caramelo competir na RoboCup@Work 2024.

## 🚀 **INÍCIO RÁPIDO - ROBÔ REAL**

### Pré-requisitos
```bash
# 1. Terminal 1: PWM Bringup
ros2 run your_package pwm_bringup

# 2. Terminal 2: Encoder Bringup  
ros2 run your_package encoder_bringup

# 3. Terminal 3: Build do sistema
cd /home/work/Caramelo_workspace
colcon build --packages-select caramelo_bringup caramelo_tasks caramelo_manipulation
source install/setup.bash
```

### Execução do Sistema Completo

#### 🏆 Modo Competição (Recomendado)
```bash
# Sistema mínimo para competição
ros2 launch caramelo_bringup competition.launch.py

# Com mapa e tarefas customizadas
ros2 launch caramelo_bringup competition.launch.py \
  map_file:=/caminho/para/mapa.yaml \
  task_file:=tarefas_competicao.yaml
```

#### 🔧 Modo Desenvolvimento (Com RViz)
```bash
# Sistema completo com visualização
ros2 launch caramelo_bringup system.launch.py

# Com configurações customizadas
ros2 launch caramelo_bringup system.launch.py \
  map_file:=/caminho/para/mapa.yaml \
  task_file:=tarefas_teste.yaml \
  initial_pose_x:=1.0 \
  initial_pose_y:=2.0 \
  initial_pose_yaw:=0.0
```

## 📋 **CONFIGURAÇÃO DE TAREFAS**

### Formato do Arquivo YAML
Criar arquivo em `/home/work/Caramelo_workspace/src/caramelo_tasks/config/`:

```yaml
task_list:
  - object: "R20"
    pick_from: "WS3"
    place_to: "WS5"
  - object: "M20"
    pick_from: "WS1"
    place_to: "WS2"
  - object: "F20_20_B"
    pick_from: "WS4"
    place_to: "WS6"

workstations:
  WS1:
    position: [1.0, 1.0]
    orientation: [0.0, 0.0, 0.0, 1.0]
  WS2:
    position: [2.0, 1.0]
    orientation: [0.0, 0.0, 0.0, 1.0]
  WS3:
    position: [1.0, 2.0]
    orientation: [0.0, 0.0, 0.0, 1.0]
  WS4:
    position: [2.0, 2.0]
    orientation: [0.0, 0.0, 0.0, 1.0]
  WS5:
    position: [3.0, 1.0]
    orientation: [0.0, 0.0, 0.0, 1.0]
  WS6:
    position: [3.0, 2.0]
    orientation: [0.0, 0.0, 0.0, 1.0]
```

### Coordenadas das Workstations
Use o sistema de waypoints já criado ou crie novos:

```bash
# Criar waypoints interativamente
ros2 launch caramelo_navigation interactive_waypoint_creator.launch.py

# Converter waypoints para workstations
# Use as coordenadas dos waypoints WS01, WS02, etc. como WS1, WS2, etc.
```

## 🏗️ **ARQUITETURA DO SISTEMA**

### Componentes Principais

1. **caramelo_navigation** - Navegação autônoma
   - Nav2 stack completo
   - AMCL para localização
   - Evitação de obstáculos dinâmicos

2. **caramelo_tasks** - Executor de tarefas
   - Lê arquivos YAML
   - Coordena navegação e manipulação
   - Tolerância a falhas (até 3 tentativas)

3. **caramelo_manipulation** - Sistema de manipulação
   - Interface mock para testes
   - Integração com manipulador real
   - Detecção e manipulação de objetos

4. **caramelo_bringup** - Lançamento do sistema
   - Launch files integrados
   - Configuração automatizada
   - Modo competição e desenvolvimento

## 🔧 **TESTES E DEBUGGING**

### Testar Componentes Individuais

```bash
# 1. Testar apenas navegação
ros2 launch caramelo_navigation autonomous_navigation.launch.py

# 2. Testar apenas task executor
ros2 run caramelo_tasks task_executor_node --ros-args -p task_file:=test_tasks.yaml

# 3. Testar apenas manipulação
ros2 run caramelo_manipulation manipulation_interface

# 4. Testar navegação por waypoints
ros2 run caramelo_navigation autonomous_waypoint_navigator
```

### Logs e Monitoramento

```bash
# Ver logs do sistema
ros2 topic echo /rosout

# Monitorar status da navegação
ros2 topic echo /navigate_to_pose/_action/status

# Monitorar comandos de manipulação
ros2 topic echo /manipulation_command

# Ver transformações
ros2 run tf2_tools view_frames
```

## 🎯 **WORKFLOW PARA COMPETIÇÃO**

### Preparação
1. ✅ Verificar hardware (PWM, encoders, LiDAR)
2. ✅ Carregar mapa do ambiente
3. ✅ Configurar tarefas no YAML
4. ✅ Testar sistema completo

### Execução
1. 🚀 Iniciar PWM e encoder bringup
2. 🚀 Executar `ros2 launch caramelo_bringup competition.launch.py`
3. 🚀 Robô executa tarefas automaticamente
4. 🚀 Monitorar logs para debug se necessário

### Fallbacks
- Sistema continua mesmo se uma tarefa falhar
- Navegação robusta com evitação de obstáculos
- Manipulação com múltiplas tentativas

## 📊 **MÉTRICAS E PERFORMANCE**

O sistema registra:
- ✅ Tempo por tarefa
- ✅ Taxa de sucesso na navegação
- ✅ Taxa de sucesso na manipulação
- ✅ Obstáculos detectados e evitados
- ✅ Falhas e recuperações

## 🔮 **PRÓXIMOS PASSOS**

1. **Integração com Visão Real**
   - Substituir detecção mock por OpenCV/ZED
   - Reconhecimento de objetos com IA

2. **Manipulador Físico**
   - Integração com ROS 2 Control
   - Controle preciso de posição

3. **Otimizações**
   - Planejamento de trajetória otimizado
   - Algoritmos de busca mais eficientes

---

## 🆘 **TROUBLESHOOTING**

### Problemas Comuns

**Navegação não funciona:**
- Verificar TF tree: `ros2 run tf2_tools view_frames`
- Verificar odometria: `ros2 topic echo /odom`
- Verificar mapa: `ros2 topic echo /map`

**Task executor não inicia:**
- Verificar arquivo YAML: `cat /path/to/tasks.yaml`
- Verificar Nav2: `ros2 node list | grep nav`

**Manipulação falha sempre:**
- Verificar interface: `ros2 topic list | grep manipulation`
- Verificar logs: `ros2 topic echo /rosout`

### Contatos
- **Desenvolvimento:** Time RoboFEI
- **Issues:** GitHub do projeto
- **Competição:** RoboCup@Work 2024

---

**🏆 Bom trabalho e boa sorte na competição! 🏆**
