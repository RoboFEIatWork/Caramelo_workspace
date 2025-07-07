# 🚀 Sistema de Inicialização do Robô Caramelo

## 📋 Ordem de Inicialização para Competição RoboCup@Work

### 1. **Pré-requisitos (Terminais Separados)**
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

### 2. **Sistema Principal (Terminal 3)**
```bash
# Terminal 3 - Sistema Completo
cd /home/work/Caramelo_workspace
source install/setup.bash

# Opção 1: Sistema padrão
ros2 launch caramelo_bringup system.launch.py

# Opção 2: Sistema com parâmetros customizados
ros2 launch caramelo_bringup system.launch.py \
  map_file:=/home/work/Caramelo_workspace/meu_mapa.yaml \
  task_file:=competicao_tasks.yaml \
  initial_pose_x:=0.5 \
  initial_pose_y:=0.5 \
  initial_pose_yaw:=1.57 \
  use_rviz:=true
```

## 🔧 O que o `system.launch.py` faz

### ✅ Componentes Incluídos:
1. **Robot Description** - URDF do robô
2. **Navigation Stack** - Nav2, AMCL, Map Server
3. **Task Executor** - Executor de tarefas YAML
4. **RViz** - Visualização (opcional)

### ❌ Componentes NÃO Incluídos:
- **PWM Bringup** - Deve estar em terminal separado
- **Encoder Bringup** - Deve estar em terminal separado
- **LiDAR** - Assumido que já está rodando

### 🎯 Razão do Design:
- **Flexibilidade**: PWM e Encoder podem ser reiniciados independentemente
- **Debugging**: Cada componente tem seu próprio terminal para logs
- **Robustez**: Se um componente falhar, outros continuam funcionando
- **Restrição do Sistema**: Conforme solicitado, PWM e Encoder em terminais separados

## 📊 Fluxo de Dados do Sistema

```
[PWM Bringup] → /cmd_vel → [Motores do Robô]
     ↑
[Task Executor] → [Navigation Stack] → [PWM Bringup]
     ↑                    ↑
[Arquivo YAML]       [Encoder Bringup] → /odom
                          ↑
                     [LiDAR] → /scan
```

## 🔍 Verificação do Sistema

### Verificar se tudo está funcionando:
```bash
# Verificar tópicos ativos
ros2 topic list

# Verificar se os nós estão rodando
ros2 node list

# Verificar transformações
ros2 run tf2_tools view_frames

# Verificar status da navegação
ros2 topic echo /amcl_pose
```

### Tópicos Importantes:
- `/cmd_vel` - Comandos para os motores
- `/odom` - Odometria dos encoders
- `/scan` - Dados do LiDAR
- `/amcl_pose` - Posição estimada do robô
- `/goal_pose` - Goals de navegação

## 🚨 Troubleshooting

### Se PWM não responder:
1. Verificar se o terminal do PWM está ativo
2. Verificar se `/cmd_vel` está sendo publicado
3. Reiniciar apenas o PWM bringup

### Se Encoder não funcionar:
1. Verificar se o terminal do Encoder está ativo
2. Verificar se `/odom` está sendo publicado
3. Reiniciar apenas o Encoder bringup

### Se Navegação falhar:
1. Verificar se o mapa está correto
2. Verificar se AMCL está localizando o robô
3. Verificar se há obstáculos no caminho

## 🎯 Para Competição

### Checklist de Inicialização:
1. ✅ **Terminal 1**: PWM Bringup rodando
2. ✅ **Terminal 2**: Encoder Bringup rodando  
3. ✅ **Terminal 3**: Sistema principal iniciado
4. ✅ **Verificar**: Todos os tópicos ativos
5. ✅ **Confirmar**: Robô respondendo aos comandos

## 🏁 Autonomia Total

Uma vez que todos os componentes estão rodando:
1. **O robô opera autonomamente**
2. **Executa tarefas do arquivo YAML**
3. **Navega entre workstations**
4. **Tenta manipular objetos**
5. **Continua mesmo com falhas**

O sistema está desenhado para **máxima autonomia** após a inicialização!
