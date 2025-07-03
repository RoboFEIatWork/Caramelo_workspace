# 🤖 Sistema de Mapeamento Manual Seguro - Robô Caramelo

## 🎯 Visão Geral

Sistema de mapeamento manual com navegação por goals do RViz e proteção contra colisão integrada.

## 🔧 Melhorias Implementadas

### 🛡️ Proteção Contra Colisão
- **Detecção de obstáculos**: LIDAR monitora região frontal (±45°)
- **Distância de segurança**: 50cm mínimo para obstáculos
- **Parada automática**: Robô para imediatamente se obstáculo detectado
- **Cancelamento de goal**: Objetivo é cancelado em caso de risco de colisão

### 🚦 Filtro de Segurança
- **Filtro CMD_VEL**: Todos os comandos passam por verificação de segurança
- **Limites globais**: Velocidade máxima controlada (30cm/s linear, 0.5rad/s angular)
- **Timeout de segurança**: Comandos zero enviados se sistema inativo

### 🎮 Navegação Melhorada
- **Velocidades reduzidas**: 20cm/s linear máximo, 0.3rad/s angular máximo
- **Tolerância aumentada**: 15cm para posição, evita oscilações
- **Dupla verificação**: Obstáculos checados antes de cada movimento

## 📋 Como Usar

### 1. Terminal 1 - Hardware
```bash
cd /home/work/Caramelo_workspace
source install/setup.bash
ros2 launch caramelo_bringup pwm_bringup.launch.py
```

### 2. Terminal 2 - Mapeamento Manual
```bash
cd /home/work/Caramelo_workspace
source install/setup.bash
ros2 launch caramelo_bringup manual_mapping.launch.py
```

### 3. Navegação no RViz
1. **RViz abrirá automaticamente**
2. **Use "2D Nav Goal"** para enviar pontos de navegação
3. **Robô navegará com segurança** para cada ponto
4. **Mapa será construído** conforme o robô explora

### 4. Salvar Mapa
```bash
ros2 run nav2_map_server map_saver_cli -f maps/mapa_seguro
```

## 🛠️ Monitoramento (Opcional)

### Monitor de Comandos
```bash
ros2 run caramelo_bringup cmd_vel_monitor
```

### Status da Navegação
```bash
ros2 topic echo /navigation_status
```

## 🔍 Arquitetura do Sistema

```
RViz "2D Nav Goal" 
    ↓
/move_base_simple/goal (remapeado para /goal_pose)
    ↓
Simple Waypoint Navigator (com detecção de obstáculos)
    ↓
/cmd_vel_raw
    ↓
CMD_VEL Safety Filter (limites globais)
    ↓
/cmd_vel
    ↓
Twist Converter Node
    ↓
/mecanum_drive_controller/cmd_vel (TwistStamped)
    ↓
Mecanum Drive Controller
    ↓
Hardware do Robô
```

## ⚠️ Características de Segurança

1. **Detecção Ativa**: LIDAR monitora constantemente obstáculos à frente
2. **Parada Imediata**: Robô para ao detectar obstáculo < 50cm
3. **Cancelamento de Goal**: Objetivos são cancelados em situações de risco
4. **Filtro Global**: Todos os comandos passam por limites de segurança
5. **Timeout**: Sistema para automaticamente se não receber comandos

## 🎛️ Parâmetros Ajustáveis

### No Simple Waypoint Navigator:
- `min_obstacle_distance`: 0.5m (distância mínima para obstáculos)
- `max_linear_speed`: 0.20 m/s (velocidade linear máxima)
- `max_angular_speed`: 0.3 rad/s (velocidade angular máxima)
- `linear_tolerance`: 0.15m (tolerância de posição)

### No CMD_VEL Safety Filter:
- `max_linear_speed`: 0.3 m/s (limite global linear)
- `max_angular_speed`: 0.5 rad/s (limite global angular)

## 🚨 Resolução de Problemas

### Robô não se move
1. Verificar se hardware está ligado
2. Verificar tópicos: `ros2 topic list | grep cmd_vel`
3. Monitorar comandos: `ros2 run caramelo_bringup cmd_vel_monitor`

### Robô para sempre
1. Verificar LIDAR: `ros2 topic echo /scan`
2. Verificar logs de obstáculos nos terminais
3. Ajustar `min_obstacle_distance` se necessário

### Mapa com baixa qualidade
1. Navegar mais devagar (goals próximos)
2. Verificar odometria: `ros2 topic echo /odom`
3. Verificar TF: `ros2 run tf2_tools view_frames`

## ✅ Sistema Testado e Funcionando

- ✅ Goal pose do RViz funcionando
- ✅ Navegação segura até objetivos
- ✅ Parada automática por obstáculos
- ✅ Mapa de alta qualidade gerado
- ✅ Proteção contra colisão ativa
- ✅ Sistema isolado (sem explorador autônomo interferindo)

## 🎉 Pronto para Uso!

O sistema agora está seguro para mapeamento manual. Use o RViz para enviar goals e o robô navegará com segurança, evitando colisões e gerando mapas de alta qualidade.
