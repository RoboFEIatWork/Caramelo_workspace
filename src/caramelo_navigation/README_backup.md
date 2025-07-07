# Caramelo Navigation

Pacote de navegação completo ### 1. Navegação Autônoma (Sistema Completo)

```bash
# ⚠️ IMPORTANTE: Primeiro rodar PWM e Encoder bringup!
# Depois usar um dos comandos abaixo:

# Navegação com arquivos padrão
ros2 launch caramelo_navigation autonomous_navigation.launch.py

# Navegação com arquivos customizados
ros2 launch caramelo_navigation autonomous_navigation.launch.py \
  map_file:=/home/work/Caramelo_workspace/mapa_customizado.yaml \
  waypoints_file:=/home/work/Caramelo_workspace/src/caramelo_navigation/config/waypoints_customizado.json
```ramelo com navegação autônoma por waypoints.

## ⚠️ **CHECKLIST PARA ROBÔ REAL**

Antes de testar, verificar:
- [ ] Computador conectado nos USBs do robô
- [ ] PWM bringup rodando (`ros2 run your_package pwm_bringup`)
- [ ] Encoder bringup rodando (`ros2 run your_package encoder_bringup`)
- [ ] Mapa existe e está correto
- [ ] Waypoints criados e salvos no JSON
- [ ] LiDAR funcionando (verificar `/scan`)

## Arquivos Importantes

## Funcionalidades

### 🤖 Navegação Autônoma
- Navegação automática pelos waypoints WS01, WS02, WS03, etc.
- Desvio de obstáculos dinâmicos
- Posicionamento preciso com orientação correta
- Recuperação automática quando não consegue chegar no waypoint

### 📍 Criação de Waypoints
- Sistema interativo usando RViz
- Salvamento automático no formato JSON
- Edição manual de waypoints

### 🗺️ Sistemas de Mapeamento
- SLAM para criação de mapas
- Navegação com mapas existentes
- Localização com AMCL

## Scripts Principais

- `autonomous_waypoint_navigator.py` - **Navegador autônomo** (NOVO!)
- `interactive_robot_positioner.py` - Criador interativo de waypoints
- `simple_waypoint_navigator.py` - Navegador básico
- `checkpoint_navigator.py` - Navegador de checkpoints

## Como Usar

### 🤖 **INSTRUÇÕES PARA ROBÔ REAL - NÃO ESQUECER!**

```bash
# 1. Preparar terminais (PWM e Encoder bringup)
# Terminal 1: ros2 run your_package pwm_bringup
# Terminal 2: ros2 run your_package encoder_bringup

# 2. Build e source
cd /home/work/Caramelo_workspace
colcon build --packages-select caramelo_navigation
source install/setup.bash

# 3. Navegação com arquivos padrão
ros2 launch caramelo_navigation autonomous_navigation.launch.py

# 4. Ou com arquivos personalizados
ros2 launch caramelo_navigation autonomous_navigation.launch.py \
  map_file:=/caminho/para/seu_mapa.yaml \
  waypoints_file:=/caminho/para/seus_waypoints.json
```

### 1. Navegação Autônoma (Recomendado)

```bash
# Build e source
cd /home/work/Caramelo_workspace
colcon build --packages-select caramelo_navigation
source install/setup.bash

# Navegação completa (mapa + navegação + waypoints)
ros2 launch caramelo_navigation autonomous_navigation.launch.py

# Ou apenas o navegador (se Nav2 já está rodando)
ros2 run caramelo_navigation autonomous_waypoint_navigator
```

### 2. Criar Waypoints Interativamente

```bash
# Inicie o sistema de criação de waypoints
ros2 launch caramelo_navigation interactive_waypoint_creator.launch.py

# No RViz:
# - Use "2D Pose Estimate" para MOVER o robô virtual
# - Use "2D Nav Goal" para SALVAR waypoint com orientação
# - Use "Publish Point" para SALVAR waypoint com orientação padrão
```

### 3. Apenas Navegação (sem waypoints automáticos)

```bash
# Navegação básica
ros2 launch caramelo_navigation navigation_launch.py

# Carregar mapa
ros2 launch caramelo_navigation load_map.launch.py

# SLAM
ros2 launch caramelo_navigation slam_launch.py
```

## Arquivo de Waypoints

Localização: `/home/work/Caramelo_workspace/src/caramelo_navigation/config/waypoints.json`

Exemplo:
```json
{
  "frame_id": "map",
  "waypoints": [
    {
      "name": "START",
      "position": {"x": 0.0, "y": 0.0, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    },
    {
      "name": "WS01",
      "position": {"x": 2.64, "y": 2.32, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": -0.999, "w": 0.002}
    },
    {
      "name": "WS02",
      "position": {"x": 0.94, "y": 2.26, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": -0.0003, "w": 0.999}
    },
    {
      "name": "FINISH",
      "position": {"x": 0.0, "y": 0.0, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    }
  ]
}
```

## Nomenclatura de Waypoints

- **START**: Posição inicial (0,0,0)
- **WS01, WS02, WS03, ...**: Work Stations numeradas
- **FINISH**: Posição final (0,0,0)

## Características da Navegação Autônoma

### ✅ Funcionalidades
- **Navegação cíclica**: Vai de WS01 → WS02 → WS03 → WS01...
- **Desvio de obstáculos**: Usa Nav2 para planejar rotas alternativas
- **Detecção de obstáculos dinâmicos**: Para quando detecta obstáculo próximo
- **Posicionamento preciso**: Controle fino para chegar na posição exata
- **Controle de orientação**: Gira para ficar na orientação correta
- **Recuperação automática**: Tenta aproximação manual se Nav2 falhar

### ⚙️ Configurações Ajustáveis
```python
self.approach_distance = 0.3      # Distância para considerar "próximo" (metros)
self.orientation_tolerance = 0.1  # Tolerância de orientação (radianos)
self.max_approach_time = 30.0     # Tempo máximo para chegar no waypoint
self.obstacle_distance = 0.5      # Distância mínima para obstáculos
```

## Requisitos

- **Nav2**: Sistema de navegação do ROS2
- **Map**: Mapa do ambiente carregado
- **Localization**: AMCL ou sistema similar
- **LiDAR**: Para detecção de obstáculos (`/scan`)
- **TF**: Transformações entre `map` e `base_link`

## Tópicos ROS2

- **Subscribe**: `/scan` (LaserScan) - Detecção de obstáculos
- **Publish**: `/cmd_vel` (Twist) - Controle de movimento
- **Action**: `/navigate_to_pose` (NavigateToPose) - Navegação Nav2
- **TF**: `map` → `base_link` - Localização do robô

## Monitoramento

O sistema mostra mensagens informativas:
- 🚀 **Iniciando navegação pelos waypoints**
- 📍 **Carregados X waypoints**
- 🎯 **Enviando goal para WS01**
- ✅ **Chegou em WS01!**
- 🏁 **Todos os waypoints foram visitados!**

## Troubleshooting

### "Nav2 server não disponível"
- Certifique-se que o Nav2 está rodando
- Use: `ros2 launch caramelo_navigation autonomous_navigation.launch.py`

### Robô não se move
- Verificar se o mapa está carregado
- Verificar TF entre map e base_link
- Verificar se AMCL está funcionando

### Não encontra waypoints
- Verificar se `waypoints.json` existe
- Verificar se tem waypoints "WS01", "WS02", etc.

Para mais detalhes, consulte: `NAVEGACAO_AUTONOMA.md`
