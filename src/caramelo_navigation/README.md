# Caramelo Navigation

Pacote de navegação simplificado para o robô Caramelo.

## Estrutura do Pacote

### Scripts Principais
- `simple_waypoint_navigator.py` - Navegador básico de waypoints
- `checkpoint_navigator.py` - Navegador de checkpoints
- `interactive_robot_positioner.py` - Criador interativo de waypoints

### Launch Files
- `interactive_waypoint_creator.launch.py` - Lança o sistema de criação de waypoints
- `mapping_launch.py` - Sistema de mapeamento
- `navigation_launch.py` - Sistema de navegação
- `load_map.launch.py` - Carrega mapa existente
- `map_display.launch.py` - Exibe mapa no RViz
- `slam_launch.py` - SLAM
- `teleop_mapping.launch.py` - Teleoperação + mapeamento
- `checkpoint_navigation.launch.py` - Navegação por checkpoints

### Configurações
- `waypoints.json` - Arquivo de waypoints (WS01, WS02, START, FINISH)
- `checkpoint_creator_final.rviz` - Configuração do RViz
- `*.yaml` - Parâmetros de navegação, SLAM, AMCL, etc.

## Como Usar

### 1. Criar Waypoints Interativamente

```bash
# Inicie o sistema de criação de waypoints
ros2 launch caramelo_navigation interactive_waypoint_creator.launch.py

# No RViz:
# - Use "2D Pose Estimate" para MOVER o robô virtual
# - Use "2D Nav Goal" para SALVAR waypoint com orientação
# - Use "Publish Point" para SALVAR waypoint com orientação padrão
```

### 2. Editar Waypoints Manualmente

Edite o arquivo: `/home/work/Caramelo_workspace/src/caramelo_navigation/config/waypoints.json`

Estrutura:
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
      "position": {"x": 1.0, "y": 2.0, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.707, "w": 0.707}
    }
  ]
}
```

### 3. Navegação

```bash
# Navegação básica
ros2 run caramelo_navigation simple_waypoint_navigator

# Navegação por checkpoints
ros2 run caramelo_navigation checkpoint_navigator
```

## Naming Convention

- **START**: Posição inicial (0,0,0)
- **WS01, WS02, WS03, ...**: Work Stations numeradas
- **FINISH**: Posição final (0,0,0)

## Orientação (Quaternions)

- 0°: `(0, 0, 0, 1)`
- 90°: `(0, 0, 0.707, 0.707)`
- 180°: `(0, 0, 1, 0)`
- 270°: `(0, 0, -0.707, 0.707)`
