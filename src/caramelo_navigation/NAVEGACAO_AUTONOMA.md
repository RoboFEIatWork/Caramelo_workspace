# Navegação Autônoma com Waypoints

## Descrição

O sistema de navegação autônoma permite que o robô Caramelo navegue pelos waypoints definidos no arquivo `waypoints.json`, desviando de obstáculos dinâmicos e tentando chegar o mais próximo possível de cada waypoint com a orientação correta.

## Características

- ✅ **Navegação autônoma** usando Nav2
- ✅ **Desvio de obstáculos** dinâmicos
- ✅ **Posicionamento preciso** nos waypoints
- ✅ **Controle de orientação** para chegar na direção correta
- ✅ **Recuperação automática** quando não consegue chegar no waypoint
- ✅ **Navegação cíclica** pelos waypoints WS01, WS02, WS03, etc.

## Como Usar

### 1. Preparar o Ambiente

```bash
# 1. Build do pacote
cd /home/work/Caramelo_workspace
colcon build --packages-select caramelo_navigation
source install/setup.bash

# 2. Verificar waypoints
# O arquivo waypoints.json deve conter pelo menos waypoints WS01, WS02, etc.
```

### 2. Navegação Autônoma

```bash
# Opção 1: Navegação completa (mapa + navegação + waypoints)
ros2 launch caramelo_navigation autonomous_navigation.launch.py

# Opção 2: Apenas o navegador (se já tem nav2 rodando)
ros2 run caramelo_navigation autonomous_waypoint_navigator
```

### 3. Monitoramento

O sistema mostra mensagens no terminal:
- 🚀 **Iniciando navegação pelos waypoints**
- 🎯 **Enviando goal para WS01**
- ✅ **Goal aceito!**
- ✅ **Navegação concluída com sucesso!**
- ✅ **Chegou em WS01!**
- 🏁 **Todos os waypoints foram visitados!**

## Configurações Ajustáveis

No arquivo `autonomous_waypoint_navigator.py`, você pode ajustar:

```python
# Configurações
self.approach_distance = 0.3      # Distância para considerar "próximo" (metros)
self.orientation_tolerance = 0.1  # Tolerância de orientação (radianos)
self.max_approach_time = 30.0     # Tempo máximo para chegar no waypoint (segundos)
self.obstacle_distance = 0.5      # Distância mínima para obstáculos (metros)
```

## Estados da Navegação

O sistema opera em diferentes estados:

1. **IDLE**: Aguardando para iniciar
2. **NAVIGATING**: Navegando usando Nav2
3. **APPROACHING**: Tentando se aproximar do waypoint
4. **POSITIONING**: Posicionamento fino (posição + orientação)
5. **COMPLETED**: Waypoint alcançado, indo para o próximo
6. **FAILED**: Falha na navegação

## Tratamento de Obstáculos

- **Obstáculos estáticos**: Nav2 planeja rota alternativa
- **Obstáculos dinâmicos**: Sistema para movimento linear quando detecta obstáculo próximo
- **Recuperação**: Se não conseguir chegar no waypoint, tenta aproximação manual

## Estrutura dos Waypoints

O sistema lê waypoints do arquivo `waypoints.json`:

```json
{
  "frame_id": "map",
  "waypoints": [
    {
      "name": "WS01",
      "position": {"x": 2.64, "y": 2.32, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": -0.999, "w": 0.002}
    },
    {
      "name": "WS02", 
      "position": {"x": 0.94, "y": 2.26, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": -0.0003, "w": 0.999}
    }
  ]
}
```

## Requisitos

- **Nav2**: Sistema de navegação do ROS2
- **Map**: Mapa do ambiente carregado
- **Localization**: AMCL ou sistema similar
- **Sensor**: LiDAR para detecção de obstáculos (`/scan`)
- **TF**: Transformações entre `map` e `base_link`

## Tópicos Usados

- **Subscribe**: `/scan` (LaserScan) - Detecção de obstáculos
- **Publish**: `/cmd_vel` (Twist) - Controle de movimento
- **Action**: `/navigate_to_pose` (NavigateToPose) - Navegação Nav2
- **TF**: `map` → `base_link` - Localização do robô

## Troubleshooting

### Robô não se move
- Verificar se Nav2 está rodando
- Verificar se o mapa está carregado
- Verificar TF entre map e base_link

### Não encontra waypoints
- Verificar se waypoints.json existe
- Verificar se tem waypoints com nome "WS01", "WS02", etc.
- Verificar permissões do arquivo

### Fica parado em obstáculos
- Ajustar `obstacle_distance` no código
- Verificar se sensor LiDAR está funcionando
- Verificar tópico `/scan`

### Não chega na orientação correta
- Ajustar `orientation_tolerance` no código
- Verificar se waypoints têm orientação válida
- Verificar controle de orientação do robô
