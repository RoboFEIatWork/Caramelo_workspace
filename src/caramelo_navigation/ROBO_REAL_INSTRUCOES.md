# Como Usar o Sistema de Navegação Autônoma com o Robô Real

## Pré-requisitos

### 1. Hardware Conectado
- ✅ **Computador ligado nos USBs do robô**
- ✅ **PWM bringup rodando** (controle dos motores)
- ✅ **Encoder bringup rodando** (odometria)
- ✅ **LiDAR funcionando** (detecção de obstáculos)

### 2. Arquivos Necessários
- 🗺️ **Mapa salvo**: `/home/work/Caramelo_workspace/mapa_20250704_145039.yaml`
- 📍 **Waypoints**: `/home/work/Caramelo_workspace/src/caramelo_navigation/config/waypoints.json`

## Preparação dos Terminais

### Terminal 1: PWM Bringup
```bash
# Iniciar controle dos motores
ros2 run your_package pwm_bringup
```

### Terminal 2: Encoder Bringup
```bash
# Iniciar odometria
ros2 run your_package encoder_bringup
```

### Terminal 3: Build e Source
```bash
cd /home/work/Caramelo_workspace
colcon build --packages-select caramelo_navigation
source install/setup.bash
```

## Navegação Autônoma

### Uso Básico (com arquivos padrão)
```bash
# Terminal 4: Navegação completa
ros2 launch caramelo_navigation autonomous_navigation.launch.py
```

### Uso com Arquivos Personalizados
```bash
# Especificar mapa e waypoints customizados
ros2 launch caramelo_navigation autonomous_navigation.launch.py \
  map_file:=/caminho/para/seu_mapa.yaml \
  waypoints_file:=/caminho/para/seus_waypoints.json
```

### Exemplos de Uso

#### Exemplo 1: Waypoints de Produção
```bash
ros2 launch caramelo_navigation autonomous_navigation.launch.py \
  waypoints_file:=/home/work/Caramelo_workspace/src/caramelo_navigation/config/waypoints_producao.json
```

#### Exemplo 2: Mapa Específico
```bash
ros2 launch caramelo_navigation autonomous_navigation.launch.py \
  map_file:=/home/work/Caramelo_workspace/mapa_novo.yaml \
  waypoints_file:=/home/work/Caramelo_workspace/src/caramelo_navigation/config/waypoints_novo.json
```

## Comportamento do Sistema

### O que o robô faz:
1. 📍 **Carrega waypoints** do arquivo JSON especificado
2. 🗺️ **Usa mapa existente** (não cria mapa novo)
3. 🎯 **Navega para WS01** automaticamente
4. 🚨 **Detecta obstáculos** dinâmicos na frente
5. ⏸️ **Para quando encontra obstáculo** próximo
6. 🔄 **Retoma navegação** quando obstáculo sai
7. ✅ **Posiciona-se precisamente** em cada waypoint
8. 🔁 **Continua ciclicamente** (WS01 → WS02 → WS03 → WS01...)

### Detecção de Obstáculos:
- 🔍 **Monitora área frontal** (±30° na frente)
- 📏 **Distância de segurança**: 0.6m
- ⏳ **Aguarda 3 segundos** para obstáculo sair
- 🔄 **Replaneja rota** se obstáculo persistir

## Monitoramento

### Mensagens do Sistema:
- 🚀 `Iniciando navegação pelos waypoints!`
- 🎯 `Enviando goal para WS01`
- 🚨 `Obstáculo detectado a X.XXm!`
- ⏹️ `Parado - aguardando obstáculo sair`
- ✅ `Caminho livre!`
- ✅ `Chegou em WS01!`

### Em Caso de Problemas:
- ❌ `Nav2 server não disponível!` → Verificar se PWM/Encoder estão rodando
- ❌ `Arquivo de waypoints não encontrado` → Verificar caminho do arquivo
- ❌ `TF lookup failed` → Verificar se AMCL está funcionando

## Parâmetros Configuráveis

Você pode ajustar no arquivo `autonomous_waypoint_navigator.py`:

```python
# Configurações de navegação
self.approach_distance = 0.25      # Distância para considerar "próximo"
self.orientation_tolerance = 0.15  # Tolerância de orientação
self.max_approach_time = 45.0      # Tempo máximo para chegar

# Configurações de obstáculos
self.obstacle_distance = 0.6       # Distância mínima para obstáculos
self.obstacle_check_angle = 60.0   # Ângulo de verificação (±30°)
self.obstacle_wait_time = 3.0      # Tempo para aguardar obstáculo sair
```

## Arquivos de Exemplo

### Múltiplos Waypoints JSON:
```bash
# Waypoints de teste
/home/work/Caramelo_workspace/src/caramelo_navigation/config/waypoints.json

# Waypoints de produção (criar se necessário)
/home/work/Caramelo_workspace/src/caramelo_navigation/config/waypoints_producao.json

# Waypoints de manutenção (criar se necessário)
/home/work/Caramelo_workspace/src/caramelo_navigation/config/waypoints_manutencao.json
```

### Múltiplos Mapas:
```bash
# Mapa atual
/home/work/Caramelo_workspace/mapa_20250704_145039.yaml

# Outros mapas (criar se necessário)
/home/work/Caramelo_workspace/mapa_producao.yaml
/home/work/Caramelo_workspace/mapa_manutencao.yaml
```

## Parar o Sistema

Para parar o robô:
```bash
# Ctrl+C no terminal da navegação
# O robô para automaticamente
```

## Troubleshooting

### Robô não se move:
1. Verificar se PWM bringup está rodando
2. Verificar se encoder bringup está rodando
3. Verificar se há obstáculos na frente

### Navegação imprecisa:
1. Verificar se o mapa está correto
2. Verificar se AMCL está funcionando
3. Verificar se há interferência no LiDAR

### Obstáculos não detectados:
1. Verificar se LiDAR está funcionando
2. Verificar tópico `/scan`
3. Ajustar `obstacle_distance` no código

**O sistema está otimizado para robô real com detecção de obstáculos dinâmicos!** 🤖
