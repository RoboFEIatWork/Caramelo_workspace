# 🚀 Sistema de Mapeamento Reorganizado - Robô Caramelo

## 📁 Organização dos Pacotes

### 🔧 **caramelo_bringup** - Hardware & Básico
- `encoder_bringup.launch.py` - URDF + Encoders + Odometria
- `pwm_bringup.launch.py` - Controle PWM dos motores
- `teleop_keyboard.launch.py` - Controle por teclado
- `twist_converter_node.py` - Conversão Twist→TwistStamped

### 🗺️ **caramelo_navigation** - Navegação & Mapeamento
- `mapping_launch.py` - SLAM + LIDAR + Filtros
- `teleop_mapping.launch.py` - **NOVO** Mapeamento com teclado
- `manual_mapping.launch.py` - Mapeamento com goals RViz
- `lidar_filter.py` - Filtro LIDAR (remove carcaça)
- `simple_waypoint_navigator.py` - Navegação por goals
- `cmd_vel_safety_filter.py` - Filtro de segurança
- `autonomous_explorer.py` - Exploração autônoma

## 🎯 Modos de Mapeamento

### 1. **🎮 Mapeamento com Teleop (RECOMENDADO)**
```bash
# Terminal 1 - Hardware
ros2 launch caramelo_bringup pwm_bringup.launch.py

# Terminal 2 - Mapeamento + Teleop
ros2 launch caramelo_navigation teleop_mapping.launch.py
```

**Vantagens:**
- ✅ Controle total sobre movimento
- ✅ LIDAR filtrado (remove carcaça)
- ✅ Velocidades controladas
- ✅ Mapa de alta qualidade

**Controles:**
- `w/s` - Frente/Trás
- `a/d` - Esquerda/Direita  
- `q/e` - Rotação
- `x` - Parar

### 2. **🎯 Mapeamento com Goals (RViz)**
```bash
# Terminal 1 - Hardware
ros2 launch caramelo_bringup pwm_bringup.launch.py

# Terminal 2 - Mapeamento + Goals
ros2 launch caramelo_bringup manual_mapping.launch.py
```

**Vantagens:**
- ✅ Navegação por cliques no RViz
- ✅ Proteção contra colisão
- ✅ Parada automática em obstáculos

## 🔧 Melhorias Implementadas

### 🛡️ **Filtro LIDAR Melhorado**
- **Distância mínima:** 20cm (remove carcaça do robô)
- **Filtro de ruído:** Remove pontos isolados
- **Ângulos bloqueados:** Configurável para suportes específicos
- **Fluxo:** `/scan` → filtro → `/scan_filtered` → SLAM

### 📊 **Organização de Pacotes**
- **caramelo_bringup:** Apenas hardware básico
- **caramelo_navigation:** Toda navegação e mapeamento
- **Dependências claras:** Cada pacote tem função específica

### 🚦 **Segurança Aprimorada**
- Filtro de segurança para todos os comandos
- Detecção de obstáculos no waypoint navigator
- Velocidades limitadas globalmente

## 📋 Como Usar

### **Opção 1: Mapeamento com Teclado (Mais Controle)**
```bash
# 1. Hardware
ros2 launch caramelo_bringup pwm_bringup.launch.py

# 2. Mapeamento + Teleop
ros2 launch caramelo_navigation teleop_mapping.launch.py

# 3. Salvar mapa
ros2 run nav2_map_server map_saver_cli -f maps/teleop_map_$(date +%Y%m%d_%H%M%S)
```

### **Opção 2: Mapeamento com Goals (Mais Autônomo)**
```bash
# 1. Hardware  
ros2 launch caramelo_bringup pwm_bringup.launch.py

# 2. Mapeamento + Goals
ros2 launch caramelo_bringup manual_mapping.launch.py

# 3. Usar "2D Nav Goal" no RViz
# 4. Salvar mapa
ros2 run nav2_map_server map_saver_cli -f maps/goal_map_$(date +%Y%m%d_%H%M%S)
```

## 🔍 Troubleshooting

### **LIDAR detecta carcaça do robô**
Ajustar parâmetros em `caramelo_navigation/caramelo_navigation/lidar_filter.py`:
```python
self.min_range = 0.25  # Aumentar se necessário
self.blocked_angles = [
    (-0.5, -0.3),  # Bloquear ângulos específicos
    (0.3, 0.5),    # conforme necessário
]
```

### **Mapa com qualidade baixa**
1. Usar **teleop_mapping.launch.py** para controle manual
2. Mover mais devagar
3. Fazer movimentos suaves
4. Evitar rotações rápidas

### **Robô não para em obstáculos**
Verificar logs do waypoint navigator:
```bash
ros2 topic echo /navigation_status
```

## 🎉 Sistema Pronto!

Agora você tem:
- ✅ **2 modos de mapeamento** (teleop + goals)
- ✅ **LIDAR filtrado** (sem interferência da carcaça)
- ✅ **Pacotes organizados** (bringup vs navigation)
- ✅ **Segurança aprimorada** (filtros e detecção)
- ✅ **Mapas de alta qualidade** com odometria precisa

**Recomendação:** Use `teleop_mapping.launch.py` para máximo controle e qualidade!
