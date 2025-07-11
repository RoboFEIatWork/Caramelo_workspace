# CARAMELO - NAVEGAÇÃO AUTÔNOMA COMPLETA

## Sistema Fail-Proof para Robô Real

### 🚀 USO RÁPIDO

Após qualquer `pkill ros2` ou reinicialização, rode apenas **UM COMANDO**:

```bash
cd /home/work/Caramelo_workspace
./start_caramelo_navigation.sh
```

**OU**

```bash
cd /home/work/Caramelo_workspace
source install/setup.bash
ros2 launch caramelo_navigation caramelo_complete.launch.py
```

### 📋 O QUE O SISTEMA FAZ

O launch `caramelo_complete.launch.py` inicia **TUDO** em sequência cronometrada:

1. **Hardware (0s)**: PWM + Encoder
2. **Sensores (2s)**: LiDAR RPLidar S2  
3. **Conversores (4s)**: Twist Converter (`/cmd_vel` → `/mecanum_drive_controller/cmd_vel`)
4. **Navegação (6s)**: Nav2 Stack completo + AMCL + Mapa estático
5. **Visualização (12s)**: RViz 
6. **Autonomia (20s)**: Navegação por waypoints automática

### 🎯 ARQUIVOS DE CONFIGURAÇÃO

- **Mapa**: `/home/work/Caramelo_workspace/maps/arena_fei/map.yaml`
- **Missão**: `/home/work/Caramelo_workspace/maps/arena_fei/mission.yaml`
- **Nav2**: `/home/work/Caramelo_workspace/src/caramelo_navigation/config/caramelo_nav2.yaml`

### 📍 EXEMPLO DE MISSÃO

Arquivo `mission.yaml`:
```yaml
waypoints:
  - x: 2.0
    y: 0.0
    yaw: 0.0
  - x: 2.0
    y: 2.0
    yaw: 1.57
  - x: 0.0
    y: 2.0
    yaw: 3.14
  - x: 0.0
    y: 0.0
    yaw: 0.0
```

### 🔧 TROUBLESHOOTING

#### Problema: "Nav2 não está disponível"
- **Solução**: Aguarde mais tempo. O Nav2 leva ~10s para inicializar

#### Problema: "Robô não se move"
- **Verificar**: Se o `twist_converter_node` está rodando
- **Comando**: `ros2 node list | grep twist_converter`

#### Problema: "AMCL não localiza"
- **Verificar**: Se o frame `map` existe no TF
- **Comando**: `ros2 run tf2_tools view_frames`

#### Problema: "Arquivo de mapa não encontrado"
- **Verificar**: Se existe `/home/work/Caramelo_workspace/maps/arena_fei/map.yaml`

### 📊 MONITORAMENTO

```bash
# Ver todos os nodes ativos
ros2 node list

# Ver tópicos ativos  
ros2 topic list

# Monitorar navegação
ros2 topic echo /navigate_to_pose/_action/feedback

# Ver status AMCL
ros2 topic echo /amcl_pose

# Ver comandos enviados ao robô
ros2 topic echo /mecanum_drive_controller/cmd_vel
```

### 🛑 PARAR SISTEMA

```bash
pkill ros2
# ou
pkill -f "ros2 launch"
```

### 🔄 REINICIALIZAR

Após parar, rode novamente:
```bash
./start_caramelo_navigation.sh
```

### ⚠️ IMPORTANTE

- **Hardware**: O sistema assume que PWM e Encoder estão funcionais
- **Mapa**: Certifique-se que o mapa `arena_fei` está correto
- **Pose inicial**: É publicada automaticamente na origem (0,0,0)
- **Twist Converter**: Essencial para movimento físico do robô
- **Timing**: Os delays são otimizados para hardware real

### 📂 ESTRUTURA DO SISTEMA

```
Caramelo_workspace/
├── start_caramelo_navigation.sh          # Script principal
├── maps/arena_fei/
│   ├── map.yaml                          # Mapa estático
│   └── mission.yaml                      # Waypoints
└── src/caramelo_navigation/
    ├── launch/
    │   ├── caramelo_complete.launch.py   # Launch completo
    │   └── caramelo_nav.launch.py        # Launch só navegação
    ├── config/
    │   └── caramelo_nav2.yaml           # Parâmetros Nav2
    └── caramelo_navigation/
        └── caramelo_waypoint_nav.py     # Node de navegação
```

---

**Sistema desenvolvido para robô Caramelo - FEI**  
**ROS2 Jazzy + Nav2 + Hardware Real**
