# Caramelo Navigation

## Descrição
Pacote responsável pela navegação, SLAM e fusão sensorial do robô Caramelo. Implementa estratégias de navegação autônoma e mapeamento.

## Funcionalidades
- **EKF Sensor Fusion**: Fusão de odometria dos encoders com IMU
- **SLAM**: Mapeamento simultâneo e localização com LiDAR
- **Nav2 Integration**: Navegação autônoma com planejamento de trajetória
- **Configurações Otimizadas**: Para robôs mecanum com deslizamento

## Launches Disponíveis

### Fusão Sensorial
```bash
# EKF para fusão encoder + IMU
ros2 launch caramelo_navigation ekf_encoder_imu_launch.py

# ZED IMU apenas (quando disponível)
ros2 launch caramelo_navigation zed_imu_launch.py
```

### SLAM e Navegação
```bash
# SLAM com SLAM Toolbox
ros2 launch caramelo_navigation slam_launch.py

# Mapeamento
ros2 launch caramelo_navigation mapping_launch.py

# Navegação completa com Nav2
ros2 launch caramelo_navigation nav2_slam_launch.py
```

## Configurações Principais

### EKF (Fusão Sensorial)
- **Arquivo**: `config/ekf_encoder_imu_params.yaml`
- **Sensores**: Encoders (velocidades) + IMU ZED (orientação)
- **Saída**: `/odom` fusionado e robusto

### SLAM Toolbox
- **Arquivo**: `config/slam_toolbox_params.yaml`
- **Otimizado**: Para robôs com deslizamento
- **Saída**: Mapa 2D em `/map`

### Nav2
- **Arquivo**: `config/nav2_params.yaml`
- **Configurado**: Para robô omnidirecional mecanum
- **Funcionalidades**: Planejamento, controle local, comportamentos

## Tópicos Principais

### Entradas
- `/odom` - Odometria dos encoders
- `/scan` - Dados LiDAR
- `/zed/zed_node/imu/data` - IMU da ZED
- `/cmd_vel` - Comandos de navegação

### Saídas
- `/odom` - Odometria fusionada (EKF)
- `/map` - Mapa gerado (SLAM)
- `/plan` - Trajetórias planejadas
- `/local_costmap/costmap` - Mapa de custos local

## Estratégia de Navegação

1. **Sensor Fusion**: EKF combina encoders + IMU para odometria robusta
2. **SLAM**: LiDAR gera mapa 2D do ambiente
3. **Planning**: Nav2 planeja trajetórias no mapa
4. **Control**: DWB controla movimento omnidirecional
5. **Recovery**: Comportamentos de recuperação em falhas

## Configuração para Mecanum

### Características Especiais
- **Movimentação lateral**: Configurada para vy ≠ 0
- **Deslizamento compensado**: Covariâncias ajustadas
- **Modelo omnidirecional**: AMCL configurado para mecanum
- **DWB otimizado**: Samples em vx, vy e vtheta

### Ajustes de Deslizamento
- **Encoder vx**: Covariância 3e-2
- **Encoder vy**: Covariância 5e-2 (mais deslizamento lateral)
- **IMU orientação**: Covariância baixa (sensor confiável)

## Dependências
- nav2_bringup
- slam_toolbox
- robot_localization
- dwb_core
- nav2_amcl
