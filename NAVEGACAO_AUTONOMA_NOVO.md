# NAVEGAÇÃO AUTÔNOMA CARAMELO - MANUAL PRÁTICO

## Sistema Fail-Proof com ROS2 Launch

### 🎯 **OBJETIVO**
Navegação autônoma robusta para o robô Caramelo usando múltiplos terminais com `ros2 launch`. Sistema funciona após reboot/pkill, com integração completa Nav2 + AMCL + waypoint navigation.

---

## 📋 **PRÉ-REQUISITOS**

1. **Mapa estático** disponível em: `/home/work/Caramelo_workspace/maps/arena_fei/`
   - `map.pgm` e `map.yaml` (mapa SLAM)
   
2. **Waypoints definidos** em: `/home/work/Caramelo_workspace/maps/arena_fei/waypoints.json`
   - Database de waypoints com coordenadas precisas
   
3. **Missão configurada** em: `/home/work/Caramelo_workspace/maps/arena_fei/mission.yaml`
   - Sequência de waypoints por nome (não coordenadas)

---

## 🚀 **SEQUÊNCIA DE INICIALIZAÇÃO**

### **TERMINAL 1: Hardware (PWM + Encoders)**
```bash
cd /home/work/Caramelo_workspace
source install/setup.bash
ros2 launch caramelo_navigation caramelo_hardware.launch.py
```

**O que faz:**
- Inicia PWM controller (esp32_pwm_node)
- Inicia encoder reading (encoder_node) 
- Inicia twist converter (twist_converter_node) - **ESSENCIAL para movimento físico**
- Configura hardware básico para operação

---

### **TERMINAL 2: Navegação (Nav2 + AMCL + Mapa)**
```bash
cd /home/work/Caramelo_workspace
source install/setup.bash
ros2 launch caramelo_navigation caramelo_nav2.launch.py
```

**O que faz:**
- Carrega mapa estático (`map.yaml`)
- Inicia Nav2 stack completo
- Configura AMCL para localização
- Inicia AMCL initializer (publica pose inicial automaticamente)
- Estabelece frame `map` no TF tree

---

### **TERMINAL 3: Interface (RViz + Waypoint Navigation)**
```bash
cd /home/work/Caramelo_workspace
source install/setup.bash
ros2 launch caramelo_navigation caramelo_autonomous.launch.py
```

**O que faz:**
- Abre RViz2 com configuração de navegação
- Inicia waypoint navigation node
- Carrega missão automática de `mission.yaml`
- Executa navegação sequencial pelos waypoints

---

## 📊 **VERIFICAÇÃO DO SISTEMA**

### **Verificar se todos os nodes estão ativos:**
```bash
ros2 node list | grep -E "(esp32_pwm|encoder|twist_converter|amcl|bt_navigator|controller_server|planner_server|waypoint_nav)"
```

### **Verificar TF tree (deve incluir frame 'map'):**
```bash
ros2 run tf2_tools view_frames
```

### **Verificar tópicos críticos:**
```bash
ros2 topic list | grep -E "(cmd_vel|scan|odom|initialpose|goal_pose)"
```

### **Monitor da navegação:**
```bash
# Terminal adicional - Logs da missão
ros2 topic echo /rosout | grep waypoint_nav

# Terminal adicional - Status do Nav2
ros2 topic echo /navigate_to_pose/_action/status
```

---

## 🎮 **CONTROLE MANUAL DE EMERGÊNCIA**

### **Parar navegação autônoma:**
```bash
pkill -f caramelo_waypoint_nav
```

### **Enviar goal manual via RViz:**
1. RViz → `2D Goal Pose`
2. Clique no mapa para definir destino
3. Robô navegará automaticamente

### **Resetar pose do AMCL:**
```bash
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "
header:
  frame_id: 'map'
pose:
  pose:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
" --once
```

---

## 🔧 **TROUBLESHOOTING**

### **Problema: Robô não se move fisicamente**
- **Causa:** `twist_converter_node` não está rodando
- **Solução:** Verificar Terminal 1 (Hardware)

### **Problema: AMCL não funciona / sem frame 'map'**
- **Causa:** Nav2 ou mapa não carregado
- **Solução:** Verificar Terminal 2 (Navegação)

### **Problema: Waypoints não encontrados**
- **Causa:** Arquivo `waypoints.json` ou `mission.yaml` inválido
- **Solução:** Verificar formato dos arquivos

### **Problema: Nav2 falha ao iniciar**
- **Causa:** Parâmetros inválidos ou dependências
- **Solução:** 
  ```bash
  pkill -f nav2
  # Reiniciar Terminal 2
  ```

### **Reset completo do sistema:**
```bash
# Mata todos os processos ROS
pkill -f ros2

# Aguarda 3 segundos
sleep 3

# Reinicia sequência completa
# Terminal 1: Hardware
# Terminal 2: Navegação  
# Terminal 3: Interface
```

---

## 📁 **ESTRUTURA DE ARQUIVOS**

```
/home/work/Caramelo_workspace/maps/arena_fei/
├── map.pgm                    # Mapa estático (imagem)
├── map.yaml                   # Metadados do mapa
├── waypoints.json             # Database de waypoints
└── mission.yaml               # Sequência da missão

/home/work/Caramelo_workspace/src/caramelo_navigation/launch/
├── caramelo_hardware.launch.py    # Hardware bringup
├── caramelo_nav2.launch.py        # Nav2 + AMCL + Mapa
└── caramelo_autonomous.launch.py  # RViz + Waypoint Nav

/home/work/Caramelo_workspace/src/caramelo_navigation/config/
└── caramelo_nav2.yaml         # Configuração do Nav2
```

---

## ✅ **VALIDAÇÃO FINAL**

### **Sistema funcionando corretamente quando:**
1. ✅ Todos os 3 terminais rodando sem erros
2. ✅ RViz mostra mapa, robô localizado, scan do LIDAR
3. ✅ Robô executa missão automaticamente
4. ✅ Robô se move fisicamente no mundo real
5. ✅ Logs mostram progressão pelos waypoints
6. ✅ TF tree inclui frame `map` → `odom` → `base_link`

### **Teste de robustez:**
```bash
# Terminal de teste
pkill -f ros2
sleep 5
# Reiniciar os 3 terminais conforme sequência
# Robô deve retomar navegação automaticamente
```

---

## 🎯 **MISSÃO ATUAL**

**Arquivo:** `/home/work/Caramelo_workspace/maps/arena_fei/mission.yaml`

**Sequência:**
1. `START` → Origem (0,0)
2. `WS01` → Workspace 1
3. `WS02` → Workspace 2  
4. `FINISH` → Retorno à origem

**Estimativa:** 5 minutos de navegação autônoma

---

> **💡 DICA:** Use `tmux` ou terminais em abas para organizar os 3 launches simultaneamente. O sistema é projetado para ser **fail-proof** e funcionar consistentemente após reinicializações.
