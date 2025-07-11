# CARAMELO - NAVEGAÇÃO AUTÔNOMA EM 3 TERMINAIS

## 🎯 **ESTRUTURA SIMPLES E CONTROLADA**

### **Terminal 1: Encoder**
```bash
cd /home/work/Caramelo_workspace
source install/setup.bash
ros2 launch caramelo_bringup encoder_bringup.launch.py
```

### **Terminal 2: PWM**
```bash
cd /home/work/Caramelo_workspace
source install/setup.bash
ros2 launch caramelo_bringup pwm_bringup.launch.py
```

### **Terminal 3: Navegação Completa**
```bash
cd /home/work/Caramelo_workspace
source install/setup.bash
ros2 launch caramelo_navigation caramelo_navigation.launch.py
```

---

## 🔧 **O QUE CADA TERMINAL FAZ**

### **Terminal 1 - Encoder**
- ✅ Leitura dos encoders
- ✅ Publicação de odometria
- ✅ Joint states
- ✅ TF: `base_footprint` → `base_link`

### **Terminal 2 - PWM**
- ✅ Controle PWM do ESP32
- ✅ Interface com motores
- ✅ Recebe comandos de velocidade

### **Terminal 3 - Navegação**
- ✅ Twist converter (`/cmd_vel` → `/mecanum_drive_controller/cmd_vel`)
- ✅ Map server (carrega mapa estático)
- ✅ AMCL (localização + **TF: `map` → `odom`**)
- ✅ Nav2 stack completo
- ✅ Lifecycle manager (gerencia inicialização)
- ✅ AMCL initializer (pose inicial automática)
- ✅ RViz2 (visualização)
- ✅ Waypoint navigation (missão automática)

---

## 📊 **CADEIA DE TF COMPLETA**

```
map → odom → base_footprint → base_link
```

- **`map` → `odom`**: Publicado pelo AMCL (Terminal 3)
- **`odom` → `base_footprint`**: Publicado pelo encoder (Terminal 1)
- **`base_footprint` → `base_link`**: Publicado pelo encoder (Terminal 1)

---

## ✅ **VERIFICAÇÃO DO SISTEMA**

### **Nodes ativos esperados:**
```bash
ros2 node list | grep -E "(encoder|esp32_pwm|twist_converter|amcl|bt_navigator|controller_server|planner_server|waypoint_nav)"
```

### **TF completo:**
```bash
ros2 run tf2_ros tf2_echo map base_link
```

### **Tópicos críticos:**
```bash
ros2 topic list | grep -E "(cmd_vel|scan|odom|initialpose|map)"
```

---

## 🎯 **MISSÃO ATUAL**

**Arquivo de waypoints:** `/home/work/Caramelo_workspace/maps/arena_fei/waypoints.json`
**Arquivo de missão:** `/home/work/Caramelo_workspace/maps/arena_fei/mission.yaml`

**Sequência automática:**
1. `START` (0,0) → `WS01` (2.64, 2.32) → `WS02` (0.94, 2.26) → `FINISH` (0,0)

---

## 🔧 **TROUBLESHOOTING**

### **Problema: Robô não se move**
- **Verificar:** Terminal 2 (PWM) e Terminal 3 (twist_converter)

### **Problema: AMCL não funciona**  
- **Verificar:** Terminal 3 (map_server, amcl)

### **Problema: TF incompleto**
- **Verificar:** Terminal 1 (encoder) e Terminal 3 (amcl)

### **Reset completo:**
```bash
pkill -f ros2
sleep 3
# Reiniciar os 3 terminais na ordem
```

---

> **💡 Controle total:** Cada terminal tem logs independentes e específicos para diagnóstico preciso!
