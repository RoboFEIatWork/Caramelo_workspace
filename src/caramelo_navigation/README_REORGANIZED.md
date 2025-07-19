# 🚀 CARAMELO NAVIGATION - SISTEMA REORGANIZADO

## 📋 Visão Geral

O pacote `caramelo_navigation` foi reorganizado para ser mais modular e fácil de usar. Agora inclui um **Servidor de Navegação por Workstations** que permite tanto ao `caramelo_tasks` quanto a desenvolvedores testarem a navegação facilmente.

## 🏗️ Arquitetura

```
📦 caramelo_navigation
├── 🎯 workstation_navigation_server    # Servidor principal de navegação
├── 🧪 navigation_test_client          # Cliente para testes
├── 📡 task_navigation_client          # Cliente para caramelo_tasks
├── 🚀 basic_navigation.launch.py      # Launch só navegação
├── 🚀 complete_navigation.launch.py   # Launch com hardware
└── 📄 NavigateToWorkstation.srv       # Interface de serviço
```

## 🎯 Workstation Navigation Server

### Funcionalidades
- ✅ Carrega waypoints automaticamente da arena
- ✅ Expõe serviço `/navigate_to_workstation`
- ✅ Suporta navegação síncrona e assíncrona
- ✅ Gerencia estado para evitar navegações simultâneas
- ✅ Integração completa com Nav2

### Parâmetros
- `arena`: Nome da arena (padrão: `arena_fei`)
- `waypoints_base_path`: Caminho base dos mapas (padrão: `/home/work/Caramelo_workspace/maps`)
- `default_timeout`: Timeout padrão em segundos (padrão: `120.0`)

## 🚀 Como Usar

### 1. Sistema Básico (sem hardware)
Use quando já tiver hardware rodando separadamente:
```bash
# Terminal 1: Hardware
ros2 launch caramelo_bringup odometry_bringup.launch.py
ros2 launch caramelo_bringup lidar_bringup.launch.py  
ros2 launch caramelo_bringup actuators_bringup.launch.py

# Terminal 2: Navegação
ros2 launch caramelo_navigation basic_navigation.launch.py arena:=arena_fei
```

### 2. Sistema Completo (tudo junto)
Inicia tudo de uma vez:
```bash
ros2 launch caramelo_navigation complete_navigation.launch.py arena:=arena_fei
```

## 🧪 Testando Navegação

### Cliente de Teste Direto
```bash
# Navegar para WS01 (assíncrono)
ros2 run caramelo_navigation navigation_test_client WS01

# Navegar para WS02 aguardando conclusão
ros2 run caramelo_navigation navigation_test_client WS02 --wait

# Navegar com arena específica
ros2 run caramelo_navigation navigation_test_client START --arena arena_robocup25

# Navegar com timeout customizado
ros2 run caramelo_navigation navigation_test_client FINISH --wait --timeout 180
```

### Via Serviço ROS
```bash
ros2 service call /navigate_to_workstation caramelo_navigation/srv/NavigateToWorkstation "{workstation_name: 'WS01', arena_name: '', wait_for_completion: true, timeout: 120.0}"
```

## 📡 Integração com caramelo_tasks

### Importar e Usar
```python
from caramelo_navigation.task_navigation_client import TaskNavigationClient

# No seu nó do caramelo_tasks
class TaskOrchestrator(Node):
    def __init__(self):
        super().__init__('task_orchestrator')
        
        # Criar cliente de navegação
        self.nav_client = TaskNavigationClient(node=self, arena="arena_fei")
    
    def execute_task(self):
        # Navegar para WS01 (assíncrono)
        success = self.nav_client.navigate_to_workstation("WS01")
        
        if success:
            self.get_logger().info("Navegação iniciada!")
            # Continuar com outras tarefas...
        
        # Navegar para WS02 aguardando conclusão
        success = self.nav_client.navigate_to_workstation("WS02", wait=True)
        
        if success:
            self.get_logger().info("Chegou em WS02! Executar manipulação...")
```

### Função de Conveniência
```python
from caramelo_navigation.task_navigation_client import navigate_to_ws

# Uso simples (cria e destrói cliente automaticamente)
success = navigate_to_ws("WS01", arena="arena_fei", wait=True)
```

## 🗂️ Estrutura de Arquivos

### Mapas (não alterar!)
```
/home/work/Caramelo_workspace/maps/
├── arena_fei/
│   ├── map.yaml
│   ├── map.pgm
│   └── waypoints.json
├── arena_robocup25/
│   ├── map.yaml
│   ├── map.pgm
│   └── waypoints.json
└── ...
```

### Tasks (não alterar!)
```
src/caramelo_tasks/
├── BMT/
│   └── task.yaml
├── BTT1/
│   └── task.yaml
└── ...
```

## 📋 Interface do Serviço

### NavigateToWorkstation.srv
```
# Request
string workstation_name        # Nome da workstation (ex: "WS01", "WS02", "START", "FINISH")
string arena_name             # Nome da arena (opcional, usa padrão se vazio)
float32 timeout              # Timeout em segundos (opcional, usa padrão se 0.0)
bool wait_for_completion     # Se deve aguardar a conclusão da navegação

---

# Response
bool success                 # Se a navegação foi iniciada/concluída com sucesso
string message              # Mensagem de status ou erro
float32 distance_traveled   # Distância percorrida (se wait_for_completion=true)
float32 time_elapsed        # Tempo decorrido em segundos
```

## 🔧 Vantagens da Nova Arquitetura

### ✅ Para Desenvolvimento
- **Testes fáceis**: Testar navegação sem precisar implementar tasks
- **Depuração simples**: Logs claros e estados bem definidos
- **Modularidade**: Hardware e navegação independentes

### ✅ Para caramelo_tasks  
- **Interface simples**: Uma função para navegar
- **Flexibilidade**: Navegação síncrona ou assíncrona conforme necessário
- **Reutilização**: Mesmo servidor para todas as tasks

### ✅ Para Sistema Completo
- **Escalabilidade**: Funciona com qualquer arena/task
- **Robustez**: Gestão de estado e timeouts
- **Compatibilidade**: Mantém launches existentes

## 🚨 Notas Importantes

1. **Não alterar arquivos de mapas/tasks**: O sistema lê automaticamente
2. **Compilar após mudanças**: `colcon build --packages-select caramelo_navigation`
3. **Hardware primeiro**: Se usar basic_navigation, certifique-se que hardware está rodando
4. **Timeouts**: Ajustar conforme necessário para arena grande/pequena

## 📝 Próximos Passos

1. **Compilar o pacote** com as novas interfaces
2. **Testar navegação básica** com cliente de teste
3. **Integrar no caramelo_tasks** usando TaskNavigationClient
4. **Configurar launch files** conforme necessário para cada arena

---

**Autor**: GitHub Copilot  
**Data**: 2025-01-18  
**Versão**: 1.0
