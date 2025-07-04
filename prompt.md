# Planejamento Técnico: Sistema Funcional do Robô Caramelo para a RoboCup@Work 2024

## 📌 Contexto

O robô omnidirecional Caramelo já possui o mapeamento do ambiente realizado com sucesso usando sensores como LiDAR. A partir deste ponto, o objetivo é tornar o robô funcional para a competição da RoboCup@Work, permitindo que ele realize tarefas de navegação e manipulação de forma autônoma, mesmo com o manipulador ainda em fase de desenvolvimento ou instável.

---

## 🎯 Objetivo

Desenvolver um sistema modular em ROS 2 (versão: Jazzy), onde o robô seja capaz de:

- Ler um arquivo `.yaml` contendo tarefas organizadas por estações.
- Navegar entre as estações (workstations) com base no mapa gerado.
- Tentar detectar e manipular objetos com até 3 tentativas.
- Lidar com falhas (por exemplo, ausência do objeto ou falha na manipulação).
- Continuar a execução da próxima tarefa mesmo se falhar uma estação.
- Operar de forma autônoma e sem intervenção após o início do teste.

---

## 📂 Estrutura de Pacotes ROS 2 (Caramelo_workspace/src/)

Caramelo_workspace/src/
├── caramelo_description/ # URDF/Xacro do robô, meshes, links, sensores
├── caramelo_navigation/ # Mapa, parâmetros do Nav2, AMCL, planner
├── caramelo_tasks/ # Execução de tarefas com base em arquivo YAML
├── caramelo_manipulation/ # Lógica do manipulador (mock ou real)
├── caramelo_bringup/ # Arquivo launch geral que inicia todos os nós


---

## 🚀 Etapas de Implementação

### 1. Configurar Navegação Autônoma (Nav2)

- Criar o pacote `caramelo_navigation`.
- Adicionar:
  - `map.yaml` e `map.pgm` gerados a partir do SLAM.
  - `nav2_params.yaml` com configuração dos nós `planner_server`, `controller_server`, `amcl`, `map_server`, `bt_navigator`.
  - `navigation.launch.py` que inicializa o Nav2 completo.
- Validar se o robô responde a goals via `/goal_pose` ou `PoseStamped` por action.

---

### 2. Criar Executor de Tarefas Baseado em YAML

- Criar o pacote `caramelo_tasks`.
- O nó principal (`task_executor_node.py`) deve:
  - Carregar um arquivo `tasks.yaml` no formato:

    ```yaml
    task_list:
      - object: "R20"
        pick_from: "WS3"
        place_to: "WS5"
    workstations:
      WS3:
        position: [2.0, 1.5]
      WS5:
        position: [4.0, 1.0]
    ```

  - Navegar até `pick_from`, tentar detectar e manipular o objeto com até 3 tentativas.
  - Se bem-sucedido, navegar até `place_to` e realizar a entrega (simulada).
  - Se falhar 3 vezes, pular para a próxima tarefa.
  - Exibir logs claros no terminal com `INFO`, `WARN` e `ERROR`.

---

### 3. Programar Função de Navegação `navigate_to(position)`

- Publicar `geometry_msgs/msg/PoseStamped` no tópico `/goal_pose` ou usar cliente de ação Nav2.
- Esperar resultado (`SUCCEEDED`, `FAILED`, `TIMEOUT`).
- Implementar fallback se não alcançar o destino em tempo razoável.

---

### 4. Lógica de Manipulação com Tolerância a Falhas

- Criar o pacote `caramelo_manipulation`.
- Incluir funções mock:
  - `detect_object(obj)`: retorna `True` com 50% de chance.
  - `manipulate_object(obj)`: retorna `True` com 30% de chance.
- No futuro, essas funções serão substituídas por:
  - Visão computacional (ex: MediaPipe, ZED)
  - Controle do manipulador real (com ROS 2 ou comandos diretos)

---

### 5. Lançamento Unificado do Sistema

- Criar o pacote `caramelo_bringup`.
- Implementar `system.launch.py` que:
  - Inicia:
    - Nav2 (mapa, amcl, planner)
    - Executor de tarefas
    - (opcional) módulos de visualização (RViz)
  - Recebe parâmetros como:
    - `map_file`
    - `task_file`
    - `initial_pose`

---

## ✅ Requisitos Funcionais

- O robô deve:
  - Ser autônomo após o comando de start.
  - Executar uma lista de tarefas YAML sequencialmente.
  - Lidar com falhas sem parar o sistema.
  - Ser modular, com pacotes isolados por função.

---

## 📈 Expansões Futuras

- Substituir funções mock por visão real com câmeras e IA.
- Controlar o manipulador físico com ROS 2 Control ou comandos diretos.
- Publicar estado e progresso do robô via `/caramelo/status`.
- Registrar métricas como tempo por tarefa, sucesso/falha e logs em disco.

---

## 🏁 Meta Final

Permitir que o robô Caramelo execute tarefas definidas por YAML, com navegação robusta, manipulação tolerante a falhas, e operação 100% autônoma, respeitando os critérios da RoboCup@Work. O sistema será expansível e sustentável para testes futuros e aplicação em ambientes industriais simulados.
