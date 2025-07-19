import json
import time
from dynamixel_sdk import PortHandler
from dynamixel_motor import DynamixelMotor  # importe sua classe correta
import os

# === Configurações ===

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CAMINHO_CONTAINER = os.path.join(BASE_DIR, "containers.json")
CAMINHO_TARGET = os.path.join(BASE_DIR, "target_filtrado.json")

VELOCIDADE_PADRAO = 30

POSICAO_INICIAL = {
    1: 1055,
    2: 1800,
    3: 3400,
    4: 2100
}

enderecos_por_tipo = {
    "10": [
        {"motor": 2, "pos": 1800},
        {"motor": 3, "pos": 3340},
        {"motor": 1, "pos": 3380},
        {"motor": 4, "pos": 2150},
        {"motor": 2, "pos": 2000}
    ],
    "15": [
        {"motor": 2, "pos": 2100},
        {"motor": 3, "pos": 3100},
        {"motor": 1, "pos": 2900}
    ],
    "20": [
        {"motor": 3, "pos": 3300},
        {"motor": 4, "pos": 2200}
    ]
}

enderecos_containers = {
    "A": [
        {"motor": 2, "pos": 1800},
        {"motor": 3, "pos": 3250},
        {"motor": 1, "pos": 2790},
        {"motor": 4, "pos": 2230},
        {"motor": 2, "pos": 2100}
    ],
    "B": [
        {"motor": 2, "pos": 1800},
        {"motor": 3, "pos": 3000},
        {"motor": 1, "pos": 3060},
        {"motor": 4, "pos": 2157},
        {"motor": 2, "pos": 2280}
    ],
    "C": [
        {"motor": 2, "pos": 1800},
        {"motor": 3, "pos": 3340},
        {"motor": 1, "pos": 3380},
        {"motor": 4, "pos": 2150},
        {"motor": 2, "pos": 2000}
    ]
}

# === Funções auxiliares ===
def carregar_json(caminho):
    with open(caminho, "r") as f:
        return json.load(f)

def salvar_json(caminho, dados):
    with open(caminho, "w") as f:
        json.dump(dados, f, indent=2)

def encontrar_primeiro_workspace_valido(target_data):
    for ws, info in target_data.items():
        if info.get("manipulado"):
            continue
        objs_validos = [obj for obj in info.get("objects", []) if not obj.get("decoy", False)]
        if objs_validos:
            return ws, objs_validos
    return None, []

def encontrar_container_por_id(container_data, id_objeto):
    for nome_container, dados in container_data.items():
        if dados.get("ocupado") and dados.get("id") == id_objeto:
            return nome_container
    return None

def aguardar_movimento(motor, posicao_alvo, tolerancia=20, timeout=10.0):
    inicio = time.time()
    while time.time() - inicio < timeout:
        pos_atual = motor.get_present_position()
        if abs(pos_atual - posicao_alvo) <= tolerancia:
            return True
        time.sleep(0.05)
    print(f"⚠️ Timeout motor {motor.motor_id}. Última posição: {pos_atual}")
    return False

def mover_motores(motores, movimentos):
    for mov in movimentos:
        motor_id = mov["motor"]
        pos = mov["pos"]
        motores[motor_id].set_profile(acceleration=10, velocity=VELOCIDADE_PADRAO)
        motores[motor_id].set_goal_position(pos)

        # Espera o motor chegar
        #while abs(motores[motor_id].get_present_position() - pos) > 20:  # tolerância de 20 tics
         #   time.sleep(0.05)
        aguardar_movimento(motores[motor_id],pos)

def voltar_posicao_inicial(motores):
    print("🔙 Retornando para posição inicial (2 → 3 → 4 → 1)...")
    for motor_id in [2, 3, 4, 1]:
        pos = POSICAO_INICIAL[motor_id]
        motores[motor_id].set_goal_position(pos)
        aguardar_movimento(motores[motor_id], pos)


def controlar_garra(m6, m7, delta_ticks):
    pos6 = m6.get_present_position()
    pos7 = m7.get_present_position()

    nova_pos6 = max(0, min(4095, pos6 + delta_ticks))
    nova_pos7 = max(0, min(4095, pos7 - delta_ticks))

    print(f"🔧 Garra ajustada → M6: {pos6} ➜ {nova_pos6} | M7: {pos7} ➜ {nova_pos7}")

    m6.set_profile(10, 40)
    m7.set_profile(10, 40)

    m6.set_goal_position(nova_pos6)
    m7.set_goal_position(nova_pos7)

    aguardar_movimento(m6, nova_pos6)
    aguardar_movimento(m7, nova_pos7)

# === Main ===
def main():
    print("📦 Iniciando sequência place.py...")

    # Abrir porta serial e criar objetos motor
    port_handler = PortHandler('/dev/ttyUSB3')
    if not port_handler.openPort():
        print("❌ Não foi possível abrir a porta COM3")
        return
    if not port_handler.setBaudRate(1000000):
        print("❌ Não foi possível configurar baudrate")
        return

    motores = {}
    for i in range(1, 8):
        motor = DynamixelMotor(i, port_handler)
        motor.enable_torque()
        motores[i] = motor

    # Carregar dados JSON
    container_data = carregar_json(CAMINHO_CONTAINER)
    target_data = carregar_json(CAMINHO_TARGET)

    ws_selecionado, objetos_a_manipular = encontrar_primeiro_workspace_valido(target_data)
    if ws_selecionado is None:
        print("✅ Nenhum workspace válido para manipular")
        port_handler.closePort()
        return

    print(f"▶️ Workspace selecionado: {ws_selecionado}")

    for obj in objetos_a_manipular:
        id_obj = obj["name"]
        print(f"🔍 Manipulando objeto ID {id_obj}...")

        container = encontrar_container_por_id(container_data, id_obj)
        if container is None:
            print(f"⚠️ Objeto {id_obj} não encontrado em containers.")
            continue

        print(f"📍 Objeto {id_obj} encontrado no container {container}")

        # Mover até container
        mover_motores(motores, enderecos_containers[container])

        # Fechar garra para pegar o objeto (automático)
        controlar_garra(motores[6],motores[7],-600)

        # Voltar para posição inicial
        voltar_posicao_inicial(motores)

        tipo_ws = target_data[ws_selecionado].get("type")
        if tipo_ws in enderecos_por_tipo:
            print(f"🧭 Executando sequência adicional para tipo {tipo_ws}")
            mover_motores(motores, enderecos_por_tipo[tipo_ws])
        else:
            print(f"ℹ️ Nenhuma sequência definida para tipo {tipo_ws}")

        # Abrir garra ao chegar na base
        controlar_garra(motores[6],motores[7],600)

        # Atualizar containers.json para remover o objeto do container
        container_data[container]["ocupado"] = False
        container_data[container]["id"] = None

    # Marcar workspace como manipulado e salvar
    target_data[ws_selecionado]["manipulado"] = True
    salvar_json(CAMINHO_TARGET, target_data)
    salvar_json(CAMINHO_CONTAINER, container_data)

    # Desabilitar torque e fechar porta
    port_handler.closePort()

    print("✅ Sequência place finalizada.")

if __name__ == "__main__":
    main()
