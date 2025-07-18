import math
import time
import numpy as np
import cv2
import pyrealsense2 as rs
import json
import os
import sys
import random 

from pupil_apriltags import Detector
from dynamixel_sdk import PortHandler
from dynamixel_motor import DynamixelMotor

# === CONFIGURAÇÃO SERIAL ===
DEVICENAME = '/dev/ttyUSB0'
BAUDRATE = 1000000

# === OFFSETS (zero DH → motor) ===
offsets = {
    1: 270,
    2: 94,
    3: 188,
    4: 73,
    5: 180
}

import os
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CAMINHO_JSON = os.path.join(BASE_DIR, "containers.json")
CAMINHO_START_JSON = os.path.join(BASE_DIR, "start_filtrado.json")


def carregar_estado_containers():
    if not os.path.exists(CAMINHO_JSON):
        estado_inicial = {
            "A": {"ocupado": False, "id": None},
            "B": {"ocupado": False, "id": None},
            "C": {"ocupado": False, "id": None}
        }
        with open(CAMINHO_JSON, 'w') as f:
            json.dump(estado_inicial, f, indent=4)
        return estado_inicial
    with open(CAMINHO_JSON, 'r') as f:
        return json.load(f)

def salvar_estado_containers(estado):
    with open(CAMINHO_JSON, 'w') as f:
        json.dump(estado, f, indent=4)

def resetar_containers():
    salvar_estado_containers({
        "A": {"ocupado": False, "id": None},
        "B": {"ocupado": False, "id": None},
        "C": {"ocupado": False, "id": None}
    })
    print("🧹 Containers resetados.")

enderecos_containers = {
    "A": [
        {"motor": 2, "pos": 1800},
        {"motor": 3, "pos": 3250},
        {"motor": 1, "pos": 2790},
        {"motor": 4, "pos": 2230},
        {"motor": 2, "pos": 2100},
    ],
    "B": [
        {"motor": 2, "pos": 1800},
        {"motor": 3, "pos": 3000},
        {"motor": 1, "pos": 3060},
        {"motor": 4, "pos": 2157},
        {"motor": 2, "pos": 2280},
    ],
    "C": [
        {"motor": 2, "pos": 1800},
        {"motor": 3, "pos": 3340},
        {"motor": 1, "pos": 3380},
        {"motor": 4, "pos": 2150},
        {"motor": 2, "pos": 2000},
    ]
}

POSICAO_INICIAL = {
    1: 1055,
    2: 1800,
    3: 3050,
    4: 2180
}

def aguardar_movimento(motor, posicao_alvo, tolerancia=20, timeout=5.0):
    inicio = time.time()
    while time.time() - inicio < timeout:
        pos_atual = motor.get_present_position()
        if abs(pos_atual - posicao_alvo) <= tolerancia:
            return True
        time.sleep(0.05)
    print(f"⚠️ Timeout motor {motor.motor_id}. Última posição: {pos_atual}")
    return False

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

def voltar_posicao_inicial(motores):
    print("🔙 Retornando para posição inicial (2 → 3 → 4 → 1)...")
    for motor_id in [2, 3, 4, 1]:
        pos = POSICAO_INICIAL[motor_id]
        motores[motor_id].set_goal_position(pos)
        aguardar_movimento(motores[motor_id], pos)

def ticks_para_graus(ticks):
    return (ticks % 4096) * 360.0 / 4096.0

def get_angulo_atual(motores, offsets):
    A = {}
    for i in range(1, 6):
        ang_motor = motores[i].get_present_position_deg()
        A[i] = (ang_motor - offsets[i]) % 360
    return A

def calcular_T15(A1, A2, A3, A4, A5):
    c1 = math.cos(math.radians(A1))
    s1 = math.sin(math.radians(A1))
    c5 = math.cos(math.radians(A5))
    s5 = math.sin(math.radians(A5))

    soma234 = A2 + A3 + A4
    c234 = math.cos(math.radians(soma234))
    s234 = math.sin(math.radians(soma234))

    c23 = math.cos(math.radians(A2 + A3))
    s23 = math.sin(math.radians(A2 + A3))

    c2 = math.cos(math.radians(A2))
    s2 = math.sin(math.radians(A2))

    T15 = [
        [
            s1 * s5 + c234 * c1 * c5,
            c5 * s1 - c234 * c1 * s5,
            s234 * c1,
            (c1 * (41 * c23 + 37 * c2)) / 200 + (219 * s234 * c1) / 1250
        ],
        [
            c234 * c5 * s1 - c1 * s5,
            - c1 * c5 - c234 * s1 * s5,
            s234 * s1,
            (s1 * (41 * c23 + 37 * c2)) / 200 + (219 * s234 * s1) / 1250
        ],
        [
            s234 * c5,
            - s234 * s5,
            - c234,
            (41 * s23) / 200 - (219 * c234) / 1250 + (37 * s2) / 200 + 1963 / 20000
        ],
        [0, 0, 0, 1]
    ]
    return T15

def seguir_em_x_y_z(coord_xyz, pixel_c, centro_alvo_px, motores,
                    tolerancia_px=100, tolerancia_z=0.03,
                    max_ajuste_x=30, max_ajuste_y=30, max_ajuste_z=10):
    cx, cy = pixel_c
    centro_x, centro_y = centro_alvo_px
    _, _, Z = coord_xyz

    comandos = {}

    # Correção em X (motor 1)
    diff_x = cx - centro_x
    if abs(diff_x) > tolerancia_px:
        ajuste_x = int(diff_x * 3)
        ajuste_x = max(-max_ajuste_x, min(max_ajuste_x, ajuste_x))
        atual = motores[1].get_present_position()
        nova_pos = atual + ajuste_x
        comandos[1] = max(0, min(4095, nova_pos))
        print(f"🔧 Ajuste em X (motor 1): {ajuste_x} ticks (nova pos: {comandos[1]})")

    # Correção em Y (motor 4)
    diff_y = cy - centro_y
    if abs(diff_y) > tolerancia_px:
        ajuste_y = int(diff_y * 5)
        ajuste_y = max(-max_ajuste_y, min(max_ajuste_y, ajuste_y))
        atual_4 = motores[4].get_present_position()
        nova_pos_4 = atual_4 - ajuste_y
        comandos[4] = max(0, min(4095, nova_pos_4))
        print(f"🔧 Ajuste em Y (motor 4): {ajuste_y} ticks (nova pos: {comandos[4]})")

    # Correção em Z (motores 2 e 3)
    if max_ajuste_z > 0 and abs(Z - 0.4) > tolerancia_z:
        ajuste_z = int((Z - 0.4) * 120)

        if ajuste_z == 0:
            ajuste_z = 12 if (Z - 0.4) > 0 else -12

        ajuste_z = max(-max_ajuste_z, min(max_ajuste_z, ajuste_z))
        for mid in [2, 3]:
            atual = motores[mid].get_present_position()
            nova_pos = atual - ajuste_z
            nova_pos = max(0, min(4095, nova_pos))

            if nova_pos != atual:
                comandos[mid] = nova_pos
        print(f"🔧 Ajuste em Z (motores 2 e 3): {ajuste_z} ticks (posições: {[comandos.get(2, 'sem mudança'), comandos.get(3, 'sem mudança')]})")

    return comandos

def marcar_ws_como_manipulada(ws_nome):
    if not os.path.exists(CAMINHO_START_JSON):
        print(f"❌ Arquivo {CAMINHO_START_JSON} não encontrado.")
        return

    with open(CAMINHO_START_JSON, 'r') as f:
        dados = json.load(f)

    if ws_nome in dados:
        dados[ws_nome]["manipulado"] = True
        with open(CAMINHO_START_JSON, 'w') as f:
            json.dump(dados, f, indent=4)
        print(f"✅ Workspace '{ws_nome}' marcada como manipulada.")
    else:
        print(f"⚠️ Workspace '{ws_nome}' não encontrada.")

def armazenar_tag_no_json(tag_id):
    estado = carregar_estado_containers()
    destino = next((nome for nome, dados in estado.items() if not dados["ocupado"]), None)

    if destino is None:
        print("❌ Todos os containers estão ocupados.")
        return None

    estado[destino]["ocupado"] = True
    estado[destino]["id"] = tag_id
    salvar_estado_containers(estado)
    print(f"✅ Objeto com ID {tag_id} armazenado no container {destino}.")
    return destino  # <--- retorna o container usado


def carregar_tags_da_ws(ws_nome="WS07"):
    if not os.path.exists(CAMINHO_START_JSON):
        print(f"❌ Arquivo {CAMINHO_START_JSON} não encontrado.")
        return []

    with open(CAMINHO_START_JSON, 'r') as f:
        dados = json.load(f)

    if ws_nome not in dados:
        print(f"❌ Workspace '{ws_nome}' não encontrada no JSON.")
        return []

    if dados[ws_nome].get("manipulado", False):
        print(f"⏭️ Workspace '{ws_nome}' já foi manipulada. Pulando.")
        return []

    objetos = dados[ws_nome].get("objects", [])
    if not objetos:
        print(f"⚠️ Nenhum objeto encontrado na workspace {ws_nome}.")
        return []

    # Extrai apenas os objetos válidos (sem decoy)
    return [obj["name"] for obj in objetos if not obj.get("decoy", False)]

def buscar_primeira_ws_nao_manipulada():
    if not os.path.exists(CAMINHO_START_JSON):
        print(f"❌ Arquivo {CAMINHO_START_JSON} não encontrado.")
        return None

    with open(CAMINHO_START_JSON, 'r') as f:
        dados = json.load(f)

    for ws_nome, ws_dados in dados.items():
        if not ws_dados.get("manipulado", False):
            return ws_nome
    return None

def main():
    ws_alvo = buscar_primeira_ws_nao_manipulada()
    if not ws_alvo:
        print("🎉 Todas as workspaces já foram manipuladas. Encerrando.")
        return

    print(f"🔍 Manipulando workspace: {ws_alvo}")
    tags_sequencia = carregar_tags_da_ws(ws_alvo)
    if not tags_sequencia:
        print("⚠️ Sequência de tags vazia. Saindo.")
        return
    port_handler = PortHandler(DEVICENAME)
    if not port_handler.openPort():
        print("❌ Erro ao abrir a porta serial.")
        return
    if not port_handler.setBaudRate(BAUDRATE):
        print("❌ Erro ao configurar a baudrate.")
        return

    motores = {i: DynamixelMotor(i, port_handler) for i in range(1, 8)}
    for m in motores.values():
        m.enable_torque()
        m.set_profile(acceleration=10, velocity=30)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)
    detector = Detector(families="tag36h11", nthreads=4)

    tolerancia_alinhamento = 20  # pixels tolerados para considerar alinhado
    garra_acionada = False  # flag para indicar garra fechada
    ultima_tag_id = None

    # Carrega a sequência de tags a pegar
    #tags_sequencia = carregar_tags_da_ws("WS07")
    indice_tag_atual = 0

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            depth_frame = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            h, w = gray.shape

            terco_h, terco_w = h // 3, w // 3
            for i in range(1, 3):
                cv2.line(color_image, (0, i * terco_h), (w, i * terco_h), (255, 255, 255), 1)
                cv2.line(color_image, (i * terco_w, 0), (i * terco_w, h), (255, 255, 255), 1)

            centro_bloco_2_x = terco_w * 1 + terco_w // 2
            centro_bloco_2_y = terco_h * 0 + terco_h // 2

            tags = detector.detect(gray)

            status_msg = ""
            tag_detectada_atual = False

            # Procura pela tag atual na lista
            tag_esperada = tags_sequencia[indice_tag_atual]

            for tag in tags:
                if tag.tag_id == tag_esperada:
                    tag_detectada_atual = True
                    cx, cy = int(tag.center[0]), int(tag.center[1])
                    distancia = depth_frame.get_distance(cx, cy)
                    intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

                    if distancia == 0.0:
                        T_tag_metros = 0.05
                        focal_length_px = intrinsics.fx
                        corners = tag.corners
                        largura_px = np.linalg.norm(np.array(corners[0]) - np.array(corners[1]))
                        altura_px = np.linalg.norm(np.array(corners[1]) - np.array(corners[2]))
                        tamanho_px = (largura_px + altura_px) / 2

                        if tamanho_px > 0:
                            distancia_estimativa = (focal_length_px * T_tag_metros) / tamanho_px
                            distancia = distancia_estimativa
                            print(f"⚠️ Distância estimada pela AprilTag: {distancia:.3f} m")
                        else:
                            print("❌ Estimativa pela tag falhou.")
                            distancia = 0.0
                    else:
                        print(f"📏 Distância RealSense: {distancia:.3f} m")

                    coord_xyz = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], distancia)
                    A = get_angulo_atual(motores, offsets)
                    T = calcular_T15(A[1], A[2], A[3], A[4], A[5])
                    pos_efetuador = np.array([T[0][3], T[1][3], T[2][3]])
                    print(f"Pos efetuador: {pos_efetuador} | Tag: {coord_xyz}")

                    comandos = {}

                    diff_x = abs(cx - centro_bloco_2_x)
                    diff_y = abs(cy - centro_bloco_2_y)

                    if not garra_acionada:
                        if distancia > 0.15:
                            numero_aleatorio = random.randint(0, 100)  # Gera um número aleatório entre 0 e 100
                            #sei q n faz sentido mas confia no pai(ass. galera)
                            if numero_aleatorio % 2 == 0:  
                                atual = motores[2].get_present_position()
                                nova_pos = atual + 1
                                nova_pos = max(0, min(4095, nova_pos))
                                motores[2].set_goal_position(nova_pos)
                            if diff_x > tolerancia_alinhamento or diff_y > tolerancia_alinhamento:
                                comandos = seguir_em_x_y_z(
                                    coord_xyz, (cx, cy),
                                    (centro_bloco_2_x, centro_bloco_2_y),
                                    motores,
                                    tolerancia_px=tolerancia_alinhamento,
                                    max_ajuste_x=15,
                                    max_ajuste_y=15,
                                    max_ajuste_z=0 if distancia > 0.4 else 0
                                )
                                atual = motores[2].get_present_position()
                                nova_pos = atual + 1
                                nova_pos = max(0, min(4095, nova_pos))
                                motores[2].set_goal_position(nova_pos)
                                status_msg = "⏳ Alinhando X e Y antes da aproximação em Z..."
                            else:
                                comandos = seguir_em_x_y_z(
                                    coord_xyz, (cx, cy),
                                    (centro_bloco_2_x, centro_bloco_2_y),
                                    motores,
                                    tolerancia_px=tolerancia_alinhamento,
                                    max_ajuste_x=0,
                                    max_ajuste_y=0,
                                    max_ajuste_z=1
                                )
                        else:
                            if diff_x > tolerancia_alinhamento or diff_y > tolerancia_alinhamento:
                                comandos = seguir_em_x_y_z(
                                    coord_xyz, (cx, cy),
                                    (centro_bloco_2_x, centro_bloco_2_y),
                                    motores,
                                    tolerancia_px=tolerancia_alinhamento,
                                    max_ajuste_x=15,
                                    max_ajuste_y=15,
                                    max_ajuste_z=0
                                )
                                status_msg = "⏳ Distância pequena. Ajustando X e Y..."
                            else:
                                # Alinhado e perto, ativa garra
                                garra_acionada = True
                                status_msg = "✊ Alinhado e perto - preparando para fechar garra..."

                        for mid in sorted(comandos):
                            motores[mid].set_goal_position(comandos[mid])

                    else:
                        controlar_garra(motores[6], motores[7], -600)  # Fecha garra

                        destino = armazenar_tag_no_json(tag_esperada)
                        if destino:
                            estado = carregar_estado_containers()
                            destino = next((nome for nome, dados in estado.items() if dados["id"] == tag_esperada), None)
                            if destino:
                                print(f"🏁 Levando objeto para container {destino}...")
                                for passo in enderecos_containers[destino]:
                                    motor_obj = motores[passo["motor"]]
                                    motor_obj.set_goal_position(passo["pos"])
                                    aguardar_movimento(motor_obj, passo["pos"])

                                print("✅ Chegou no container, abrindo garra para soltar objeto.")
                                controlar_garra(motores[6], motores[7], 600)  # Abre garra
                                time.sleep(1)

                                voltar_posicao_inicial(motores)
                                garra_acionada = False
                                ultima_tag_id = None

                                # Passa para a próxima tag na sequência
                                indice_tag_atual += 1
                                if indice_tag_atual >= len(tags_sequencia):
                                    print("🎉 Todas as tags da workspace foram processadas.")

                                    # Apaga a workspace usada
                                    marcar_ws_como_manipulada(ws_alvo)

                                    sys.exit(1)

                                else:
                                    print(f"➡️ Próxima tag: {tags_sequencia[indice_tag_atual]}")

                                print("✔️ Ciclo completo.")

                    break  # Processa apenas a tag esperada no frame

            if not tag_detectada_atual:
                status_msg = f"🔍 Esperando a tag {tag_esperada}..."

            cv2.putText(color_image, status_msg, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            if len(tags) == 0:
                cv2.putText(color_image, "Nenhuma AprilTag detectada", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            cv2.imshow("Seguindo Tag (X,Y,Z) - BLOCO 2", color_image)
            if cv2.waitKey(1) & 0xFF == 27:
                print("⏹ Saindo...")
                break

    finally:
        port_handler.closePort()
        pipeline.stop()
        cv2.destroyAllWindows()
        print("✔️ Finalizado.")

if __name__ == "__main__":
    main()
