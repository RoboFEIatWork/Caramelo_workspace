from manipulador import Manipulador5DOF
import time


def executar_movimento_seguindo_dh(encoders, motores, tolerancia=20, timeout=5.0):
    manipulador = Manipulador5DOF()

    if manipulador.check_collision(encoders):
        print("🚫 Movimento abortado! Posição final muito próxima do chão.")
        return False

    # Executa motor por motor
    for i, pos in enumerate(encoders, start=1):
        if i not in motores:
            print(f"⚠️ Motor {i} não encontrado!")
            continue

        motores[i].set_goal_position(pos)
        print(f"🔄 Motor {i} indo para {pos}...")

        inicio = time.time()
        while True:
            atual = motores[i].get_present_position()
            if abs(atual - pos) <= tolerancia:
                print(f"✅ Motor {i} chegou na posição {atual}")
                break
            if time.time() - inicio > timeout:
                print(f"⚠️ Timeout aguardando Motor {i}")
                break

    return True
