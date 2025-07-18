import subprocess
import os

import os
BASE_DIR = os.path.dirname(os.path.abspath(__file__))


def main():
    base_path = os.path.dirname(os.path.abspath(__file__))

    print("🚀 Iniciando execução de translate.py...")
    subprocess.run(["python3", os.path.join(BASE_DIR, "translate.py")])
    print("✅ translate.py finalizado com sucesso.\n")

    print("🚀 Iniciando execução de pegada.py...")
    subprocess.run(["python3", os.path.join(BASE_DIR, "pegada.py")])
    print("✅ pegada.py finalizado com sucesso.\n")

    print("🚀 Iniciando execução de place.py...")
    subprocess.run(["python3", os.path.join(BASE_DIR, "place.py")])
    print("✅ place.py finalizado com sucesso.")

if __name__ == "__main__":
    main()
