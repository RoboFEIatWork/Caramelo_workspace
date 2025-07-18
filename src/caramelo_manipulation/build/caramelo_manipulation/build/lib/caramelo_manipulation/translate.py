import yaml
import json
import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CAMINHO_YAML = os.path.join(BASE_DIR, "output.yaml")

# Lê o arquivo YAML
if not os.path.exists(CAMINHO_YAML):
    print(f"❌ Arquivo {CAMINHO_YAML} não encontrado. Abortando.")
    exit(1)

with open(CAMINHO_YAML, 'r') as arquivo:
    dados = yaml.safe_load(arquivo)

# Função para filtrar e reestruturar os dados
def filtrar_secao(secao_nome):
    resultado = {}
    for item in dados.get(secao_nome, []):
        nome = item['name']
        resultado[nome] = {
            'type': item['type'],
            'objects': item['objects'],
            'manipulado': False
        }
    return resultado

start_filtrado = filtrar_secao('start')
target_filtrado = filtrar_secao('target')

with open(os.path.join(BASE_DIR, "start_filtrado.json"), "w") as f:
    json.dump(start_filtrado, f, indent=2)

with open(os.path.join(BASE_DIR, "target_filtrado.json"), "w") as f:
    json.dump(target_filtrado, f, indent=2)
