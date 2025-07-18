import yaml

# Caminho para o YAML exportado do MATLAB
yaml_path = 'arena_state.yaml'

with open(yaml_path, 'r') as f:
    data = yaml.safe_load(f)

start = data['start']
target = data['target']

# Copiar lista para permitir marcação
remaining_targets = target.copy()
transitions = []

for s_obj in start:
    for i, t_obj in enumerate(remaining_targets):
        # Casar por ID e decoy (poderia incluir outros critérios se necessário)
        if s_obj['id'] == t_obj['id'] and s_obj['decoy'] == t_obj['decoy']:
            transitions.append({
                'id': s_obj['id'],
                'from_ws': s_obj['ws'],
                'from_type': s_obj['type'],
                'to_ws': t_obj['ws'],
                'to_type': t_obj['type'],
                'decoy': s_obj['decoy']
            })
            # Remover o item correspondente da lista de destino para evitar duplicatas
            del remaining_targets[i]
            break  # ir para o próximo item da origem

# Exibir transições
for t in transitions:
    print(f"FROM: {t['from_ws']} (Type {t['from_type']})")
    print(f"TO:   {t['to_ws']} (Type {t['to_type']})")
    print(f"ID:   {t['id']} | Decoy: {t['decoy']}\n")
