#!/bin/bash

# Diretório onde estão os arquivos .bag
BAG_DIR="$HOME"
# Encontra o mais recente arquivo .bag
LATEST_BAG=$(ls -t "$BAG_DIR"/*.bag 2>/dev/null | head -n 1)

# Caminho de saída fixo
OUTPUT_FILE="$HOME/Caramelo_workspace/src/caramelo_tasks"

# Verificação
if [ -z "$LATEST_BAG" ]; then
    echo "❌ Nenhum arquivo .bag encontrado em $BAG_DIR"
    exit 1
fi

echo "✅ Usando bagfile: $LATEST_BAG"
echo "📁 Salvando YAML em: $OUTPUT_FILE"

# Caminho do script MATLAB
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/../matlab"

/usr/local/MATLAB/R2025a/bin/matlab -batch "process_bagfile('$LATEST_BAG', '$OUTPUT_FILE')"
