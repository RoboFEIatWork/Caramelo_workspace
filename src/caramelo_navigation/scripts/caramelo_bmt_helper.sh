#!/bin/bash
"""
CARAMELO BMT NAVIGATION - Helper Script
Sistema integrado de navegação RoboCup com arquivos reais
"""

echo "🤖 CARAMELO BMT NAVIGATION - Sistema RoboCup Integrado"
echo "======================================================="
echo ""

# Verificar arenas disponíveis
echo "📍 ARENAS DISPONÍVEIS:"
for arena in /home/work/Caramelo_workspace/maps/*/; do
    arena_name=$(basename "$arena")
    if [ -f "$arena/map.yaml" ] && [ -f "$arena/waypoints.json" ]; then
        echo "   ✅ $arena_name (mapa + waypoints)"
    elif [ -f "$arena/map.yaml" ]; then
        echo "   ⚠️  $arena_name (apenas mapa)"
    else
        echo "   ❌ $arena_name (incompleto)"
    fi
done

echo ""
echo "📋 TIPOS DE TASK DISPONÍVEIS:"
echo "   - BMT  (Basic Manipulation Test)"
echo "   - BTT1 (Basic Transportation Test 1)"  
echo "   - BTT2 (Basic Transportation Test 2)"

echo ""
echo "🚀 COMO USAR:"
echo ""
echo "1. Para arena teste_robocup25 com BMT:"
echo "   ros2 launch caramelo_navigation caramelo_bmt_navigation.launch.py arena:=teste_robocup25 task_type:=BMT"
echo ""
echo "2. Para arena_robocup25 com BTT1:"
echo "   ros2 launch caramelo_navigation caramelo_bmt_navigation.launch.py arena:=arena_robocup25 task_type:=BTT1"
echo ""
echo "3. Para lab_fei com BTT2:" 
echo "   ros2 launch caramelo_navigation caramelo_bmt_navigation.launch.py arena:=teste_lab task_type:=BTT2"
echo ""
echo "📌 PARÂMETROS OBRIGATÓRIOS:"
echo "   - arena:     Nome da pasta em /maps/ (teste_robocup25, arena_robocup25, etc.)"
echo ""
echo "📌 PARÂMETROS OPCIONAIS:"
echo "   - task_type: BMT, BTT1, BTT2 (padrão: BMT)"
echo "   - auto_start: true/false (padrão: true)"
echo ""
echo "✅ Sistema pronto! Baseado no commit funcional 7e8a309 + integração RoboCup"
