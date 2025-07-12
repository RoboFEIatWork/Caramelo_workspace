#!/usr/bin/env python3
"""
DIAGNÓSTICO DE ESCALA DO MAPA
Verifica se a escala do mapa está correta comparando com o modelo URDF
"""

import os
import sys

import yaml


def analyze_map_scale():
    """Analisa a escala atual do mapa"""
    
    # Carregar configuração do mapa
    map_file = "/home/work/Caramelo_workspace/maps/arena_fei/map.yaml"
    
    if not os.path.exists(map_file):
        print(f"❌ Arquivo de mapa não encontrado: {map_file}")
        return
        
    with open(map_file, 'r') as f:
        map_config = yaml.safe_load(f)
    
    resolution = map_config.get('resolution', 0.02)
    origin = map_config.get('origin', [0, 0, 0])
    
    print("🗺️  ANÁLISE DE ESCALA DO MAPA")
    print("=" * 50)
    print(f"📏 Resolução atual: {resolution} m/pixel")
    print(f"📍 Origem: {origin}")
    
    # Calcular tamanho do mapa em metros
    # Assumindo tamanho de 359x425 pixels (do log)
    map_width_pixels = 359
    map_height_pixels = 425
    
    map_width_meters = map_width_pixels * resolution
    map_height_meters = map_height_pixels * resolution
    
    print(f"🗺️  Tamanho do mapa:")
    print(f"   - Largura: {map_width_pixels} pixels = {map_width_meters:.2f} metros")
    print(f"   - Altura: {map_height_pixels} pixels = {map_height_meters:.2f} metros")
    
    # Calcular limites do mapa
    min_x = origin[0]
    max_x = origin[0] + map_width_meters
    min_y = origin[1] 
    max_y = origin[1] + map_height_meters
    
    print(f"🌐 Limites do mapa em metros:")
    print(f"   - X: {min_x:.2f} a {max_x:.2f} metros")
    print(f"   - Y: {min_y:.2f} a {max_y:.2f} metros")
    
    # Verificar waypoints dentro dos limites
    waypoints_file = "/home/work/Caramelo_workspace/maps/arena_fei/waypoints.json"
    if os.path.exists(waypoints_file):
        import json
        with open(waypoints_file, 'r') as f:
            waypoints_data = json.load(f)
            
        print(f"\n🎯 VERIFICAÇÃO DE WAYPOINTS:")
        waypoints = waypoints_data.get('waypoints', [])
        
        for wp in waypoints:
            name = wp.get('name', 'Sem nome')
            pos = wp.get('position', {})
            x = pos.get('x', 0)
            y = pos.get('y', 0)
            
            # Verificar se está dentro dos limites
            x_ok = min_x <= x <= max_x
            y_ok = min_y <= y <= max_y
            
            status = "✅" if (x_ok and y_ok) else "❌"
            print(f"   {status} {name}: ({x:.2f}, {y:.2f})")
            
            if not x_ok:
                print(f"      ⚠️  X fora dos limites! ({min_x:.2f} a {max_x:.2f})")
            if not y_ok:
                print(f"      ⚠️  Y fora dos limites! ({min_y:.2f} a {max_y:.2f})")
    
    # Sugestões de correção
    print(f"\n🔧 SUGESTÕES DE CORREÇÃO:")
    
    # Verificar se resolução está razoável
    if resolution < 0.01:
        print(f"   ⚠️  Resolução muito alta ({resolution}m) - considere 0.02-0.05m")
    elif resolution > 0.1:
        print(f"   ⚠️  Resolução muito baixa ({resolution}m) - considere 0.02-0.05m")
    else:
        print(f"   ✅ Resolução adequada ({resolution}m)")
    
    # Verificar origem
    if abs(origin[0]) > 10 or abs(origin[1]) > 10:
        print(f"   ⚠️  Origem muito deslocada {origin[:2]} - considere valores próximos de [0,0]")
    else:
        print(f"   ✅ Origem adequada {origin[:2]}")
        
    # Sugerir resolução alternativa
    print(f"\n💡 TESTE DE RESOLUÇÕES ALTERNATIVAS:")
    for test_res in [0.01, 0.03, 0.05, 0.1]:
        test_width = map_width_pixels * test_res
        test_height = map_height_pixels * test_res
        print(f"   - {test_res}m/pixel: {test_width:.1f}m x {test_height:.1f}m")

if __name__ == "__main__":
    analyze_map_scale()
