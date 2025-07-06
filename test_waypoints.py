#!/usr/bin/env python3
"""
Teste do Interactive Robot Positioner
"""
import json
import os

# Testar se o arquivo waypoints.json existe
waypoints_file = "/home/work/Caramelo_workspace/src/caramelo_navigation/config/waypoints.json"

if os.path.exists(waypoints_file):
    with open(waypoints_file, 'r') as f:
        data = json.load(f)
    
    print(f"✅ Arquivo waypoints.json encontrado!")
    print(f"📍 Frame ID: {data.get('frame_id')}")
    print(f"📊 Total waypoints: {len(data.get('waypoints', []))}")
    
    # Contar waypoints WS
    ws_count = 0
    for wp in data.get('waypoints', []):
        if wp.get('name', '').startswith('WS'):
            ws_count += 1
    
    print(f"🔢 Waypoints WS: {ws_count}")
    
    # Mostrar todos os waypoints
    print("\n📋 Lista de waypoints:")
    for wp in data.get('waypoints', []):
        name = wp.get('name', 'Unknown')
        pos = wp.get('position', {})
        x, y = pos.get('x', 0), pos.get('y', 0)
        print(f"  - {name}: ({x:.2f}, {y:.2f})")
        
else:
    print("❌ Arquivo waypoints.json não encontrado!")
