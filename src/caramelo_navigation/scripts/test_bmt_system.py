#!/usr/bin/env python3
"""
TESTE RÁPIDO DO SISTEMA BMT INTEGRADO
Script para validar implementação baseada no sistema funcional
"""

import json
import os

import yaml


def test_waypoints_file():
    """Testa se o arquivo de waypoints está correto"""
    waypoints_file = '/home/work/Caramelo_workspace/maps/teste_robocup25/waypoints.json'
    
    try:
        with open(waypoints_file, 'r') as f:
            data = json.load(f)
            
        waypoints = data.get('waypoints', [])
        print(f"✅ Waypoints carregados: {len(waypoints)} pontos")
        
        for wp in waypoints:
            name = wp.get('name', 'UNNAMED')
            pos = wp.get('position', {})
            print(f"   - {name}: ({pos.get('x', 0):.2f}, {pos.get('y', 0):.2f})")
            
        return True
        
    except Exception as e:
        print(f"❌ Erro ao carregar waypoints: {e}")
        return False

def test_task_file():
    """Testa se o arquivo de tasks está correto"""
    task_file = '/home/work/Caramelo_workspace/src/caramelo_tasks/BMT/task.yaml'
    
    try:
        with open(task_file, 'r') as f:
            data = yaml.safe_load(f)
            
        # Formato RoboCup
        if 'start' in data and 'target' in data:
            start_tasks = data.get('start', [])
            target_tasks = data.get('target', [])
            
            print(f"✅ Tasks RoboCup carregadas:")
            print(f"   - Start workstations: {len(start_tasks)}")
            for ws in start_tasks:
                ws_name = ws.get('ws', 'N/A')
                ws_type = ws.get('type', 'N/A')
                objects = ws.get('objects', [])
                print(f"     {ws_name} (tipo {ws_type}): {len(objects)} objetos")
                
            print(f"   - Target workstations: {len(target_tasks)}")
            for ws in target_tasks:
                ws_name = ws.get('ws', 'N/A')
                ws_type = ws.get('type', 'N/A')
                objects = ws.get('objects', [])
                print(f"     {ws_name} (tipo {ws_type}): {len(objects)} objetos")
        else:
            print(f"❌ Formato de task.yaml não é RoboCup válido")
            return False
            
        return True
        
    except Exception as e:
        print(f"❌ Erro ao carregar tasks: {e}")
        return False

def test_config_files():
    """Testa se os arquivos de configuração existem"""
    config_files = [
        '/home/work/Caramelo_workspace/src/caramelo_navigation/config/caramelo_nav2_functional.yaml',
        '/home/work/Caramelo_workspace/src/caramelo_navigation/launch/caramelo_bmt_navigation.launch.py',
        '/home/work/Caramelo_workspace/src/caramelo_navigation/caramelo_navigation/caramelo_bmt_waypoint_nav.py'
    ]
    
    all_good = True
    
    for config_file in config_files:
        if os.path.exists(config_file):
            print(f"✅ {os.path.basename(config_file)} existe")
        else:
            print(f"❌ {os.path.basename(config_file)} NÃO EXISTE")
            all_good = False
            
    return all_good

def main():
    print("🧪 TESTE DO SISTEMA BMT INTEGRADO")
    print("=" * 50)
    
    print("\n📍 Testando Waypoints...")
    wp_ok = test_waypoints_file()
    
    print("\n📋 Testando Tasks BMT...")
    task_ok = test_task_file()
    
    print("\n📁 Testando Arquivos de Configuração...")
    config_ok = test_config_files()
    
    print("\n" + "=" * 50)
    
    if wp_ok and task_ok and config_ok:
        print("🎉 TODOS OS TESTES PASSARAM!")
        print("✅ Sistema BMT integrado pronto para navegação RoboCup")
        print("\n🚀 Para iniciar com BMT:")
        print("   ros2 launch caramelo_navigation caramelo_bmt_navigation.launch.py task_type:=BMT")
        print("\n🚀 Para iniciar com BTT1:")
        print("   ros2 launch caramelo_navigation caramelo_bmt_navigation.launch.py task_type:=BTT1")
        print("\n🚀 Para iniciar com BTT2:")
        print("   ros2 launch caramelo_navigation caramelo_bmt_navigation.launch.py task_type:=BTT2")
    else:
        print("❌ ALGUNS TESTES FALHARAM!")
        print("⚠️  Verifique os erros acima antes de prosseguir")

if __name__ == '__main__':
    main()
