#!/usr/bin/env python3
"""
TESTE DE CONFIGURAÇÃO DE COSTMAPS
Verifica se os parâmetros de resolução e inflação estão corretos
"""

import os

import yaml


def test_costmap_config():
    """Testa se as configurações de costmap estão corretas"""
    config_file = '/home/work/Caramelo_workspace/src/caramelo_navigation/config/caramelo_nav2_functional.yaml'
    
    try:
        with open(config_file, 'r') as f:
            data = yaml.safe_load(f)
        
        print("🔍 VERIFICANDO CONFIGURAÇÕES DE COSTMAP")
        print("=" * 50)
        
        # Verificar local_costmap
        local_costmap = data.get('local_costmap', {}).get('local_costmap', {}).get('ros__parameters', {})
        local_resolution = local_costmap.get('resolution', 'N/A')
        local_inflation = None
        
        inflation_layer = local_costmap.get('inflation_layer', {})
        if inflation_layer:
            local_inflation = inflation_layer.get('inflation_radius', 'N/A')
        
        print(f"📍 LOCAL COSTMAP:")
        print(f"   - Resolução: {local_resolution} ({'✅ 2cm' if local_resolution == 0.02 else '❌ Deveria ser 0.02'})")
        print(f"   - Inflação:  {local_inflation} ({'✅ 1cm' if local_inflation == 0.01 else '❌ Deveria ser 0.01'})")
        
        # Verificar global_costmap
        global_costmap = data.get('global_costmap', {}).get('global_costmap', {}).get('ros__parameters', {})
        global_resolution = global_costmap.get('resolution', 'N/A')
        global_inflation = None
        
        inflation_layer = global_costmap.get('inflation_layer', {})
        if inflation_layer:
            global_inflation = inflation_layer.get('inflation_radius', 'N/A')
        
        print(f"🌍 GLOBAL COSTMAP:")
        print(f"   - Resolução: {global_resolution} ({'✅ 2cm' if global_resolution == 0.02 else '❌ Deveria ser 0.02'})")
        print(f"   - Inflação:  {global_inflation} ({'✅ 1cm' if global_inflation == 0.01 else '❌ Deveria ser 0.01'})")
        
        # Verificar robot_radius
        local_radius = local_costmap.get('robot_radius', 'N/A')
        global_radius = global_costmap.get('robot_radius', 'N/A')
        
        print(f"🤖 ROBOT RADIUS:")
        print(f"   - Local:  {local_radius} ({'✅ 18.5cm' if local_radius == 0.185 else '❌'})")
        print(f"   - Global: {global_radius} ({'✅ 18.5cm' if global_radius == 0.185 else '❌'})")
        
        print("\n" + "=" * 50)
        
        # Verificar se todas as configurações estão corretas
        configs_ok = (
            local_resolution == 0.02 and
            local_inflation == 0.01 and
            global_resolution == 0.02 and
            global_inflation == 0.01 and
            local_radius == 0.185 and
            global_radius == 0.185
        )
        
        if configs_ok:
            print("🎉 TODAS AS CONFIGURAÇÕES ESTÃO CORRETAS!")
            print("✅ Resolução: 2cm (0.02)")
            print("✅ Inflação: 1cm (0.01) - MÍNIMA para navegação precisa")
            print("✅ Robot radius: 18.5cm (0.185)")
            print("\n💡 Benefícios desta configuração:")
            print("   - Maior precisão de mapeamento (2cm vs 5cm)")
            print("   - Inflação mínima (1cm) permite navegação muito próxima às paredes")
            print("   - Mantém segurança com robot_radius de 18.5cm")
            print("   - Ideal para corredores estreitos e navegação precisa")
        else:
            print("❌ ALGUMAS CONFIGURAÇÕES PRECISAM SER AJUSTADAS")
            print("⚠️  Verifique os valores acima")
        
        return configs_ok
        
    except Exception as e:
        print(f"❌ Erro ao carregar configuração: {e}")
        return False

def main():
    print("🧪 TESTE DE CONFIGURAÇÃO DE COSTMAPS")
    print("Verificando resolução de 2cm e inflação de 4cm\n")
    
    test_costmap_config()

if __name__ == '__main__':
    main()
