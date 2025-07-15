#!/bin/bash

# Script para converter XACRO para URDF
# Usado pelo complete_navigation.launch.py

XACRO_FILE="/home/work/Caramelo_workspace/src/caramelo_description/URDF/robot.urdf.xacro"
URDF_FILE="/home/work/Caramelo_workspace/src/caramelo_description/urdf/caramelo.urdf"

# Criar diretório se não existir
mkdir -p /home/work/Caramelo_workspace/src/caramelo_description/urdf

# Converter XACRO para URDF
ros2 run xacro xacro "$XACRO_FILE" > "$URDF_FILE"

echo "URDF gerado em: $URDF_FILE"
