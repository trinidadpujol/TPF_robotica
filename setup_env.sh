#!/bin/bash

# Script para configurar el entorno de forma portable
# Este script debe ejecutarse desde la raíz del workspace

# Obtener la ruta absoluta del directorio actual (workspace)
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Configurar variables de entorno de TurtleBot3
export TURTLEBOT3_MODEL=burger

# Configurar el path de modelos de Gazebo para incluir nuestro modelo personalizado
export GAZEBO_MODEL_PATH="$WORKSPACE_DIR/archivos:$GAZEBO_MODEL_PATH"

# Configurar el workspace de ROS2
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
    echo "✅ Entorno configurado correctamente"
    echo "📁 Workspace: $WORKSPACE_DIR"
    echo "🤖 Modelo TurtleBot3: $TURTLEBOT3_MODEL"
    echo "🌍 Modelos Gazebo: $GAZEBO_MODEL_PATH"
else
    echo "❌ Error: No se encontró install/setup.bash"
    echo "   Ejecuta 'colcon build' primero"
    exit 1
fi
