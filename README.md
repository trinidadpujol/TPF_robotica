# TPF Robótica - SLAM con TurtleBot3

Trabajo Práctico Final de Robótica Autónoma - Implementación de SLAM usando FastSLAM.

## 📋 Requisitos Previos

- Ubuntu 22.04
- ROS2 Humble
- TurtleBot3 packages instalados
- Gazebo
- tf_transformations: `pip install transforms3d`

## 🚀 Instalación y Configuración

### 1. Clonar/Descargar el proyecto
```bash
# Descargar el proyecto en tu directorio preferido
cd ~/Documentos  # o donde quieras
# Descomprimir/clonar el proyecto aquí
```

### 2. Instalar dependencias
```bash
# Instalar tf_transformations
pip install transforms3d

# Verificar que TurtleBot3 esté instalado
ros2 pkg list | grep turtlebot
```

### 3. Compilar el workspace
```bash
cd TPF_robotica  # o el nombre de tu carpeta
colcon build
```

### 4. Configurar entorno (IMPORTANTE)
```bash
# Desde la raíz del workspace
source setup_env.sh
```

## 🎮 Ejecutar el Proyecto

### Terminal 1: Ejecutar SLAM
```bash
# Desde la raíz del workspace
source setup_env.sh
ros2 launch turtlebot3_slam_mapper python_slam_maze.launch.py
```

### Terminal 2: Teleoperación
```bash
# En una nueva terminal
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

## 📊 Qué deberías ver

1. **Gazebo**: Robot TurtleBot3 en un laberinto
2. **RViz**: 
   - Robot con LIDAR (rayos rojos)
   - Mapa que se construye en tiempo real (negro=ocupado, blanco=libre)
   - Partículas del filtro (opcional)

## 🎯 Controles

- `w/s`: Avanzar/Retroceder
- `a/d`: Girar izquierda/derecha
- `x`: Detener
- `q/z`: Aumentar/Disminuir velocidad

## 💾 Guardar Mapa

```bash
# En una nueva terminal
ros2 run nav2_map_server map_saver_cli -f ~/mi_mapa
```

## 🔧 Estructura del Proyecto

```
TPF_robotica/
├── archivos/
│   ├── turtlebot3_maze.world          # Mundo del laberinto
│   ├── turtlebot3_maze.launch.py      # Launch de Gazebo
│   └── my_png_model/                  # Modelo del laberinto
├── turtlebot3_slam_mapper/            # Paquete ROS2 principal
│   ├── launch/
│   │   └── python_slam_maze.launch.py # Launch principal
│   ├── turtlebot3_slam_mapper/
│   │   └── python_slam_node.py        # Nodo SLAM (FastSLAM)
│   └── rviz/
│       └── slam_maze.rviz             # Configuración RViz
├── setup_env.sh                       # Script de configuración
└── README.md                          # Este archivo
```

## 🛠️ Solución de Problemas

### Error: "No module named 'tf_transformations'"
```bash
pip install transforms3d
```

### Error: "Unable to find uri[model://my_png_model]"
```bash
# Asegúrate de ejecutar setup_env.sh
source setup_env.sh
```

### Gazebo se abre vacío
- Verifica que `turtlebot3_maze.world` exista en `archivos/`
- Ejecuta `source setup_env.sh` antes del launch

### No se ve el mapa en RViz
- Verifica que el nodo `python_slam_node` esté ejecutándose
- Mueve el robot con teleop para que construya el mapa

## 👥 Autores

- Agustin Manzano
- Trinidad Pujol

## 📅 Fecha límite

29/06/25, 23:59hs