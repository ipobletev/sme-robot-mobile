# SME Robot Mobile Navigation

Paquete de navegación autónoma para el robot móvil SME basado en **ROS 2 Humble**. Utiliza el stack de **Navigation 2 (Nav2)** para localización, planificación y ejecución de comportamientos.

## Launch Files

### 1. `navigation.launch.py`
**Navegación completa con mapa y simulación**

Inicia el entorno de simulación en Gazebo, el stack de Nav2 y RViz en un solo comando.

```bash
ros2 launch sme_robot_mobile_navigation navigation.launch.py
```

**Componentes:**
- **Gazebo Sim**: Simulación física del robot y entorno.
- **Nav2 Bringup**: Stack completo de navegación.
  - **AMCL**: Localización probabilística basada en partículas.
  - **Planner Server**: Planificación de rutas globales.
  - **Controller Server**: Seguimiento de trayectorias locales (DWB).
  - **Behavior Server**: Comportamientos de recuperación (giros, retroceso).
  - **Map Server**: Carga el mapa estático.
- **RViz2**: Visualización preconfigurada para navegación.

**Argumentos:**
- `path_to_map`: Ruta al archivo `.yaml` del mapa (default: `map/map.yaml`).
- `nav2_params_file`: Ruta al archivo de parámetros de Nav2 (default: `config/nav2_params.yaml`).
- `use_rviz`: Abrir RViz2 automáticamente (default: `true`).

---

### 2. SLAM (Construcción de Mapas)
**Uso de SLAM Toolbox**

Para construir un mapa nuevo, se recomienda usar `slam_toolbox`. Puedes ejecutarlo junto con la descripción del robot:

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

**Guardar el mapa:**
Una vez explorado el entorno, guarda el mapa usando el plugin de SLAM Toolbox en RViz o vía terminal:
```bash
ros2 run nav2_map_server map_saver_cli -f src/sme_robot_mobile_navigation/map/map
```

---

## Configuración

Los parámetros de navegación se centralizan en un único archivo YAML compatible con ROS 2:

- **Nav2 Params:** [nav2_params.yaml](file:///home/isma/Desktop/sme-robot-mobile/src/sme_robot_mobile_navigation/config/nav2_params.yaml)
  - Configuración de AMCL, Costmaps (Global/Local), Planners y Controllers.

## Dependencias

- `nav2_bringup`
- `nav2_amcl`
- `nav2_map_server`
- `nav2_lifecycle_manager`
- `slam_toolbox`
- `ros_gz_sim` (Gazebo)
- `sme_robot_mobile_description`