# SME Robot Mobile Navigation

Paquete de navegación autónoma para el robot móvil SME. Incluye SLAM, localización (AMCL) y planificación de trayectorias (Move Base).

## Launch Files

### 1. `navigation.launch`
**Navegación con mapa pre-construido**

Inicia el stack completo de navegación usando un mapa previamente guardado.

```bash
roslaunch sme_robot_mobile_navigation navigation.launch
```

**Componentes:**
- Gazebo (simulación)
- Map Server (carga mapa estático)
- AMCL (localización)
- Move Base (planificación global y local con TEB)
- RViz (visualización)

**Argumentos:**
- `map_file`: Ruta al archivo `.yaml` del mapa (default: `maps/map.yaml`)
- `open_rviz`: Abrir RViz automáticamente (default: `true`)
- `move_forward_only`: Restringir movimiento solo hacia adelante (default: `false`)

---

### 2. `slam_gmapping.launch`
**SLAM para construcción de mapas**

Usa GMapping para crear mapas de forma autónoma. Solo mapeo, sin navegación.

```bash
roslaunch sme_robot_mobile_navigation slam_gmapping.launch
```

**Componentes:**
- Gazebo (simulación)
- GMapping (SLAM)
- RViz (visualización)

**Uso:** Teleopera el robot para explorar y construir el mapa. Guarda el mapa con:
```bash
rosrun map_server map_saver -f nombre_mapa
```

---

### 3. `slam_navigation.launch`
**SLAM + Navegación simultánea (GMapping)**

Combina GMapping con Move Base para navegar mientras construye el mapa.

```bash
roslaunch sme_robot_mobile_navigation slam_navigation.launch
```

**Componentes:**
- Gazebo (simulación con mundo `test_room.world`)
- GMapping (SLAM)
- Move Base (navegación autónoma)
- RViz (visualización)

**Uso:** Envía goals en RViz. El robot navegará y construirá el mapa simultáneamente.

---

### 4. `slam_toolbox_navigation.launch` ⭐ **Recomendado**
**SLAM + Navegación con SLAM Toolbox**

Usa SLAM Toolbox (superior a GMapping) para mapeo dinámico y navegación.

```bash
roslaunch sme_robot_mobile_navigation slam_toolbox_navigation.launch
```

**Componentes:**
- Gazebo (simulación con mundo `test_room.world`)
- SLAM Toolbox (SLAM con soporte dinámico)
- Move Base (navegación autónoma)
- RViz (visualización)

**Ventajas sobre GMapping:**
- ✅ Maneja entornos dinámicos (personas, objetos móviles)
- ✅ Actualización de mapa cuando se remueven obstáculos
- ✅ Mejor cierre de bucles en entornos grandes
- ✅ Mapeo de por vida (lifelong mapping)
- ✅ Serialización de mapas (guardar/cargar)

**Uso:** Envía goals en RViz. El mapa se actualiza dinámicamente cuando cambia el entorno.

---

## Configuración

Los parámetros de navegación se encuentran en `config/`:

- **Costmaps:** `common/costmap_common_params.yaml`, `local_costmap_params.yaml`, `global_costmap_params.yaml`
- **Planificadores:**
  - Global: `global_planner_params.yaml`, `navfn_planner_params.yaml`
  - Local: `teb_local_planner_params.yaml`
- **SLAM:**
  - GMapping: `gmapping/gmapping_params.yaml`
  - SLAM Toolbox: `slam_toolbox/slam_toolbox_params.yaml`
- **Move Base:** `move_base_params.yaml`

## Dependencias

- `gazebo_ros`
- `move_base`
- `amcl`
- `map_server`
- `gmapping`
- `slam_toolbox`
- `teb_local_planner`
- `sme_robot_mobile_description`