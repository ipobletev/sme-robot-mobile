# SME Robot Mobile Navigation

Autonomous navigation package for the SME mobile robot based on **ROS 2 Humble**. It uses the **Navigation 2 (Nav2)** stack for localization, planning, and behavior execution.

## Navigation Modes

### 1. SLAM Navigation (No AMCL)
Used when a SLAM system (like **Cartographer**) is providing the `map` frame and the `map` topic. It only launches the Nav2 planner, controller, and behavior trees.

*   **Launch File:** `navigation_slam.launch.py`
*   **Params:** `nav2_params_slam.yaml`

### 2. Static Map Navigation (With AMCL)
Used with a pre-saved map. It launches the `map_server` to load the `.yaml` map and **AMCL** for localization within that map.

*   **Launch File:** `navigation_static.launch.py`
*   **Params:** `nav2_params_static.yaml`

## Launch Files

### 1. `navigation_slam.launch.py` / `navigation_static.launch.py`
Starts the Nav2 stack nodes according to the selected mode.

**Arguments:**
- `use_sim_time`: Set to `true` when running in Gazebo (default: `false`).
- `map`: Path to the map YAML file (only for `navigation_static`).
- `params_file`: Path to Nav2 parameters file.

### 2. `nav_rviz.launch.py`
Standalone RViz interface with the navigation configuration.

**Arguments:**
- `use_sim_time`: Set to `true` when running in Gazebo (default: `false`).

## Usage Instructions

### A. Navigation with SLAM (Cartographer)

1. **Start the simulation:**
   ```bash
   ros2 launch sme_robot_mobile_description gazebo.launch.py
   ```

2. **Start Cartographer:**
   ```bash
   ros2 launch sme_robot_mobile_cartographer cartographer.launch.py use_sim_time:=true
   ```

3. **Start SLAM Navigation:**
   ```bash
   ros2 launch sme_robot_mobile_navigation navigation_slam.launch.py use_sim_time:=true
   ```

### B. Navigation with Static Map (AMCL)

1. **Start the simulation:**
   ```bash
   ros2 launch sme_robot_mobile_description gazebo.launch.py
   ```

2. **Start Static Navigation:**
   ```bash
   ros2 launch sme_robot_mobile_navigation navigation_static.launch.py use_sim_time:=true map:=path/to/your/map.yaml
   ```

3. **Start RViz:**
   ```bash
   ros2 launch sme_robot_mobile_navigation nav_rviz.launch.py use_sim_time:=true
   ```