# SME Robot Mobile Navigation

Autonomous navigation package for the SME mobile robot based on **ROS 2 Humble**. It uses the **Navigation 2 (Nav2)** stack for localization, planning, and behavior execution.

## Launch Files

### 1. `navigation.launch.py`
Starts the Nav2 stack (map server, AMCL, controller, planner, etc.).

**Arguments:**
- `use_sim_time`: Set to `true` when running in Gazebo (default: `false`).
- `map`: Path to the map YAML file.
- `params_file`: Path to Nav2 parameters file.

### 2. `rviz.launch.py`
Standalone RViz interface with the navigation configuration.

**Arguments:**
- `use_sim_time`: Set to `true` when running in Gazebo (default: `false`).

## Usage Instructions

### A. Running in Simulation (Gazebo)

1. **Start the Gazebo simulation:**
   ```bash
   ros2 launch sme_robot_mobile_description gazebo.launch.py
   ```

2. **Start the navigation stack:**
   ```bash
   ros2 launch sme_robot_mobile_navigation navigation.launch.py use_sim_time:=true
   ```

3. **Start RViz (Optional):**
   ```bash
   ros2 launch sme_robot_mobile_navigation rviz.launch.py use_sim_time:=true
   ```

### B. Running on the Real Robot

1. **Start the navigation stack:**
   ```bash
   ros2 launch sme_robot_mobile_navigation navigation.launch.py
   ```

2. **Start RViz (on your workstation):**
   ```bash
   ros2 launch sme_robot_mobile_navigation rviz.launch.py
   ```

### C. Using a Custom Map
```bash
ros2 launch sme_robot_mobile_navigation navigation.launch.py map:=/path/to/your/map.yaml
```