# SME Robot Mobile Cartographer

This package provides SLAM (Simultaneous Localization and Mapping) capabilities for the SME mobile robot using Google Cartographer.

## Package Overview

The package is configured to work with the robot's sensors (Lidar and IMU) to create 2D maps of the environment.

### Key Files:
- `config/cartographer.lua`: Cartographer configuration parameters.
- `launch/cartographer.launch.py`: Main launch file for SLAM and visualization.
- `rviz/cartographer.rviz`: Pre-configured RViz workspace for mapping.

## Usage

### 1. Build the package
Ensure you are in the workspace root:
```bash
colcon build --packages-select sme_robot_mobile_cartographer
source install/setup.bash
```

### 2. Launch SLAM
Make sure the robot (real or Gazebo) is already running.

#### For Simulation (Gazebo):
```bash
ros2 launch sme_robot_mobile_cartographer cartographer.launch.py use_sim_time:=true
```

#### For Real Robot:
```bash
ros2 launch sme_robot_mobile_cartographer cartographer.launch.py
```

## Configuration Details

The current configuration (`cartographer.lua`) is set up with:
- **Tracking Frame**: `imu_link`
- **Map Frame**: `map`
- **Odom Frame**: `odom`
- **Lidar Range**: 0.12m to 3.5m

> [!NOTE]
> The launch file currently references some resources from `turtlebot3_cartographer`. Ensure that package is installed or update the paths in `cartographer.launch.py` to point to the local `config` and `rviz` directories.

## Saving the Map
Once you have mapped the area, use the `nav2_map_server` to save it:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```
