# sme_robot_mobile_navigation

This package provides navigation capabilities for the SME mobile robot including localization, path planning, and obstacle avoidance in Gazebo simulation.

## Overview

The `sme_robot_mobile_navigation` package configures the ROS navigation stack for autonomous mobile robot navigation. It integrates Gazebo simulation with AMCL for localization, Move Base for path planning, and TEB local planner for dynamic trajectory generation.

## Package Contents

### Launch Files (`launch/`)

#### [navigation.launch](file:///home/isma/Desktop/sme-robot-mobile/src/sme_robot_mobile_navigation/launch/navigation.launch)

Complete navigation stack with Gazebo simulation, localization, and path planning.

**Usage:**
```bash
roslaunch sme_robot_mobile_navigation navigation.launch
```

**Arguments:**
- `map_file` - Path to map YAML file (default: maps/map.yaml)
- `open_rviz` - Launch RViz for visualization (default: true)
- `paused` - Start Gazebo paused (default: false)
- `gui` - Launch Gazebo GUI (default: true)
- `move_forward_only` - Restrict robot to forward motion (default: false)

**Components Launched:**
1. **Gazebo Simulation** - Empty world with robot spawned
2. **Robot Description** - URDF loaded and state publishers
3. **Navigation Stack:**
   - **map_server** - Serves the pre-built map
   - **amcl** - Adaptive Monte Carlo Localization for robot localization
   - **move_base** - Global and local path planning with TEB planner
4. **Visualization** - RViz (optional)

#### [slam_gmapping.launch](file:///home/isma/Desktop/sme-robot-mobile/src/sme_robot_mobile_navigation/launch/slam_gmapping.launch)

SLAM launch file for autonomous map building using gmapping.

**Usage:**
```bash
roslaunch sme_robot_mobile_navigation slam_gmapping.launch
```

**Arguments:**
- `open_rviz` - Launch RViz for visualization (default: true)
- `paused` - Start Gazebo paused (default: false)
- `gui` - Launch Gazebo GUI (default: true)

**Components Launched:**
1. **Gazebo Simulation** - Empty world with robot spawned
2. **Robot Description** - URDF loaded and state publishers
3. **SLAM Stack:**
   - **gmapping** - SLAM algorithm for map building
4. **Visualization** - RViz with SLAM-specific configuration (optional)

#### [slam_navigation.launch](file:///home/isma/Desktop/sme-robot-mobile/src/sme_robot_mobile_navigation/launch/slam_navigation.launch)

**SLAM + Navigation combined** - Build maps while navigating autonomously.

**Usage:**
```bash
roslaunch sme_robot_mobile_navigation slam_navigation.launch
```

**Arguments:**
- `open_rviz` - Launch RViz for visualization (default: true)
- `paused` - Start Gazebo paused (default: false)
- `gui` - Launch Gazebo GUI (default: true)
- `move_forward_only` - Restrict robot to forward motion (default: false)

**Components Launched:**
1. **Gazebo Simulation** - Empty world with robot spawned
2. **Robot Description** - URDF loaded and state publishers
3. **SLAM Stack:**
   - **gmapping** - SLAM algorithm for real-time map building
4. **Navigation Stack:**
   - **move_base** - Autonomous navigation with dynamic map
5. **Visualization** - RViz with navigation configuration (optional)

> [!NOTE]
> This mode allows the robot to navigate autonomously while simultaneously building a map. You can send navigation goals via RViz, and the robot will plan paths on the map being built in real-time.

### Configuration Files (`config/`)

#### Costmap Configuration

- **[costmap_common_params.yaml](file:///home/isma/Desktop/sme-robot-mobile/src/sme_robot_mobile_navigation/config/costmap_common_params.yaml)** - Common costmap parameters (footprint, inflation, obstacle layers)
- **[global_costmap_params.yaml](file:///home/isma/Desktop/sme-robot-mobile/src/sme_robot_mobile_navigation/config/global_costmap_params.yaml)** - Global costmap configuration
- **[local_costmap_params.yaml](file:///home/isma/Desktop/sme-robot-mobile/src/sme_robot_mobile_navigation/config/local_costmap_params.yaml)** - Local costmap configuration

#### Planner Configuration

- **[move_base_params.yaml](file:///home/isma/Desktop/sme-robot-mobile/src/sme_robot_mobile_navigation/config/move_base_params.yaml)** - Move Base parameters
- **[global_planner_params.yaml](file:///home/isma/Desktop/sme-robot-mobile/src/sme_robot_mobile_navigation/config/global_planner_params.yaml)** - Global planner (A*) parameters
- **[teb_local_planner_params.yaml](file:///home/isma/Desktop/sme-robot-mobile/src/sme_robot_mobile_navigation/config/teb_local_planner_params.yaml)** - TEB local planner parameters
- **[dwa_local_planner_params.yaml](file:///home/isma/Desktop/sme-robot-mobile/src/sme_robot_mobile_navigation/config/dwa_local_planner_params.yaml)** - DWA local planner parameters (alternative, currently not active)

### Maps (`maps/`)

Pre-built maps for navigation testing in Gazebo environments.

### RViz Configuration (`rviz/`)

- **navigation.rviz** - RViz configuration for navigation visualization

## Usage Examples

### Basic Navigation

```bash
# Launch navigation stack with Gazebo and default map
roslaunch sme_robot_mobile_navigation navigation.launch

# Use custom map
roslaunch sme_robot_mobile_navigation navigation.launch map_file:=/path/to/map.yaml

# Disable RViz
roslaunch sme_robot_mobile_navigation navigation.launch open_rviz:=false

# Start Gazebo without GUI (headless)
roslaunch sme_robot_mobile_navigation navigation.launch gui:=false

# Restrict to forward-only motion
roslaunch sme_robot_mobile_navigation navigation.launch move_forward_only:=true
```

### Send Navigation Goals

Using RViz:
1. Launch navigation stack
2. Click "2D Pose Estimate" to set initial pose
3. Click "2D Nav Goal" to send navigation goal

Using command line:
```bash
# Send goal via rostopic
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "header:
  frame_id: 'map'
pose:
  position: {x: 2.0, y: 1.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```

### Monitor Navigation Status

```bash
# Check move_base status
rostopic echo /move_base/status

# View current goal
rostopic echo /move_base/current_goal

# Monitor robot pose
rostopic echo /amcl_pose

# View local plan (TEB trajectory)
rostopic echo /move_base/TebLocalPlannerROS/local_plan
```

## Key Topics

### Subscribed Topics
- `/scan` - Laser scan data for obstacle detection
- `/odom` - Odometry for localization
- `/map` - Static map from map_server

### Published Topics
- `/cmd_vel` - Velocity commands to robot
- `/move_base/global_costmap/costmap` - Global costmap
- `/move_base/local_costmap/costmap` - Local costmap
- `/amcl_pose` - Estimated robot pose
- `/move_base/TebLocalPlannerROS/local_plan` - TEB local trajectory
- `/move_base/TebLocalPlannerROS/teb_markers` - TEB visualization markers

## Configuration Notes

### AMCL Parameters

The package uses AMCL with the following key settings:
- Particle filter: 500-3000 particles
- Laser model: likelihood_field
- Odometry model: differential drive (diff)
- Transform tolerance: 0.5s
- Update thresholds: 0.2m translation, 0.2 rad rotation

### TEB Local Planner

The TEB (Timed Elastic Band) planner is currently active and configured for:
- Dynamic obstacle avoidance
- Time-optimal trajectory generation
- Smooth velocity profiles
- Velocity constraints based on robot capabilities
- Support for differential drive kinematics

> [!NOTE]
> The package includes DWA planner configuration files, but TEB is currently set as the active local planner in `navigation.launch`. To switch back to DWA, uncomment the DWA lines and comment out the TEB lines in the launch file.

### Global Planner

Uses A* algorithm (navfn/NavfnROS) for global path planning with configurable parameters.

## Dependencies

- `gazebo_ros` - Gazebo simulation
- `map_server` - Map serving
- `amcl` - Localization
- `move_base` - Navigation framework
- `gmapping` - SLAM for map building
- `teb_local_planner` - Local trajectory planning
- `dwa_local_planner` - Alternative local planner
- `robot_state_publisher` - TF publishing
- `joint_state_publisher` - Joint states
- `rviz` - Visualization
- `tf` - Transform library

## Related Packages

- **sme_robot_mobile_description** - Robot URDF and visualization

## SLAM vs Navigation

This package provides both **SLAM for map building** and **navigation with pre-existing maps**.

### When to Use SLAM

Use SLAM (Simultaneous Localization and Mapping) when:
- You don't have a map of the environment yet
- You need to update or rebuild an existing map
- The environment has changed significantly

### When to Use Navigation

Use navigation mode when:
- You already have a map of the environment
- You want the robot to autonomously navigate to goals
- You need path planning and obstacle avoidance

### Workflow: SLAM → Navigation

**Option 1: Manual Exploration (Teleop)**

1. **Build a map using SLAM:**
   ```bash
   roslaunch sme_robot_mobile_navigation slam_gmapping.launch
   ```

2. **Drive the robot around to explore** (in new terminal):
   ```bash
   rosrun teleop_twist_keyboard teleop_twist_keyboard.py
   ```

3. **Save the map:**
   ```bash
   rosrun map_server map_saver -f ~/Desktop/sme-robot-mobile/src/sme_robot_mobile_navigation/maps/my_new_map
   ```

4. **Use the map for navigation:**
   ```bash
   roslaunch sme_robot_mobile_navigation navigation.launch map_file:=$(rospack find sme_robot_mobile_navigation)/maps/my_new_map.yaml
   ```

**Option 2: Autonomous Exploration (SLAM + Navigation)**

1. **Build map while navigating autonomously:**
   ```bash
   roslaunch sme_robot_mobile_navigation slam_navigation.launch
   ```

2. **Send navigation goals via RViz** - Robot will navigate and build map simultaneously

3. **Save the map when done exploring:**
   ```bash
   rosrun map_server map_saver -f ~/Desktop/sme-robot-mobile/src/sme_robot_mobile_navigation/maps/my_new_map
   ```

4. **Use the saved map for navigation:**
   ```bash
   roslaunch sme_robot_mobile_navigation navigation.launch map_file:=$(rospack find sme_robot_mobile_navigation)/maps/my_new_map.yaml
   ```

### SLAM Configuration

The gmapping SLAM algorithm is configured in [gmapping_params.yaml](file:///home/isma/Desktop/sme-robot-mobile/src/sme_robot_mobile_navigation/config/gmapping_params.yaml) with parameters optimized for:
- Differential drive robot kinematics
- 360-degree lidar sensor (10m range, 360 samples)
- Map resolution: 0.05m (5cm per cell)
- Particle filter: 30 particles