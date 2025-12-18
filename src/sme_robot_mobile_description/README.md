# sme_robot_mobile_description

This package contains the URDF description, visualization configurations, and launch files for the SME mobile robot.

## Overview

The `sme_robot_mobile_description` package defines a 4-wheeled differential drive mobile robot with integrated sensors (lidar and camera). It provides tools for visualizing the robot in RViz and simulating it in Gazebo with full navigation capabilities.

## Package Contents

### URDF Files (`urdf/`)

- **[sme_robot_mobile_robot.xacro](file:///home/isma/Desktop/sme-robot-mobile/src/sme_robot_mobile_description/urdf/sme_robot_mobile_robot.xacro)** - Main robot description file that assembles the complete robot
- **[sme_robot_mobile_base.xacro](file:///home/isma/Desktop/sme-robot-mobile/src/sme_robot_mobile_description/urdf/sme_robot_mobile_base.xacro)** - Mobile base definition with 4-wheel chassis
  - Footprint link for navigation
  - Chassis (0.5m × 0.3m × 0.1m)
  - Four wheels (0.2m diameter, 0.05m width)
  - Wheel separation: 0.35m
- **[sme_robot_mobile_sensors.xacro](file:///home/isma/Desktop/sme-robot-mobile/src/sme_robot_mobile_description/urdf/sme_robot_mobile_sensors.xacro)** - Sensor definitions
  - Lidar sensor (360° scanning, 10m range)
  - Camera sensor

### Launch Files (`launch/`)

#### [display.launch](file:///home/isma/Desktop/sme-robot-mobile/src/sme_robot_mobile_description/launch/display.launch)

Visualize the robot model in RViz without simulation.

**Usage:**
```bash
roslaunch sme_robot_mobile_description display.launch
```

**Arguments:**
- `model` - Path to robot URDF/xacro file (default: `sme_robot_mobile_robot.xacro`)
- `gui` - Show joint state publisher GUI (default: `true`)
- `rvizconfig` - RViz configuration file (default: `rviz/rviz.rviz`)

**Features:**
- Loads robot description from xacro
- Launches joint state publisher (with GUI option)
- Starts robot state publisher for TF transforms
- Opens RViz with custom configuration

---

#### [gazebo.launch](file:///home/isma/Desktop/sme-robot-mobile/src/sme_robot_mobile_description/launch/gazebo.launch)

Launch the robot in Gazebo simulation with full physics and sensor simulation.

**Usage:**
```bash
roslaunch sme_robot_mobile_description gazebo.launch
```

**Features:**
- Starts Gazebo with empty world
- Spawns robot from URDF with skid-steer drive controller
- Publishes odometry on `/odom` topic
- Accepts velocity commands on `/cmd_vel` topic
- Publishes laser scan data on `/scan` topic
- Launches robot state publisher for TF transforms
- Launches joint state publisher

**Gazebo Plugins:**
- **Skid Steer Drive Controller** - Controls 4-wheel differential drive
  - Update rate: 100 Hz
  - Wheel torque: 20 Nm
  - Broadcasts TF from `odom` to `robot_footprint`

### RViz Configuration (`rviz/`)

- **rviz.rviz** - Default RViz configuration for robot visualization

### Configuration (`config/`)

- **[nav_params.yaml](file:///home/isma/Desktop/sme-robot-mobile/src/sme_robot_mobile_description/config/nav_params.yaml)** - Navigation parameters for costmaps
  - Robot radius: 0.3m
  - Inflation radius: 0.55m
  - Obstacle range: 2.5m
  - Laser scan sensor configuration

### Scripts (`scripts/`)

- **[simple_behavior_tree.py](file:///home/isma/Desktop/sme-robot-mobile/src/sme_robot_mobile_description/scripts/simple_behavior_tree.py)** - Behavior tree implementation for robot control

## Robot Specifications

### Physical Dimensions
- **Chassis:** 0.5m (L) × 0.3m (W) × 0.1m (H)
- **Wheel Diameter:** 0.2m (20cm)
- **Wheel Width:** 0.05m (5cm)
- **Wheel Separation:** 0.35m
- **Robot Radius:** 0.3m (for navigation)
- **Mass:** 10kg (chassis) + 4kg (wheels)

### Sensors
- **Lidar:** 360° scanning, 10m max range, 360 samples, 10 Hz update rate
- **Camera:** Visual perception sensor

### TF Frames
- `robot_footprint` - Base footprint on ground plane
- `robot_chassis` - Main chassis link
- `lidar_link` - Lidar sensor frame
- `camera_link` - Camera sensor frame
- `front_left_wheel`, `front_right_wheel`, `rear_left_wheel`, `rear_right_wheel` - Wheel frames

## Usage Examples

### Visualize Robot in RViz

```bash
# With joint state publisher GUI
roslaunch sme_robot_mobile_description display.launch

# Without GUI
roslaunch sme_robot_mobile_description display.launch gui:=false
```

### Simulate Robot in Gazebo

```bash
# Launch Gazebo simulation
roslaunch sme_robot_mobile_description gazebo.launch
```

### Control Robot in Simulation

```bash
# Send velocity commands to move the robot
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.2"
```

### Monitor Sensor Data

```bash
# View laser scan data
rostopic echo /scan

# View odometry
rostopic echo /odom
```

### Load Robot Description Programmatically

```python
import rospy

# Robot description is loaded to parameter server
robot_description = rospy.get_param('/robot_description')
```

## Dependencies

- `urdf`
- `xacro`
- `robot_state_publisher`
- `joint_state_publisher`
- `joint_state_publisher_gui`
- `rviz`
- `gazebo_ros`
- `move_base`
- `amcl`
- `nav_msgs`
- `moveit_core`
- `moveit_ros_planning_interface`
- `py_trees`
- `py_trees_ros`

## Related Packages

- **sme_robot_mobile_navigation** - Navigation stack configuration with AMCL and move_base
- **sme_robot_mobile_simulation** - Comprehensive simulation scenarios (if available)
