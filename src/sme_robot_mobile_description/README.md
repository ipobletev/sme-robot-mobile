# sme_robot_mobile_description

This package contains the URDF description, visualization configurations, and launch files for the SME mobile robot, updated for ROS 2.

## Overview

The `sme_robot_mobile_description` package defines a 4-wheeled differential drive mobile robot with integrated sensors (lidar and camera). It provides tools for visualizing the robot in RViz 2 and simulating it in Gazebo with full navigation capabilities using Nav2.

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

#### [display.launch.py](file:///home/isma/Desktop/sme-robot-mobile/src/sme_robot_mobile_description/launch/display.launch.py)

Visualize the robot model in RViz 2 without simulation.

**Usage:**
```bash
ros2 launch sme_robot_mobile_description display.launch.py
```

**Arguments:**
- `model` - Path to robot URDF/xacro file (default: `sme_robot_mobile_robot.xacro`)
- `gui` - Show joint state publisher GUI (default: `true`)
- `rvizconfig` - RViz configuration file (default: `rviz/rviz.rviz`)

**Features:**
- Loads robot description from xacro
- Launches `joint_state_publisher` (with GUI option)
- Starts `robot_state_publisher` for TF transforms
- Opens RViz 2 with custom configuration

---

#### [gazebo.launch.py](file:///home/isma/Desktop/sme-robot-mobile/src/sme_robot_mobile_description/launch/gazebo.launch.py)

Launch the robot in Gazebo simulation with full physics and sensor simulation.

**Usage:**
```bash
ros2 launch sme_robot_mobile_description gazebo.launch.py
```

**Features:**
- Starts Gazebo with empty world
- Spawns robot from URDF with differential drive controller
- Publishes odometry on `/odom` topic
- Accepts velocity commands on `/cmd_vel` topic
- Publishes laser scan data on `/scan` topic
- Launches `robot_state_publisher` for TF transforms
- Launches `joint_state_publisher`

**Gazebo Plugins:**
- **Differential Drive Controller** - Controls 4-wheel differential drive
  - Update rate: 100 Hz
  - Wheel torque: 20 Nm
  - Broadcasts TF from `odom` to `robot_footprint`

### RViz Configuration (`rviz/`)

- **rviz.rviz** - Default RViz configuration for robot visualization

### Configuration (`config/`)

- **[nav2_params.yaml](file:///home/isma/Desktop/sme-robot-mobile/src/sme_robot_mobile_description/config/nav2_params.yaml)** - Navigation parameters for Nav2
  - Robot radius: 0.3m
  - Inflation radius: 0.55m
  - Obstacle range: 2.5m
  - Laser scan sensor configuration

### Scripts (`scripts/`)

- **[simple_behavior_tree.py](file:///home/isma/Desktop/sme-robot-mobile/src/sme_robot_mobile_description/scripts/simple_behavior_tree.py)** - Behavior tree implementation for robot control (requires ROS 2 compatible libraries)

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

### Visualize Robot in RViz 2

```bash
# With joint state publisher GUI
ros2 launch sme_robot_mobile_description display.launch.py

# Without GUI
ros2 launch sme_robot_mobile_description display.launch.py gui:=false
```

### Simulate Robot in Gazebo

```bash
# Launch Gazebo simulation
ros2 launch sme_robot_mobile_description gazebo.launch.py
```

### Control Robot in Simulation

```bash
# Send velocity commands to move the robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
```

### Monitor Sensor Data

```bash
# View laser scan data
ros2 topic echo /scan

# View odometry
ros2 topic echo /odom
```

### Load Robot Description Programmatically (Python)

```python
import rclpy
from rclpy.node import Node

# In ROS 2, robot_description is usually a topic or a parameter of robot_state_publisher
# You can get it from the /robot_description topic
```

## Dependencies (ROS 2)

- `urdf`
- `xacro`
- `robot_state_publisher`
- `joint_state_publisher`
- `joint_state_publisher_gui`
- `rviz2`
- `gazebo_ros_pkgs`
- `nav2_common`
- `nav2_bringup`
- `rclpy`

## Related Packages

- **sme_robot_mobile_navigation** - Navigation stack configuration with Nav2
- **sme_robot_mobile_simulation** - Comprehensive simulation scenarios
