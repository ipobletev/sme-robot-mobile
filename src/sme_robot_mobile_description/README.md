# sme_robot_mobile_description

This package contains the URDF description, visualization configurations, and launch files for the SME mobile robot, based on the **TurtleBot3 Waffle** platform and updated for ROS 2.

## Overview

The `sme_robot_mobile_description` package defines a mobile robot based on TurtleBot3 architecture with integrated sensors (lidar and camera). It provides tools for visualizing the robot in RViz 2 and simulating it in Gazebo.

## Package Contents

### URDF Files (`urdf/`)

This directory contains the robot model definitions, organized into the following key files:

- **[turtlebot3_waffle.urdf](file:///home/isma/Desktop/sme-robot-mobile/src/sme_robot_mobile_description/urdf/turtlebot3/turtlebot3_waffle.urdf)**: Main robot description file. It defines the physical structure (links) and connections (joints) of the SME mobile robot.
- **[turtlebot3_waffle.gazebo.xacro](file:///home/isma/Desktop/sme-robot-mobile/src/sme_robot_mobile_description/urdf/turtlebot3/turtlebot3_waffle.gazebo.xacro)**: Contains Gazebo-specific extensions and plugins. It configures:
    - Differential drive controller.
    - Lidar and IMU sensor simulation.
    - Joint state publishing for Gazebo.
- **[common_properties.urdf](file:///home/isma/Desktop/sme-robot-mobile/src/sme_robot_mobile_description/urdf/turtlebot3/common_properties.urdf)**: Defines common material colors and physical properties used across the model.

The robot model is based on the **TurtleBot3 Waffle** structure but is specifically tailored for this project with configurations compatible with **ROS 2 Jazzy** and **Ignition/Gazebo Sim**.
### Launch Files (`launch/`)

#### [display.launch.py](file:///home/isma/Desktop/sme-robot-mobile/src/sme_robot_mobile_description/launch/display.launch.py)

Visualize the robot model in RViz 2.

**Usage:**
```bash
ros2 launch sme_robot_mobile_description display.launch.py
```

---

#### [gazebo.launch.py](file:///home/isma/Desktop/sme-robot-mobile/src/sme_robot_mobile_description/launch/gazebo.launch.py)

Launch the robot in Gazebo simulation with full physics and sensor simulation.

**Usage:**
```bash
ros2 launch sme_robot_mobile_description gazebo.launch.py
