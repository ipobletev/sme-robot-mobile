# Robot Project

This project contains a ROS 2 Humble workspace for a mobile robot. It includes packages for simulation, SLAM, and navigation.

## Launch Commands

Before running any commands, ensure you have sourced the workspace:

```bash
colcon build && source install/setup.bash
```

### 1. Mobile Robot Description Package
Visualizes the robot model in RViz
*   `ros2 launch sme_robot_mobile_description rviz.launch.py`

Starts the simulation environment in Gazebo.
*   `ros2 launch sme_robot_mobile_description gazebo.launch.py`

### 2. Mobile Robot Navigation Package

Runs the SLAM system to build a map.
*   `ros2 launch sme_robot_mobile_cartographer cartographer.launch.py`

#### 3.1 Navigation
Use only one of the following commands:

Use this while building a map with Cartographer.
*   `ros2 launch sme_robot_mobile_navigation navigation_slam.launch.py`

Use this with a pre-saved map (static map) and AMCL.
*   `ros2 launch sme_robot_mobile_navigation navigation_static.launch.py map:=test_map.yaml`

#### 4.2 Navigation visualize in RViz
*   `ros2 launch sme_robot_mobile_navigation nav_rviz.launch.py`

### 5. Test commands
#### 5.1 Navigation static map
    ros2 launch sme_robot_mobile_description gazebo.launch.py

    ros2 launch sme_robot_mobile_navigation navigation_static.launch.py map:=test_map.yaml use_sim_time:=true
    
    ros2 launch sme_robot_mobile_navigation nav_rviz.launch.py use_sim_time:=true`

#### 5.2 Navigation SLAM
    ros2 launch sme_robot_mobile_description gazebo.launch.py
    
    ros2 launch sme_robot_mobile_cartographer cartographer.launch.py  use_sim_time:=true
    
    ros2 launch sme_robot_mobile_navigation navigation_slam.launch.py use_sim_time:=true
    
    ros2 launch sme_robot_mobile_navigation nav_rviz.launch.py use_sim_time:=true