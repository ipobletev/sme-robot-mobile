# Robot Project

This project contains a ROS 2 Humble workspace for a mobile robot. It includes packages for simulation, and navigation.

## Launch Commands

Before running any commands, ensure you have sourced the workspace:

```bash
colcon build
source install/setup.bash
```

### Mobile Robot Description Package

*   **1.1** - `ros2 launch sme_robot_mobile_description rviz.launch.py`: Visualizes the robot model in RViz. Useful for checking the URDF model and joint states.
*   **1.2** - `ros2 launch sme_robot_mobile_description gazebo.launch.py`: Starts the full simulation environment in Gazebo. It spawns the robot and starts sensors.

### Mobile Robot Navigation Package

*   **2.1** - `ros2 launch sme_robot_mobile_navigation navigation.launch.py`: Launches the navigation stack (Map Server, AMCL, Nav2 controllers/planners).
    *   *For simulation:* Use Gazebo (**1.2**) command first, then run with `use_sim_time:=true`.
*   **2.2** - `ros2 launch sme_robot_mobile_navigation rviz.launch.py`: Launches RViz pre-configured for navigation visualization.
    *   *For simulation:* Run with `use_sim_time:=true`.