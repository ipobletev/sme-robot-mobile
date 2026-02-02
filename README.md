# Robot Project

This project contains a ROS 2 Humble workspace for a mobile robot. It includes packages for simulation, and navigation.

## Launch Commands

Before running any commands, ensure you have sourced the workspace:

```bash
colcon build
source install/setup.bash
```

### Mobile Robot Description Package

*   `ros2 launch sme_robot_mobile_description display.launch.py`: Visualizes the robot model in Rviz. Useful for checking the URDF model.
*   `ros2 launch sme_robot_mobile_description gazebo.launch.py`: Starts the full simulation environment in Gazebo. It spawns the robot, starts state publishers, and launches a behavior tree node.

### Mobile Robot Navigation Package

*   `ros2 launch sme_robot_mobile_navigation navigation.launch.py`: Launches the complete navigation stack. This includes the Map Server, AMCL for localization, Move Base for path planning and control, and Rviz for visualization. With simulation first launch gazebo: `ros2 launch sme_robot_mobile_description gazebo.launch.py`