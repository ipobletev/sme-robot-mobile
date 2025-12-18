# Robot Project

This project contains a ROS 1 workspace for a mobile robot. It includes packages for simulation, and navigation.

## Launch Commands

Before running any commands, ensure you have sourced the workspace:

```bash
source /opt/ros/noetic/setup.bash && source devel/setup.bash
```

### Mobile Robot Package

*   `roslaunch sme_robot_mobile_description display.launch`: Visualizes the robot model in Rviz. Useful for checking the URDF model.
*   `roslaunch sme_robot_mobile_description gazebo.launch`: Starts the full simulation environment in Gazebo. It spawns the robot, starts state publishers, and launches a behavior tree node.

### Mobile Robot Navigation Package

*   `roslaunch sme_robot_mobile_navigation navigation.launch`: Launches the complete navigation stack. This includes the Map Server, AMCL for localization, Move Base for path planning and control, and Rviz for visualization.