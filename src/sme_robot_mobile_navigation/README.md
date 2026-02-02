# SME Robot Mobile Navigation

Autonomous navigation package for the SME mobile robot based on **ROS 2 Humble**. It uses the **Navigation 2 (Nav2)** stack for localization, planning, and behavior execution.

## Launch Files

### 1. `navigation.launch.py`
**Full navigation with map and RViz**

This file starts the Nav2 stack and the RViz interface. For correct operation in a simulated environment, follow these steps:

1. **Start the Gazebo simulation:**
   ```bash
   ros2 launch sme_robot_mobile_description gazebo.launch.py
   ```

   ros2 launch sme_robot_mobile_navigation navigation.launch.py use_sim_time:=true
   ```

3. **Start the navigation stack with other map:**
   ```bash
   ros2 launch sme_robot_mobile_navigation navigation.launch.py use_sim_time:=true map:=sme-robot-mobile/src/sme_robot_mobile_navigation/map/test_map.yaml
   