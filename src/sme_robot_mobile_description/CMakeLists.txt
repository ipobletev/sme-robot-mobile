cmake_minimum_required(VERSION 3.0.2)
project(sme_robot_mobile_description)

# Find the catkin build system and other required ROS packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  urdf
  xacro
  gazebo_ros
  nav_msgs
)


# Define the catkin package parameters
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sme_robot_mobile_description
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

# Install directories (launch files, config, urdf, etc.)
install(DIRECTORY launch config urdf meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
