import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_description = get_package_share_directory('sme_robot_mobile_description')
    pkg_navigation = get_package_share_directory('sme_robot_mobile_navigation')
    
    # 1. Start Gazebo + Bridge + Robot State Publisher
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'gazebo.launch.py')
        )
    )

    # 2. Start Navigation Stack + RViz
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, 'launch', 'navigation.launch.py')
        )
    )

    return LaunchDescription([
        gazebo_launch,
        navigation_launch
    ])
