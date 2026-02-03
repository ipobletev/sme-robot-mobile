import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Packages
    pkg_mob_nav = get_package_share_directory('sme_robot_mobile_navigation')
    
    # Files
    rviz_config_path = os.path.join(pkg_mob_nav, 'rviz', 'navigation.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])
