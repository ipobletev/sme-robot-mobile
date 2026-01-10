import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('sme_robot_mobile_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Full path to the ROS2 parameters file to use for all launched nodes'
        ),
        
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_share, 'maps', 'my_map.yaml'), # Replace with your default map if any
            description='Full path to map yaml file to load'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_sim_time': 'true',
                'params_file': LaunchConfiguration('params_file')
            }.items()
        )
    ])
