import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def resolve_map_path(context, *args, **kwargs):
    pkg_mob_nav = get_package_share_directory('sme_robot_mobile_navigation')
    map_arg = LaunchConfiguration('map').perform(context)
    
    if not os.path.isabs(map_arg):
        # Look in the package's map directory
        map_path = os.path.join(pkg_mob_nav, 'map', map_arg)
        # If not found there, maybe it's relative to CWD? 
        # But usually in ROS 2 we prefer the package dir for simplified names
        if not os.path.exists(map_path):
             map_path = os.path.abspath(map_arg)
    else:
        map_path = map_arg
    
    context.launch_configurations['map_path'] = map_path
    return []

def generate_launch_description():
    pkg_mob_nav = get_package_share_directory('sme_robot_mobile_navigation')
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    nav2_params_file_path = os.path.join(pkg_mob_nav, 'param', 'nav2_params_static.yaml')
    
    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value='map.yaml',
        description='Full path to map yaml file or name of map file in the map directory'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file_path,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    # Use IncludeLaunchDescription for bringup_launch.py
    # We use a special trick to pass the resolved map path
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
        launch_arguments={
            'map': LaunchConfiguration('map_path'),
            'use_sim_time': use_sim_time,
            'params_file': LaunchConfiguration('params_file')}.items()
    )

    return LaunchDescription([
        declare_map_cmd,
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        OpaqueFunction(function=resolve_map_path),
        nav2_bringup_launch
    ])