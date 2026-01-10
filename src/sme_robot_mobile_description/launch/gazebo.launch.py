import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('sme_robot_mobile_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Check if we should use a custom world
    # For now we use empty world but you can change this
    world_file = os.path.join(pkg_share, 'worlds', 'sme_robot.world')
    
    # If world file doesn't exist, use empty world
    # We will just pass the world argument if needed
    
    # Process the URDF file
    xacro_file = os.path.join(pkg_share, 'urdf', 'sme_robot_mobile_robot.xacro')
    
    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            ),
            # launch_arguments={'world': world_file}.items() # Add this if you want a custom world
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': ParameterValue(robot_description, value_type=str), 'use_sim_time': True}]
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'sme_robot_mobile', '-topic', 'robot_description'],
            output='screen'
        ),
    ])
