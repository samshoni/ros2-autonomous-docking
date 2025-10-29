#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_automated_docking = get_package_share_directory('automated_docking_project')
    
    # World file path
    world_file_name = 'docking_world.world'
    world_path = os.path.join(pkg_automated_docking, 'worlds', world_file_name)
    
    return LaunchDescription([
        # Launch Gazebo with custom world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_path, 'verbose': 'true'}.items()
        ),
        
        # Spawn TurtleBot3
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_turtlebot3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': '0.0',
                'y_pose': '0.0',
                'z_pose': '0.01',
                'yaw': '0.0'
            }.items()
        ),
        
        # Battery Monitor Node
        Node(
            package='automated_docking_project',
            executable='battery_monitor',
            name='battery_monitor',
            output='screen'
        ),
        
        # Docking Controller Node
        Node(
            package='automated_docking_project',
            executable='docking_controller',
            name='docking_controller',
            output='screen'
        ),
        
        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])
