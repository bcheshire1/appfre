# File: launch_nav2.launch.py
# Author: Ben Cheshire
# Date: 21/03/2024
# Source: Josh Ewans (Articulated Robotics)
# 
# Description:
# This launch file launches all relevant processes for the Bunker Mini robot in the appfre project
# to the the navigation2 stack in Gazebo Classic simulation

import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


# This function assigns each launch entity to a variable with all the relevant parameters and arguments
def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled

    package_name='bunker_mini'

    # Robot State publisher: publishes the robot's state (position, velocity etc.)
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'bunker_mini'],
                        output='screen')

    # Start RViz2 with the specific setup found in nav2_config.rviz
    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 'src/bunker_mini/config/nav2_config.rviz'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')
    
    # Launches the SLAM toolbox for mapping and localization
    slam_toolbox = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('slam_toolbox'),'launch','online_async_launch.py'
                )]), launch_arguments={'slam_params_file': './src/bunker_mini/config/mapper_params_online_async.yaml',
                                       'use_sim_time': use_sim_time,
                                       }.items()
    )
    
    # Start navigation2
    navigation2 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('nav2_bringup'),'launch','navigation_launch.py'
                )]), launch_arguments={'params_file': './src/bunker_mini/config/nav2_params.yaml',
                                       }.items()
    )


    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        rviz2,
        slam_toolbox,
        navigation2,
    ])