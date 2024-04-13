import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled

    package_name='bunker_mini'

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 'src/bunker_mini/config/nav2_config.rviz'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')
    
    slam_toolbox = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('slam_toolbox'),'launch','online_async_launch.py'
                )]), launch_arguments={'slam_params_file': './src/bunker_mini/config/mapper_params_online_async.yaml',
                                       'use_sim_time': use_sim_time,
                                       }.items()
    )
    
    navigation2 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('nav2_bringup'),'launch','navigation_launch.py'
                )]), launch_arguments={'params_file': './src/bunker_mini/config/nav2_params.yaml',
                                       }.items()
    )


    # Launch them all!
    return LaunchDescription([
        rsp,
        rviz2,
        slam_toolbox,
        navigation2,
    ])