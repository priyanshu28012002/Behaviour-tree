#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    turtlebot3_model_env = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_navigation2'),
                         'launch', 'navigation2.launch.py')
        ),
        launch_arguments={'map': '/home/octobotics/ros2_ws/src/Behavior\ Tree/patrolling_bot/map/my_map2.yaml'}.items()
    )

    return LaunchDescription([
        turtlebot3_model_env,
        navigation_launch
    ])