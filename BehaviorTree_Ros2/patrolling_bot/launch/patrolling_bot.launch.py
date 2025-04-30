#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    patrolling_bot_dir = get_package_share_directory('patrolling_bot')
    bt_bot_env_setup_dir = get_package_share_directory('bt_bot_env_setup')

    # Define delay durations (in seconds)
    GAZEBO_DELAY = 0.0     # Start immediately
    RVIZ_DELAY = 5.0       # Start 5 seconds after Gazebo
    BT_DELAY = 10.0        # Start 10 seconds after Gazebo
    ENV_SETUP_DELAY = 15.0 # Start 15 seconds after Gazebo

    # Include the spinner launch files with delays
    gazeebo_launch = TimerAction(
        period=GAZEBO_DELAY,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(patrolling_bot_dir, 'launch', 'gazeebo.launch.py')
                )
            )
        ]
    )

    rviz_nav2_turtleBot_launch = TimerAction(
        period=RVIZ_DELAY,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(patrolling_bot_dir, 'launch', 'rviz_nav2_turtleBot.launch.py')
                )
            )
        ]
    )

    bt_launch = TimerAction(
        period=BT_DELAY,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(patrolling_bot_dir, 'launch', 'bt.launch.py')
                )
            )
        ]
    )

    env_setup_launch = TimerAction(
        period=ENV_SETUP_DELAY,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bt_bot_env_setup_dir, 'launch', 'env_setup.launch.py')
                )
            )
        ]
    )

    return LaunchDescription([
        # gazeebo_launch,
        rviz_nav2_turtleBot_launch,
        bt_launch,
        env_setup_launch
    ])