# # spinner_launch.py

# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='bt_topic_spiner',  # Replace with your actual package name
#             executable='bt_topic_spiner_node',  # Replace with your actual executable name
#             name='bt_spiner_node',
#             output='screen',
#             parameters=['config/spinner_params.yaml']
#         ),
        

#     ])


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('bt_topic_spiner'),
        'config',
        'spinner_params.yaml'
        )
        
    node=Node(
        package = 'bt_topic_spiner',
        name = 'bt_topic_spiner_node',
        executable = 'bt_topic_spiner_node',
        parameters = [config]
    )

    ld.add_action(node)
    return ld