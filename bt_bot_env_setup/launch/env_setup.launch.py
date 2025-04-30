# #!/usr/bin/env python3

# import os
# from launch import LaunchDescription
# from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():

   
 

#     bot_curr_battery_node = Node(
#         package='bt_bot_env_setup',
#         executable='bot_curr_battery_node',
#         output='screen'
#     )

#     risk_pub_node = Node(
#         package='bt_bot_env_setup',
#         executable='risk_pub_node',
#         output='screen'
#     )




#     return LaunchDescription([
    
#         bot_curr_battery_node,
#         risk_pub_node
#     ])


#!/usr/bin/env python3

# import os
# from launch import LaunchDescription
# from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     # Set ROS domain ID if needed (optional)
#     # env = SetEnvironmentVariable('ROS_DOMAIN_ID', '0')

#     # Nodes from bt_bot_env_setup package
#     bot_curr_battery_node = Node(
#         package='bt_bot_env_setup',
#         executable='bot_curr_battery_node',
#         output='screen'
#     )

#     risk_pub_node = Node(
#         package='bt_bot_env_setup',
#         executable='risk_pub_node',
#         output='screen'
#     )
#     bt_topic_spiner_node = Node(
#         package='bt_bot_env_setup',
#         executable='odom_pub_node',
#         output='screen',
        
#     )

#     # # Node for bt_topic_spiner
#     # bt_topic_spiner_node = Node(
#     #     package='bt_topic_spiner',
#     #     executable='bt_topic_spiner_node',
#     #     output='screen',
        
#     # )

   

#     return LaunchDescription([
#         bot_curr_battery_node,
#         risk_pub_node,
#         bt_topic_spiner_node,
#     ])

#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the bt_topic_spiner package
    bt_topic_spiner_dir = get_package_share_directory('bt_topic_spiner')

    # Include the spinner launch file
    spinner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bt_topic_spiner_dir, 'launch', 'spinner_launch.launch.py')
        )
    )

    bot_curr_battery_node = Node(
        package='bt_bot_env_setup',
        executable='bot_curr_battery_node',
        output='screen'
    )

    risk_pub_node = Node(
        package='bt_bot_env_setup',
        executable='risk_pub_node',
        output='screen'
    )

    return LaunchDescription([
        spinner_launch,
        bot_curr_battery_node,
        risk_pub_node,
    ])