cmake_minimum_required(VERSION 3.8)
project(bt_bot_env_setup)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
set(COMMON_DEPENDENCIES ament_cmake rclcpp std_msgs rclcpp_action std_srvs behaviortree_cpp_v3 geometry_msgs sensor_msgs nav_msgs  visualization_msgs)

foreach(DEPENDENCY ${COMMON_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

install(
  DIRECTORY include launch 
  DESTINATION share/${PROJECT_NAME}
)



add_executable(risk_pub_node src/risk_pub.cpp)
ament_target_dependencies(risk_pub_node ament_cmake rclcpp std_msgs std_srvs behaviortree_cpp_v3 geometry_msgs sensor_msgs nav_msgs visualization_msgs rclcpp_action )


add_executable(bot_curr_battery_node src/bot_curr_battery.cpp)
ament_target_dependencies(bot_curr_battery_node ament_cmake rclcpp std_msgs std_srvs behaviortree_cpp_v3 geometry_msgs sensor_msgs nav_msgs visualization_msgs rclcpp_action )

add_executable(odom_pub_node src/odom_pub.cpp)
ament_target_dependencies(odom_pub_node ament_cmake rclcpp std_msgs std_srvs behaviortree_cpp_v3 geometry_msgs sensor_msgs nav_msgs visualization_msgs rclcpp_action )

add_executable(odom_kf_node src/odom_kf.cpp)
ament_target_dependencies(odom_kf_node ament_cmake rclcpp std_msgs std_srvs behaviortree_cpp_v3 geometry_msgs sensor_msgs nav_msgs visualization_msgs rclcpp_action )


add_executable(joy_to_cmd_node src/joy_to_cmd.cpp)
ament_target_dependencies(joy_to_cmd_node ament_cmake rclcpp std_msgs std_srvs behaviortree_cpp_v3 geometry_msgs sensor_msgs nav_msgs visualization_msgs rclcpp_action )


install(TARGETS
risk_pub_node
odom_kf_node
  bot_curr_battery_node
  joy_to_cmd_node
  odom_pub_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS

  risk_pub_node
  odom_kf_node
  odom_pub_node
  joy_to_cmd_node
  bot_curr_battery_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)


ament_package()
