cmake_minimum_required(VERSION 3.8)
project(patrolling_bot)

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
  DIRECTORY include launch bt_tree
  DESTINATION share/${PROJECT_NAME}
)

add_library(bt_nodes src/bt_nodes.cpp)
target_include_directories(bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(bt_nodes ament_cmake rclcpp std_msgs std_srvs behaviortree_cpp_v3 geometry_msgs sensor_msgs nav_msgs visualization_msgs rclcpp_action )

add_executable(bt_control_node src/bt_control.cpp)
target_link_libraries(bt_control_node bt_nodes)
ament_target_dependencies(bt_control_node ament_cmake rclcpp std_msgs std_srvs behaviortree_cpp_v3 geometry_msgs sensor_msgs nav_msgs visualization_msgs rclcpp_action )





install(TARGETS
  bt_nodes
  bt_control_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  bt_nodes
  bt_control_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)


  if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(play_motion2_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)

  # tuck_arm_test
  ament_add_gtest_executable(tuck_arm_test
    test/test_node.cpp
  )

  ament_target_dependencies(tuck_arm_test play_motion2_msgs launch_testing_ament_cmake ament_cmake rclcpp std_msgs std_srvs behaviortree_cpp_v3 geometry_msgs sensor_msgs nav_msgs visualization_msgs rclcpp_action )


  install(
    TARGETS tuck_arm_test
    DESTINATION share/${PROJECT_NAME}/test
  )

endif()

ament_package()
