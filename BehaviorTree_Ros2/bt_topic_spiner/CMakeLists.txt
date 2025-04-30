cmake_minimum_required(VERSION 3.8)
project(bt_topic_spiner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

# ✅ Add all message dependencies you are using or may use
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# Declare your executable
add_executable(bt_topic_spiner_node src/bt_topic_spiner.cpp)

# Link dependencies to the executable
ament_target_dependencies(bt_topic_spiner_node
  rclcpp
  std_msgs
  rcl_interfaces
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  builtin_interfaces
  diagnostic_msgs
)

# Include your headers
include_directories(
  include
)

# Install the executable
install(TARGETS
  bt_topic_spiner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers and any launch/config files
install(DIRECTORY
  include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
