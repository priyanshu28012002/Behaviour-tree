cmake_minimum_required(VERSION 3.8)
project(bt_custom_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(bt_custom_node REQUIRED)
set(COMMON_DEPENDENCIES ament_cmake rclcpp std_msgs std_srvs behaviortree_cpp_v3 geometry_msgs sensor_msgs nav_msgs visualization_msgs)

foreach(DEPENDENCY ${COMMON_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

file(GLOB_RECURSE CPP_FILES src/*.cpp)

foreach(CPP_FILE ${CPP_FILES})
  get_filename_component(EXE_NAME ${CPP_FILE} NAME_WE)

  add_executable(${EXE_NAME}_node ${CPP_FILE})
  ament_target_dependencies(${EXE_NAME}_node ${COMMON_DEPENDENCIES})
  install(TARGETS ${EXE_NAME}_node DESTINATION lib/${PROJECT_NAME})

endforeach()


install(
  DIRECTORY include launch bt_tree
  DESTINATION share/${PROJECT_NAME}
)

# Install map files
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
