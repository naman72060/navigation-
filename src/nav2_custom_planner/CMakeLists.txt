cmake_minimum_required(VERSION 3.8)
project(nav2_custom_planner)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)

# Include headers
include_directories(
  include
)

# Plugin source
add_library(nav2_custom_planner_plugin SHARED
  src/custom_planner.cpp
)

# Link dependencies to plugin target
ament_target_dependencies(nav2_custom_planner_plugin
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_util
  nav2_costmap_2d
  tf2
  tf2_ros
  geometry_msgs
  nav_msgs
  pluginlib
)

# Plugin XML export (ensure filename matches your actual file)
pluginlib_export_plugin_description_file(nav2_core gobal_planner_plugin.xml)

# Install compiled library
install(
  TARGETS nav2_custom_planner_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install plugin.xml file
install(
  FILES gobal_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Linting and testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
