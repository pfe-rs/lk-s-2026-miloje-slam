cmake_minimum_required(VERSION 3.8)
project(lidar_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)
find_package(lidar_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Install the Python script to the lib directory
install(PROGRAMS
  scripts/lidar_scan_node.py
  scripts/vector_deducer_node.py
  scripts/slam_mapping_node.py
  scripts/path_planner_node.py
  scripts/motion_planner_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
