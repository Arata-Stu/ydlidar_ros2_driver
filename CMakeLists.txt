cmake_minimum_required(VERSION 3.8)
project(ydlidar_ros2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ydlidar_sdk REQUIRED)
find_package(rclcpp_components REQUIRED)

ament_auto_find_build_dependencies()

# Include directories
include_directories(include)

# 1. Composable Node (Shared Library)
ament_auto_add_library(ydlidar_ros2_driver_component SHARED
  src/ydlidar_ros2_driver_component.cpp
)

target_link_libraries(ydlidar_ros2_driver_component ydlidar_sdk)

rclcpp_components_register_nodes(ydlidar_ros2_driver_component
  "ydlidar_ros2_driver::YdlidarRos2DriverComponent"
)

# 2. Standalone Node Executable
ament_auto_add_executable(ydlidar_ros2_driver_node
  src/ydlidar_ros2_driver_node.cpp
)

target_link_libraries(ydlidar_ros2_driver_node ydlidar_ros2_driver_component)

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
