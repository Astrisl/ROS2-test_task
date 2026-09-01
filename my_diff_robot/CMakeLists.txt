cmake_minimum_required(VERSION 3.8)
project(my_diff_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY
    urdf
    launch
    config
    rviz
    worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(
  PROGRAMS
    scripts/simple_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(safe_node src/safe_node.cpp)
ament_target_dependencies(safe_node rclcpp sensor_msgs geometry_msgs)

install(TARGETS
  safe_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
