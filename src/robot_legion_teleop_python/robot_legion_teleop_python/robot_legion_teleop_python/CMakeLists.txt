cmake_minimum_required(VERSION 3.5)
project(robot_legion_teleop_python)

# This package is ament_python-based but still uses CMake as the build system.

# Core build tools
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Runtime dependencies (we use them just to express the dependency graph)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install the Python package (the folder robot_legion_teleop_python/)
ament_python_install_package(${PROJECT_NAME})

# Install the teleop executable script into the standard ROS 2 location:
#   lib/robot_legion_teleop_python/legion_teleop_key
# so that:
#   ros2 run robot_legion_teleop_python legion_teleop_key
# works as expected (via setup.py entry_points).
install(PROGRAMS
  robot_legion_teleop_python/teleop_legion_key.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
