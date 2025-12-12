cmake_minimum_required(VERSION 3.8)
project(my_bot)

# Set standards
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)  # Jazzy prefers >=17
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------
# Find required packages
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(ros2_control REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(controller_manager REQUIRED)
find_package(robot_state_publisher REQUIRED)

# ------------------------------------------------------------------------------
# Install robot resources (URDF, XACRO, config, launch files)
# ------------------------------------------------------------------------------
install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Export dependencies
# ------------------------------------------------------------------------------
ament_export_dependencies(xacro)
ament_export_dependencies(ros2_control)
ament_export_dependencies(gazebo_ros2_control)
ament_export_dependencies(controller_manager)
ament_export_dependencies(robot_state_publisher)

ament_package()
