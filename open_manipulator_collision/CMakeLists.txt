################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(open_manipulator_playground)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

################################################################################
# Build
################################################################################
add_executable(omy_3m_hello_moveit src/omy_3m_hello_moveit.cpp)
target_include_directories(omy_3m_hello_moveit
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(omy_3m_hello_moveit
  rclcpp
  moveit_ros_planning_interface
)

add_executable(omy_f3m_hello_moveit src/omy_f3m_hello_moveit.cpp)
target_include_directories(omy_f3m_hello_moveit
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(omy_f3m_hello_moveit
  rclcpp
  moveit_ros_planning_interface
)

add_executable(open_manipulator_x_hello_moveit src/open_manipulator_x_hello_moveit.cpp)
target_include_directories(open_manipulator_x_hello_moveit
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(open_manipulator_x_hello_moveit
  rclcpp
  moveit_ros_planning_interface
)

################################################################################
# Install
################################################################################
# C++ nodes
install(TARGETS
  omy_3m_hello_moveit
  omy_f3m_hello_moveit
  open_manipulator_x_hello_moveit
  DESTINATION lib/${PROJECT_NAME}
)

# Headers (if used)
install(DIRECTORY include/
  DESTINATION include/
)

# Launch files (if exist)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

# Python scripts 
install(PROGRAMS
  src/drawing_shape_omx.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  src/__init__.py
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_export_dependencies(
  rclcpp
  moveit_ros_planning_interface
)

ament_export_include_directories(
  include
)

ament_package()
