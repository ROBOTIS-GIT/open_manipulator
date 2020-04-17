################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(open_manipulator_x_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(open_manipulator_msgs REQUIRED)
find_package(open_manipulator_x_libs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robotis_manipulator REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
)

set(dependencies
  "geometry_msgs" 
  "open_manipulator_msgs"
  "open_manipulator_x_libs" 
  "rclcpp" 
  "robotis_manipulator" 
  "sensor_msgs" 
  "std_msgs" 
  "trajectory_msgs"
)

set(EXEC_NAME "open_manipulator_x_controller")

add_executable(${EXEC_NAME} src/open_manipulator_x_controller.cpp)
ament_target_dependencies(${EXEC_NAME} ${dependencies})

################################################################################
# Install
################################################################################
install(TARGETS ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES 99-open-manipulator-cdc.rules
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/create_udev_rules
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(open_manipulator_msgs)
ament_export_dependencies(open_manipulator_x_libs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(robotis_manipulator)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(trajectory_msgs)
ament_package()
