################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.8)
project(open_manipulator_x_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(open_manipulator_x_bringup REQUIRED)
find_package(open_manipulator_x_moveit_config REQUIRED)
find_package(open_manipulator_x_description REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_action REQUIRED)

################################################################################
# Build
################################################################################
add_executable(open_manipulator_x_teleop src/open_manipulator_x_teleop.cpp)
target_include_directories(open_manipulator_x_teleop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  open_manipulator_x_teleop
  "rclcpp"
  "open_manipulator_x_bringup"
  "open_manipulator_x_moveit_config"
  "open_manipulator_x_description"
  "geometry_msgs"
  "nav_msgs"
  "control_msgs"
  "sensor_msgs"
  "std_srvs"
  "rclcpp_action"
)

################################################################################
# Install
################################################################################
install(TARGETS open_manipulator_x_teleop
  DESTINATION lib/${PROJECT_NAME})

  ################################################################################
# Macro for ament package
################################################################################
ament_package()
