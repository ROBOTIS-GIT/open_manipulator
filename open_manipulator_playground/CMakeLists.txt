################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.10)
project(open_manipulator_playground)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(hello_moveit_omx src/hello_moveit_omx.cpp)
target_include_directories(hello_moveit_omx PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(hello_moveit_omx PUBLIC cxx_std_17)
target_link_libraries(hello_moveit_omx PUBLIC
  moveit_ros_planning_interface::moveit_move_group_interface
  rclcpp::rclcpp
)

add_executable(hello_moveit_omy src/hello_moveit_omy.cpp)
target_include_directories(hello_moveit_omy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(hello_moveit_omy PUBLIC cxx_std_17)
target_link_libraries(hello_moveit_omy PUBLIC
  moveit_ros_planning_interface::moveit_move_group_interface
  rclcpp::rclcpp
)

################################################################################
# Install
################################################################################
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS hello_moveit_omx hello_moveit_omy
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_package()
