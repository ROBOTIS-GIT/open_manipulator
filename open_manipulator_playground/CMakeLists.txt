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

add_executable(open_manipulator_x_hello_moveit src/open_manipulator_x_hello_moveit.cpp)
target_include_directories(open_manipulator_x_hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(open_manipulator_x_hello_moveit PUBLIC cxx_std_17)
target_link_libraries(open_manipulator_x_hello_moveit PUBLIC
  moveit_ros_planning_interface::moveit_move_group_interface
  rclcpp::rclcpp
)

add_executable(omy_f3m_hello_moveit src/omy_f3m_hello_moveit.cpp)
target_include_directories(omy_f3m_hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(omy_f3m_hello_moveit PUBLIC cxx_std_17)
target_link_libraries(omy_f3m_hello_moveit PUBLIC
  moveit_ros_planning_interface::moveit_move_group_interface
  rclcpp::rclcpp
)

add_executable(omy_3m_hello_moveit src/omy_3m_hello_moveit.cpp)
target_include_directories(omy_3m_hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(omy_3m_hello_moveit PUBLIC cxx_std_17)
target_link_libraries(omy_3m_hello_moveit PUBLIC
  moveit_ros_planning_interface::moveit_move_group_interface
  rclcpp::rclcpp
)

################################################################################
# Install
################################################################################
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS open_manipulator_x_hello_moveit omy_f3m_hello_moveit omy_3m_hello_moveit
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
