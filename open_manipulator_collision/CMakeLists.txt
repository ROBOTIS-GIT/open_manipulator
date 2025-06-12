################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(open_manipulator_collision)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fcl REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)

################################################################################
# Build
################################################################################
add_executable(self_collision_node src/self_collision_node.cpp)

target_include_directories(self_collision_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${urdf_INCLUDE_DIRS}
    ${kdl_parser_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(self_collision_node
  PUBLIC
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${kdl_parser_TARGETS}
    ${urdf_TARGETS}
    Eigen3::Eigen
    fcl
)

################################################################################
# Install
################################################################################
install(TARGETS
  self_collision_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  kdl_parser
  urdf
)

ament_export_include_directories(
  include
  ${urdf_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

ament_export_libraries(self_collision_node)

ament_package()
