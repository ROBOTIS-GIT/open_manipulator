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

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

################################################################################
# Build
################################################################################
add_executable(self_collision_node src/self_collision_node.cpp)

ament_target_dependencies(self_collision_node
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  urdf
  kdl_parser
  fcl
  Eigen3
)

target_link_libraries(self_collision_node fcl)

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
# Ament package
################################################################################
ament_package()
