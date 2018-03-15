################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(open_manipulator_description)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs
)

################################################################################
# Build
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(open_manipulator_chain_rviz_pub src/open_manipulator_chain_rviz_pub.cpp)
add_dependencies(open_manipulator_chain_rviz_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(open_manipulator_chain_rviz_pub ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS open_manipulator_chain_rviz_pub
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
