################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(open_manipulator_gazebo)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  gazebo_ros_control
  position_controllers
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
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs gazebo_ros_control position_controllers
)

################################################################################
# Build
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(open_manipulator_gazebo_grip_pub src/open_manipulator_gazebo_grip_pub.cpp)
target_link_libraries(open_manipulator_gazebo_grip_pub ${catkin_LIBRARIES})
add_dependencies(open_manipulator_gazebo_grip_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

################################################################################
# Install
################################################################################
install(TARGETS open_manipulator_gazebo_grip_pub
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
