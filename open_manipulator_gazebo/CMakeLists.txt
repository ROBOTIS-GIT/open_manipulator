################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(open_manipulator_gazebo)

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
#  INCLUDE_DIRS include
#  LIBRARIES open_manipulator_gazebo
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
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

################################################################################
# Test
################################################################################
