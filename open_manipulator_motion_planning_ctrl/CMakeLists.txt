################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(open_manipulator_motion_planning_ctrl)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  control_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  cmake_modules

  open_manipulator_msgs
)

find_package(Eigen REQUIRED)

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
  LIBRARIES open_manipulator_motion_planning_ctrl
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib EIGEN3
)

################################################################################
# Build
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(motion_planning_controller src/motion_planning_controller.cpp)
add_dependencies(motion_planning_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motion_planning_controller ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################
