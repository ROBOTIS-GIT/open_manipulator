################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(open_manipulator_with_tb3_waffle_pi_moveit)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  moveit_ros_move_group
  moveit_kinematics
  moveit_planners_ompl
  moveit_ros_visualization
  joint_state_publisher
  robot_state_publisher
  xacro
  urdf
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
  INCLUDE_DIRS include
  CATKIN_DEPENDS moveit_ros_move_group moveit_kinematics moveit_planners_ompl moveit_ros_visualization joint_state_publisher robot_state_publisher xacro urdf
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

################################################################################
# Install
################################################################################
install(DIRECTORY include/industrial_trajectory_filters/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE
)

install(FILES planning_request_adapters_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
