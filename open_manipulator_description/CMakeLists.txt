################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(open_manipulator_description)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
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
#  LIBRARIES open_manipulator_description
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

################################################################################
# Build
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(open_manipulator_description_node src/open_manipulator_description_node.cpp)
add_dependencies(open_manipulator_description_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(open_manipulator_description_node ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################
