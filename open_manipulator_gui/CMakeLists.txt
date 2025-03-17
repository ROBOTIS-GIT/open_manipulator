################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(open_manipulator_gui)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

################################################################################
# Define project sources and headers
################################################################################
set(PROJECT_SOURCES_X
  src/om_x/main.cpp
  src/om_x/main_window.cpp
  src/om_x/qnode.cpp
)

set(PROJECT_HEADERS_X
  include/open_manipulator_x_gui/main_window.hpp
  include/open_manipulator_x_gui/qnode.hpp
)

set(PROJECT_UI_X
  ui/x_main_window.ui
)

set(PROJECT_SOURCES_Y
  src/om_y/main.cpp
  src/om_y/main_window.cpp
  src/om_y/qnode.cpp
)

set(PROJECT_HEADERS_Y
  include/open_manipulator_y_gui/main_window.hpp
  include/open_manipulator_y_gui/qnode.hpp
)

set(PROJECT_UI_Y
  ui/y_main_window.ui
)

################################################################################
# Qt specific configurations
################################################################################
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ui)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

qt5_wrap_cpp(PROJECT_MOCS_X ${PROJECT_HEADERS_X})
qt5_wrap_ui(PROJECT_UI_HEADERS_X ${PROJECT_UI_X})
qt5_wrap_cpp(PROJECT_MOCS_Y ${PROJECT_HEADERS_Y})
qt5_wrap_ui(PROJECT_UI_HEADERS_Y ${PROJECT_UI_Y})

################################################################################
# Include directories
################################################################################
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
)

################################################################################
# Build
################################################################################
add_executable(open_manipulator_x_gui_node
  ${PROJECT_SOURCES_X}
  ${PROJECT_MOCS_X}
  ${PROJECT_UI_HEADERS_X}
)

ament_target_dependencies(open_manipulator_x_gui_node
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  Eigen3
  Qt5Widgets
)

add_executable(open_manipulator_y_gui_node
  ${PROJECT_SOURCES_Y}
  ${PROJECT_MOCS_Y}
  ${PROJECT_UI_HEADERS_Y}
)

ament_target_dependencies(open_manipulator_y_gui_node
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  Eigen3
  Qt5Widgets
)

################################################################################
# Install
################################################################################
install(TARGETS open_manipulator_x_gui_node
  DESTINATION lib/open_manipulator_gui
)

install(TARGETS open_manipulator_y_gui_node
  DESTINATION lib/open_manipulator_gui
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch ui
  DESTINATION share/open_manipulator_gui
)

install(DIRECTORY config DESTINATION share/open_manipulator_gui)

################################################################################
# Export information
################################################################################
ament_export_include_directories(include)
ament_export_libraries(open_manipulator_x_gui_node)
ament_export_libraries(open_manipulator_y_gui_node)

################################################################################
# Macro for ament package
################################################################################
ament_package()
