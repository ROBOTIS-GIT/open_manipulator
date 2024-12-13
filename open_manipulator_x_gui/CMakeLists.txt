################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(open_manipulator_x_gui)

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
set(PROJECT_SOURCES
  src/main.cpp
  src/main_window.cpp
  src/qnode.cpp
)

set(PROJECT_HEADERS
  include/${PROJECT_NAME}/main_window.hpp
  include/${PROJECT_NAME}/qnode.hpp
)

set(PROJECT_UI
  ui/main_window.ui
)

################################################################################
# Qt specific configurations
################################################################################
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ui)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

qt5_wrap_cpp(PROJECT_MOCS ${PROJECT_HEADERS})
qt5_wrap_ui(PROJECT_UI_HEADERS ${PROJECT_UI})

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
add_executable(${PROJECT_NAME}_node
  ${PROJECT_SOURCES}
  ${PROJECT_MOCS}
  ${PROJECT_UI_HEADERS}
)

ament_target_dependencies(${PROJECT_NAME}_node
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
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch ui
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})


################################################################################
# Export information
################################################################################
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_node)

################################################################################
# Macro for ament package
################################################################################
ament_package()
