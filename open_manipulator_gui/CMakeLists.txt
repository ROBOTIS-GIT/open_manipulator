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
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create output directories for UI files
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/open_manipulator_x_gui)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/open_manipulator_y_gui)

# Generate UI headers
qt5_wrap_ui(PROJECT_UI_HEADERS_X ${PROJECT_UI_X})
qt5_wrap_ui(PROJECT_UI_HEADERS_Y ${PROJECT_UI_Y})

qt5_wrap_cpp(PROJECT_MOCS_X ${PROJECT_HEADERS_X})
qt5_wrap_cpp(PROJECT_MOCS_Y ${PROJECT_HEADERS_Y})

################################################################################
# Build
################################################################################
add_executable(open_manipulator_x_gui_node
  ${PROJECT_SOURCES_X}
  ${PROJECT_MOCS_X}
  ${PROJECT_UI_HEADERS_X}
)

add_dependencies(open_manipulator_x_gui_node copy_ui_headers)

################################################################################
# Include directories
################################################################################
target_include_directories(open_manipulator_x_gui_node PUBLIC
  include
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/ui
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(open_manipulator_x_gui_node
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  moveit_ros_planning_interface::moveit_move_group_interface
  ${EIGEN3_TARGETS}
  Qt5::Widgets
)

add_executable(open_manipulator_y_gui_node
  ${PROJECT_SOURCES_Y}
  ${PROJECT_MOCS_Y}
  ${PROJECT_UI_HEADERS_Y}
)

add_dependencies(open_manipulator_y_gui_node copy_ui_headers)

target_include_directories(open_manipulator_y_gui_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  include
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/ui
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(open_manipulator_y_gui_node
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  moveit_ros_planning_interface::moveit_move_group_interface
  ${EIGEN3_TARGETS}
  Qt5::Widgets
)

# Copy generated UI headers to the correct location
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/open_manipulator_x_gui/ui_x_main_window.h
  COMMAND ${CMAKE_COMMAND} -E copy
    ${CMAKE_CURRENT_BINARY_DIR}/ui_x_main_window.h
    ${CMAKE_CURRENT_BINARY_DIR}/open_manipulator_x_gui/ui_x_main_window.h
  DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/ui_x_main_window.h
)

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/open_manipulator_y_gui/ui_y_main_window.h
  COMMAND ${CMAKE_COMMAND} -E copy
    ${CMAKE_CURRENT_BINARY_DIR}/ui_y_main_window.h
    ${CMAKE_CURRENT_BINARY_DIR}/open_manipulator_y_gui/ui_y_main_window.h
  DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/ui_y_main_window.h
)

add_custom_target(copy_ui_headers ALL
  DEPENDS
    ${CMAKE_CURRENT_BINARY_DIR}/open_manipulator_x_gui/ui_x_main_window.h
    ${CMAKE_CURRENT_BINARY_DIR}/open_manipulator_y_gui/ui_y_main_window.h
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
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_libraries(open_manipulator_x_gui_node)
ament_export_libraries(open_manipulator_y_gui_node)
ament_package()
