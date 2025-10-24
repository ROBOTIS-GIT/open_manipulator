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
set(PROJECT_SOURCES_open_manipulator_x
  src/open_manipulator_x/main.cpp
  src/open_manipulator_x/main_window.cpp
  src/open_manipulator_x/qnode.cpp
)

set(PROJECT_HEADERS_open_manipulator_x
  include/open_manipulator_x_gui/main_window.hpp
  include/open_manipulator_x_gui/qnode.hpp
)

set(PROJECT_UI_open_manipulator_x
  ui/open_manipulator_x_main_window.ui
)

################################################################################
# OMY 3M
################################################################################
set(PROJECT_SOURCES_OMY3M
  src/omy_3m/main.cpp
  src/omy_3m/main_window.cpp
  src/omy_3m/qnode.cpp
)

set(PROJECT_HEADERS_OMY3M
  include/omy_3m_gui/main_window.hpp
  include/omy_3m_gui/qnode.hpp
)

set(PROJECT_UI_OMY3M
  ui/omy_3m_main_window.ui
)


################################################################################
# OMY F3M
################################################################################
set(PROJECT_SOURCES_OMYF3M
  src/omy_f3m/main.cpp
  src/omy_f3m/main_window.cpp
  src/omy_f3m/qnode.cpp
)

set(PROJECT_HEADERS_OMYF3M
  include/omy_f3m_gui/main_window.hpp
  include/omy_f3m_gui/qnode.hpp
)

set(PROJECT_UI_OMYF3M
  ui/omy_f3m_main_window.ui
)


################################################################################
# OMX F
################################################################################
set(PROJECT_SOURCES_OMXF
  src/omx_f/main.cpp
  src/omx_f/main_window.cpp
  src/omx_f/qnode.cpp
)

set(PROJECT_HEADERS_OMXF
  include/omx_f_gui/main_window.hpp
  include/omx_f_gui/qnode.hpp
)

set(PROJECT_UI_OMXF
  ui/omx_f_main_window.ui
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
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/omy_3m_gui)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/omy_f3m_gui)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/omx_f_gui)

# Generate UI headers
qt5_wrap_ui(PROJECT_UI_HEADERS_open_manipulator_x ${PROJECT_UI_open_manipulator_x})
qt5_wrap_ui(PROJECT_UI_HEADERS_OMY3M ${PROJECT_UI_OMY3M})
qt5_wrap_ui(PROJECT_UI_HEADERS_OMYF3M ${PROJECT_UI_OMYF3M})
qt5_wrap_ui(PROJECT_UI_HEADERS_OMXF ${PROJECT_UI_OMXF})

qt5_wrap_cpp(PROJECT_MOCS_open_manipulator_x ${PROJECT_HEADERS_open_manipulator_x})
qt5_wrap_cpp(PROJECT_MOCS_OMY3M ${PROJECT_HEADERS_OMY3M})
qt5_wrap_cpp(PROJECT_MOCS_OMYF3M ${PROJECT_HEADERS_OMYF3M})
qt5_wrap_cpp(PROJECT_MOCS_OMXF ${PROJECT_HEADERS_OMXF})

################################################################################
# Build
################################################################################
add_executable(open_manipulator_x_gui_node
  ${PROJECT_SOURCES_open_manipulator_x}
  ${PROJECT_MOCS_open_manipulator_x}
  ${PROJECT_UI_HEADERS_open_manipulator_x}
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

add_dependencies(open_manipulator_x_gui_node copy_ui_headers)

################################################################################
# OMY 3M
################################################################################
add_executable(omy_3m_gui_node
  ${PROJECT_SOURCES_OMY3M}
  ${PROJECT_MOCS_OMY3M}
  ${PROJECT_UI_HEADERS_OMY3M}
)

add_dependencies(omy_3m_gui_node copy_ui_headers)

target_include_directories(omy_3m_gui_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  include
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/ui
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(omy_3m_gui_node
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  moveit_ros_planning_interface::moveit_move_group_interface
  ${EIGEN3_TARGETS}
  Qt5::Widgets
)

################################################################################
# OMY F3M
################################################################################
add_executable(omy_f3m_gui_node
  ${PROJECT_SOURCES_OMYF3M}
  ${PROJECT_MOCS_OMYF3M}
  ${PROJECT_UI_HEADERS_OMYF3M}
)

add_dependencies(omy_f3m_gui_node copy_ui_headers)

target_include_directories(omy_f3m_gui_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  include
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/ui
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(omy_f3m_gui_node
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  moveit_ros_planning_interface::moveit_move_group_interface
  ${EIGEN3_TARGETS}
  Qt5::Widgets
)

################################################################################
# OMX F
################################################################################
add_executable(omx_f_gui_node
  ${PROJECT_SOURCES_OMXF}
  ${PROJECT_MOCS_OMXF}
  ${PROJECT_UI_HEADERS_OMXF}
)

add_dependencies(omx_f_gui_node copy_ui_headers)

target_include_directories(omx_f_gui_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  include
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/ui
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(omx_f_gui_node
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  moveit_ros_planning_interface::moveit_move_group_interface
  ${EIGEN3_TARGETS}
  Qt5::Widgets
)

################################################################################
# Copy generated UI headers to the correct location
################################################################################
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/open_manipulator_x_gui/ui_open_manipulator_x_main_window.h
  COMMAND ${CMAKE_COMMAND} -E copy
    ${CMAKE_CURRENT_BINARY_DIR}/ui_open_manipulator_x_main_window.h
    ${CMAKE_CURRENT_BINARY_DIR}/open_manipulator_x_gui/ui_open_manipulator_x_main_window.h
  DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/ui_open_manipulator_x_main_window.h
)

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/omy_3m_gui/ui_omy_3m_main_window.h
  COMMAND ${CMAKE_COMMAND} -E copy
    ${CMAKE_CURRENT_BINARY_DIR}/ui_omy_3m_main_window.h
    ${CMAKE_CURRENT_BINARY_DIR}/omy_3m_gui/ui_omy_3m_main_window.h
  DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/ui_omy_3m_main_window.h
)

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/omy_f3m_gui/ui_omy_f3m_main_window.h
  COMMAND ${CMAKE_COMMAND} -E copy
    ${CMAKE_CURRENT_BINARY_DIR}/ui_omy_f3m_main_window.h
    ${CMAKE_CURRENT_BINARY_DIR}/omy_f3m_gui/ui_omy_f3m_main_window.h
  DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/ui_omy_f3m_main_window.h
)

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/omx_f_gui/ui_omx_f_main_window.h
  COMMAND ${CMAKE_COMMAND} -E copy
    ${CMAKE_CURRENT_BINARY_DIR}/ui_omx_f_main_window.h
    ${CMAKE_CURRENT_BINARY_DIR}/omx_f_gui/ui_omx_f_main_window.h
  DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/ui_omx_f_main_window.h
)

add_custom_target(copy_ui_headers ALL
  DEPENDS
    ${CMAKE_CURRENT_BINARY_DIR}/open_manipulator_x_gui/ui_open_manipulator_x_main_window.h
    ${CMAKE_CURRENT_BINARY_DIR}/omy_3m_gui/ui_omy_3m_main_window.h
    ${CMAKE_CURRENT_BINARY_DIR}/omy_f3m_gui/ui_omy_f3m_main_window.h
    ${CMAKE_CURRENT_BINARY_DIR}/omx_f_gui/ui_omx_f_main_window.h
)

################################################################################
# Install
################################################################################
install(TARGETS open_manipulator_x_gui_node
  DESTINATION lib/open_manipulator_gui
)

install(TARGETS omy_3m_gui_node
  DESTINATION lib/open_manipulator_gui
)

install(TARGETS omy_f3m_gui_node
  DESTINATION lib/open_manipulator_gui
)

install(TARGETS omx_f_gui_node
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
ament_export_libraries(omy_3m_gui_node)
ament_export_libraries(omy_f3m_gui_node)
ament_export_libraries(omx_f_gui_node)

################################################################################
# Macro for ament package
################################################################################
ament_package()
