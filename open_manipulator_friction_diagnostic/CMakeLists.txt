################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.16)
project(open_manipulator_friction_diagnostic LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=unused-but-set-variable
                      -Werror=return-type -Werror=shadow -Werror=format -Werror=range-loop-construct
                      -Werror=missing-braces)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  angles
  std_srvs
  sensor_msgs
  std_msgs
  control_msgs
  control_toolbox
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tl_expected
  urdf
  kdl_parser
  orocos_kdl
)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

################################################################################
# Build
################################################################################
generate_parameter_library(friction_compensation_controller_parameters
  src/friction_compensation_controller_parameters.yaml
)

add_library(friction_compensation_controller SHARED
  src/friction_compensation_controller.cpp
  src/current_probe_controller.cpp
)
target_compile_features(friction_compensation_controller PUBLIC cxx_std_17)
target_include_directories(friction_compensation_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/friction_compensation_controller>
)
target_link_libraries(friction_compensation_controller PUBLIC
  controller_interface::controller_interface
  hardware_interface::mock_components
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${std_srvs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${control_msgs_TARGETS}
  realtime_tools::realtime_tools
  ${tl_expected_LIBRARIES}
  ${urdf_LIBRARIES}
  ${kdl_parser_TARGETS}
  ${orocos_kdl_TARGETS}
  friction_compensation_controller_parameters
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(friction_compensation_controller PRIVATE "friction_compensation_controller_BUILDING_DLL" "_USE_MATH_DEFINES")
pluginlib_export_plugin_description_file(controller_interface friction_compensation_plugin.xml)

################################################################################
# Install
################################################################################
install(
  DIRECTORY include/
  DESTINATION include/friction_compensation_controller
)
install(TARGETS
    friction_compensation_controller
    friction_compensation_controller_parameters
  EXPORT export_friction_compensation_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

################################################################################
# Macro for ament package
################################################################################
ament_export_targets(export_friction_compensation_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_python_install_package(${PROJECT_NAME})
install(FILES launch/calibrate.launch.py launch/bringup.launch.py launch/gazebo.launch.py
  launch/joint_impedance.launch.py
  DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
# A robot's fitted coefficients are runtime data, never a default for other units.
install(DIRECTORY calibration/geometry_cache DESTINATION share/${PROJECT_NAME}/calibration)
install(DIRECTORY calibration/trajectories DESTINATION share/${PROJECT_NAME}/calibration)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/friction_runtime scripts/calibration_collect scripts/calibration_video
  scripts/joint_impedance scripts/friction_device
  DESTINATION lib/${PROJECT_NAME})
add_subdirectory(vendor)
pluginlib_export_plugin_description_file(hardware_interface vendor/grouped_dynamixel_hardware_plugin.xml)
ament_package()
