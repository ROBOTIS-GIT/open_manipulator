cmake_minimum_required(VERSION 3.16)
project(om_torque_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=unused-but-set-variable
                      -Werror=return-type -Werror=shadow -Werror=format -Werror=range-loop-construct
                      -Werror=missing-braces)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  kdl_parser
  orocos_kdl
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  trajectory_msgs
  tl_expected
  urdf
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(torque_controller_parameters
  src/torque_controller_parameters.yaml
)

add_library(torque_controller SHARED
  src/torque_controller.cpp
)
target_compile_features(torque_controller PUBLIC cxx_std_17)
target_include_directories(torque_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/torque_controller>
)
target_link_libraries(torque_controller PUBLIC
  controller_interface::controller_interface
  hardware_interface::mock_components
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  ${backward_ros_LIBRARIES}
  ${tl_expected_LIBRARIES}
  ${urdf_LIBRARIES}
  ${kdl_parser_TARGETS}
  ${orocos_kdl_TARGETS}
  torque_controller_parameters
)

target_compile_definitions(torque_controller PRIVATE "TORQUE_CONTROLLER_BUILDING_DLL" "_USE_MATH_DEFINES")
pluginlib_export_plugin_description_file(controller_interface torque_controller_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/torque_controller
)
install(TARGETS
    torque_controller
    torque_controller_parameters
  EXPORT export_torque_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_torque_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
