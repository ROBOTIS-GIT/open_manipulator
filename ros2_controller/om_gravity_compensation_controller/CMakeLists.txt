cmake_minimum_required(VERSION 3.16)
project(om_gravity_compensation_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=unused-but-set-variable
                      -Werror=return-type -Werror=shadow -Werror=format -Werror=range-loop-construct
                      -Werror=missing-braces)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  angles
  control_msgs
  control_toolbox
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  rsl
  tl_expected
  urdf
  kdl_parser
  orocos_kdl_vendor
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(gravity_compensation_controller_parameters
  src/gravity_compensation_controller_parameters.yaml
)

add_library(gravity_compensation_controller SHARED
  src/gravity_compensation_controller.cpp
)
target_compile_features(gravity_compensation_controller PUBLIC cxx_std_17)
target_include_directories(gravity_compensation_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/gravity_compensation_controller>
)
target_link_libraries(gravity_compensation_controller PUBLIC
  gravity_compensation_controller_parameters
)
ament_target_dependencies(gravity_compensation_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(gravity_compensation_controller PRIVATE "gravity_compensation_controller_BUILDING_DLL" "_USE_MATH_DEFINES")
pluginlib_export_plugin_description_file(controller_interface gravity_compensation_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/gravity_compensation_controller
)
install(TARGETS
    gravity_compensation_controller
    gravity_compensation_controller_parameters
  EXPORT export_gravity_compensation_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_gravity_compensation_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
