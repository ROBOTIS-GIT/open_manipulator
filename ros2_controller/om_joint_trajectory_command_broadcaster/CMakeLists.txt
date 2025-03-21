cmake_minimum_required(VERSION 3.16)
project(om_joint_trajectory_command_broadcaster LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=conversion -Werror=unused-but-set-variable
                      -Werror=return-type -Werror=shadow -Werror=format -Werror=range-loop-construct
                      -Werror=missing-braces)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  builtin_interfaces
  control_msgs
  controller_interface
  generate_parameter_library
  pluginlib
  rclcpp_lifecycle
  rcutils
  realtime_tools
  sensor_msgs
  urdf
  trajectory_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(joint_trajectory_command_broadcaster_parameters
  src/joint_trajectory_command_broadcaster_parameters.yaml
)

add_library(joint_trajectory_command_broadcaster SHARED
  src/joint_trajectory_command_broadcaster.cpp
)
target_compile_features(joint_trajectory_command_broadcaster PUBLIC cxx_std_17)
target_include_directories(joint_trajectory_command_broadcaster PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/joint_trajectory_command_broadcaster>
)
target_link_libraries(joint_trajectory_command_broadcaster PUBLIC
  joint_trajectory_command_broadcaster_parameters
)
ament_target_dependencies(joint_trajectory_command_broadcaster PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(joint_trajectory_command_broadcaster PRIVATE "JOINT_TRAJECTORY_COMMAND_BROADCASTER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface joint_trajectory_command_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/joint_trajectory_command_broadcaster
)
install(
  TARGETS
    joint_trajectory_command_broadcaster
    joint_trajectory_command_broadcaster_parameters
  EXPORT export_joint_trajectory_command_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_joint_trajectory_command_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
