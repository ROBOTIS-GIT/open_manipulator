cmake_minimum_required(VERSION 3.16)
project(om_joint_trajectory_command_broadcaster LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(
    -Wall -Wextra -Wpedantic
    -Werror=conversion -Werror=unused-but-set-variable
    -Werror=return-type -Werror=shadow -Werror=format
    -Werror=range-loop-construct -Werror=missing-braces
  )
endif()

# ---------------- Deps ----------------
set(THIS_PACKAGE_INCLUDE_DEPENDS
  builtin_interfaces
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcutils
  realtime_tools
  trajectory_msgs
  urdf
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# -------------- Parameters ------------
generate_parameter_library(joint_trajectory_command_broadcaster_parameters
  src/joint_trajectory_command_broadcaster_parameters.yaml
)

# --------------- Library --------------
add_library(joint_trajectory_command_broadcaster SHARED
  src/joint_trajectory_command_broadcaster.cpp
)

target_compile_features(joint_trajectory_command_broadcaster PUBLIC cxx_std_17)

target_include_directories(joint_trajectory_command_broadcaster PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/joint_trajectory_command_broadcaster>
)

# PLAIN signature only
target_link_libraries(joint_trajectory_command_broadcaster
  pluginlib::pluginlib
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  urdf::urdf
  joint_trajectory_command_broadcaster_parameters
  ${backward_ros_LIBRARIES}
)

ament_target_dependencies(joint_trajectory_command_broadcaster
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  rcutils
  realtime_tools
  trajectory_msgs
  builtin_interfaces
)

target_compile_definitions(joint_trajectory_command_broadcaster PRIVATE
  "JOINT_TRAJECTORY_COMMAND_BROADCASTER_BUILDING_DLL"
)

# IMPORTANT: use the XML filename that actually exists in this package
pluginlib_export_plugin_description_file(
  controller_interface joint_trajectory_command_plugin.xml
)

# ---------------- Install -------------
install(
  DIRECTORY include/
  DESTINATION include/joint_trajectory_command_broadcaster
)

install(TARGETS
  joint_trajectory_command_broadcaster
  joint_trajectory_command_broadcaster_parameters
  EXPORT export_joint_trajectory_command_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(FILES joint_trajectory_command_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# ---------------- Export --------------
ament_export_targets(export_joint_trajectory_command_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
