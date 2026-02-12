cmake_minimum_required(VERSION 3.16)
project(om_spring_actuator_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(
    -Wall -Wextra -Wpedantic
    -Werror=unused-but-set-variable
    -Werror=return-type -Werror=shadow -Werror=format
    -Werror=range-loop-construct -Werror=missing-braces
  )
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

# ------------------------------------------------------------------------------
# Find dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# ------------------------------------------------------------------------------
# Build
# ------------------------------------------------------------------------------
generate_parameter_library(spring_actuator_controller_parameters
  src/spring_actuator_controller_parameters.yaml
)

add_library(spring_actuator_controller SHARED
  src/spring_actuator_controller.cpp
)

target_compile_features(spring_actuator_controller PUBLIC cxx_std_17)

target_include_directories(spring_actuator_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/spring_actuator_controller>
)

# Plain signature (no PUBLIC/PRIVATE/INTERFACE keywords)
target_link_libraries(spring_actuator_controller
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  spring_actuator_controller_parameters
)

# Keep deps here — it uses the plain signature internally
ament_target_dependencies(spring_actuator_controller
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  pluginlib
  realtime_tools
)

# Visibility + plugin export
target_compile_definitions(spring_actuator_controller PRIVATE
  "spring_actuator_controller_BUILDING_DLL" "_USE_MATH_DEFINES"
)

pluginlib_export_plugin_description_file(
  controller_interface spring_actuator_plugin.xml
)

# ------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------
install(
  DIRECTORY include/
  DESTINATION include/spring_actuator_controller
)

install(TARGETS
    spring_actuator_controller
    spring_actuator_controller_parameters
  EXPORT export_spring_actuator_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Install plugin XML so downstream can discover the controller
install(FILES spring_actuator_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Export
# ------------------------------------------------------------------------------
ament_export_targets(export_spring_actuator_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
