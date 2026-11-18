# Package-owned transport variant; the installed upstream model tables remain authoritative.
find_package(ament_index_cpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_interfaces REQUIRED)

add_library(grouped_dynamixel_hardware SHARED
  src/dynamixel_hardware_interface.cpp
  src/dynamixel/dynamixel_info.cpp
  src/dynamixel/dynamixel.cpp
)
target_compile_features(grouped_dynamixel_hardware PRIVATE cxx_std_17)
target_include_directories(grouped_dynamixel_hardware PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${dynamixel_sdk_INCLUDE_DIRS}
  ${hardware_interface_INCLUDE_DIRS}
  ${realtime_tools_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
)
target_link_libraries(grouped_dynamixel_hardware PRIVATE
  ament_index_cpp::ament_index_cpp
  ${hardware_interface_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${dynamixel_sdk_LIBRARIES}
  ${std_srvs_TARGETS}
  ${dynamixel_interfaces_TARGETS}
  ${realtime_tools_TARGETS}
)
install(TARGETS grouped_dynamixel_hardware LIBRARY DESTINATION lib)
install(FILES LICENSE source_manifest.json README.md grouped_write.patch read_service.patch write_alert.patch
  DESTINATION share/${PROJECT_NAME}/vendor)
