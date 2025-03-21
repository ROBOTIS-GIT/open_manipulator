cmake_minimum_required(VERSION 3.10)
project(open_manipulator_playground)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Executable 1
add_executable(hello_moveit_omx src/hello_moveit_omx.cpp)
target_include_directories(hello_moveit_omx PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(hello_moveit_omx PUBLIC cxx_std_17)
ament_target_dependencies(hello_moveit_omx
  moveit_ros_planning_interface
  rclcpp
)
install(TARGETS hello_moveit_omx
  DESTINATION lib/${PROJECT_NAME}
)

# Executable 2
add_executable(hello_moveit_omy src/hello_moveit_omy.cpp)
target_include_directories(hello_moveit_omy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(hello_moveit_omy PUBLIC cxx_std_17)
ament_target_dependencies(hello_moveit_omy
  moveit_ros_planning_interface
  rclcpp
)
install(TARGETS hello_moveit_omy
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
