################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(open_manipulator_x_libs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robotis_manipulator REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
)
include_directories(${EIGEN3_INCLUDE_DIRS})

set(dependencies_lib
  "EIGEN3"
  "dynamixel_workbench_toolbox"
  "rclcpp"
  "robotis_manipulator"
)

set(LIB_NAME "open_manipulator_x_libs")

add_library(${LIB_NAME} SHARED
  "src/custom_trajectory.cpp"
  "src/dynamixel.cpp"
  "src/kinematics.cpp"
  "src/open_manipulator_x.cpp"
)
ament_target_dependencies(${LIB_NAME} ${dependencies_lib})
target_link_libraries(${LIB_NAME} ${Eigen3_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS ${LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(Eigen3)
ament_export_dependencies(dynamixel_workbench_toolbox)
ament_export_dependencies(rclcpp)
ament_export_dependencies(robotis_manipulator)
ament_export_libraries(${LIB_NAME})
ament_package()
