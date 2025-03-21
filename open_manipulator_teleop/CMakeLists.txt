cmake_minimum_required(VERSION 3.8)
project(open_manipulator_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/keyboard_control_x.py
  scripts/keyboard_control_y.py
  scripts/keyboard_control_y_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
