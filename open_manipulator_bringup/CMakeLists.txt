################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(open_manipulator_bringup)

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED)

################################################################################
# Install
################################################################################
install(
  DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

install(FILES open-manipulator-x-cdc.rules
  DESTINATION share/${PROJECT_NAME}
)

install(FILES open-manipulator-y-cdc.rules
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/x_create_udev_rules
  scripts/y_create_udev_rules
  scripts/init_position_for_x.py
  scripts/init_position_for_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
