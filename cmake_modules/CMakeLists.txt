cmake_minimum_required(VERSION 3.5)
project(cmake_modules)

# Add support for C++11
if(NOT WIN32)
  add_definitions(-std=c++11)
endif()

find_package(ament_cmake REQUIRED)

set(INCLUDE_DIRS ${ament_cmake_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${ament_cmake_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${ament_cmake_LIBRARIES})

## Install all cmake files
install(DIRECTORY cmake/Modules
  DESTINATION share/${PROJECT_NAME}/cmake)

ament_export_dependencies(ament_cmake)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package(
  CONFIG_EXTRAS "cmake/cmake_modules-extras.cmake.in"
)
