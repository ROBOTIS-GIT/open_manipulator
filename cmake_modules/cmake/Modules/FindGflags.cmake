###############################################################################
#
# CMake script for finding the GFlags library.
#
# https://gflags.github.io/gflags/
#
# Redistribution and use is allowed according to the terms of the 3-clause BSD
# license.
#
# Input variables:
#
# - Gflags_FIND_REQUIRED: throws an error if gflags is not found but it is required
#
# Output variables:
#
# - Gflags_FOUND: Boolean that indicates if the package was found
# - Gflags_LIBRARIES: Package libraries
# - Gflags_INCLUDE_DIRS: Absolute path to package headers
#
# Example usage:
#
#   find_package(Gflags REQUIRED)
#
#   include_directories(
#     ${Gflags_INCLUDE_DIRS}
#   )
#
#   add_executable(main src/main.cpp)
#   target_link_libraries(main
#     ${Gflags_LIBRARIES}
#   )
###############################################################################


find_path(Gflags_INCLUDE_PATH gflags/gflags.h)

find_library(Gflags_LIBRARY NAMES gflags libgflags)

if(Gflags_INCLUDE_PATH AND Gflags_LIBRARY)
  set(Gflags_FOUND TRUE)
endif(Gflags_INCLUDE_PATH AND Gflags_LIBRARY)

if(Gflags_FOUND)
  message(STATUS "Found gflags: ${Gflags_LIBRARY}")
  # Output variables
  set(Gflags_INCLUDE_DIRS ${Gflags_INCLUDE_DIR})
  set(Gflags_LIBRARIES ${Gflags_LIBRARY})
else(Gflags_FOUND)
  if(Gflags_FIND_REQUIRED)
    message(FATAL_ERROR "Could not find gflags library.")
  endif(Gflags_FIND_REQUIRED)
endif(Gflags_FOUND)
