
cmake_minimum_required(VERSION 2.8)

project(artoolkit)

find_package(catkin REQUIRED)

# add forward declaration for ARToolkit target below
add_custom_target(ARToolkit_catkin)

catkin_package(
	LIBRARIES GL glut ARgsub AR ARMulti ARvideo GL glut
	EXPORTED_TARGETS ARToolkit_catkin
)

include(ExternalProject)
ExternalProject_Add(ARToolkit
	# Settings:
	#  out of source build is not supported
	BINARY_DIR ARToolkit-prefix/src/ARToolkit

	# Download
	SVN_REPOSITORY svn://svn.code.sf.net/p/artoolkit/code/trunk/artoolkit
	UPDATE_COMMAND echo "Do not update"
	# Patch
	PATCH_COMMAND COMMAND patch -p0 < ${CMAKE_CURRENT_SOURCE_DIR}/patch_auto_config
		COMMAND patch -p0 < ${CMAKE_CURRENT_SOURCE_DIR}/patch_v4l
		COMMAND patch -p1 < ${CMAKE_CURRENT_SOURCE_DIR}/use-relative-include-paths-in-headers.patch
		COMMAND sh ${CMAKE_CURRENT_SOURCE_DIR}/cleanup.sh

	# Configure (with V4L driver)
	CONFIGURE_COMMAND ./Configure 1 n n n

	# Build
	BUILD_COMMAND make

	INSTALL_COMMAND mkdir -p ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
		                 ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}
		COMMAND cp -r include/AR ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
		COMMAND cp -r lib/libAR.a lib/libARgsub.a lib/libARgsub_lite.a lib/libARMulti.a lib/libARvideo.a lib/libARgsubUtil.a
			${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/
)

# Resolve forward declaration
add_dependencies(ARToolkit_catkin ARToolkit)

# Add cmake targets for each library to make inter-package dependency
# resolution possible
foreach(library AR ARgsub ARgsub_lite ARMulti ARvideo ARgsubUtil)
	add_library(${library} STATIC IMPORTED GLOBAL)
	set_target_properties(${library} PROPERTIES
		IMPORTED_LOCATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/lib${library}.a
	)
	set_target_properties(${library} PROPERTIES LINKER_LANGUAGE CXX)
endforeach()
