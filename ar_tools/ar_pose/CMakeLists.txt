
cmake_minimum_required(VERSION 2.8)

project(ar_pose)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	visualization_msgs
	artoolkit
	geometry_msgs
	tf
	resource_retriever
	image_transport
	cv_bridge
	message_generation
)


add_message_files(
  DIRECTORY msg
  FILES
	ARMarker.msg
	ARMarkers.msg
)

generate_messages(DEPENDENCIES
	geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    visualization_msgs
    artoolkit
    message_runtime
)

include_directories(${catkin_INCLUDE_DIRS} include)

add_executable(ar_single
	src/ar_single.cpp
	src/object.cpp
)
target_link_libraries(ar_single
	${catkin_LIBRARIES}
)
add_dependencies(ar_single
	${PROJECT_NAME}_generate_messages_cpp
	${artoolkit_EXPORTED_TARGETS}
)

add_executable(ar_multi
	src/ar_multi.cpp
	src/object.cpp
)
target_link_libraries(ar_multi
	${catkin_LIBRARIES}
)
add_dependencies(ar_multi
	${PROJECT_NAME}_generate_messages_cpp
	${artoolkit_EXPORTED_TARGETS}
)

add_executable(ar_manipulator_demo
  src/ar_manipulator_demo.cpp
)
target_link_libraries(ar_manipulator_demo
  ${catkin_LIBRARIES}
)
add_dependencies(ar_manipulator_demo
	${PROJECT_NAME}_generate_messages_cpp
	${artoolkit_EXPORTED_TARGETS}
)

install(TARGETS ar_single
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ar_multi
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY demo/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/demo
)
install(DIRECTORY data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
)
