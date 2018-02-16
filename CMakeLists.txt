cmake_minimum_required(VERSION 2.8.3)
project(camera1394stereo)

find_package(catkin REQUIRED COMPONENTS 
	camera_info_manager 
	dynamic_reconfigure 
	image_transport 
	nodelet 
	roscpp
	rospy
	tf)

# This driver uses Boost threads
find_package(Boost REQUIRED COMPONENTS thread)

# Resolve system dependency on libdc1394, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(DC1394_PKGCONF REQUIRED libdc1394-2)
find_path(DC1394_INCLUDE_DIR
  NAMES dc1394.h
  PATHS ${DC1394_PKGCONF_INCLUDE_DIRS}
)
find_library(DC1394_LIBRARY
  NAMES dc1394
  PATHS ${DC1394_PKGCONF_LIBRARY_DIRS}
)

# auto-generated dynamic reconfiguration GUI
generate_dynamic_reconfigure_options(cfg/Camera1394Stereo.cfg)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
)

include_directories(cfg/cpp ${catkin_INCLUDE_DIRS})

# common source files for both node and nodelet driver interfaces
set(DRIVER_SOURCES 
  src/nodes/driver1394stereo.cpp
  src/nodes/dev_camera1394stereo.cpp
  src/nodes/featuresstereo.cpp
  src/nodes/format7stereo.cpp
  src/nodes/modes.cpp
  src/nodes/trigger.cpp)

# Mac OSX requires extra linker parameters
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  set(DC1394 dc1394 "-framework CoreServices")
else(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  set(DC1394 dc1394)
endif(CMAKE_SYSTEM_NAME MATCHES "Darwin")


add_executable(camera1394stereo_node src/nodes/camera1394stereo_node.cpp ${DRIVER_SOURCES})
add_dependencies(camera1394stereo_node camera1394stereo_gencfg) 
target_link_libraries(camera1394stereo_node ${DC1394} ${catkin_LIBRARIES})

add_library(camera1394stereo_nodelet src/nodes/nodelet.cpp ${DRIVER_SOURCES})
add_dependencies(camera1394stereo_nodelet camera1394stereo_gencfg) 
target_link_libraries(camera1394stereo_nodelet ${DC1394} ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS camera1394stereo_node camera1394stereo_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY src/nodes
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/stereo_camera_full.launch
  launch/stereo_camera.launch
  launch/nodelet_manager.launch
  launch/nodelet_standalone.launch
  launch/nodelet_pipeline.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
