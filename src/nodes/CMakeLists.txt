# common source files for both node and nodelet driver interfaces
set(DRIVER_SOURCES 
  driver1394stereo.cpp
  dev_camera1394stereo.cpp
  featuresstereo.cpp
  format7stereo.cpp
  modes.cpp)

# Mac OSX requires extra linker parameters
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  set(DC1394 dc1394 "-framework CoreServices")
else(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  set(DC1394 dc1394)
endif(CMAKE_SYSTEM_NAME MATCHES "Darwin")

rosbuild_add_executable(camera1394stereo_node camera1394stereo_node.cpp ${DRIVER_SOURCES})
target_link_libraries(camera1394stereo_node ${DC1394})

rosbuild_add_library(camera1394stereo_nodelet nodelet.cpp ${DRIVER_SOURCES})
target_link_libraries(camera1394stereo_nodelet ${DC1394})

#rosbuild_add_executable(camera1394_node node.cpp)
