cmake_minimum_required(VERSION 2.8.3)
project(feynman_camera)

find_package(catkin REQUIRED camera_info_manager dynamic_reconfigure sensor_msgs roscpp message_generation)
SET(CMAKE_CXX_FLAGS "-g")
#MACHINE = $(shell uname -m)
execute_process(COMMAND uname -m  OUTPUT_VARIABLE  MACHINES )
execute_process(COMMAND getconf LONG_BIT  OUTPUT_VARIABLE  MACHINES_BIT )
MESSAGE(STATUS "NEXTVPU Machine : ${MACHINES}")
MESSAGE(STATUS "NEXTVPU Machine Bits : ${MACHINES_BIT}")
IF ( (${MACHINES} MATCHES "x86_64") AND (${MACHINES_BIT} MATCHES "64")  )
      set(HOST_PLATFORM  "x64")
# for schroot enviroment
ELSEIF ( (${MACHINES} MATCHES "x86_64") AND (${MACHINES_BIT} MATCHES "32")  )
      set(HOST_PLATFORM  "x86")
ELSEIF ( ${MACHINES} MATCHES "x86" )
ELSEIF ( ${MACHINES} MATCHES "x86" )
      set(HOST_PLATFORM  "x86")
ELSEIF ( ${MACHINES} MATCHES "i686" )
      set(HOST_PLATFORM  "x86")
ELSEIF ( ${MACHINES} MATCHES "i386" )
      set(HOST_PLATFORM  "x86")
ELSEIF ( ${MACHINES} MATCHES "arm" )
      set(HOST_PLATFORM  "arm")
ELSEIF ( (${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "64")  )
      set(HOST_PLATFORM  "arm64")
ELSEIF ( (${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "32")  )
      set(HOST_PLATFORM  "arm")
ENDIF ()

message(STATUS "NEXTVPU : ${HOST_PLATFORM}")

add_service_files(
  FILES
  SetExposure.srv
  SetStreamMode.srv
  SetDepthMode.srv
  SetProjector.srv
  GetCameraParam.srv
  SwitchRectify.srv
  SaveDepth.srv
  RemoveDark.srv
)

add_message_files(FILES
  temp_info.msg
  cnn_info.msg
  cnn_box.msg
  imu_frame.msg
  imu_info.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS camera_info_manager dynamic_reconfigure  sensor_msgs roscpp message_runtime
  
)

include_directories(include /usr/include/libusb-1.0 /usr/include/pcl-1.8 /usr/include/eigen3
                    ${catkin_INCLUDE_DIRS}
                    )

add_executable(feynman_camera
   ros/feynman_camera_node.cpp
   src/feynman_sdk.cpp
   src/feynman_upgrade_module.cpp
   src/ring_queue.cpp
   src/yuv_rgb.c
)
add_dependencies(feynman_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#pragma comment(lib,"opencv_core453d.lib")
#pragma comment(lib,"opencv_imgproc453d.lib")
#pragma comment(lib,"ippicvmt.lib")
#pragma comment(lib,"IlmImfd.lib")
#pragma comment(lib,"ippiwd.lib")
#pragma comment(lib,"ittnotifyd.lib")
#pragma comment(lib,"zlibd.lib")

target_link_libraries(feynman_camera ${catkin_LIBRARIES} usb-1.0 opencv_core opencv_imgproc)
add_dependencies(feynman_camera ${PROJECT_NAME}_generate_messages_cpp)

install(TARGETS feynman_camera ${ADDITIONAL_EXECUTABLES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES nextvpu-usb.rules
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}
)

install(FILES nextvpu-usb.rules
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
