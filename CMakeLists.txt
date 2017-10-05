cmake_minimum_required(VERSION 2.8.3)
project(guidance_uart)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  roslaunch
)

add_message_files(
  FILES
  Motion.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
	CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

message("System is: " ${CMAKE_SYSTEM_PROCESSOR})
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  if (CMAKE_SIZEOF_VOID_P EQUAL 4)
    message("-- 32bit detected")
    link_directories(lib/x86)
  elseif (CMAKE_SIZEOF_VOID_P EQUAL 8)
    message("-- 64bit detected")
    link_directories(lib/x64)
  endif ()
else()
  message("-- Non-linux platform detected but sorry we do not support :D")
endif ()

if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "armv7l" )
  message("-- " ${CMAKE_SYSTEM_PROCESSOR} " detected")
  link_directories(lib/XU3)
else ()
endif ()

link_libraries(
  ${catkin_LIBRARIES}
  DJI_guidance
  usb-1.0
  yaml-cpp)

#set(SOURCES file.cpp file2.cpp ${YOUR_DIRECTORY}/file1.h ${YOUR_DIRECTORY}/file2.h)
#add_executable(test ${SOURCES})

add_executable(guidanceUARTNode
  src/main.cpp
  src/serial.cpp
  src/crc16.cpp
  src/crc32.cpp
  src/protocal_uart_sdk.cpp
  ) 

roslaunch_add_file_check(launch)

#  vim: set et fenc=utf-8 ff=unix sts=0 sw=2 ts=2 :
