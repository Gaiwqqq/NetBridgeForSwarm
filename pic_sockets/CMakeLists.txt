cmake_minimum_required(VERSION 3.0.2)
project(pic_sockets)
# Set C++11 standard
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -O3 -fPIC")
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(JPEG REQUIRED)
include_directories(${JPEG_INCLUDE_DIR})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES pic_sockets
    CATKIN_DEPENDS roscpp rospy std_msgs
    DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(pic_sockets
    src/network.cpp
    src/udp_sender.cpp
    src/udp_receiver.cpp
    src/opencv.cpp
    src/pic_socket.cpp
)

target_link_libraries(pic_sockets
    ${catkin_LIBRARIES}
    pthread
    ${OpenCV_LIBS}
    ${JPEG_LIBRARIES}
)

