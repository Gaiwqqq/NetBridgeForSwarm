cmake_minimum_required(VERSION 3.0.2)
project(swarm_ros_bridge)

add_compile_options(-std=c++17)
find_program(CCACHE_FOUND ccache)
set(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

get_filename_component(SWARM_ROS_BRIDGE_REAL_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" REALPATH)
set(SWARM_ROS_BRIDGE_VENDOR_DIR "${SWARM_ROS_BRIDGE_REAL_SOURCE_DIR}/../third_party")

set(FTXUI_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(FTXUI_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(FTXUI_BUILD_MODULES OFF CACHE BOOL "" FORCE)
set(FTXUI_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(FTXUI_BUILD_TESTS_FUZZER OFF CACHE BOOL "" FORCE)
set(FTXUI_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
set(FTXUI_QUIET ON CACHE BOOL "" FORCE)

set(DRACO_BUILD_EXECUTABLES OFF CACHE BOOL "" FORCE)
set(DRACO_INSTALL OFF CACHE BOOL "" FORCE)
set(DRACO_TESTS OFF CACHE BOOL "" FORCE)
set(DRACO_WASM OFF CACHE BOOL "" FORCE)
set(DRACO_JS_GLUE OFF CACHE BOOL "" FORCE)

add_subdirectory(${SWARM_ROS_BRIDGE_VENDOR_DIR}/FTXUI
                 ${CMAKE_CURRENT_BINARY_DIR}/third_party/ftxui EXCLUDE_FROM_ALL)
add_subdirectory(${SWARM_ROS_BRIDGE_VENDOR_DIR}/draco
                 ${CMAKE_CURRENT_BINARY_DIR}/third_party/draco EXCLUDE_FROM_ALL)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  topic_tools
#        cv_bridge
  cv_bridge_noetic_fit_version
  pic_sockets
  image_transport
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
find_package(JPEG REQUIRED)

add_message_files(
  FILES
  NetworkInfo.msg
  NetworkArray.msg
  PtCloudCompress.msg
)

add_service_files(
  FILES
  GetOdomTopic.srv
  GetRecvTopic.srv
  AddTwoInts.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS message_runtime cv_bridge_noetic_fit_version pic_sockets
)

include_directories(
    include
    SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${JPEG_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${SWARM_ROS_BRIDGE_VENDOR_DIR}/FTXUI/include
    ${CMAKE_BINARY_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/third_party/draco
    ${SWARM_ROS_BRIDGE_VENDOR_DIR}/draco/src
)

catkin_install_python(PROGRAMS
    scripts/listener.py
    scripts/talker.py
    scripts/odom_talker.py
    scripts/drone_talker.py
    scripts/goalset_talker.py
    scripts/bridge_test_publisher.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(bridge_new
    src/bridge_node_main.cpp
    src/bridge_factory.cpp
    src/topic_factory.cpp
    src/service_factory.cpp
    src/logging/bridge_logger.cpp
    src/config/bridge_config.cpp
    src/config/config_loader.cpp
    src/diagnostics/diagnostics_cache.cpp
    src/diagnostics/topic_metrics.cpp
    src/transport/pointcloud_codec.cpp
    src/transport/draco_pointcloud_codec.cpp
)
add_dependencies(bridge_new
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(bridge_new
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    zmq
    zmqpp
    pthread
    ${JPEG_LIBRARIES}
    ${PCL_LIBRARIES}
    draco::draco
)

add_executable(bridge_tui
    src/nodes/bridge_tui_main.cpp
    src/tui/app.cpp
    src/tui/overview_screen.cpp
    src/tui/topics_screen.cpp
    src/tui/hosts_screen.cpp
    src/tui/logs_screen.cpp
    src/logging/bridge_logger.cpp
    src/config/bridge_config.cpp
    src/config/config_loader.cpp
    src/diagnostics/diagnostics_cache.cpp
    src/diagnostics/topic_metrics.cpp
    src/transport/pointcloud_codec.cpp
    src/transport/draco_pointcloud_codec.cpp
)
add_dependencies(bridge_tui
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(bridge_tui
    ${catkin_LIBRARIES}
    pthread
    ftxui::ftxui
    ${PCL_LIBRARIES}
    draco::draco
)

add_executable(test_md5 test/md5test.cpp)
add_dependencies(test_md5
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(test_md5
        ${catkin_LIBRARIES}
)

add_executable(a_simple_server test/a_simple_server.cpp)
add_dependencies(a_simple_server
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(a_simple_server
        ${catkin_LIBRARIES}
)

add_executable(a_simple_client test/a_simple_client.cpp)
add_dependencies(a_simple_client
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(a_simple_client
        ${catkin_LIBRARIES}
)
