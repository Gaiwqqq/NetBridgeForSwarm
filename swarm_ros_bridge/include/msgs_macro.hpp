

#ifndef __MSGS_MACRO__
#define __MSGS_MACRO__
#include <ros/ros.h>

#include "logging/bridge_logger.hpp"

#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Image.h>

#include <std_srvs/Empty.h>
#include <swarm_ros_bridge/AddTwoInts.h>
// include your msg type here

#define INFO_MSG(str)        BRIDGE_LOG_INFO("Legacy", "", str)
#define INFO_MSG_RED(str)    BRIDGE_LOG_ERROR("Legacy", "", str)
#define INFO_MSG_GREEN(str)  BRIDGE_LOG_INFO("Legacy", "", str)
#define INFO_MSG_YELLOW(str) BRIDGE_LOG_WARN("Legacy", "", str)
#define INFO_MSG_BLUE(str)   BRIDGE_LOG_INFO("Legacy", "", str)

// Use X macro
#define MSGS_MACRO \
  X("sensor_msgs/Image", sensor_msgs::Image)                           \
  X("std_msgs/String", std_msgs::String)                               \
  X("nav_msgs/Odometry", nav_msgs::Odometry)                           \
  X("geometry_msgs/PoseStamped", geometry_msgs::PoseStamped)           \
  X("sensor_msgs/PointCloud2", sensor_msgs::PointCloud2)               \
  X("visualization_msgs/Marker", visualization_msgs::Marker)           \
  X("visualization_msgs/MarkerArray", visualization_msgs::MarkerArray)

#define SRVS_MACRO \
  X("std_srvs/Empty", std_srvs::Empty) \
  X("swarm_ros_bridge/AddTwoInts", swarm_ros_bridge::AddTwoInts)

#endif

