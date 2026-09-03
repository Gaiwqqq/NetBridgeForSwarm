

#ifndef __MSGS_MACRO__
#define __MSGS_MACRO__
#include <ros/ros.h>

#include "logging/bridge_logger.hpp"

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <std_srvs/Empty.h>
#include <swarm_ros_bridge/AddTwoInts.h>
// Include only message types that require field-level bridge behavior here.

#define INFO_MSG(str)        BRIDGE_LOG_INFO("Zenoh", "", str)
#define INFO_MSG_RED(str)    BRIDGE_LOG_ERROR("Zenoh", "", str)
#define INFO_MSG_GREEN(str)  BRIDGE_LOG_INFO("Zenoh", "", str)
#define INFO_MSG_YELLOW(str) BRIDGE_LOG_WARN("Zenoh", "", str)
#define INFO_MSG_BLUE(str)   BRIDGE_LOG_INFO("Zenoh", "", str)

// Built-in wire-codec specializations. Ordinary ROS topics do not need to be
// listed: they are forwarded as ShapeShifter serialization bytes.
#define SPECIALIZED_MSGS_MACRO \
  X("sensor_msgs/Image", sensor_msgs::Image)                           \
  X("nav_msgs/Odometry", nav_msgs::Odometry)                           \
  X("sensor_msgs/PointCloud2", sensor_msgs::PointCloud2)

// Register only custom messages whose contents the bridge must inspect, such
// as a std::vector<uint8_t> to_drone_ids field. Leave empty otherwise.
#define MSGS_MACRO

#define SRVS_MACRO \
  X("std_srvs/Empty", std_srvs::Empty) \
  X("swarm_ros_bridge/AddTwoInts", swarm_ros_bridge::AddTwoInts)

#endif
