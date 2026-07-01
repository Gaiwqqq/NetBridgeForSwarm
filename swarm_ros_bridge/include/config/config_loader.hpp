#ifndef SWARM_ROS_BRIDGE_CONFIG_CONFIG_LOADER_HPP_
#define SWARM_ROS_BRIDGE_CONFIG_CONFIG_LOADER_HPP_

#include "config/bridge_config.hpp"

#include <ros/ros.h>

namespace swarm_ros_bridge {
namespace config {

class ConfigLoader {
 public:
  static bool LoadFromRosParams(const ros::NodeHandle& nh, BridgeConfig* config);
};

}  // namespace config
}  // namespace swarm_ros_bridge

#endif  // SWARM_ROS_BRIDGE_CONFIG_CONFIG_LOADER_HPP_
