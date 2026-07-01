#ifndef SWARM_ROS_BRIDGE_CONFIG_BRIDGE_CONFIG_HPP_
#define SWARM_ROS_BRIDGE_CONFIG_BRIDGE_CONFIG_HPP_

#include <map>
#include <string>
#include <vector>

namespace swarm_ros_bridge {
namespace config {

struct TopicRule {
  std::string topic_name;
  std::string msg_type;
  std::vector<std::string> src_hosts;
  std::vector<std::string> dst_hosts;
  int src_port{0};
  double max_freq_hz{10.0};
  bool prefix{true};
  bool same_prefix{false};
  bool cloud_compress{false};
  double cloud_downsample{-1.0};
  double image_resize_rate{1.0};
  int image_jpeg_quality{80};
  bool image_adaptive_quality{false};
  int image_min_jpeg_quality{45};
  int image_max_jpeg_quality{90};
  double image_target_bandwidth_kbps{1200.0};
  int image_quality_step{5};
  int image_adapt_cooldown_frames{8};
  std::string cloud_codec{"raw"};
};

struct ServiceRule {
  std::string service_name;
  std::string service_type;
  std::string server_host;
  std::vector<std::string> client_hosts;
  int src_port{0};
  bool prefix{true};
};

struct RuntimeOptions {
  bool debug{false};
  bool odom_convert{true};
  bool monitor_node{true};
  int warn_threshold{3};
  int monitor_rate_hz{500};
};

struct BridgeConfig {
  std::string hostname;
  std::map<std::string, std::string> ip_map;
  RuntimeOptions runtime;
  std::vector<TopicRule> topics;
  std::vector<ServiceRule> services;
};

BridgeConfig MakeDefaultConfig();

}  // namespace config
}  // namespace swarm_ros_bridge

#endif  // SWARM_ROS_BRIDGE_CONFIG_BRIDGE_CONFIG_HPP_
