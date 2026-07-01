#ifndef SWARM_ROS_BRIDGE_DIAGNOSTICS_TOPIC_METRICS_HPP_
#define SWARM_ROS_BRIDGE_DIAGNOSTICS_TOPIC_METRICS_HPP_

#include <ros/ros.h>

#include <cstddef>
#include <deque>
#include <string>

namespace swarm_ros_bridge {
namespace diagnostics {

struct TopicMetrics {
  std::string topic_name;
  std::string msg_type;
  std::string direction;
  std::string codec;
  double configured_rate_hz{0.0};
  double send_rate_hz{0.0};
  double recv_rate_hz{0.0};
  double avg_latency_ms{0.0};
  double jitter_ms{0.0};
  double bandwidth_kbps{0.0};
  double stability_score{100.0};
  double last_recv_age_ms{0.0};
  bool adaptive_quality_enabled{false};
  std::uint32_t configured_jpeg_quality{0};
  std::uint32_t current_jpeg_quality{0};
  double target_bandwidth_kbps{0.0};
  std::uint32_t packet_size{0};
  std::size_t total_messages{0};
  std::size_t dropped_messages{0};
};

struct TopicRuntimeState {
  std::string topic_name;
  std::string msg_type;
  std::string direction;
  std::string codec;
  double configured_rate_hz{0.0};
  std::size_t total_sent{0};
  std::size_t total_received{0};
  std::size_t dropped_messages{0};
  std::size_t total_send_bytes{0};
  std::size_t total_recv_bytes{0};
  std::uint32_t packet_size{0};
  ros::Time window_start;
  ros::Time last_send_time;
  ros::Time last_recv_time;
  std::deque<ros::Time> recent_send_times;
  std::deque<ros::Time> recent_recv_times;
  std::deque<std::pair<ros::Time, std::size_t>> recent_send_bytes;
  std::deque<std::pair<ros::Time, std::size_t>> recent_recv_bytes;
  double rate_window_sec{3.0};
  bool adaptive_quality_enabled{false};
  std::uint32_t configured_jpeg_quality{0};
  std::uint32_t current_jpeg_quality{0};
  double target_bandwidth_kbps{0.0};
  std::uint32_t quality_step{0};
  std::uint32_t adapt_cooldown_frames{0};
  std::size_t last_adapt_total_sent{0};
  double avg_latency_ms{0.0};
  double jitter_ms{0.0};
  double last_latency_ms{-1.0};
};

TopicMetrics MakeTopicMetrics(const TopicRuntimeState& state, const ros::Time& now);

}  // namespace diagnostics
}  // namespace swarm_ros_bridge

#endif  // SWARM_ROS_BRIDGE_DIAGNOSTICS_TOPIC_METRICS_HPP_
