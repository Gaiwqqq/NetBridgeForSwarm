#ifndef SWARM_ROS_BRIDGE_TUI_HISTORY_STORE_HPP_
#define SWARM_ROS_BRIDGE_TUI_HISTORY_STORE_HPP_

#include <swarm_ros_bridge/NetworkInfo.h>

#include <cstddef>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace swarm_ros_bridge {
namespace tui {

// Rolling window of bandwidth samples used by the sparkline widgets.
// Sampling happens on the UI refresh cadence; lookups are read-only.
class HistoryStore {
 public:
  void Sample(const std::vector<swarm_ros_bridge::NetworkInfo>& snapshot);
  std::vector<float> TotalBandwidthKbps() const;
  std::vector<float> BandwidthKbps(const std::vector<std::string>& keys) const;
  float LatestTotalKbps() const;

 private:
  static void Push(std::deque<float>* series, float value);

  mutable std::mutex mutex_;
  std::deque<float> total_;
  std::map<std::string, std::deque<float>> per_key_;
  std::size_t sample_count_{0};
  static constexpr std::size_t kCapacity = 120;
};

}  // namespace tui
}  // namespace swarm_ros_bridge

#endif  // SWARM_ROS_BRIDGE_TUI_HISTORY_STORE_HPP_
