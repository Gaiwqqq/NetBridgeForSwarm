#ifndef SWARM_ROS_BRIDGE_DIAGNOSTICS_DIAGNOSTICS_CACHE_HPP_
#define SWARM_ROS_BRIDGE_DIAGNOSTICS_DIAGNOSTICS_CACHE_HPP_

#include <swarm_ros_bridge/NetworkArray.h>
#include <swarm_ros_bridge/NetworkInfo.h>

#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace swarm_ros_bridge {
namespace diagnostics {

class DiagnosticsCache {
 public:
  void Update(const swarm_ros_bridge::NetworkArray& message);
  bool HasReceivedData() const;
  bool IsFresh(std::chrono::milliseconds max_age =
                   std::chrono::milliseconds(2500)) const;
  std::vector<swarm_ros_bridge::NetworkInfo> Snapshot() const;
  std::vector<swarm_ros_bridge::NetworkInfo> NodeSnapshot() const;
  bool LookupNode(const std::string& hostname,
                  swarm_ros_bridge::NetworkInfo* out) const;
  bool Lookup(const std::string& topic_name, swarm_ros_bridge::NetworkInfo* out) const;
  bool LookupDirected(const std::vector<std::string>& topic_names,
                      const std::string& direction,
                      swarm_ros_bridge::NetworkInfo* out) const;
  bool LookupAny(const std::vector<std::string>& topic_names,
                 swarm_ros_bridge::NetworkInfo* out) const;

 private:
  bool IsFreshLocked(std::chrono::milliseconds max_age) const;

  mutable std::mutex mutex_;
  std::map<std::string, swarm_ros_bridge::NetworkInfo> latest_;
  std::chrono::steady_clock::time_point last_update_at_;
  bool has_update_{false};
};

}  // namespace diagnostics
}  // namespace swarm_ros_bridge

#endif  // SWARM_ROS_BRIDGE_DIAGNOSTICS_DIAGNOSTICS_CACHE_HPP_
