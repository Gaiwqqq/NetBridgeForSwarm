#ifndef SWARM_ROS_BRIDGE_DIAGNOSTICS_DIAGNOSTICS_CACHE_HPP_
#define SWARM_ROS_BRIDGE_DIAGNOSTICS_DIAGNOSTICS_CACHE_HPP_

#include <swarm_ros_bridge/NetworkArray.h>
#include <swarm_ros_bridge/NetworkInfo.h>

#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace swarm_ros_bridge {
namespace diagnostics {

class DiagnosticsCache {
 public:
  void Update(const swarm_ros_bridge::NetworkArray& message);
  std::vector<swarm_ros_bridge::NetworkInfo> Snapshot() const;
  bool Lookup(const std::string& topic_name, swarm_ros_bridge::NetworkInfo* out) const;
  bool LookupDirected(const std::vector<std::string>& topic_names,
                      const std::string& direction,
                      swarm_ros_bridge::NetworkInfo* out) const;
  bool LookupAny(const std::vector<std::string>& topic_names,
                 swarm_ros_bridge::NetworkInfo* out) const;

 private:
  mutable std::mutex mutex_;
  std::map<std::string, swarm_ros_bridge::NetworkInfo> latest_;
};

}  // namespace diagnostics
}  // namespace swarm_ros_bridge

#endif  // SWARM_ROS_BRIDGE_DIAGNOSTICS_DIAGNOSTICS_CACHE_HPP_
