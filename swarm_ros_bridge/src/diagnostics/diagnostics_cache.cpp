#include "diagnostics/diagnostics_cache.hpp"

namespace swarm_ros_bridge {
namespace diagnostics {

void DiagnosticsCache::Update(const swarm_ros_bridge::NetworkArray& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_.clear();
  for (const auto& item : message.info) {
    latest_[item.name] = item;
  }
}

std::vector<swarm_ros_bridge::NetworkInfo> DiagnosticsCache::Snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<swarm_ros_bridge::NetworkInfo> items;
  items.reserve(latest_.size());
  for (const auto& kv : latest_) {
    items.push_back(kv.second);
  }
  return items;
}

bool DiagnosticsCache::Lookup(const std::string& topic_name,
                              swarm_ros_bridge::NetworkInfo* out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = latest_.find(topic_name);
  if (it == latest_.end() || out == nullptr) {
    return false;
  }
  *out = it->second;
  return true;
}

bool DiagnosticsCache::LookupDirected(
    const std::vector<std::string>& topic_names,
    const std::string& direction,
    swarm_ros_bridge::NetworkInfo* out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (out == nullptr) {
    return false;
  }
  for (const auto& topic_name : topic_names) {
    const auto it = latest_.find(topic_name);
    if (it != latest_.end() && it->second.direction == direction) {
      *out = it->second;
      return true;
    }
  }
  return false;
}

bool DiagnosticsCache::LookupAny(const std::vector<std::string>& topic_names,
                                 swarm_ros_bridge::NetworkInfo* out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (out == nullptr) {
    return false;
  }
  for (const auto& topic_name : topic_names) {
    const auto it = latest_.find(topic_name);
    if (it != latest_.end()) {
      *out = it->second;
      return true;
    }
  }
  return false;
}

}  // namespace diagnostics
}  // namespace swarm_ros_bridge
