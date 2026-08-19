#include "diagnostics/diagnostics_cache.hpp"

namespace swarm_ros_bridge {
namespace diagnostics {

void DiagnosticsCache::Update(const swarm_ros_bridge::NetworkArray& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_.clear();
  for (const auto& item : message.info) {
    latest_[item.name] = item;
  }
  last_update_at_ = std::chrono::steady_clock::now();
  has_update_ = true;
}

bool DiagnosticsCache::IsFresh(std::chrono::milliseconds max_age) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return IsFreshLocked(max_age);
}

bool DiagnosticsCache::HasReceivedData() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return has_update_;
}

bool DiagnosticsCache::IsFreshLocked(std::chrono::milliseconds max_age) const {
  return has_update_ &&
         std::chrono::steady_clock::now() - last_update_at_ <= max_age;
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

std::vector<swarm_ros_bridge::NetworkInfo> DiagnosticsCache::NodeSnapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<swarm_ros_bridge::NetworkInfo> nodes;
  for (const auto& item : latest_) {
    if (item.second.msg_type == "presence") {
      nodes.push_back(item.second);
    }
  }
  return nodes;
}

bool DiagnosticsCache::LookupNode(
    const std::string& hostname,
    swarm_ros_bridge::NetworkInfo* out) const {
  if (out == nullptr) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!IsFreshLocked(std::chrono::milliseconds(2500))) {
    return false;
  }
  const auto item = latest_.find("@zenoh/node/" + hostname);
  if (item == latest_.end() || item->second.msg_type != "presence") {
    return false;
  }
  *out = item->second;
  return true;
}

bool DiagnosticsCache::Lookup(const std::string& topic_name,
                              swarm_ros_bridge::NetworkInfo* out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!IsFreshLocked(std::chrono::milliseconds(2500))) {
    return false;
  }
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
  if (!IsFreshLocked(std::chrono::milliseconds(2500))) {
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
  if (!IsFreshLocked(std::chrono::milliseconds(2500))) {
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
