#include "tui/history_store.hpp"

#include <algorithm>
#include <iterator>
#include <utility>

namespace swarm_ros_bridge {
namespace tui {

namespace {
bool IsTopicRow(const swarm_ros_bridge::NetworkInfo& info) {
  return info.msg_type != "transport" && info.msg_type != "presence";
}
}  // namespace

void HistoryStore::Push(std::deque<float>* series, float value) {
  series->push_back(value);
  while (series->size() > kCapacity) {
    series->pop_front();
  }
}

void HistoryStore::Sample(
    const std::vector<swarm_ros_bridge::NetworkInfo>& snapshot) {
  std::lock_guard<std::mutex> lock(mutex_);
  ++sample_count_;
  float total = 0.0F;
  for (const auto& info : snapshot) {
    if (!IsTopicRow(info)) {
      continue;
    }
    total += info.bandwidth_kbps;
    Push(&per_key_[info.name], info.bandwidth_kbps);
  }
  Push(&total_, total);
}

std::vector<float> HistoryStore::TotalBandwidthKbps() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return std::vector<float>(total_.begin(), total_.end());
}

std::vector<float> HistoryStore::BandwidthKbps(
    const std::vector<std::string>& keys) const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<float> merged;
  merged.reserve(kCapacity);
  // Merge every alias series by taking the max sample at each offset; aliases
  // refer to the same physical stream so only one of them reports at a time.
  for (const auto& key : keys) {
    const auto it = per_key_.find(key);
    if (it == per_key_.end()) {
      continue;
    }
    if (merged.empty()) {
      merged.assign(it->second.begin(), it->second.end());
      continue;
    }
    const std::size_t overlap = std::min(merged.size(), it->second.size());
    const std::size_t offset_merged = merged.size() - overlap;
    const std::size_t offset_other = it->second.size() - overlap;
    for (std::size_t i = 0; i < overlap; ++i) {
      merged[offset_merged + i] =
          std::max(merged[offset_merged + i],
                   (*std::next(it->second.begin(),
                               static_cast<long>(offset_other + i))));
    }
  }
  return merged;
}

float HistoryStore::LatestTotalKbps() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return total_.empty() ? 0.0F : total_.back();
}

}  // namespace tui
}  // namespace swarm_ros_bridge
