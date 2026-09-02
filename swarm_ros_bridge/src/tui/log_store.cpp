#include "tui/log_store.hpp"

namespace swarm_ros_bridge {
namespace tui {

namespace {
constexpr int kLevelAll = 0;
constexpr int kLevelInfo = 10;
constexpr int kLevelWarn = 20;
constexpr int kLevelError = 30;
}  // namespace

int LogStore::LevelRank(const std::string& level) {
  if (level == "INFO" || level == "Info") {
    return kLevelInfo;
  }
  if (level == "WARN" || level == "Warn") {
    return kLevelWarn;
  }
  if (level == "ERROR" || level == "Error" || level == "FATAL") {
    return kLevelError;
  }
  return kLevelAll;  // DEBUG, All and anything else.
}

void LogStore::Append(LogRecord record) {
  std::lock_guard<std::mutex> lock(mutex_);
  records_.push_back(std::move(record));
  while (records_.size() > kCapacity) {
    records_.pop_front();
  }
}

std::vector<LogRecord> LogStore::Records(const std::string& min_level) const {
  const int threshold = LevelRank(min_level);
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<LogRecord> output;
  output.reserve(records_.size());
  for (const auto& record : records_) {
    if (LevelRank(record.level) >= threshold) {
      output.push_back(record);
    }
  }
  return output;
}

std::size_t LogStore::TotalCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return records_.size();
}

}  // namespace tui
}  // namespace swarm_ros_bridge
