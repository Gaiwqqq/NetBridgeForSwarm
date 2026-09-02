#ifndef SWARM_ROS_BRIDGE_TUI_LOG_STORE_HPP_
#define SWARM_ROS_BRIDGE_TUI_LOG_STORE_HPP_

#include <cstddef>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

namespace swarm_ros_bridge {
namespace tui {

struct LogRecord {
  std::string stamp;
  std::string level;
  std::string node;
  std::string message;
};

// Ring buffer of rosout records. The ROS spinner thread appends, the render
// thread reads; every access is guarded.
class LogStore {
 public:
  void Append(LogRecord record);
  std::vector<LogRecord> Records(const std::string& min_level) const;
  std::size_t TotalCount() const;

  static int LevelRank(const std::string& level);

 private:
  mutable std::mutex mutex_;
  std::deque<LogRecord> records_;
  static constexpr std::size_t kCapacity = 500;
};

}  // namespace tui
}  // namespace swarm_ros_bridge

#endif  // SWARM_ROS_BRIDGE_TUI_LOG_STORE_HPP_
