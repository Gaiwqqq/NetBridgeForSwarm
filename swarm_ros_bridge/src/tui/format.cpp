#include "tui/format.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <iomanip>
#include <sstream>
#include <ctime>

namespace swarm_ros_bridge {
namespace tui {

std::string MiddleEllipsis(const std::string& text, std::size_t max_width) {
  if (text.size() <= max_width || max_width < 7) {
    return text;
  }
  const std::size_t head = (max_width - 3) / 2;
  const std::size_t tail = max_width - 3 - head;
  return text.substr(0, head) + "..." + text.substr(text.size() - tail);
}

std::string EndEllipsis(const std::string& value, int max_width) {
  if (max_width <= 3 || static_cast<int>(value.size()) <= max_width) {
    return value;
  }
  return value.substr(0, static_cast<std::size_t>(max_width - 3)) + "...";
}

std::string JoinHosts(const std::vector<std::string>& hosts) {
  if (hosts.empty()) {
    return "-";
  }
  std::string joined = hosts.front();
  for (std::size_t i = 1; i < hosts.size(); ++i) {
    joined += ", " + hosts[i];
  }
  return joined;
}

std::string FormatMetric(double value, const std::string& suffix) {
  if (value < 0.0) {
    return "--";
  }
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(value >= 100.0 ? 0 : 1);
  stream << value << suffix;
  return stream.str();
}

std::string FormatAge(std::uint64_t milliseconds) {
  const std::uint64_t seconds = milliseconds / 1000U;
  if (seconds < 1U) {
    return "<1s";
  }
  if (seconds < 60U) {
    return std::to_string(seconds) + "s";
  }
  const std::uint64_t minutes = seconds / 60U;
  if (minutes < 60U) {
    return std::to_string(minutes) + "m " +
           std::to_string(seconds % 60U) + "s";
  }
  const std::uint64_t hours = minutes / 60U;
  return std::to_string(hours) + "h " + std::to_string(minutes % 60U) + "m";
}

std::string FormatClockNow() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm local{};
  localtime_r(&time, &local);
  std::ostringstream stream;
  stream << std::setfill('0') << std::setw(2) << local.tm_hour << ":"
         << std::setw(2) << local.tm_min;
  return stream.str();
}

}  // namespace tui
}  // namespace swarm_ros_bridge
