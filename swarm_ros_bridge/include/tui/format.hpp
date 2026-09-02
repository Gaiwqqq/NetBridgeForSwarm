#ifndef SWARM_ROS_BRIDGE_TUI_FORMAT_HPP_
#define SWARM_ROS_BRIDGE_TUI_FORMAT_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace swarm_ros_bridge {
namespace tui {

std::string MiddleEllipsis(const std::string& text, std::size_t max_width);
std::string EndEllipsis(const std::string& value, int max_width);
std::string JoinHosts(const std::vector<std::string>& hosts);
std::string FormatMetric(double value, const std::string& suffix);
std::string FormatAge(std::uint64_t milliseconds);
std::string FormatClockNow();

}  // namespace tui
}  // namespace swarm_ros_bridge

#endif  // SWARM_ROS_BRIDGE_TUI_FORMAT_HPP_
