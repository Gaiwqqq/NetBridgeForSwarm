#ifndef SWARM_ROS_BRIDGE_TUI_SCREEN_COMMON_HPP_
#define SWARM_ROS_BRIDGE_TUI_SCREEN_COMMON_HPP_

#include <ftxui/dom/elements.hpp>

#include <string>

namespace swarm_ros_bridge {
namespace tui {

ftxui::Element Panel(const std::string& title, ftxui::Element body);
ftxui::Element KeyHint(const std::string& key, const std::string& description);
ftxui::Element MetricCard(const std::string& title,
                          const std::string& value,
                          const std::string& note,
                          const ftxui::Color& accent_color);

}  // namespace tui
}  // namespace swarm_ros_bridge

#endif  // SWARM_ROS_BRIDGE_TUI_SCREEN_COMMON_HPP_
