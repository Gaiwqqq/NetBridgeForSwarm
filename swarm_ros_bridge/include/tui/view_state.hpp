#ifndef SWARM_ROS_BRIDGE_TUI_VIEW_STATE_HPP_
#define SWARM_ROS_BRIDGE_TUI_VIEW_STATE_HPP_

#include <string>
#include <vector>

namespace swarm_ros_bridge {
namespace tui {

struct ViewState {
  int selected_nav{0};
  int selected_topic{0};
  int selected_host{0};
  int selected_log_level{0};
  int focused_panel{0};
  std::vector<std::string> nav_items{
      "Overview", "Topics", "Hosts", "Logs"};
  std::vector<std::string> log_levels{
      "All", "Info", "Warn", "Error"};
};

}  // namespace tui
}  // namespace swarm_ros_bridge

#endif  // SWARM_ROS_BRIDGE_TUI_VIEW_STATE_HPP_
