#ifndef SWARM_ROS_BRIDGE_TUI_LOGS_SCREEN_HPP_
#define SWARM_ROS_BRIDGE_TUI_LOGS_SCREEN_HPP_

#include "config/bridge_config.hpp"
#include "tui/view_state.hpp"

#include <ftxui/dom/elements.hpp>

namespace swarm_ros_bridge {
namespace tui {

ftxui::Element RenderLogsScreen(const config::BridgeConfig& config,
                                const ViewState& state);

}  // namespace tui
}  // namespace swarm_ros_bridge

#endif  // SWARM_ROS_BRIDGE_TUI_LOGS_SCREEN_HPP_
