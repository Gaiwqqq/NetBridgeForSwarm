#ifndef SWARM_ROS_BRIDGE_TUI_OVERVIEW_SCREEN_HPP_
#define SWARM_ROS_BRIDGE_TUI_OVERVIEW_SCREEN_HPP_

#include "config/bridge_config.hpp"
#include "diagnostics/diagnostics_cache.hpp"

#include <ftxui/dom/elements.hpp>
#include <memory>

namespace swarm_ros_bridge {
namespace tui {

ftxui::Element RenderOverviewScreen(
    const config::BridgeConfig& config,
    const std::shared_ptr<diagnostics::DiagnosticsCache>& diagnostics_cache);

}  // namespace tui
}  // namespace swarm_ros_bridge

#endif  // SWARM_ROS_BRIDGE_TUI_OVERVIEW_SCREEN_HPP_
