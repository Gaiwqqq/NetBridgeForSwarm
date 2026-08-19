#ifndef SWARM_ROS_BRIDGE_TUI_HOSTS_SCREEN_HPP_
#define SWARM_ROS_BRIDGE_TUI_HOSTS_SCREEN_HPP_

#include "config/bridge_config.hpp"
#include "diagnostics/diagnostics_cache.hpp"
#include "tui/screen_common.hpp"
#include "tui/view_state.hpp"

#include <ftxui/component/component.hpp>
#include <ftxui/dom/elements.hpp>

#include <memory>
#include <string>
#include <vector>

namespace swarm_ros_bridge {
namespace tui {

ftxui::Element RenderHostsScreen(const config::BridgeConfig& config,
                                 const ViewState& state,
                                 const std::shared_ptr<diagnostics::DiagnosticsCache>&
                                     diagnostics_cache,
                                 const std::vector<std::string>& host_entries,
                                 ftxui::Component host_list,
                                 const LayoutContext& layout);

}  // namespace tui
}  // namespace swarm_ros_bridge

#endif  // SWARM_ROS_BRIDGE_TUI_HOSTS_SCREEN_HPP_
