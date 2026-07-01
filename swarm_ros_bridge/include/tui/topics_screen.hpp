#ifndef SWARM_ROS_BRIDGE_TUI_TOPICS_SCREEN_HPP_
#define SWARM_ROS_BRIDGE_TUI_TOPICS_SCREEN_HPP_

#include "config/bridge_config.hpp"
#include "diagnostics/diagnostics_cache.hpp"
#include "tui/view_state.hpp"

#include <ftxui/component/component.hpp>
#include <ftxui/dom/elements.hpp>
#include <memory>

namespace swarm_ros_bridge {
namespace tui {

ftxui::Element RenderTopicsScreen(const config::BridgeConfig& config,
                                  const ViewState& state,
                                  const std::shared_ptr<diagnostics::DiagnosticsCache>& diagnostics_cache,
                                  ftxui::Component topic_list);

}  // namespace tui
}  // namespace swarm_ros_bridge

#endif  // SWARM_ROS_BRIDGE_TUI_TOPICS_SCREEN_HPP_
