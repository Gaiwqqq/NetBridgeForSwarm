#ifndef SWARM_ROS_BRIDGE_TUI_LOGS_SCREEN_HPP_
#define SWARM_ROS_BRIDGE_TUI_LOGS_SCREEN_HPP_

#include "tui/log_store.hpp"
#include "tui/screen_common.hpp"
#include "tui/view_state.hpp"

#include <ftxui/component/component.hpp>
#include <ftxui/dom/elements.hpp>

namespace swarm_ros_bridge {
namespace tui {

ftxui::Element RenderLogsScreen(const ViewState& state,
                                const LogStore& log_store,
                                ftxui::Component level_filter,
                                const LayoutContext& layout);

}  // namespace tui
}  // namespace swarm_ros_bridge

#endif  // SWARM_ROS_BRIDGE_TUI_LOGS_SCREEN_HPP_
