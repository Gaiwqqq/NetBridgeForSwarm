#ifndef SWARM_ROS_BRIDGE_TUI_TOPICS_SCREEN_HPP_
#define SWARM_ROS_BRIDGE_TUI_TOPICS_SCREEN_HPP_

#include "config/bridge_config.hpp"
#include "diagnostics/diagnostics_cache.hpp"
#include "tui/history_store.hpp"
#include "tui/screen_common.hpp"
#include "tui/view_state.hpp"

#include <ftxui/component/component.hpp>
#include <ftxui/dom/elements.hpp>

#include <memory>
#include <string>
#include <vector>

namespace swarm_ros_bridge {
namespace tui {

struct TopicPaneGeometry {
  bool split{false};
  bool detailed_columns{false};
  int matrix_width{80};
  int matrix_inner_width{78};
  int inspector_width{80};
  int name_width{29};
};

TopicPaneGeometry MakeTopicPaneGeometry(const LayoutContext& layout);

std::vector<std::string> TopicAliases(const config::TopicRule& topic);

// One row of the topic matrix, shared by the interactive list and the tests.
// `live_info` may be null when no live sample exists yet.
ftxui::Element TopicRowElement(const config::TopicRule& topic,
                               const swarm_ros_bridge::NetworkInfo* live_info,
                               bool highlighted,
                               int name_width,
                               bool detailed);

ftxui::Element RenderTopicsScreen(
    const config::BridgeConfig& config,
    const ViewState& state,
    const std::shared_ptr<diagnostics::DiagnosticsCache>& diagnostics_cache,
    const HistoryStore& history,
    ftxui::Component topic_list,
    const LayoutContext& layout);

}  // namespace tui
}  // namespace swarm_ros_bridge

#endif  // SWARM_ROS_BRIDGE_TUI_TOPICS_SCREEN_HPP_
