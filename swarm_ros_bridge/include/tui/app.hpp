#ifndef SWARM_ROS_BRIDGE_TUI_APP_HPP_
#define SWARM_ROS_BRIDGE_TUI_APP_HPP_

#include "config/bridge_config.hpp"
#include "diagnostics/diagnostics_cache.hpp"
#include "tui/view_state.hpp"

#include <ftxui/component/component.hpp>
#include <memory>
#include <vector>

namespace swarm_ros_bridge {
namespace tui {

class App {
 public:
  App(config::BridgeConfig config,
      std::shared_ptr<diagnostics::DiagnosticsCache> diagnostics_cache);
  int Run();

 private:
  void BuildTopicEntries();
  void BuildHostEntries();

  config::BridgeConfig config_;
  std::shared_ptr<diagnostics::DiagnosticsCache> diagnostics_cache_;
  ViewState state_;
  std::vector<std::string> host_entries_;
  std::vector<std::string> topic_entries_;
};

}  // namespace tui
}  // namespace swarm_ros_bridge

#endif  // SWARM_ROS_BRIDGE_TUI_APP_HPP_
