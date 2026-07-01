#include "tui/hosts_screen.hpp"

#include "tui/screen_common.hpp"

#include <ftxui/dom/elements.hpp>

#include <algorithm>
#include <string>
#include <vector>

namespace swarm_ros_bridge {
namespace tui {

ftxui::Element RenderHostsScreen(const config::BridgeConfig& config,
                                 const ViewState& state,
                                 ftxui::Component host_list) {
  using namespace ftxui;

  std::vector<std::pair<std::string, std::string>> hosts(config.ip_map.begin(),
                                                          config.ip_map.end());
  std::sort(hosts.begin(), hosts.end());

  Element detail = text("No host entries loaded.") | color(Color::GrayLight);
  if (!hosts.empty()) {
    const auto& host = hosts[std::min<int>(state.selected_host, hosts.size() - 1)];
    detail = vbox({
                 text(host.first) | bold | color(Color::White),
                 separator(),
                 text("Address  " + host.second),
                 text("Role     " +
                      std::string(host.first == config.hostname ? "Current node" : "Peer node")),
                 text("Scope    " +
                      std::string(host.first.find("drone") == 0 ? "Drone" : "Ground / custom")),
             }) |
             color(Color::GrayLight);
  }

  return hbox({
             Panel("Host Directory", host_list->Render() | frame | vscroll_indicator) | flex,
             Panel("Host Detail", detail) | size(WIDTH, EQUAL, 38),
         }) |
         flex;
}

}  // namespace tui
}  // namespace swarm_ros_bridge
