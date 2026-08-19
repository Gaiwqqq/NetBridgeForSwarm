#include "tui/hosts_screen.hpp"

#include "tui/screen_common.hpp"

#include <ftxui/dom/elements.hpp>

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

namespace swarm_ros_bridge {
namespace tui {

namespace {

std::string FormatAge(std::uint64_t milliseconds) {
  const std::uint64_t seconds = milliseconds / 1000U;
  if (seconds < 1U) {
    return "<1s";
  }
  if (seconds < 60U) {
    return std::to_string(seconds) + "s";
  }
  const std::uint64_t minutes = seconds / 60U;
  if (minutes < 60U) {
    return std::to_string(minutes) + "m " +
           std::to_string(seconds % 60U) + "s";
  }
  const std::uint64_t hours = minutes / 60U;
  return std::to_string(hours) + "h " +
         std::to_string(minutes % 60U) + "m";
}

}  // namespace

ftxui::Element RenderHostsScreen(const config::BridgeConfig& config,
                                 const ViewState& state,
                                 const std::shared_ptr<diagnostics::DiagnosticsCache>&
                                     diagnostics_cache,
                                 const std::vector<std::string>& host_entries,
                                 ftxui::Component host_list,
                                 const LayoutContext& layout) {
  using namespace ftxui;
  const bool has_diagnostics =
      diagnostics_cache != nullptr && diagnostics_cache->HasReceivedData();
  const bool diagnostics_fresh =
      diagnostics_cache != nullptr && diagnostics_cache->IsFresh();

  int online_count = 0;
  int offline_count = 0;
  int unknown_count = 0;
  for (const auto& hostname : host_entries) {
    swarm_ros_bridge::NetworkInfo presence;
    if (diagnostics_cache == nullptr ||
        !diagnostics_cache->LookupNode(hostname, &presence)) {
      ++unknown_count;
    } else if (presence.node_online) {
      ++online_count;
    } else {
      ++offline_count;
    }
  }

  Element summary;
  if (layout.compact()) {
    summary = hbox({
        text(" UP " + std::to_string(online_count) + " ") | bold |
            color(Color::Black) | bgcolor(Color::GreenLight),
        text(" DOWN " + std::to_string(offline_count) + " ") | bold |
            color(Color::White) | bgcolor(Color::Red),
        text(" ? " + std::to_string(unknown_count) + " ") |
            color(Color::GrayLight),
        filler(),
        text(diagnostics_fresh ? " LIVE "
                               : (has_diagnostics ? " STALE " : " WAIT ")) |
            bold |
            color(diagnostics_fresh ? Color::GreenLight : Color::YellowLight),
    });
  } else {
    summary = hbox({
        text(" ONLINE " + std::to_string(online_count) + " ") | bold |
            color(Color::Black) | bgcolor(Color::GreenLight),
        text("  OFFLINE " + std::to_string(offline_count) + " ") | bold |
            color(Color::White) | bgcolor(Color::Red),
        text("  UNKNOWN " + std::to_string(unknown_count) + " ") |
            color(Color::GrayLight),
        filler(),
        text(diagnostics_fresh
                 ? " DIAGNOSTICS LIVE "
                 : (has_diagnostics ? " DIAGNOSTICS STALE "
                                    : " DIAGNOSTICS WAIT ")) |
            bold |
            color(diagnostics_fresh ? Color::GreenLight : Color::YellowLight),
    });
  }

  Element detail = text("No configured or discovered nodes.") |
                   color(Color::GrayLight);
  Element compact_detail = detail;
  Element short_detail = detail;
  if (!host_entries.empty()) {
    const std::string& hostname =
        host_entries[std::min<int>(state.selected_host,
                                   host_entries.size() - 1)];
    swarm_ros_bridge::NetworkInfo presence;
    const bool known = diagnostics_cache != nullptr &&
                       diagnostics_cache->LookupNode(hostname, &presence);
    const bool online = known && presence.node_online;
    const std::string status = !known ? "UNKNOWN" : (online ? "ONLINE" : "OFFLINE");
    const auto status_color = !known ? Color::GrayLight
                                     : (online ? Color::GreenLight : Color::RedLight);
    const auto configured = config.ip_map.find(hostname);
    const bool is_configured = configured != config.ip_map.end();
    const std::string address =
        !is_configured || configured->second.empty()
            ? "Zenoh discovery"
            : configured->second;

    std::vector<Element> rows{
        text(hostname) | bold | color(Color::White),
        text(" " + status + " ") | bold | color(Color::Black) |
            bgcolor(status_color),
        separator(),
        text("Role        " +
             std::string(hostname == config.hostname ? "Current node"
                                                      : "Peer node")),
        text("Scope       " +
             std::string(hostname.find("drone") == 0 ? "Drone"
                                                      : "Ground / custom")),
        text("Configured  " + std::string(is_configured ? "yes" : "no")),
        text("Address     " + address),
        text("Discovery   Zenoh liveliness token"),
    };
    if (known) {
      rows.push_back(text(std::string(online ? "Online for  " : "Offline for ") +
                          FormatAge(presence.node_state_age_ms)));
      rows.push_back(text("Online count " +
                          std::to_string(presence.node_online_transitions)));
    } else {
      rows.push_back(
          text(has_diagnostics && !diagnostics_fresh
                   ? "Last event   local diagnostics stale"
                   : "Last event   not observed") |
          color(Color::YellowLight));
    }
    detail = vbox({
                 vbox(std::move(rows)),
                 separator(),
                 text(known
                          ? "State comes from live Zenoh PUT/DELETE events."
                          : (has_diagnostics && !diagnostics_fresh
                                 ? "Local bridge diagnostics are older than 2.5s."
                                 : "Waiting for the node's first liveliness event.")) |
                     dim | color(Color::GrayDark),
             }) |
             color(Color::GrayLight);
    compact_detail = vbox({
        paragraph(hostname) | bold | color(Color::White),
        hbox({
            text(" " + status + " ") | bold | color(Color::Black) |
                bgcolor(status_color),
            text("  "),
            text(hostname == config.hostname ? "current" : "peer") |
                color(Color::GrayLight),
            filler(),
            text(known
                     ? FormatAge(presence.node_state_age_ms)
                     : (has_diagnostics && !diagnostics_fresh ? "stale"
                                                              : "not seen")) |
                color(known ? Color::White : Color::YellowLight),
        }),
    });
    short_detail = hbox({
        text(" " + hostname) | bold | color(Color::White),
        filler(),
        text(status + " ") | bold | color(status_color),
        text(known
                 ? FormatAge(presence.node_state_age_ms)
                 : (has_diagnostics && !diagnostics_fresh ? "stale"
                                                          : "not seen")) |
            color(known ? Color::White : Color::YellowLight),
        text(" "),
    });
  }

  if (layout.compact()) {
    if (layout.short_height) {
      return vbox({
                 summary | bgcolor(Color::RGB(18, 23, 36)),
                 host_list->Render() | frame | vscroll_indicator |
                     size(HEIGHT, EQUAL,
                          std::max(1, layout.content_height - 2)),
                 short_detail |
                     size(HEIGHT, EQUAL, 1) |
                     bgcolor(Color::RGB(18, 23, 36)),
             }) |
             flex;
    }
    return vbox({
               summary | bgcolor(Color::RGB(18, 23, 36)),
               Panel("Nodes",
                     host_list->Render() | frame | vscroll_indicator) |
                   flex,
               Panel("Selected", compact_detail) |
                   size(HEIGHT, EQUAL, 6),
           }) |
           flex;
  }

  const int detail_width =
      layout.wide() ? std::min(48, std::max(40, layout.content_width / 3))
                    : std::max(34, layout.content_width / 2);
  return vbox({
             Panel("Network Presence", summary),
             hbox({
                 Panel("Nodes", host_list->Render() | frame |
                                    vscroll_indicator) |
                     flex,
                 Panel("Node Detail", detail | frame | vscroll_indicator) |
                     size(WIDTH, EQUAL, detail_width),
             }) |
                 flex,
         }) |
         flex;
}

}  // namespace tui
}  // namespace swarm_ros_bridge
