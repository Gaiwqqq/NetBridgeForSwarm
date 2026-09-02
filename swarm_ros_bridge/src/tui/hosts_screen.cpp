#include "tui/hosts_screen.hpp"

#include "tui/format.hpp"
#include "tui/theme.hpp"

#include <ftxui/dom/elements.hpp>

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

namespace swarm_ros_bridge {
namespace tui {

ftxui::Element HostRowElement(
    const std::string& hostname,
    const swarm_ros_bridge::NetworkInfo* presence,
    bool selected,
    bool focused,
    int name_width) {
  using namespace ftxui;
  const bool known = presence != nullptr;
  const bool online = known && presence->node_online;
  const auto state_color =
      !known ? theme::TextDim()
             : (online ? theme::Success() : theme::Error());
  const std::string dot = !known ? "? " : (online ? "● " : "○ ");
  const std::string cursor = selected ? "▸ " : (focused ? "› " : "  ");
  const auto name_color = selected
                              ? theme::Text()
                              : (focused ? theme::PrimarySoft()
                                         : theme::TextMuted());
  auto row = hbox({
      text(cursor) | color(focused ? theme::Primary() : theme::TextDim()),
      text(dot) | color(state_color),
      text(MiddleEllipsis(hostname,
                          static_cast<std::size_t>(std::max(8, name_width)))) |
          color(name_color),
      filler(),
      text(known ? FormatAge(presence->node_state_age_ms) : "") |
          color(theme::TextDim()),
      text(" "),
  });
  return selected ? row | bgcolor(theme::BackgroundSelected()) : row;
}

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

  const std::string diag_label =
      diagnostics_fresh ? "LIVE" : (has_diagnostics ? "STALE" : "WAIT");
  const auto diag_color =
      diagnostics_fresh ? theme::Success() : theme::Warning();
  const int detail_width =
      layout.compact()
          ? layout.content_width
          : (layout.wide()
                 ? std::min(48, std::max(40, layout.content_width / 3))
                 : std::max(34, layout.content_width / 2));
  const int detail_body_width = std::max(
      20, detail_width - (layout.compact() && layout.short_height ? 0 : 2));

  auto summary = hbox({
      text(" ● " + std::to_string(online_count) + " up ") | bold |
          color(theme::Success()),
      text("  ● " + std::to_string(offline_count) + " down ") | bold |
          color(theme::Error()),
      text("  ? " + std::to_string(unknown_count) + " ") |
          color(theme::TextMuted()),
      filler(),
      text("diagnostics ") | color(theme::TextMuted()),
      text(diag_label) | bold | color(diag_color),
      text(" "),
  });

  Element detail = text("No configured or discovered nodes.") |
                   color(theme::TextDim());
  Element short_detail = detail;
  if (!host_entries.empty()) {
    const std::string& hostname =
        host_entries[std::min<int>(state.selected_host,
                                   host_entries.size() - 1)];
    swarm_ros_bridge::NetworkInfo presence;
    const bool known = diagnostics_cache != nullptr &&
                       diagnostics_cache->LookupNode(hostname, &presence);
    const bool online = known && presence.node_online;
    const std::string status =
        !known ? "UNKNOWN" : (online ? "ONLINE" : "OFFLINE");
    const auto status_color =
        !known ? theme::TextMuted()
               : (online ? theme::Success() : theme::Error());
    const auto configured = config.ip_map.find(hostname);
    const bool is_configured = configured != config.ip_map.end();
    const std::string address =
        !is_configured || configured->second.empty()
            ? "Zenoh discovery"
            : configured->second;

    std::vector<Element> rows;
    if (detail_body_width <= 38) {
      rows.push_back(text(hostname) | bold | color(theme::Text()));
      rows.push_back(
          FieldRow("Status", status, status_color, detail_body_width));
    } else {
      rows.push_back(hbox({
          text(hostname) | bold | color(theme::Text()) | flex,
          Badge(status, status_color),
      }));
    }
    rows.push_back(separatorStyled(BorderStyle::LIGHT) |
                   color(theme::BorderSubtle()));
    rows.push_back(FieldRow(
        "Role", hostname == config.hostname ? "Current node" : "Peer node",
        theme::Text(), detail_body_width));
    rows.push_back(FieldRow(
        "Scope", hostname.rfind("drone", 0) == 0 ? "Drone" : "Ground / custom",
        theme::Text(), detail_body_width));
    rows.push_back(FieldRow("Configured", is_configured ? "yes" : "no",
                            is_configured ? theme::Text() : theme::TextMuted(),
                            detail_body_width));
    rows.push_back(
        FieldRow("Address", address, theme::Text(), detail_body_width));
    rows.push_back(FieldRow("Discovery", "Zenoh liveliness token",
                            theme::TextMuted(), detail_body_width));
    if (known) {
      rows.push_back(FieldRow(
          online ? "Online for" : "Offline for",
          FormatAge(presence.node_state_age_ms), theme::Text(),
          detail_body_width));
      rows.push_back(FieldRow(
          "Online count",
          std::to_string(presence.node_online_transitions), theme::Text(),
          detail_body_width));
    } else {
      rows.push_back(FieldRow(
          "Last event",
          has_diagnostics && !diagnostics_fresh
              ? "local diagnostics stale"
              : "not observed",
          theme::Warning(), detail_body_width));
    }
    rows.push_back(separatorStyled(BorderStyle::LIGHT) |
                   color(theme::BorderSubtle()));
    rows.push_back(
        WrappedText(known ? "State comes from live Zenoh PUT/DELETE events."
                          : (has_diagnostics && !diagnostics_fresh
                                 ? "Local bridge diagnostics are older than 2.5s."
                                 : "Waiting for the node's first liveliness event."),
                    detail_body_width) |
        dim | color(theme::TextDim()) | flex);
    detail = vbox(std::move(rows));

    short_detail = hbox({
        text(" " + EndEllipsis(hostname, std::max(8, layout.content_width - 32))) |
            bold | color(theme::Text()),
        filler(),
        text(status + " ") | bold | color(status_color),
        text(known ? FormatAge(presence.node_state_age_ms)
                   : (has_diagnostics && !diagnostics_fresh ? "stale"
                                                            : "not seen")) |
            color(known ? theme::TextMuted() : theme::Warning()),
        text(" "),
    });
  }

  if (layout.compact()) {
    if (layout.short_height) {
      return vbox({
          summary | bgcolor(theme::BackgroundElement()),
          host_list->Render() | frame | vscroll_indicator |
              size(HEIGHT, EQUAL, std::max(1, layout.content_height - 2)),
          short_detail | bgcolor(theme::BackgroundElement()),
      });
    }
    return vbox({
        summary | bgcolor(theme::BackgroundElement()),
        Panel("Nodes", host_list->Render() | frame | vscroll_indicator) |
            flex,
        Panel("Node Detail", detail | frame | vscroll_indicator) |
            size(HEIGHT, EQUAL, std::max(6, layout.content_height / 2)),
    });
  }

  return vbox({
      summary | bgcolor(theme::BackgroundElement()),
      hbox({
          Panel("Nodes", host_list->Render() | frame | vscroll_indicator) |
              flex,
          Panel("Node Detail", detail | frame | vscroll_indicator) |
              size(WIDTH, EQUAL, detail_width),
      }) |
          flex,
  });
}

}  // namespace tui
}  // namespace swarm_ros_bridge
