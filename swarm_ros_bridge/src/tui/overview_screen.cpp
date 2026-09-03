#include "tui/overview_screen.hpp"

#include "tui/format.hpp"
#include "tui/theme.hpp"

#include <ftxui/dom/elements.hpp>

#include <algorithm>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace swarm_ros_bridge {
namespace tui {

namespace {

struct OverviewStats {
  int unhealthy_topics{0};
  double avg_latency_ms{0.0};
  double avg_stability{100.0};
};

OverviewStats BuildOverviewStats(
    const std::shared_ptr<diagnostics::DiagnosticsCache>& diagnostics_cache) {
  OverviewStats stats;
  if (diagnostics_cache == nullptr || !diagnostics_cache->IsFresh()) {
    return stats;
  }
  const auto snapshot = diagnostics_cache->Snapshot();
  double latency_sum = 0.0;
  double stability_sum = 0.0;
  int latency_count = 0;
  int topic_count = 0;
  for (const auto& item : snapshot) {
    if (item.msg_type == "transport" || item.msg_type == "presence") {
      continue;
    }
    ++topic_count;
    if (item.stability_score < 80.0F || item.last_recv_age_ms > 3000.0F) {
      ++stats.unhealthy_topics;
    }
    stability_sum += item.stability_score;
    if (item.avg_latency_ms > 0.0F) {
      latency_sum += item.avg_latency_ms;
      ++latency_count;
    }
  }
  stats.avg_stability =
      topic_count == 0 ? 100.0 : stability_sum / topic_count;
  stats.avg_latency_ms = latency_count == 0 ? 0.0 : latency_sum / latency_count;
  return stats;
}

struct AlertRow {
  std::string name;
  std::string reason;
  float severity{0.0F};
};

std::vector<AlertRow> BuildAlerts(
    const std::shared_ptr<diagnostics::DiagnosticsCache>& diagnostics_cache) {
  std::vector<AlertRow> alerts;
  if (diagnostics_cache == nullptr || !diagnostics_cache->IsFresh()) {
    return alerts;
  }
  for (const auto& item : diagnostics_cache->Snapshot()) {
    if (item.msg_type == "transport" || item.msg_type == "presence") {
      continue;
    }
    std::vector<std::string> reasons;
    float severity = 0.0F;
    if (item.stability_score < 80.0F) {
      reasons.push_back(FormatMetric(item.stability_score, "% stability"));
      severity = std::max(severity, 100.0F - item.stability_score);
    }
    if (item.last_recv_age_ms > 3000.0F) {
      reasons.push_back("stale " +
                       FormatMetric(item.last_recv_age_ms / 1000.0F, "s"));
      severity = std::max(severity, 50.0F);
    }
    if (reasons.empty()) {
      continue;
    }
    std::string joined = reasons.front();
    for (std::size_t i = 1; i < reasons.size(); ++i) {
      joined += " · " + reasons[i];
    }
    alerts.push_back({item.name, joined, severity});
  }
  std::sort(alerts.begin(), alerts.end(),
            [](const AlertRow& a, const AlertRow& b) {
              return a.severity > b.severity;
            });
  return alerts;
}

std::string SessionShortName(const swarm_ros_bridge::NetworkInfo& info) {
  const std::string& name = info.direction.empty() ? info.name : info.direction;
  return name.empty() ? info.name : name;
}

}  // namespace

ftxui::Element RenderOverviewScreen(
    const config::BridgeConfig& config,
    const std::shared_ptr<diagnostics::DiagnosticsCache>& diagnostics_cache,
    const HistoryStore& history,
    const LayoutContext& layout) {
  using namespace ftxui;

  const bool has_diagnostics =
      diagnostics_cache != nullptr && diagnostics_cache->HasReceivedData();
  const bool diagnostics_fresh =
      diagnostics_cache != nullptr && diagnostics_cache->IsFresh();
  const auto stats = BuildOverviewStats(diagnostics_cache);
  const auto live_snapshot =
      !diagnostics_fresh ? std::vector<swarm_ros_bridge::NetworkInfo>()
                         : diagnostics_cache->Snapshot();

  std::set<std::string> node_names;
  for (const auto& host : config.ip_map) {
    if (host.first != "all" && host.first != "all_drone") {
      node_names.insert(host.first);
    }
  }
  std::map<std::string, bool> observed_nodes;
  if (diagnostics_cache != nullptr) {
    for (const auto& node : diagnostics_cache->NodeSnapshot()) {
      if (!node.node_hostname.empty()) {
        node_names.insert(node.node_hostname);
        if (diagnostics_fresh) {
          observed_nodes[node.node_hostname] = node.node_online;
        }
      }
    }
  }
  int online_nodes = 0;
  int offline_nodes = 0;
  int unknown_nodes = 0;
  for (const auto& hostname : node_names) {
    const auto observed = observed_nodes.find(hostname);
    if (observed == observed_nodes.end()) {
      ++unknown_nodes;
    } else if (observed->second) {
      ++online_nodes;
    } else {
      ++offline_nodes;
    }
  }
  const auto node_color = offline_nodes > 0
                              ? theme::Error()
                              : (unknown_nodes > 0 ? theme::Warning()
                                                   : theme::Success());
  const auto live_topic_count = std::count_if(
      live_snapshot.begin(), live_snapshot.end(),
      [](const swarm_ros_bridge::NetworkInfo& item) {
        return item.msg_type != "transport" && item.msg_type != "presence";
      });
  const auto cloud_topics =
      std::count_if(live_snapshot.begin(), live_snapshot.end(),
                    [](const swarm_ros_bridge::NetworkInfo& topic) {
                      return topic.msg_type == "sensor_msgs/PointCloud2";
                    });

  const bool waiting_for_diagnostics = live_snapshot.empty();
  const std::string bridge_state =
      waiting_for_diagnostics
          ? (has_diagnostics ? "STALE" : "WAIT")
          : (stats.unhealthy_topics == 0 ? "STABLE" : "ATTN");
  const auto bridge_color = waiting_for_diagnostics || stats.unhealthy_topics > 0
                                ? theme::Warning()
                                : theme::Success();
  const std::string latency =
      live_snapshot.empty()
          ? "--"
          : std::to_string(static_cast<int>(stats.avg_latency_ms)) + " ms";

  const auto bridge_tile = StatTile(
      "Bridge", bridge_state,
      waiting_for_diagnostics
          ? (has_diagnostics ? "diagnostics stale" : "awaiting diagnostics")
          : std::to_string(stats.unhealthy_topics) + " need attention",
      bridge_color);
  const auto topics_tile =
      StatTile("Topics", std::to_string(config.topics.size()),
               std::to_string(live_topic_count) + " live", theme::Primary());
  const auto nodes_tile = StatTile(
      "Nodes", std::to_string(online_nodes) + " / " +
                   std::to_string(node_names.size()),
      std::to_string(offline_nodes) + " down · " +
          std::to_string(unknown_nodes) + " unknown",
      node_color);
  const auto latency_tile =
      StatTile("Latency", latency, "avg receive", theme::Accent());

  const int column_width =
      layout.compact() ? layout.content_width : layout.content_width / 2;
  const int field_width = std::max(20, column_width - 2);

  // -- This node -------------------------------------------------------------
  auto this_node = Panel("This Node", vbox({
                         FieldRow("Hostname", config.hostname, theme::Primary(),
                                  field_width),
                         FieldRow("Host entries",
                                  std::to_string(config.ip_map.size()),
                                  theme::Text(), field_width),
                         FieldRow("Live topics",
                                  std::to_string(live_topic_count),
                                  theme::Text(), field_width),
                         FieldRow("Cloud links", std::to_string(cloud_topics),
                                  theme::Text(), field_width),
                         FieldRow("Monitor",
                                  std::string(config.runtime.monitor_node
                                                  ? "enabled"
                                                  : "disabled"),
                                  config.runtime.monitor_node
                                      ? theme::Success()
                                      : theme::TextMuted(),
                                  field_width),
                         FieldRow("Avg stability",
                                  FormatMetric(stats.avg_stability, "%"),
                                  theme::Text(), field_width),
                     }));

  // -- Transport sessions ----------------------------------------------------
  std::vector<Element> session_rows;
  for (const auto& item : live_snapshot) {
    if (item.msg_type != "transport") {
      continue;
    }
    const auto link_color =
        item.link_connected ? theme::Success() : theme::Error();
    if (column_width < 72) {
      session_rows.push_back(vbox({
          hbox({
              text("● ") | color(link_color),
              text(SessionShortName(item)) | bold | color(theme::Text()),
              filler(),
              text("link ") | color(theme::TextMuted()),
              text(item.link_connected ? "up" : "down") |
                  color(link_color),
              text(" "),
          }),
          hbox({
              text("  peers ") | color(theme::TextMuted()),
              text(std::to_string(item.connected_peer_count)) |
                  color(theme::Text()),
              text("  routers ") | color(theme::TextMuted()),
              text(std::to_string(item.connected_router_count)) |
                  color(theme::Text()),
              filler(),
              text("rc " + std::to_string(item.reconnect_count) +
                   "  qdrop " +
                   std::to_string(item.transport_queue_drops) + " ") |
                  color(theme::TextDim()),
          }),
      }));
    } else {
      session_rows.push_back(hbox({
          text("● ") | color(link_color),
          text(SessionShortName(item)) | bold | color(theme::Text()) |
              size(WIDTH, EQUAL, 8),
          text("link ") | color(theme::TextMuted()),
          text(item.link_connected ? "up" : "down") | color(link_color),
          text("  peers ") | color(theme::TextMuted()),
          text(std::to_string(item.connected_peer_count)) |
              color(theme::Text()),
          text("  routers ") | color(theme::TextMuted()),
          text(std::to_string(item.connected_router_count)) |
              color(theme::Text()),
          filler(),
          text("rc " + std::to_string(item.reconnect_count) + "  qdrop " +
               std::to_string(item.transport_queue_drops)) |
              color(theme::TextDim()),
      }));
    }
  }
  if (session_rows.empty()) {
    session_rows.push_back(
        text(has_diagnostics
                 ? "No transport telemetry in the latest diagnostics."
                 : "Waiting for transport telemetry...") |
        color(theme::TextDim()) | dim);
  }
  auto transport = Panel("Transport Sessions", vbox(std::move(session_rows)));

  // -- Bandwidth trend -------------------------------------------------------
  const auto bandwidth_series = history.TotalBandwidthKbps();
  float bandwidth_peak = 0.0F;
  for (const float sample : bandwidth_series) {
    bandwidth_peak = std::max(bandwidth_peak, sample);
  }
  auto bandwidth = Panel("Bandwidth Trend", vbox({
                            Sparkline(bandwidth_series, theme::Primary()) |
                                flex,
                            hbox({
                                FieldInline(
                                    "now ",
                                    FormatMetric(history.LatestTotalKbps(),
                                                 " kbps"),
                                    theme::Text()),
                                filler(),
                                FieldInline(
                                    "peak ",
                                    FormatMetric(bandwidth_peak, " kbps"),
                                    theme::TextMuted()),
                            }),
                        }) |
                   size(HEIGHT, GREATER_THAN, 7));

  // -- Alerts ----------------------------------------------------------------
  const auto alerts = BuildAlerts(diagnostics_cache);
  std::vector<Element> alert_rows;
  if (alerts.empty()) {
    alert_rows.push_back(
        hbox({text("● ") | color(theme::Success()),
              text(waiting_for_diagnostics
                       ? (has_diagnostics ? "Diagnostics are stale."
                                          : "Waiting for the first diagnostics.")
                       : "All topics are healthy.") |
                  color(waiting_for_diagnostics ? theme::Warning()
                                                : theme::TextMuted())}));
  } else {
    const int alert_name_width =
        std::max(14, std::min(28, column_width - 18));
    for (const auto& alert : alerts) {
      alert_rows.push_back(hbox({
          text("▲ ") | color(theme::Warning()),
          text(MiddleEllipsis(alert.name, alert_name_width)) |
              color(theme::Text()) |
              size(WIDTH, EQUAL, alert_name_width),
          text(" "),
          paragraph(alert.reason) | color(theme::TextMuted()) | flex,
      }));
    }
  }
  auto alerts_panel = Panel("Alerts", vbox(std::move(alert_rows)) | flex);

  if (layout.compact()) {
    if (layout.short_height) {
      return vbox({
          hbox({
              text(" " + bridge_state + " ") | bold |
                  color(bridge_color),
              text("  T " + std::to_string(config.topics.size()) + " ") |
                  color(theme::Primary()),
              filler(),
              text("N " + std::to_string(online_nodes) + "/" +
                   std::to_string(node_names.size()) + " ") |
                  color(node_color),
              text("  " + latency) | color(theme::Accent()),
          }),
          alerts_panel | flex,
      });
    }
    return vbox({
        hbox({bridge_tile, topics_tile}),
        hbox({nodes_tile, latency_tile}),
        this_node | flex,
        alerts_panel | flex,
    });
  }

  Element tiles = layout.wide()
                      ? Element(hbox({
                            bridge_tile,
                            separatorStyled(BorderStyle::LIGHT) |
                                color(theme::BorderSubtle()),
                            topics_tile,
                            separatorStyled(BorderStyle::LIGHT) |
                                color(theme::BorderSubtle()),
                            nodes_tile,
                            separatorStyled(BorderStyle::LIGHT) |
                                color(theme::BorderSubtle()),
                            latency_tile,
                        }))
                      : Element(hbox({
                            hbox({bridge_tile, topics_tile}) | flex,
                            hbox({nodes_tile, latency_tile}) | flex,
                        }));
  return vbox({
      tiles,
      hbox({this_node | flex, transport | flex}) | flex,
      hbox({bandwidth | flex, alerts_panel | flex}) | flex,
  });
}

}  // namespace tui
}  // namespace swarm_ros_bridge
