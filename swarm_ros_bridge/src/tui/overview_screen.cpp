#include "tui/overview_screen.hpp"

#include "tui/screen_common.hpp"

#include <ftxui/dom/elements.hpp>

#include <algorithm>
#include <map>
#include <set>
#include <string>

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
  if (diagnostics_cache == nullptr) {
    return stats;
  }
  if (!diagnostics_cache->IsFresh()) {
    return stats;
  }

  const auto snapshot = diagnostics_cache->Snapshot();
  if (snapshot.empty()) {
    return stats;
  }

  double latency_sum = 0.0;
  double stability_sum = 0.0;
  int latency_count = 0;
  int topic_count = 0;
  for (const auto& item : snapshot) {
    // The synthetic Zenoh session row has link health rather than topic-rate
    // health. Counting it here would permanently depress the topic score.
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

}  // namespace

ftxui::Element Panel(const std::string& title, ftxui::Element body) {
  using namespace ftxui;
  return vbox({
             text(title) | bold | color(Color::CyanLight),
             separator(),
             std::move(body),
         }) |
         borderRounded | bgcolor(Color::RGB(14, 18, 30));
}

ftxui::Element KeyHint(const std::string& key, const std::string& description) {
  using namespace ftxui;
  return hbox({
      text(" " + key + " ") | bgcolor(Color::SkyBlue1) | color(Color::Black),
      text(" " + description) | color(Color::GrayLight),
  });
}

ftxui::Element MetricCard(const std::string& title,
                          const std::string& value,
                          const std::string& note,
                          const ftxui::Color& accent_color) {
  using namespace ftxui;
  return vbox({
             text(title) | color(Color::GrayLight),
             text(value) | bold | color(accent_color),
             text(note) | dim | color(Color::GrayDark),
         }) |
         size(WIDTH, GREATER_THAN, 20) | borderRounded |
         bgcolor(Color::RGB(22, 28, 44));
}

ftxui::Element RenderOverviewScreen(
    const config::BridgeConfig& config,
    const std::shared_ptr<diagnostics::DiagnosticsCache>& diagnostics_cache,
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
                              ? Color::RedLight
                              : (unknown_nodes > 0 ? Color::YellowLight
                                                   : Color::GreenLight);
  const auto live_topic_count = std::count_if(
      live_snapshot.begin(), live_snapshot.end(),
      [](const swarm_ros_bridge::NetworkInfo& item) {
        return item.msg_type != "transport" && item.msg_type != "presence";
      });

  const auto cloud_topics = std::count_if(
      config.topics.begin(), config.topics.end(), [](const config::TopicRule& topic) {
        return topic.msg_type == "sensor_msgs/PointCloud2";
      });
  const bool waiting_for_diagnostics = live_snapshot.empty();
  const std::string bridge_state =
      waiting_for_diagnostics
          ? (has_diagnostics ? "STALE" : "WAIT")
          : (stats.unhealthy_topics == 0 ? "STABLE" : "ATTN");
  const auto bridge_color =
      waiting_for_diagnostics || stats.unhealthy_topics > 0
          ? Color::YellowLight
          : Color::GreenLight;
  const std::string latency =
      live_snapshot.empty()
          ? "--"
          : std::to_string(static_cast<int>(stats.avg_latency_ms)) + " ms";
  const auto bridge_card = MetricCard(
      "Bridge State", bridge_state,
      waiting_for_diagnostics
          ? (has_diagnostics ? "Diagnostics older than 2.5 seconds"
                             : "Waiting for live diagnostics")
          : std::to_string(stats.unhealthy_topics) +
                " topic(s) need attention",
      bridge_color);
  const auto topics_card = MetricCard(
      "Topics", std::to_string(config.topics.size()),
      "Configured forwarding rules", Color::CyanLight);
  const auto nodes_card = MetricCard(
      "Nodes",
      std::to_string(online_nodes) + " / " + std::to_string(node_names.size()),
      std::to_string(offline_nodes) + " offline, " +
          std::to_string(unknown_nodes) + " unknown",
      node_color);
  const auto latency_card = MetricCard(
      "Latency", latency, "Average receive latency", Color::Magenta1);

  Element cards;
  if (layout.wide()) {
    cards = hbox({bridge_card | flex, topics_card | flex,
                  nodes_card | flex, latency_card | flex});
  } else if (layout.medium()) {
    cards = vbox({
        hbox({bridge_card | flex, topics_card | flex}),
        hbox({nodes_card | flex, latency_card | flex}),
    });
  } else {
    cards = vbox({
                hbox({
                    text(" Bridge ") | color(Color::GrayLight),
                    text(bridge_state) | bold | color(bridge_color),
                    filler(),
                    text("Nodes ") | color(Color::GrayLight),
                    text(std::to_string(online_nodes) + "/" +
                         std::to_string(node_names.size())) |
                        bold | color(node_color),
                    text(" "),
                }),
                hbox({
                    text(" Topics ") | color(Color::GrayLight),
                    text(std::to_string(config.topics.size())) | bold |
                        color(Color::CyanLight),
                    filler(),
                    text("Latency ") | color(Color::GrayLight),
                    text(latency) | color(Color::Magenta1),
                    text(" "),
                }),
            }) |
            borderRounded | bgcolor(Color::RGB(22, 28, 44));
  }

  auto spotlight = Panel(
      "Spotlight",
      vbox({
          paragraph("Hostname: " + config.hostname) | color(Color::White),
          text("Host entries: " + std::to_string(config.ip_map.size())) |
              color(Color::GrayLight),
          text("Live topics: " + std::to_string(live_topic_count)) |
              color(Color::GrayLight),
          text("Cloud links: " + std::to_string(cloud_topics)) |
              color(Color::GrayLight),
          text("Monitor: " +
               std::string(config.runtime.monitor_node ? "enabled" : "disabled")) |
              color(Color::GrayLight),
          text("Avg stability: " + std::to_string(static_cast<int>(stats.avg_stability)) + "%") |
              color(Color::GrayLight),
      }));

  auto next_steps = Panel(
      "Next Steps",
      vbox({
          text("1. Check ONLINE/OFFLINE state in the Nodes view."),
          text("2. Review live jitter/stability in Topic Matrix."),
          text("3. Watch logs for stale topics or drop spikes."),
      }) |
          color(Color::GrayLight));

  auto shortcuts = Panel(
      "Quick Hints",
      vbox({
          KeyHint("q", "Exit the TUI"),
          KeyHint("↑/↓", "Move through navigation"),
          KeyHint("Tab", "Switch focus when forms land"),
      }));

  if (layout.wide()) {
    return vbox({
               cards,
               hbox({
                   spotlight | flex,
                   vbox({next_steps, shortcuts}) | flex,
               }) |
                   flex,
           }) |
           flex;
  }
  if (layout.compact()) {
    std::vector<Element> sections{cards};
    sections.push_back(spotlight | flex);
    if (!layout.short_height) {
      sections.push_back(Panel(
          "Hints",
          hbox({KeyHint("q", "Exit"), text("  "),
                KeyHint("arrows", "Move")})));
    }
    return vbox(std::move(sections)) | flex;
  }
  return vbox({
             cards,
             hbox({spotlight | flex, next_steps | flex}) | flex,
         }) |
         flex;
}

}  // namespace tui
}  // namespace swarm_ros_bridge
