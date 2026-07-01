#include "tui/overview_screen.hpp"

#include "tui/screen_common.hpp"

#include <ftxui/dom/elements.hpp>

#include <algorithm>
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

  const auto snapshot = diagnostics_cache->Snapshot();
  if (snapshot.empty()) {
    return stats;
  }

  double latency_sum = 0.0;
  double stability_sum = 0.0;
  int latency_count = 0;
  for (const auto& item : snapshot) {
    if (item.stability_score < 80.0F || item.last_recv_age_ms > 3000.0F) {
      ++stats.unhealthy_topics;
    }
    stability_sum += item.stability_score;
    if (item.avg_latency_ms > 0.0F) {
      latency_sum += item.avg_latency_ms;
      ++latency_count;
    }
  }
  stats.avg_stability = stability_sum / snapshot.size();
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
    const std::shared_ptr<diagnostics::DiagnosticsCache>& diagnostics_cache) {
  using namespace ftxui;
  const auto stats = BuildOverviewStats(diagnostics_cache);
  const auto live_snapshot =
      diagnostics_cache == nullptr ? std::vector<swarm_ros_bridge::NetworkInfo>()
                                   : diagnostics_cache->Snapshot();

  const auto cloud_topics = std::count_if(
      config.topics.begin(), config.topics.end(), [](const config::TopicRule& topic) {
        return topic.msg_type == "sensor_msgs/PointCloud2";
      });
  auto cards = hbox({
                   MetricCard("Bridge State",
                              stats.unhealthy_topics == 0 ? "STABLE" : "ATTN",
                              live_snapshot.empty()
                                  ? "Waiting for live diagnostics"
                                  : std::to_string(stats.unhealthy_topics) +
                                        " topic(s) need attention",
                              stats.unhealthy_topics == 0 ? Color::GreenLight
                                                          : Color::YellowLight),
                   MetricCard("Topics", std::to_string(config.topics.size()),
                              "Configured forwarding rules", Color::CyanLight),
                   MetricCard("Cloud Links", std::to_string(cloud_topics),
                              "Draco migration target set", Color::YellowLight),
                   MetricCard("Latency",
                              live_snapshot.empty()
                                  ? "--"
                                  : std::to_string(static_cast<int>(stats.avg_latency_ms)) + " ms",
                              "Average receive latency", Color::Magenta1),
               });

  auto spotlight = Panel(
      "Spotlight",
      vbox({
          text("Hostname: " + config.hostname) | color(Color::White),
          text("Host entries: " + std::to_string(config.ip_map.size())) |
              color(Color::GrayLight),
          text("Live topics: " + std::to_string(live_snapshot.size())) |
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
          text("1. Review live jitter/stability in Topic Matrix."),
          text("2. Validate Draco cloud links against receiver timestamp."),
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

  return vbox({
             cards,
             hbox({
                 spotlight | flex,
                 vbox({
                     next_steps,
                     shortcuts,
                 }) |
                     flex,
             }) |
                 flex,
         }) |
         flex;
}

}  // namespace tui
}  // namespace swarm_ros_bridge
