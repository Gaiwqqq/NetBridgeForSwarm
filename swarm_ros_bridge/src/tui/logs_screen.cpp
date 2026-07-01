#include "tui/logs_screen.hpp"

#include "tui/screen_common.hpp"

#include <ftxui/dom/elements.hpp>

#include <string>
#include <vector>

namespace swarm_ros_bridge {
namespace tui {

namespace {

struct LogLine {
  std::string level;
  std::string message;
  ftxui::Color color;
};

std::vector<LogLine> BuildPreviewLogs(const config::BridgeConfig& config) {
  std::vector<LogLine> logs;
  logs.push_back({"INFO", "✅ bridge config loaded for host " + config.hostname,
                  ftxui::Color::GreenLight});
  logs.push_back({"INFO",
                  "🛰 " + std::to_string(config.topics.size()) +
                      " topic rules are ready for inspection",
                  ftxui::Color::CyanLight});
  logs.push_back({"WARN",
                  "⚠ watch latency, jitter and stability in the live topic matrix",
                  ftxui::Color::YellowLight});
  logs.push_back({"INFO", "📦 vendored libraries: FTXUI, Draco, pic_sockets",
                  ftxui::Color::Magenta1});
  return logs;
}

}  // namespace

ftxui::Element RenderLogsScreen(const config::BridgeConfig& config,
                                const ViewState& state) {
  using namespace ftxui;

  const auto logs = BuildPreviewLogs(config);
  std::vector<Element> lines;
  for (const auto& log : logs) {
    if (state.selected_log_level == 1 && log.level != "INFO") continue;
    if (state.selected_log_level == 2 && log.level != "WARN") continue;
    if (state.selected_log_level == 3 && log.level != "ERROR") continue;

    lines.push_back(hbox({
        text(" " + log.level + " ") | bold | bgcolor(log.color) | color(Color::Black),
        text(" " + log.message) | color(Color::GrayLight),
    }));
  }

  std::vector<Element> filter_chips;
  for (std::size_t i = 0; i < state.log_levels.size(); ++i) {
    const bool active = static_cast<int>(i) == state.selected_log_level;
    filter_chips.push_back(
        text(" " + state.log_levels[i] + " ") |
        (active ? bgcolor(Color::SkyBlue1) | color(Color::Black)
                : color(Color::GrayLight)));
  }

  return vbox({
             Panel("Filters", hbox(filter_chips)),
             Panel("Log Preview",
                   vbox(lines.empty() ? std::vector<Element>{text("No logs in this filter.")}
                                      : lines) |
                       frame | vscroll_indicator) |
                 flex,
         }) |
         flex;
}

}  // namespace tui
}  // namespace swarm_ros_bridge
