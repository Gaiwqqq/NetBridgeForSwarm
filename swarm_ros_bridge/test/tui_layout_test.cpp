#include "tui/hosts_screen.hpp"
#include "tui/history_store.hpp"
#include "tui/log_store.hpp"
#include "tui/logs_screen.hpp"
#include "tui/overview_screen.hpp"
#include "tui/screen_common.hpp"
#include "tui/theme.hpp"
#include "tui/topics_screen.hpp"

#include <ftxui/component/component.hpp>
#include <ftxui/dom/node.hpp>
#include <ftxui/screen/screen.hpp>

#include <cassert>
#include <cstddef>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using swarm_ros_bridge::tui::LayoutMode;
using swarm_ros_bridge::tui::MakeLayoutContext;

namespace {

ftxui::Screen RenderToScreen(const ftxui::Element& element,
                             int width,
                             int height,
                             const std::string& context) {
  assert(element != nullptr);
  ftxui::Screen screen(width, height);
  ftxui::Render(screen, element);
  const std::string text = screen.ToString();
  assert(!text.empty());
  (void)context;
  return screen;
}

int FindLine(const std::string& content, const std::string& needle) {
  std::istringstream stream(content);
  std::string line;
  int index = 0;
  while (std::getline(stream, line)) {
    if (line.find(needle) != std::string::npos) {
      return index;
    }
    ++index;
  }
  return -1;
}

}  // namespace

int main() {
  // -- responsive breakpoints (top tabs only, no sidebar) --------------------
  const auto tiny = MakeLayoutContext(40, 12);
  assert(tiny.mode == LayoutMode::kCompact);
  assert(tiny.short_height);
  assert(tiny.content_width == 40 && tiny.content_height == 7);

  const auto compact = MakeLayoutContext(48, 16);
  assert(compact.mode == LayoutMode::kCompact);
  assert(compact.content_height == 11);

  const auto medium = MakeLayoutContext(80, 24);
  assert(medium.mode == LayoutMode::kMedium);
  assert(medium.content_width == 80 && medium.content_height == 19);

  const auto wide = MakeLayoutContext(120, 30);
  assert(wide.mode == LayoutMode::kWide);
  assert(wide.content_height == 25);

  const auto large = MakeLayoutContext(160, 45);
  assert(large.mode == LayoutMode::kWide);
  assert(large.content_width == 160 && large.content_height == 40);

  const auto fallback = MakeLayoutContext(0, 0);
  assert(fallback.terminal_width == 80 && fallback.terminal_height == 24);

  // -- topic pane geometry ----------------------------------------------------
  const auto tiny_topics =
      swarm_ros_bridge::tui::MakeTopicPaneGeometry(tiny);
  assert(!tiny_topics.split && !tiny_topics.detailed_columns);
  assert(tiny_topics.matrix_width == 40 && tiny_topics.name_width == 23);

  const auto medium_topics =
      swarm_ros_bridge::tui::MakeTopicPaneGeometry(medium);
  assert(!medium_topics.split && medium_topics.detailed_columns);
  assert(medium_topics.matrix_width == 80 && medium_topics.name_width == 29);

  const auto almost_split = swarm_ros_bridge::tui::MakeTopicPaneGeometry(
      MakeLayoutContext(119, 30));
  assert(!almost_split.split && almost_split.matrix_width == 119);

  const auto screenshot_layout = MakeLayoutContext(120, 37);
  const auto screenshot_topics =
      swarm_ros_bridge::tui::MakeTopicPaneGeometry(screenshot_layout);
  assert(screenshot_topics.split && screenshot_topics.detailed_columns);
  assert(screenshot_topics.matrix_width == 80);
  assert(screenshot_topics.inspector_width == 40);
  assert(screenshot_topics.matrix_inner_width == 78);
  assert(screenshot_topics.name_width == 29);

  const auto large_topics =
      swarm_ros_bridge::tui::MakeTopicPaneGeometry(large);
  assert(large_topics.split && large_topics.inspector_width == 48);
  assert(large_topics.matrix_width == 112 && large_topics.name_width == 61);

  // Guard the inclusive-box math used by the vendored FTXUI size decorator.
  auto constrained = ftxui::text("abcdefghijkl") |
                     ftxui::size(ftxui::WIDTH, ftxui::EQUAL, 8);
  ftxui::Screen screen(12, 1);
  ftxui::Render(screen, constrained);
  assert(screen.ToString() == "abcdefgh    ");

  // -- theme primitives render ------------------------------------------------
  RenderToScreen(swarm_ros_bridge::tui::Badge("LIVE",
                                              swarm_ros_bridge::tui::theme::Success()),
                 12, 1, "badge");
  RenderToScreen(swarm_ros_bridge::tui::StatTile("Bridge", "STABLE", "ok",
                                                 swarm_ros_bridge::tui::theme::Success()),
                 24, 5, "stat tile");
  RenderToScreen(swarm_ros_bridge::tui::Panel("Panel", ftxui::text("body")),
                 24, 6, "panel");
  auto fill_panel = RenderToScreen(
      swarm_ros_bridge::tui::Panel(
          "Panel", ftxui::vbox({ftxui::text("top"), ftxui::filler(),
                                ftxui::text("bottom")})),
      24, 8, "panel body fills height");
  assert(FindLine(fill_panel.ToString(), "bottom") == 6);

  auto equal_tiles = RenderToScreen(
      ftxui::hbox({
          swarm_ros_bridge::tui::StatTile(
              "A", "1", "ok", swarm_ros_bridge::tui::theme::Primary()),
          swarm_ros_bridge::tui::StatTile(
              "B", "2", "ok", swarm_ros_bridge::tui::theme::Success()),
      }),
      40, 5, "equal full-background stat tiles");
  assert(equal_tiles.CellAt(1, 2).background_color ==
         swarm_ros_bridge::tui::theme::BackgroundElement());
  assert(equal_tiles.CellAt(38, 2).background_color ==
         swarm_ros_bridge::tui::theme::BackgroundElement());

  const auto empty_trend =
      RenderToScreen(swarm_ros_bridge::tui::Sparkline(
                         {}, swarm_ros_bridge::tui::theme::Primary()),
                     30, 3, "empty trend")
          .ToString();
  assert(empty_trend.find("Waiting for samples") != std::string::npos);
  const auto zero_trend =
      RenderToScreen(swarm_ros_bridge::tui::Sparkline(
                         {0.0F, 0.0F, 0.0F},
                         swarm_ros_bridge::tui::theme::Primary()),
                     30, 3, "zero trend")
          .ToString();
  assert(zero_trend.find("No traffic in window") != std::string::npos);
  RenderToScreen(swarm_ros_bridge::tui::Sparkline({0.0F, 100.0F, 50.0F},
                                                  swarm_ros_bridge::tui::theme::Primary()),
                 20, 3, "sparkline");
  RenderToScreen(swarm_ros_bridge::tui::Bar(0.5F,
                                            swarm_ros_bridge::tui::theme::Primary()),
                 10, 1, "bar");
  const auto zero_bar =
      RenderToScreen(swarm_ros_bridge::tui::Bar(
                         0.0F, swarm_ros_bridge::tui::theme::Primary()),
                     10, 1, "zero bar")
          .ToString();
  assert(zero_bar.find("█") == std::string::npos);
  const auto wrapped_field =
      RenderToScreen(swarm_ros_bridge::tui::FieldRow(
                         "Discovery", "Zenoh liveliness token",
                         swarm_ros_bridge::tui::theme::Text(), 38),
                     38, 2, "wrapped inspector field")
          .ToString();
  assert(wrapped_field.find("Zenoh liveliness") != std::string::npos);
  assert(wrapped_field.find("token") != std::string::npos);
  RenderToScreen(swarm_ros_bridge::tui::KeyHint("q", "quit"), 20, 1, "key hint");

  // -- topic row renders with and without live info ---------------------------
  swarm_ros_bridge::config::TopicRule rule;
  rule.topic_name = "/camera/image";
  RenderToScreen(swarm_ros_bridge::tui::TopicRowElement(rule, nullptr, false, 24,
                                                        true),
                 60, 1, "topic row without live info");
  swarm_ros_bridge::NetworkInfo live;
  live.bandwidth_kbps = 900.0F;
  live.stability_score = 95.0F;
  live.dropped_messages = 2;
  live.direction = "send";
  live.msg_type = "sensor_msgs/Image";
  live.send_rate_hz = 15.0F;
  RenderToScreen(swarm_ros_bridge::tui::TopicRowElement(rule, &live, true, 24,
                                                        true),
                 60, 1, "topic row with live info");
  RenderToScreen(swarm_ros_bridge::tui::TopicRowElement(rule, &live, false, 20,
                                                        false),
                 40, 1, "compact topic row");

  // The actual 120-column matrix keeps both the topic identity and metrics.
  rule.topic_name =
      "/drone_0_ego_planner_node/grid_map/occupancy";
  live.msg_type = "sensor_msgs/PointCloud2";
  const auto wide_topic_row =
      RenderToScreen(swarm_ros_bridge::tui::TopicRowElement(
                         rule, &live, true, screenshot_topics.name_width,
                         screenshot_topics.detailed_columns),
                     screenshot_topics.matrix_inner_width, 1,
                     "120-column topic matrix row")
          .ToString();
  assert(wide_topic_row.find("/drone_0") != std::string::npos);
  assert(wide_topic_row.find("occupancy") != std::string::npos);
  assert(wide_topic_row.find("Cloud") != std::string::npos);

  // -- topic aliases -----------------------------------------------------------
  const auto aliases = swarm_ros_bridge::tui::TopicAliases(rule);
  assert(!aliases.empty() && aliases.front() == rule.topic_name);

  // -- history store -----------------------------------------------------------
  swarm_ros_bridge::tui::HistoryStore history;
  swarm_ros_bridge::NetworkInfo row;
  row.name = "/camera/image";
  row.msg_type = "sensor_msgs/Image";
  row.bandwidth_kbps = 100.0F;
  std::vector<swarm_ros_bridge::NetworkInfo> snapshot{row};
  for (int i = 0; i < 10; ++i) {
    row.bandwidth_kbps = 100.0F + i;
    snapshot[0] = row;
    history.Sample(snapshot);
  }
  const auto series = history.TotalBandwidthKbps();
  assert(series.size() == 10);
  assert(series.front() == 100.0F);
  assert(series.back() == 109.0F);
  assert(history.LatestTotalKbps() == 109.0F);
  const auto topic_series =
      history.BandwidthKbps({"/camera/image", "/bridge/camera/image"});
  assert(topic_series.size() == 10 && topic_series.back() == 109.0F);

  // -- log store ---------------------------------------------------------------
  swarm_ros_bridge::tui::LogStore logs;
  swarm_ros_bridge::tui::LogRecord info_log{"14:00:00", "INFO", "bridge",
                                            "loaded"};
  swarm_ros_bridge::tui::LogRecord warn_log{"14:00:01", "WARN", "bridge",
                                            "latency"};
  swarm_ros_bridge::tui::LogRecord debug_log{"14:00:02", "DEBUG", "tui", "tick"};
  logs.Append(info_log);
  logs.Append(warn_log);
  logs.Append(debug_log);
  assert(logs.TotalCount() == 3);
  assert(logs.Records("All").size() == 3);
  assert(logs.Records("Info").size() == 2);
  assert(logs.Records("Warn").size() == 1);
  assert(logs.Records("Error").empty());
  assert(swarm_ros_bridge::tui::LogStore::LevelRank("FATAL") >=
         swarm_ros_bridge::tui::LogStore::LevelRank("ERROR"));

  // -- full responsive screens ------------------------------------------------
  swarm_ros_bridge::config::BridgeConfig config;
  config.hostname = "groundStation0";
  config.ip_map["drone1"] = "";
  config.ip_map["drone2"] = "tcp/192.168.1.22:7447";
  config.topics.push_back(rule);
  for (int i = 1; i < 20; ++i) {
    auto topic = rule;
    topic.topic_name = "/test/topic/" + std::to_string(i);
    config.topics.push_back(std::move(topic));
  }

  std::vector<std::string> topic_entries;
  for (const auto& topic : config.topics) {
    topic_entries.push_back(topic.topic_name);
  }
  swarm_ros_bridge::tui::ViewState topic_state;
  ftxui::MenuOption topic_option;
  topic_option.direction = ftxui::Direction::Down;
  topic_option.entries_option.transform =
      [&config, &screenshot_topics](const ftxui::EntryState& entry) {
        return swarm_ros_bridge::tui::TopicRowElement(
            config.topics[static_cast<std::size_t>(entry.index)], nullptr,
            entry.active, screenshot_topics.name_width,
            screenshot_topics.detailed_columns);
      };
  auto topic_menu =
      ftxui::Menu(&topic_entries, &topic_state.selected_topic, topic_option);
  swarm_ros_bridge::tui::HistoryStore empty_history;
  const auto topic_screen = RenderToScreen(
                                swarm_ros_bridge::tui::RenderTopicsScreen(
                                    config, topic_state, nullptr, empty_history,
                                    topic_menu, screenshot_layout),
                                screenshot_layout.content_width,
                                screenshot_layout.content_height,
                                "120x37 topics screen")
                                .ToString();
  assert(topic_screen.find("Topic Matrix") != std::string::npos);
  assert(topic_screen.find("Inspector") != std::string::npos);
  assert(topic_screen.find("/drone_0_ego_planner_node") != std::string::npos);
  assert(topic_screen.find("/grid_map") != std::string::npos);
  assert(topic_screen.find("/occupancy") != std::string::npos);
  assert(FindLine(topic_screen, "Live Transfer") >= 25);

  const auto overview_screen = RenderToScreen(
      swarm_ros_bridge::tui::RenderOverviewScreen(
          config, nullptr, empty_history, screenshot_layout),
      screenshot_layout.content_width, screenshot_layout.content_height,
      "120x37 overview screen");
  assert(overview_screen.ToString().find("Waiting for samples") !=
         std::string::npos);
  int tile_background_cells = 0;
  for (int x = 0; x < screenshot_layout.content_width; ++x) {
    if (overview_screen.CellAt(x, 2).background_color ==
        swarm_ros_bridge::tui::theme::BackgroundElement()) {
      ++tile_background_cells;
    }
  }
  assert(tile_background_cells >= 116);

  swarm_ros_bridge::NetworkInfo presence;
  presence.node_online = true;
  presence.node_state_age_ms = 42000;
  const auto selected_host = RenderToScreen(
      swarm_ros_bridge::tui::HostRowElement(
          "groundStation0", &presence, true, false, 48),
      72, 1, "selected host");
  const auto focused_host = RenderToScreen(
      swarm_ros_bridge::tui::HostRowElement(
          "groundStation0", &presence, false, true, 48),
      72, 1, "focused host");
  assert(selected_host.CellAt(40, 0).background_color ==
         swarm_ros_bridge::tui::theme::BackgroundSelected());
  assert(focused_host.CellAt(40, 0).background_color !=
         swarm_ros_bridge::tui::theme::BackgroundSelected());

  std::vector<std::string> host_entries{"drone1", "drone2"};
  int selected_host_index = 0;
  auto host_menu = ftxui::Menu(&host_entries, &selected_host_index);
  swarm_ros_bridge::tui::ViewState host_state;
  const auto hosts_screen = RenderToScreen(
                                swarm_ros_bridge::tui::RenderHostsScreen(
                                    config, host_state, nullptr, host_entries,
                                    host_menu, screenshot_layout),
                                screenshot_layout.content_width,
                                screenshot_layout.content_height,
                                "120x37 hosts screen")
                                .ToString();
  assert(hosts_screen.find("Zenoh liveliness") != std::string::npos);
  assert(hosts_screen.find("token") != std::string::npos);
  assert(hosts_screen.find("liveliness event.") != std::string::npos);
  assert(hosts_screen.find("UNKNOWN") != std::string::npos);

  swarm_ros_bridge::tui::LogStore many_logs;
  for (int i = 0; i < 30; ++i) {
    many_logs.Append({
        "14:00:" + (i < 10 ? std::string("0") : std::string()) +
            std::to_string(i),
        "INFO", "very-long-node-name",
        "message-" + std::to_string(i) +
            " with a deliberately long payload that must end with an ellipsis "
            "instead of being cut by the terminal edge",
    });
  }
  swarm_ros_bridge::tui::ViewState log_state;
  auto level_menu = ftxui::Menu(&log_state.log_levels,
                                &log_state.selected_log_level,
                                ftxui::MenuOption::Horizontal());
  const auto logs_screen = RenderToScreen(
                               swarm_ros_bridge::tui::RenderLogsScreen(
                                   log_state, many_logs, level_menu,
                                   screenshot_layout),
                               screenshot_layout.content_width,
                               screenshot_layout.content_height,
                               "120x37 logs screen")
                               .ToString();
  assert(logs_screen.find("message-0 ") == std::string::npos);
  assert(logs_screen.find("message-29") != std::string::npos);
  assert(logs_screen.find("showing 23 / 30 filtered / 30 total") !=
         std::string::npos);
  assert(logs_screen.find("...") != std::string::npos);

  const std::vector<std::pair<int, int>> viewport_sizes{
      {40, 12}, {80, 24}, {119, 30}, {120, 37}, {160, 45}};
  for (const auto& viewport : viewport_sizes) {
    const auto viewport_layout =
        MakeLayoutContext(viewport.first, viewport.second);
    const auto viewport_topics =
        swarm_ros_bridge::tui::MakeTopicPaneGeometry(viewport_layout);

    swarm_ros_bridge::tui::ViewState responsive_topic_state;
    ftxui::MenuOption responsive_topic_option;
    responsive_topic_option.direction = ftxui::Direction::Down;
    responsive_topic_option.entries_option.transform =
        [&config, viewport_topics](const ftxui::EntryState& entry) {
          return swarm_ros_bridge::tui::TopicRowElement(
              config.topics[static_cast<std::size_t>(entry.index)], nullptr,
              entry.active, viewport_topics.name_width,
              viewport_topics.detailed_columns);
        };
    auto responsive_topic_menu = ftxui::Menu(
        &topic_entries, &responsive_topic_state.selected_topic,
        responsive_topic_option);
    RenderToScreen(swarm_ros_bridge::tui::RenderTopicsScreen(
                       config, responsive_topic_state, nullptr, empty_history,
                       responsive_topic_menu, viewport_layout),
                   viewport_layout.content_width,
                   viewport_layout.content_height, "responsive topics");

    RenderToScreen(swarm_ros_bridge::tui::RenderOverviewScreen(
                       config, nullptr, empty_history, viewport_layout),
                   viewport_layout.content_width,
                   viewport_layout.content_height, "responsive overview");

    swarm_ros_bridge::tui::ViewState responsive_host_state;
    int responsive_host_index = 0;
    auto responsive_host_menu =
        ftxui::Menu(&host_entries, &responsive_host_index);
    RenderToScreen(swarm_ros_bridge::tui::RenderHostsScreen(
                       config, responsive_host_state, nullptr, host_entries,
                       responsive_host_menu, viewport_layout),
                   viewport_layout.content_width,
                   viewport_layout.content_height, "responsive hosts");

    swarm_ros_bridge::tui::ViewState responsive_log_state;
    auto responsive_level_menu =
        ftxui::Menu(&responsive_log_state.log_levels,
                    &responsive_log_state.selected_log_level,
                    ftxui::MenuOption::Horizontal());
    RenderToScreen(swarm_ros_bridge::tui::RenderLogsScreen(
                       responsive_log_state, many_logs, responsive_level_menu,
                       viewport_layout),
                   viewport_layout.content_width,
                   viewport_layout.content_height, "responsive logs");
  }

  std::cout << "TUI responsive layout + theme + stores tests passed\n";
  return 0;
}
