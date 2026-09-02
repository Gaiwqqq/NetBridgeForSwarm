#include "tui/logs_screen.hpp"

#include "tui/format.hpp"
#include "tui/theme.hpp"

#include <ftxui/dom/elements.hpp>

#include <algorithm>
#include <cstddef>
#include <string>
#include <vector>

namespace swarm_ros_bridge {
namespace tui {

namespace {

ftxui::Color LevelColor(const std::string& level) {
  if (level == "ERROR" || level == "FATAL") {
    return theme::Error();
  }
  if (level == "WARN") {
    return theme::Warning();
  }
  if (level == "INFO") {
    return theme::Info();
  }
  return theme::TextDim();  // DEBUG and anything else.
}

std::string LevelTag(const std::string& level) {
  if (level == "FATAL") {
    return "FATAL";
  }
  if (level.size() > 5) {
    return level.substr(0, 5);
  }
  return level;
}

}  // namespace

ftxui::Element RenderLogsScreen(const ViewState& state,
                                const LogStore& log_store,
                                ftxui::Component level_filter,
                                const LayoutContext& layout) {
  using namespace ftxui;

  const std::string min_level = state.log_levels[static_cast<std::size_t>(
      std::max(0, std::min<int>(state.selected_log_level,
                                static_cast<int>(state.log_levels.size()) -
                                    1)))];
  const auto records = log_store.Records(min_level);

  const bool bare_layout = layout.compact() && layout.short_height;
  const int body_width =
      std::max(1, layout.content_width - (bare_layout ? 0 : 2));
  const int visible_capacity =
      std::max(1, layout.content_height - (bare_layout ? 1 : 9));
  const std::size_t first_visible =
      records.size() > static_cast<std::size_t>(visible_capacity)
          ? records.size() - static_cast<std::size_t>(visible_capacity)
          : 0U;
  const std::size_t visible_count = records.size() - first_visible;

  const int stamp_width = body_width < 56 ? 9 : 11;
  const int level_width = 7;
  const int node_width = body_width >= 100 ? 20
                         : body_width >= 72 ? 16
                         : body_width >= 56 ? 12
                                            : 0;
  const int message_width =
      std::max(4, body_width - stamp_width - level_width - node_width);

  std::vector<Element> lines;
  lines.reserve(visible_count);
  for (std::size_t index = first_visible; index < records.size(); ++index) {
    const auto& record = records[index];
    std::vector<Element> columns{
        text(" " + record.stamp + " ") | color(theme::TextDim()) |
            size(WIDTH, EQUAL, stamp_width),
        text(" " + LevelTag(record.level) + " ") | bold |
            color(LevelColor(record.level)) |
            size(WIDTH, EQUAL, level_width),
    };
    if (node_width > 0) {
      columns.push_back(
          text(EndEllipsis(record.node, node_width - 1) + " ") |
          color(theme::Primary()) | size(WIDTH, EQUAL, node_width));
    }
    columns.push_back(
        text(EndEllipsis(record.message, message_width)) |
        color(theme::Text()) | size(WIDTH, EQUAL, message_width));
    lines.push_back(hbox(std::move(columns)));
  }

  const std::string count_label =
      layout.compact()
          ? std::to_string(visible_count) + "/" +
                std::to_string(records.size())
          : "showing " + std::to_string(visible_count) + " / " +
                std::to_string(records.size()) + " filtered / " +
                std::to_string(log_store.TotalCount()) + " total";
  auto filter_row = hbox({
      level_filter->Render(),
      filler(),
      text(count_label) | color(theme::TextDim()),
  });

  Element body;
  if (lines.empty()) {
    body = vbox({
        spinner(3, static_cast<std::size_t>(state.animation_frame)) |
            color(theme::Primary()),
        text("Waiting for /rosout messages" +
             (min_level == "All" ? std::string("...")
                                 : std::string(" at " + min_level + "+..."))) |
            color(theme::TextDim()),
    }) | center;
  } else {
    body = vbox(std::move(lines));
  }

  if (layout.compact() && layout.short_height) {
    return vbox({
        filter_row | bgcolor(theme::BackgroundElement()),
        body | flex,
    });
  }

  return vbox({
      Panel("Filters", filter_row),
      Panel("rosout", body) | flex,
  });
}

}  // namespace tui
}  // namespace swarm_ros_bridge
