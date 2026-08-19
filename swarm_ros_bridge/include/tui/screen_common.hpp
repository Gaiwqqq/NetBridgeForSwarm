#ifndef SWARM_ROS_BRIDGE_TUI_SCREEN_COMMON_HPP_
#define SWARM_ROS_BRIDGE_TUI_SCREEN_COMMON_HPP_

#include <ftxui/dom/elements.hpp>

#include <algorithm>
#include <string>

namespace swarm_ros_bridge {
namespace tui {

enum class LayoutMode { kCompact, kMedium, kWide };

struct LayoutContext {
  int terminal_width{80};
  int terminal_height{24};
  int content_width{80};
  int content_height{17};
  int sidebar_width{0};
  bool top_navigation{true};
  bool short_height{false};
  LayoutMode mode{LayoutMode::kMedium};

  bool compact() const { return mode == LayoutMode::kCompact; }
  bool medium() const { return mode == LayoutMode::kMedium; }
  bool wide() const { return mode == LayoutMode::kWide; }
};

inline LayoutContext MakeLayoutContext(int width, int height) {
  LayoutContext layout;
  layout.terminal_width = width > 0 ? width : 80;
  layout.terminal_height = height > 0 ? height : 24;
  layout.top_navigation = layout.terminal_width < 128;
  layout.sidebar_width = layout.top_navigation
                             ? 0
                             : (layout.terminal_width >= 160 ? 22 : 18);
  layout.content_width =
      std::max(1, layout.terminal_width - layout.sidebar_width);
  const int chrome_height = 7;
  layout.content_height =
      std::max(1, layout.terminal_height - chrome_height);
  layout.short_height = layout.content_height < 18;
  if (layout.content_width < 72 || layout.content_height < 14) {
    layout.mode = LayoutMode::kCompact;
  } else if (layout.content_width >= 104 && layout.content_height >= 22) {
    layout.mode = LayoutMode::kWide;
  } else {
    layout.mode = LayoutMode::kMedium;
  }
  return layout;
}

ftxui::Element Panel(const std::string& title, ftxui::Element body);
ftxui::Element KeyHint(const std::string& key, const std::string& description);
ftxui::Element MetricCard(const std::string& title,
                          const std::string& value,
                          const std::string& note,
                          const ftxui::Color& accent_color);

}  // namespace tui
}  // namespace swarm_ros_bridge

#endif  // SWARM_ROS_BRIDGE_TUI_SCREEN_COMMON_HPP_
