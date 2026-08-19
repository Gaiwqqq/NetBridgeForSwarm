#include "tui/screen_common.hpp"

#include <ftxui/dom/node.hpp>
#include <ftxui/screen/screen.hpp>

#include <cassert>
#include <iostream>

using swarm_ros_bridge::tui::LayoutMode;
using swarm_ros_bridge::tui::MakeLayoutContext;

int main() {
  const auto tiny = MakeLayoutContext(40, 12);
  assert(tiny.mode == LayoutMode::kCompact);
  assert(tiny.top_navigation && tiny.short_height);
  assert(tiny.content_width == 40 && tiny.content_height == 5);

  const auto compact = MakeLayoutContext(48, 16);
  assert(compact.mode == LayoutMode::kCompact);
  assert(compact.top_navigation && compact.content_height == 9);

  const auto medium = MakeLayoutContext(80, 24);
  assert(medium.mode == LayoutMode::kMedium);
  assert(medium.top_navigation && medium.content_width == 80);

  const auto top_wide = MakeLayoutContext(120, 30);
  assert(top_wide.mode == LayoutMode::kWide);
  assert(top_wide.top_navigation && top_wide.content_height == 23);

  const auto sidebar_wide = MakeLayoutContext(128, 30);
  assert(sidebar_wide.mode == LayoutMode::kWide);
  assert(!sidebar_wide.top_navigation && sidebar_wide.sidebar_width == 18);
  assert(sidebar_wide.content_width + sidebar_wide.sidebar_width == 128);

  const auto large = MakeLayoutContext(160, 45);
  assert(large.mode == LayoutMode::kWide);
  assert(!large.top_navigation && large.sidebar_width == 22);
  assert(large.content_width == 138 && large.content_height == 38);

  const auto fallback = MakeLayoutContext(0, 0);
  assert(fallback.terminal_width == 80 && fallback.terminal_height == 24);

  // Guard the inclusive-box math used by the vendored FTXUI size decorator.
  // Rendering an 8-cell constraint into a wider screen must leave four cells
  // untouched instead of overflowing by two columns.
  auto constrained = ftxui::text("abcdefghijkl") |
                     ftxui::size(ftxui::WIDTH, ftxui::EQUAL, 8);
  ftxui::Screen screen(12, 1);
  ftxui::Render(screen, constrained);
  assert(screen.ToString() == "abcdefgh    ");

  std::cout << "TUI responsive layout breakpoint tests passed\n";
  return 0;
}
