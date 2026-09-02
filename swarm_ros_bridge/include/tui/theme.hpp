#ifndef SWARM_ROS_BRIDGE_TUI_THEME_HPP_
#define SWARM_ROS_BRIDGE_TUI_THEME_HPP_

#include <ftxui/dom/elements.hpp>

#include <string>
#include <vector>

namespace swarm_ros_bridge {
namespace tui {

// Role based palette inspired by the opencode default dark theme.
// Every screen draws colors exclusively through these roles.
namespace theme {

inline ftxui::Color Background() { return ftxui::Color::RGB(10, 10, 10); }
inline ftxui::Color BackgroundPanel() { return ftxui::Color::RGB(20, 20, 20); }
inline ftxui::Color BackgroundElement() { return ftxui::Color::RGB(30, 30, 30); }
inline ftxui::Color BackgroundSelected() {
  return ftxui::Color::RGB(36, 44, 60);
}
inline ftxui::Color Border() { return ftxui::Color::RGB(72, 72, 72); }
inline ftxui::Color BorderActive() { return ftxui::Color::RGB(96, 96, 96); }
inline ftxui::Color BorderSubtle() { return ftxui::Color::RGB(52, 52, 52); }
inline ftxui::Color Primary() { return ftxui::Color::RGB(92, 156, 245); }
inline ftxui::Color PrimarySoft() { return ftxui::Color::RGB(140, 186, 248); }
inline ftxui::Color Accent() { return ftxui::Color::RGB(157, 124, 216); }
inline ftxui::Color Error() { return ftxui::Color::RGB(224, 108, 117); }
inline ftxui::Color Warning() { return ftxui::Color::RGB(245, 167, 66); }
inline ftxui::Color Success() { return ftxui::Color::RGB(127, 216, 143); }
inline ftxui::Color Info() { return ftxui::Color::RGB(86, 182, 194); }
inline ftxui::Color Text() { return ftxui::Color::RGB(238, 238, 238); }
inline ftxui::Color TextMuted() { return ftxui::Color::RGB(128, 128, 128); }
inline ftxui::Color TextDim() { return ftxui::Color::RGB(96, 96, 96); }

}  // namespace theme

// Filled status pill, e.g. ` LIVE `.
ftxui::Element Badge(const std::string& label, const ftxui::Color& color);
// Non filled status pill.
ftxui::Element BadgeOutline(const std::string& label,
                            const ftxui::Color& color);
// `label value` on one line: muted label, colored value.
ftxui::Element FieldInline(const std::string& label,
                           const std::string& value,
                           const ftxui::Color& value_color);
// Inspector row: muted label left, value right. Narrow rows stack vertically.
ftxui::Element FieldRow(const std::string& label,
                        const std::string& value,
                        const ftxui::Color& value_color,
                        int available_width = 0);
// Deterministically wrap plain text to the available terminal width.
ftxui::Element WrappedText(const std::string& value, int available_width);
// Opencode style stat tile: colored left bar, large value, dim note.
ftxui::Element StatTile(const std::string& label,
                        const std::string& value,
                        const std::string& note,
                        const ftxui::Color& color);
// Small section heading used inside panels.
ftxui::Element SectionLabel(const std::string& title);
// Keycap hint, e.g. ` q `.
ftxui::Element Keycap(const std::string& key);
// Keycap followed by a muted description.
ftxui::Element KeyHint(const std::string& key, const std::string& description);
// Rounded panel with a dim title, the container for every major region.
ftxui::Element Panel(const std::string& title, ftxui::Element body);
// Colored horizontal progress bar.
ftxui::Element Bar(float ratio, const ftxui::Color& color);
// Block character sparkline; empty and all-zero samples render a neutral state.
ftxui::Element Sparkline(const std::vector<float>& samples,
                         const ftxui::Color& color,
                         float scale_max = 0.0F);

// Bandwidth pressure semantics shared by list rows and inspectors.
ftxui::Color PressureColor(float bandwidth_kbps);
std::string PressureLabel(float bandwidth_kbps);
std::string PressureDot(float bandwidth_kbps);

}  // namespace tui
}  // namespace swarm_ros_bridge

#endif  // SWARM_ROS_BRIDGE_TUI_THEME_HPP_
