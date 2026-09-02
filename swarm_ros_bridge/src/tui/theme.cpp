#include "tui/theme.hpp"

#include <algorithm>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace swarm_ros_bridge {
namespace tui {

namespace {

std::vector<std::string> WrapValue(const std::string& value, int width) {
  const std::size_t line_width =
      static_cast<std::size_t>(std::max(1, width));
  std::vector<std::string> lines;
  std::size_t start = 0;
  while (start < value.size()) {
    while (start < value.size() && value[start] == ' ') {
      ++start;
    }
    if (start >= value.size()) {
      break;
    }
    std::size_t end = std::min(value.size(), start + line_width);
    if (end < value.size()) {
      std::size_t split = value.rfind(' ', end);
      if (split == std::string::npos || split <= start) {
        const std::size_t slash = value.rfind('/', end - 1);
        split = slash != std::string::npos && slash > start ? slash + 1 : end;
      }
      end = split;
    }
    std::string line = value.substr(start, end - start);
    while (!line.empty() && line.back() == ' ') {
      line.pop_back();
    }
    lines.push_back(std::move(line));
    start = end;
  }
  if (lines.empty()) {
    lines.push_back("");
  }
  return lines;
}

}  // namespace

ftxui::Element Badge(const std::string& label, const ftxui::Color& accent) {
  using namespace ftxui;
  return text(" " + label + " ") | bold | ftxui::color(Color::Black) |
         bgcolor(accent);
}

ftxui::Element BadgeOutline(const std::string& label,
                            const ftxui::Color& accent) {
  using namespace ftxui;
  return text(" " + label + " ") | ftxui::color(accent);
}

ftxui::Element FieldInline(const std::string& label,
                           const std::string& value,
                           const ftxui::Color& value_color) {
  using namespace ftxui;
  return hbox({
      text(label) | ftxui::color(theme::TextMuted()),
      text(value) | ftxui::color(value_color),
  });
}

ftxui::Element FieldRow(const std::string& label,
                        const std::string& value,
                        const ftxui::Color& value_color,
                        int available_width) {
  using namespace ftxui;
  const int row_width = available_width > 0 ? available_width : 48;
  const int label_width =
      row_width < 36 ? row_width : std::min(18, std::max(10, row_width / 2));
  const int value_width = row_width < 36 ? row_width
                                         : std::max(1, row_width - label_width - 1);
  const auto value_lines = WrapValue(value, value_width);
  std::vector<Element> rows;
  if (row_width < 36) {
    rows.push_back(text(label) | ftxui::color(theme::TextMuted()));
    for (const auto& line : value_lines) {
      rows.push_back(text(line) | ftxui::color(value_color));
    }
    return vbox(std::move(rows));
  }
  for (std::size_t index = 0; index < value_lines.size(); ++index) {
    rows.push_back(hbox({
        text(index == 0 ? label : "") |
            ftxui::color(theme::TextMuted()) |
            size(WIDTH, EQUAL, label_width),
        text(" "),
        text(value_lines[index]) | ftxui::color(value_color) |
            size(WIDTH, EQUAL, value_width) | align_right,
    }));
  }
  return vbox(std::move(rows));
}

ftxui::Element WrappedText(const std::string& value, int available_width) {
  using namespace ftxui;
  std::vector<Element> rows;
  for (const auto& line : WrapValue(value, available_width)) {
    rows.push_back(text(line));
  }
  return vbox(std::move(rows));
}

ftxui::Element StatTile(const std::string& label,
                        const std::string& value,
                        const std::string& note,
                        const ftxui::Color& accent) {
  using namespace ftxui;
  return hbox({
             text(" "),
             text("▌") | ftxui::color(accent),
             text(" "),
             vbox({
                 text(label) | ftxui::color(theme::TextMuted()),
                 text(value) | bold | ftxui::color(accent),
                 text(note) | ftxui::color(theme::TextDim()),
             }) |
                 flex,
             filler(),
         }) |
         xflex_grow | vcenter | size(HEIGHT, GREATER_THAN, 5) |
         bgcolor(theme::BackgroundElement()) | flex;
}

ftxui::Element SectionLabel(const std::string& title) {
  using namespace ftxui;
  return hbox({
      text("▏") | ftxui::color(theme::Primary()),
      text(title) | bold | ftxui::color(theme::Text()),
  });
}

ftxui::Element Keycap(const std::string& key) {
  using namespace ftxui;
  return text(" " + key + " ") | ftxui::color(theme::Text()) |
         bgcolor(theme::BackgroundElement());
}

ftxui::Element KeyHint(const std::string& key,
                       const std::string& description) {
  using namespace ftxui;
  return hbox({
      Keycap(key),
      text(" " + description + " ") | ftxui::color(theme::TextMuted()),
  });
}

ftxui::Element Panel(const std::string& title, ftxui::Element body) {
  using namespace ftxui;
  return vbox({
             text(" " + title) | bold | ftxui::color(theme::TextMuted()),
             separatorStyled(BorderStyle::LIGHT) | ftxui::color(theme::BorderSubtle()),
             std::move(body) | flex,
         }) |
         borderRounded | ftxui::color(theme::BorderSubtle()) |
         bgcolor(theme::BackgroundPanel());
}

ftxui::Element Bar(float ratio, const ftxui::Color& accent) {
  using namespace ftxui;
  const float clamped = std::max(0.0F, std::min(1.0F, ratio));
  if (clamped <= 0.0F) {
    return text("");
  }
  return gaugeLeft(clamped) | ftxui::color(accent);
}

ftxui::Element Sparkline(const std::vector<float>& samples,
                         const ftxui::Color& accent,
                         float scale_max) {
  using namespace ftxui;
  if (samples.empty()) {
    return text("Waiting for samples") | center | dim |
           ftxui::color(theme::TextDim());
  }
  float data_peak = 0.0F;
  for (const float sample : samples) {
    data_peak = std::max(data_peak, sample);
  }
  if (data_peak <= 0.0F) {
    return text("No traffic in window") | center | dim |
           ftxui::color(theme::TextDim());
  }
  const float peak = scale_max > 0.0F ? scale_max : data_peak;
  const float safe_peak = std::max(0.001F, peak);
  return graph(
          [samples, safe_peak](int width, int height) {
            std::vector<int> output(
                static_cast<std::size_t>(std::max(0, width)), 0);
            if (samples.empty() || width <= 0) {
              return output;
            }
            for (int x = 0; x < width; ++x) {
              // Newest samples are aligned to the right edge.
              const std::size_t index =
                  samples.size() >= static_cast<std::size_t>(width)
                      ? samples.size() -
                            (static_cast<std::size_t>(width) -
                             static_cast<std::size_t>(x))
                      : static_cast<std::size_t>(x);
              if (index >= samples.size()) {
                continue;
              }
              const float ratio =
                  std::max(0.0F,
                           std::min(1.0F, samples[index] / safe_peak));
              output[static_cast<std::size_t>(x)] =
                  static_cast<int>(ratio * static_cast<float>(height));
            }
            return output;
          }) |
      ftxui::color(accent);
}

ftxui::Color PressureColor(float bandwidth_kbps) {
  if (bandwidth_kbps <= 0.0F) {
    return theme::TextDim();
  }
  if (bandwidth_kbps < 256.0F) {
    return theme::Success();
  }
  if (bandwidth_kbps < 2048.0F) {
    return theme::Warning();
  }
  return theme::Error();
}

std::string PressureLabel(float bandwidth_kbps) {
  if (bandwidth_kbps <= 0.0F) {
    return "IDLE";
  }
  if (bandwidth_kbps < 256.0F) {
    return "LOW";
  }
  if (bandwidth_kbps < 2048.0F) {
    return "MID";
  }
  return "HIGH";
}

std::string PressureDot(float bandwidth_kbps) {
  return bandwidth_kbps <= 0.0F ? "○" : "●";
}

}  // namespace tui
}  // namespace swarm_ros_bridge
