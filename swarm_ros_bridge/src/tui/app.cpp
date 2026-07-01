#include "tui/app.hpp"
#include "tui/hosts_screen.hpp"
#include "tui/logs_screen.hpp"
#include "tui/overview_screen.hpp"
#include "tui/topics_screen.hpp"

#include <ftxui/component/component.hpp>
#include <ftxui/component/event.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/dom/elements.hpp>
#include <atomic>
#include <chrono>
#include <thread>
#include <string>
#include <utility>

namespace swarm_ros_bridge {
namespace tui {

App::App(config::BridgeConfig config,
         std::shared_ptr<diagnostics::DiagnosticsCache> diagnostics_cache)
    : config_(std::move(config)),
      diagnostics_cache_(std::move(diagnostics_cache)) {}

namespace {

std::vector<std::string> TopicAliases(const swarm_ros_bridge::config::TopicRule& topic) {
  std::vector<std::string> aliases;
  aliases.push_back(topic.topic_name);
  if (topic.same_prefix) {
    aliases.push_back("/bridge" + topic.topic_name);
  } else if (topic.prefix) {
    for (const auto& host : topic.src_hosts) {
      aliases.push_back("/" + host + topic.topic_name);
    }
  }
  return aliases;
}

std::string MiddleEllipsis(const std::string& text, std::size_t max_width) {
  if (text.size() <= max_width || max_width < 7) {
    return text;
  }
  const std::size_t head = (max_width - 3) / 2;
  const std::size_t tail = max_width - 3 - head;
  return text.substr(0, head) + "..." + text.substr(text.size() - tail);
}

ftxui::Color PressureColor(float bandwidth_kbps) {
  using ftxui::Color;
  if (bandwidth_kbps <= 0.0F) {
    return Color::GrayDark;
  }
  if (bandwidth_kbps < 256.0F) {
    return Color::GreenLight;
  }
  if (bandwidth_kbps < 2048.0F) {
    return Color::YellowLight;
  }
  return Color::RedLight;
}

}  // namespace

void App::BuildTopicEntries() {
  topic_entries_.clear();
  topic_entries_.reserve(config_.topics.size());
  for (const auto& topic : config_.topics) {
    topic_entries_.push_back(topic.topic_name);
  }
}

void App::BuildHostEntries() {
  host_entries_.clear();
  host_entries_.reserve(config_.ip_map.size());
  for (const auto& entry : config_.ip_map) {
    host_entries_.push_back(entry.first);
  }
}

int App::Run() {
  using namespace ftxui;
  BuildTopicEntries();
  BuildHostEntries();

  MenuOption nav_option;
  nav_option.direction = Direction::Down;
  nav_option.entries_option.transform = [](EntryState state) {
    auto label = text("  " + state.label + "  ");
    if (state.focused) {
      return label | bold | color(Color::Black) | bgcolor(Color::CyanLight);
    }
    if (state.active) {
      return label | color(Color::White) | bgcolor(Color::Blue);
    }
    return label | color(Color::GrayLight);
  };

  MenuOption topic_option;
  topic_option.direction = Direction::Down;
  topic_option.entries_option.transform =
      [this](const EntryState& entry_state) {
        swarm_ros_bridge::NetworkInfo live_info;
        const bool has_live_info =
            diagnostics_cache_ != nullptr &&
            entry_state.index < static_cast<int>(config_.topics.size()) &&
            diagnostics_cache_->LookupAny(TopicAliases(config_.topics[entry_state.index]),
                                          &live_info);
        const auto pressure_color =
            has_live_info ? PressureColor(live_info.bandwidth_kbps) : Color::GrayLight;
        auto label = text("  " + MiddleEllipsis(entry_state.label, 42) + "  ");
        if (entry_state.focused || entry_state.active) {
          return label | bold | color(Color::Black) | bgcolor(pressure_color);
        }
        return label | color(pressure_color);
      };

  MenuOption host_option;
  host_option.direction = Direction::Down;
  host_option.entries_option.transform =
      [this](const EntryState& entry_state) {
        auto label = text("  " + MiddleEllipsis(entry_state.label, 28) + "  ");
        if (entry_state.focused || entry_state.active) {
          return label | bold | color(Color::Black) | bgcolor(Color::CyanLight);
        }
        return label | color(Color::White);
      };

  Component nav = Menu(&state_.nav_items, &state_.selected_nav, nav_option);
  Component topics = Menu(&topic_entries_, &state_.selected_topic, topic_option);
  Component hosts = Menu(&host_entries_, &state_.selected_host, host_option);
  Component layout = Container::Horizontal({nav, topics, hosts});
  auto screen = ScreenInteractive::Fullscreen();
  std::atomic<bool> keep_refreshing{true};
  std::thread refresh_thread([&screen, &keep_refreshing]() {
    while (keep_refreshing.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      screen.PostEvent(ftxui::Event::Custom);
    }
  });
  auto renderer = Renderer(layout, [this, nav, topics, hosts] {
    using namespace ftxui;

    Element content;
    switch (state_.selected_nav) {
      case 0:
        content = RenderOverviewScreen(config_, diagnostics_cache_);
        break;
      case 1:
        content = RenderTopicsScreen(config_, state_, diagnostics_cache_, topics);
        break;
      case 2:
        content = RenderHostsScreen(config_, state_, hosts);
        break;
      case 3:
      default:
        content = RenderLogsScreen(config_, state_);
        break;
    }

    auto header = hbox({
                      vbox({
                          text("Gwq NetBridge Control Deck") | bold | color(Color::White),
                          text("TUI configuration + runtime observability") |
                              color(Color::GrayLight),
                      }),
                      filler(),
                      vbox({
                          text("Host  " + config_.hostname) | align_right |
                              color(Color::CyanLight),
                          text("Topics " + std::to_string(config_.topics.size()) +
                               "   Services " + std::to_string(config_.services.size())) |
                              align_right | color(Color::GrayLight),
                      }),
                  }) |
                  bgcolor(Color::RGB(20, 27, 45)) | borderRounded;

    auto sidebar = vbox({
                       text("Navigation") | bold | color(Color::CyanLight),
                       separator(),
                       nav->Render() | frame | vscroll_indicator | flex,
                   }) |
                   size(WIDTH, EQUAL, 24) | bgcolor(Color::RGB(17, 22, 35)) |
                   borderRounded;

    auto footer = hbox({
                      text(" q Quit ") | bgcolor(Color::DarkSlateGray1) | color(Color::Black),
                      text(" arrows Move ") | color(Color::GrayLight),
                      text("   "),
                      text(" tab Focus ") | color(Color::GrayLight),
                      text("   "),
                      text(" wheel Scroll list ") | color(Color::GrayLight),
                      text("   "),
                      text(" mouse Select item ") | color(Color::GrayLight),
                  }) |
                  bgcolor(Color::RGB(20, 27, 45)) | borderRounded;

    return vbox({
               header,
               hbox({
                   sidebar,
                   content | flex,
               }) |
                   flex,
               footer,
           }) |
           bgcolor(Color::RGB(8, 12, 20));
  });

  auto app = CatchEvent(renderer, [&screen](Event event) {
    if (event == Event::Character('q') || event == Event::Escape) {
      screen.ExitLoopClosure()();
      return true;
    }
    return false;
  });

  screen.Loop(app);
  keep_refreshing = false;
  refresh_thread.join();
  return 0;
}

}  // namespace tui
}  // namespace swarm_ros_bridge
