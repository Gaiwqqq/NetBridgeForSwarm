#include "tui/app.hpp"
#include "tui/hosts_screen.hpp"
#include "tui/logs_screen.hpp"
#include "tui/overview_screen.hpp"
#include "tui/screen_common.hpp"
#include "tui/topics_screen.hpp"

#include <ftxui/component/component.hpp>
#include <ftxui/component/event.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/dom/elements.hpp>
#include <ftxui/screen/terminal.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <set>
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
  std::set<std::string> hosts;
  for (const auto& entry : config_.ip_map) {
    if (entry.first != "all" && entry.first != "all_drone") {
      hosts.insert(entry.first);
    }
  }
  if (diagnostics_cache_ != nullptr) {
    for (const auto& node : diagnostics_cache_->NodeSnapshot()) {
      if (!node.node_hostname.empty()) {
        hosts.insert(node.node_hostname);
      }
    }
  }
  host_entries_.assign(hosts.begin(), hosts.end());
  if (host_entries_.empty()) {
    state_.selected_host = 0;
  } else {
    state_.selected_host =
        std::min<int>(state_.selected_host, host_entries_.size() - 1);
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
  MenuOption compact_nav_option = nav_option;
  compact_nav_option.direction = Direction::Right;

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
        const int terminal_width = Terminal::Size().dimx;
        const std::size_t label_width =
            terminal_width < 72 ? 28U : (terminal_width < 110 ? 38U : 42U);
        auto label = text("  " + MiddleEllipsis(entry_state.label, label_width) + "  ");
        if (entry_state.focused || entry_state.active) {
          return label | bold | color(Color::Black) | bgcolor(pressure_color);
        }
        return label | color(pressure_color);
      };

  MenuOption host_option;
  host_option.direction = Direction::Down;
  host_option.entries_option.transform =
      [this](const EntryState& entry_state) {
        swarm_ros_bridge::NetworkInfo presence;
        const bool known = diagnostics_cache_ != nullptr &&
                           diagnostics_cache_->LookupNode(entry_state.label,
                                                          &presence);
        const std::string state_label =
            !known ? "[ ? ] " : (presence.node_online ? "[UP ] " : "[DOWN] ");
        const auto state_color =
            !known ? Color::GrayLight
                   : (presence.node_online ? Color::GreenLight : Color::RedLight);
        const int terminal_width = Terminal::Size().dimx;
        const std::size_t label_width = terminal_width < 72 ? 14U : 22U;
        auto label = text(" " + state_label +
                          MiddleEllipsis(entry_state.label, label_width) + " ");
        if (entry_state.focused || entry_state.active) {
          return label | bold | color(Color::Black) | bgcolor(state_color);
        }
        return label | color(state_color);
      };

  Component nav_vertical =
      Menu(&state_.nav_items, &state_.selected_nav, nav_option);
  Component nav_horizontal =
      Menu(&state_.nav_items, &state_.selected_nav, compact_nav_option);
  int nav_mode_index = 0;
  Component nav =
      Container::Tab({nav_vertical, nav_horizontal}, &nav_mode_index);
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
  auto renderer = Renderer(layout, [this, nav, topics, hosts, &nav_mode_index] {
    using namespace ftxui;
    BuildHostEntries();
    const auto dimensions = Terminal::Size();
    const LayoutContext layout_context =
        MakeLayoutContext(dimensions.dimx, dimensions.dimy);
    nav_mode_index = layout_context.top_navigation ? 1 : 0;

    Element content;
    switch (state_.selected_nav) {
      case 0:
        content = RenderOverviewScreen(config_, diagnostics_cache_,
                                       layout_context);
        break;
      case 1:
        content = RenderTopicsScreen(config_, state_, diagnostics_cache_, topics,
                                     layout_context);
        break;
      case 2:
        content = RenderHostsScreen(config_, state_, diagnostics_cache_,
                                    host_entries_, hosts, layout_context);
        break;
      case 3:
      default:
        content = RenderLogsScreen(config_, state_, layout_context);
        break;
    }

    Element header;
    if (layout_context.top_navigation) {
      const int host_width = std::max(8, layout_context.terminal_width - 30);
      header = hbox({
                   text(" NetBridge ") | bold | color(Color::White),
                   text("ZENOH") | color(Color::CyanLight),
                   filler(),
                   text(MiddleEllipsis(config_.hostname,
                                       static_cast<std::size_t>(host_width))) |
                       color(Color::CyanLight),
                   text(" "),
               }) |
               bgcolor(Color::RGB(20, 27, 45)) | borderRounded;
    } else {
      header = hbox({
                    vbox({
                        text("Gwq NetBridge Control Deck") | bold |
                            color(Color::White),
                        text("TUI configuration + runtime observability") |
                            color(Color::GrayLight),
                    }),
                    filler(),
                    vbox({
                        text("Host  " + config_.hostname) | align_right |
                            color(Color::CyanLight),
                        text("Topics " + std::to_string(config_.topics.size()) +
                             "   Services " +
                             std::to_string(config_.services.size())) |
                            align_right | color(Color::GrayLight),
                    }),
                }) |
                bgcolor(Color::RGB(20, 27, 45)) | borderRounded;
    }

    auto sidebar = vbox({
                       text("Navigation") | bold | color(Color::CyanLight),
                       separator(),
                       nav->Render() | frame | vscroll_indicator | flex,
                   }) |
                   bgcolor(Color::RGB(17, 22, 35)) |
                   borderRounded |
                   size(WIDTH, EQUAL, layout_context.sidebar_width);

    Element footer;
    if (layout_context.top_navigation) {
      footer = hbox({
                   text(" q quit ") | bgcolor(Color::DarkSlateGray1) |
                       color(Color::Black),
                   text("  arrows move  tab focus ") | color(Color::GrayLight),
                   filler(),
                   text(std::to_string(layout_context.terminal_width) + "x" +
                        std::to_string(layout_context.terminal_height) + " ") |
                       color(Color::GrayDark),
               }) |
               bgcolor(Color::RGB(20, 27, 45));
    } else {
      footer = hbox({
                    text(" q Quit ") | bgcolor(Color::DarkSlateGray1) |
                        color(Color::Black),
                    text(" arrows Move ") | color(Color::GrayLight),
                    text("   tab Focus   wheel Scroll   mouse Select ") |
                        color(Color::GrayLight),
                    filler(),
                    text(std::to_string(layout_context.terminal_width) + "x" +
                         std::to_string(layout_context.terminal_height) + " ") |
                        color(Color::GrayDark),
                }) |
                bgcolor(Color::RGB(20, 27, 45)) | borderRounded;
    }

    Element workspace;
    auto bounded_content =
        content |
        size(WIDTH, EQUAL, layout_context.content_width) |
        size(HEIGHT, EQUAL, layout_context.content_height);
    if (layout_context.top_navigation) {
      workspace = vbox({
                      nav->Render() | borderRounded,
                      bounded_content | frame | flex,
                  }) |
                  flex;
    } else {
      workspace = hbox({
                      sidebar,
                      bounded_content | frame | flex,
                  }) |
                  flex;
    }

    return vbox({
               header,
               workspace,
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
