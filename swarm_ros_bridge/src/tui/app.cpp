#include "tui/app.hpp"
#include "tui/format.hpp"
#include "tui/hosts_screen.hpp"
#include "tui/logs_screen.hpp"
#include "tui/overview_screen.hpp"
#include "tui/screen_common.hpp"
#include "tui/theme.hpp"
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
         std::shared_ptr<diagnostics::DiagnosticsCache> diagnostics_cache,
         std::shared_ptr<LogStore> log_store)
    : config_(std::move(config)),
      diagnostics_cache_(std::move(diagnostics_cache)),
      log_store_(std::move(log_store)) {}

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

namespace {

ftxui::Element MakeHeader(const config::BridgeConfig& config,
                          const std::shared_ptr<diagnostics::DiagnosticsCache>&
                              diagnostics_cache,
                          const ViewState& state) {
  using namespace ftxui;
  const bool fresh =
      diagnostics_cache != nullptr && diagnostics_cache->IsFresh();
  const bool has_data =
      diagnostics_cache != nullptr && diagnostics_cache->HasReceivedData();
  const std::string diag_label = fresh ? "LIVE" : (has_data ? "STALE" : "WAIT");
  const auto diag_color = fresh ? theme::Success() : theme::Warning();

  Element diag;
  if (!fresh) {
    diag = hbox({
        spinner(3, static_cast<std::size_t>(state.animation_frame)) |
            color(theme::Primary()),
        text(" " + diag_label + " ") | bold | color(diag_color),
    });
  } else {
    diag = hbox({
        text("● ") | color(theme::Success()),
        text(diag_label + " ") | bold | color(diag_color),
    });
  }

  return hbox({
             text(" ◉ ") | color(theme::Primary()),
             text("NetBridge") | bold | color(theme::Text()),
             text(" zenoh") | color(theme::TextDim()),
             filler(),
             text(EndEllipsis(config.hostname, 20)) | color(theme::Info()),
             text(" · ") | color(theme::BorderSubtle()),
             std::move(diag),
             text("· ") | color(theme::BorderSubtle()),
             text(FormatClockNow()) | color(theme::TextMuted()),
             text(" "),
         }) |
         bgcolor(theme::BackgroundPanel());
}

ftxui::Element MakeTabBar(ftxui::Component nav) {
  using namespace ftxui;
  return vbox({
      nav->Render(),
      separatorStyled(BorderStyle::LIGHT) | color(theme::BorderSubtle()),
  });
}

ftxui::Element MakeFooter(const ViewState& state,
                          const LayoutContext& layout) {
  using namespace ftxui;

  std::vector<Element> hints;
  const bool roomy = !layout.compact();
  if (roomy) {
    hints.push_back(KeyHint("1-4", "tabs"));
    if (state.selected_nav == 1 || state.selected_nav == 2) {
      hints.push_back(KeyHint("↑↓", "select"));
    }
    if (state.selected_nav == 3) {
      hints.push_back(KeyHint("f", "filter"));
    }
  }
  hints.push_back(KeyHint("?", "help"));
  hints.push_back(KeyHint("q", "quit"));

  const bool pulse = state.animation_frame % 2 == 0;
  return hbox({
             hbox(std::move(hints)),
             filler(),
             text(pulse ? "● " : "○ ") |
                 color(pulse ? theme::Primary() : theme::TextDim()),
             text(std::to_string(layout.terminal_width) + "x" +
                  std::to_string(layout.terminal_height) + " ") |
                 color(theme::TextDim()),
         }) |
         bgcolor(theme::BackgroundPanel());
}

ftxui::Element MakeHelpOverlay() {
  using namespace ftxui;
  std::vector<Element> rows{
      KeyHint("1-4", "Switch tab"),
      KeyHint("←→", "Move between tabs"),
      KeyHint("Tab", "Move between tab bar and content"),
      KeyHint("↑↓", "Move inside lists"),
      KeyHint("f", "Cycle log level filter (Logs)"),
      KeyHint("mouse", "Click menu entries and chips"),
      KeyHint("?", "Toggle this help"),
      KeyHint("q", "Quit (press q twice)"),
      KeyHint("Esc", "Close overlays"),
  };
  return Panel("Keybindings", vbox(std::move(rows))) |
         size(WIDTH, EQUAL, 44);
}

ftxui::Element MakeQuitOverlay() {
  using namespace ftxui;
  return Panel("Quit", vbox({
                     text("Quit the NetBridge TUI?") | color(theme::Text()),
                     text(" "),
                     KeyHint("q", "quit now"),
                     KeyHint("other", "cancel"),
                 })) |
         size(WIDTH, EQUAL, 36);
}

}  // namespace

int App::Run() {
  using namespace ftxui;
  BuildTopicEntries();
  BuildHostEntries();

  // -- tab bar ---------------------------------------------------------------
  MenuOption nav_option;
  nav_option.direction = Direction::Right;
  nav_option.entries_option.transform = [](EntryState entry) {
    auto label = text(" " + std::to_string(entry.index + 1) + " " +
                      entry.label + " ");
    if (entry.active) {
      return label | bold | color(Color::Black) | bgcolor(theme::Primary());
    }
    if (entry.focused) {
      return label | bold | color(theme::PrimarySoft());
    }
    return label | color(theme::TextMuted());
  };
  Component nav = Menu(&state_.nav_items, &state_.selected_nav, nav_option);

  // -- topic matrix list -----------------------------------------------------
  MenuOption topic_option;
  topic_option.direction = Direction::Down;
  topic_option.entries_option.transform =
      [this](const EntryState& entry_state) {
        const int index = entry_state.index;
        if (index < 0 ||
            index >= static_cast<int>(config_.topics.size())) {
          return text(entry_state.label);
        }
        swarm_ros_bridge::NetworkInfo live_info;
        const bool has_live =
            diagnostics_cache_ != nullptr &&
            diagnostics_cache_->LookupAny(
                TopicAliases(config_.topics[static_cast<std::size_t>(index)]),
                &live_info);
        const auto dimensions = Terminal::Size();
        const auto layout = MakeLayoutContext(dimensions.dimx, dimensions.dimy);
        const auto geometry = MakeTopicPaneGeometry(layout);
        return TopicRowElement(
            config_.topics[static_cast<std::size_t>(index)],
            has_live ? &live_info : nullptr, entry_state.active,
            geometry.name_width, geometry.detailed_columns);
      };
  Component topics = Menu(&topic_entries_, &state_.selected_topic, topic_option);

  // -- host list -------------------------------------------------------------
  MenuOption host_option;
  host_option.direction = Direction::Down;
  host_option.entries_option.transform =
      [this](const EntryState& entry_state) {
        swarm_ros_bridge::NetworkInfo presence;
        const bool known =
            diagnostics_cache_ != nullptr &&
            diagnostics_cache_->LookupNode(entry_state.label, &presence);
        const auto dimensions = Terminal::Size();
        const auto layout = MakeLayoutContext(dimensions.dimx, dimensions.dimy);
        const int detail_width =
            layout.compact()
                ? 0
                : (layout.wide()
                       ? std::min(48,
                                  std::max(40, layout.content_width / 3))
                       : std::max(34, layout.content_width / 2));
        const int list_width = layout.content_width - detail_width;
        const int panel_chrome =
            layout.compact() && layout.short_height ? 0 : 2;
        const std::size_t label_width = static_cast<std::size_t>(
            std::max(8, list_width - panel_chrome - 14));
        return HostRowElement(entry_state.label, known ? &presence : nullptr,
                              entry_state.active, entry_state.focused,
                              static_cast<int>(label_width));
      };
  Component hosts = Menu(&host_entries_, &state_.selected_host, host_option);

  // -- log level filter ------------------------------------------------------
  MenuOption level_option;
  level_option.direction = Direction::Right;
  level_option.entries_option.transform = [](EntryState entry) {
    auto label = text(" " + entry.label + " ");
    if (entry.active) {
      return label | bold | color(Color::Black) | bgcolor(theme::Primary());
    }
    if (entry.focused) {
      return label | bold | color(theme::PrimarySoft());
    }
    return label | color(theme::TextMuted());
  };
  Component level_filter =
      Menu(&state_.log_levels, &state_.selected_log_level, level_option);

  // -- focus tree: only the active screen owns its interactive component ----
  Component overview_pane = Container::Vertical({});
  Component logs_pane = Container::Vertical({level_filter});
  Component content = Container::Tab({overview_pane, topics, hosts, logs_pane},
                                     &state_.selected_nav);
  Component main = Container::Vertical({nav, content});

  auto screen = ScreenInteractive::Fullscreen();
  std::atomic<bool> keep_refreshing{true};
  std::thread refresh_thread([&screen, &keep_refreshing]() {
    while (keep_refreshing.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(125));
      screen.PostEvent(ftxui::Event::Custom);
    }
  });

  auto last_sample = std::chrono::steady_clock::now() - std::chrono::seconds(1);
  auto renderer = Renderer(
      main,
      [this, nav, topics, hosts, level_filter, &last_sample]() {
    using namespace ftxui;
    state_.AdvanceFrame();
    BuildHostEntries();

    const auto now = std::chrono::steady_clock::now();
    if (now - last_sample >= std::chrono::milliseconds(500)) {
      last_sample = now;
      if (diagnostics_cache_ != nullptr && diagnostics_cache_->IsFresh()) {
        history_.Sample(diagnostics_cache_->Snapshot());
      }
    }

    const auto dimensions = Terminal::Size();
    const LayoutContext layout_context =
        MakeLayoutContext(dimensions.dimx, dimensions.dimy);

    Element content_element;
    switch (state_.selected_nav) {
      case 0:
        content_element = RenderOverviewScreen(config_, diagnostics_cache_,
                                               history_, layout_context);
        break;
      case 1:
        content_element =
            RenderTopicsScreen(config_, state_, diagnostics_cache_, history_,
                               topics, layout_context);
        break;
      case 2:
        content_element =
            RenderHostsScreen(config_, state_, diagnostics_cache_,
                              host_entries_, hosts, layout_context);
        break;
      case 3:
      default:
        content_element = RenderLogsScreen(state_, *log_store_, level_filter,
                                           layout_context);
        break;
    }

    Element document = vbox({
        MakeHeader(config_, diagnostics_cache_, state_),
        MakeTabBar(nav),
        content_element | flex,
        MakeFooter(state_, layout_context),
    });

    if (state_.show_help || state_.show_quit_confirm) {
      Element overlay = state_.show_help ? MakeHelpOverlay()
                                         : MakeQuitOverlay();
      document = dbox({
          std::move(document),
          center(std::move(overlay)) | clear_under | bgcolor(theme::Background()),
      });
    }

    return std::move(document) | bgcolor(theme::Background());
  });

  auto app = CatchEvent(renderer, [this, &screen](Event event) {
    const auto is_key = [&event](const char* key) {
      return event.character() == key;
    };
    if (state_.show_quit_confirm) {
      if (is_key("q")) {
        screen.ExitLoopClosure()();
        return true;
      }
      state_.show_quit_confirm = false;
      return true;
    }
    if (state_.show_help) {
      if (is_key("?") || event == Event::Escape) {
        state_.show_help = false;
      }
      return true;
    }
    if (is_key("?")) {
      state_.show_help = true;
      return true;
    }
    if (is_key("q")) {
      state_.show_quit_confirm = true;
      return true;
    }
    if (is_key("f") && state_.selected_nav == 3) {
      state_.selected_log_level =
          (state_.selected_log_level + 1) %
          static_cast<int>(state_.log_levels.size());
      return true;
    }
    for (int i = 0; i < 4; ++i) {
      if (event.character() ==
          std::string(1, static_cast<char>('1' + i))) {
        state_.selected_nav = i;
        return true;
      }
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
