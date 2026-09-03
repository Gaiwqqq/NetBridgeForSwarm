#include "tui/topics_screen.hpp"

#include "tui/format.hpp"
#include "tui/theme.hpp"

#include <ftxui/dom/elements.hpp>

#include <algorithm>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

namespace swarm_ros_bridge {
namespace tui {

namespace {

constexpr int kDetailedColumnsWidth = 49;
constexpr int kCompactColumnsWidth = 17;

std::string ShortType(const std::string& msg_type) {
  if (msg_type == "sensor_msgs/Image") {
    return "Img";
  }
  if (msg_type == "sensor_msgs/CompressedImage") {
    return "Jpg";
  }
  if (msg_type == "sensor_msgs/PointCloud2") {
    return "Cloud";
  }
  if (msg_type == "nav_msgs/Odometry") {
    return "Odom";
  }
  const auto slash = msg_type.find_last_of('/');
  std::string raw =
      slash == std::string::npos ? msg_type : msg_type.substr(slash + 1);
  if (raw.size() > 6) {
    raw = raw.substr(0, 5) + ".";
  }
  return raw;
}

std::string CodecLabel(const config::TopicRule& topic) {
  return topic.cloud_codec;
}

std::string TransportLabel(const config::BridgeConfig& config,
                           const swarm_ros_bridge::NetworkInfo* live_info) {
  if (live_info != nullptr && !live_info->transport.empty() &&
      live_info->transport != "pending") {
    return live_info->transport;
  }
  return config.zenoh.image_session.enabled || config.zenoh.cloud_session.enabled
             ? "Auto (schema pending)"
             : "Zenoh (schema pending)";
}

ftxui::Element DirectionStrip(const std::string& title,
                              const ftxui::Color& accent,
                              bool has_info,
                              const swarm_ros_bridge::NetworkInfo& info) {
  using namespace ftxui;
  const std::string rate =
      has_info ? FormatMetric(info.direction == "send" ? info.send_rate_hz
                                                       : info.recv_rate_hz,
                              " Hz")
               : "--";
  const std::string bandwidth =
      has_info ? FormatMetric(info.bandwidth_kbps, " kbps") : "--";
  const std::string packet =
      has_info ? std::to_string(info.packet_size) + " B" : "--";
  const std::string drops =
      has_info ? std::to_string(info.dropped_messages) : "--";
  return hbox({
             text("▏ ") | color(accent),
             text(title) | bold | color(accent) | size(WIDTH, EQUAL, 6),
             text(rate) | color(theme::Text()) | size(WIDTH, EQUAL, 10) |
                 align_right,
             text(bandwidth) | color(theme::Text()) |
                 size(WIDTH, EQUAL, 13) | align_right,
             filler(),
             text("pkt " + packet + "  drop " + drops) |
                 color(theme::TextMuted()),
         }) |
         xflex_grow | bgcolor(theme::BackgroundElement());
}

ftxui::Element InspectorField(const std::string& label,
                              const std::string& value,
                              const ftxui::Color& color,
                              int available_width) {
  return FieldRow(label, value, color, available_width);
}

ftxui::Element WrappedPath(const std::string& value, int available_width) {
  using namespace ftxui;
  const std::size_t width =
      static_cast<std::size_t>(std::max(8, available_width));
  std::vector<Element> lines;
  std::size_t start = 0;
  while (start < value.size()) {
    std::size_t end = std::min(value.size(), start + width);
    if (end < value.size()) {
      const std::size_t slash = value.rfind('/', end);
      if (slash != std::string::npos && slash > start) {
        end = slash;
      }
    }
    if (end == start) {
      end = std::min(value.size(), start + width);
    }
    lines.push_back(text(value.substr(start, end - start)) | bold |
                    color(theme::Text()));
    start = end;
  }
  if (lines.empty()) {
    lines.push_back(text("-") | color(theme::TextDim()));
  }
  return vbox(std::move(lines));
}

}  // namespace

TopicPaneGeometry MakeTopicPaneGeometry(const LayoutContext& layout) {
  TopicPaneGeometry geometry;
  geometry.split = layout.content_width >= 120 && layout.content_height >= 22;
  geometry.inspector_width =
      geometry.split
          ? std::max(40, std::min(48, layout.content_width / 3))
          : layout.content_width;
  geometry.matrix_width =
      geometry.split ? layout.content_width - geometry.inspector_width
                     : layout.content_width;
  const int panel_chrome =
      layout.compact() && layout.short_height ? 0 : 2;
  geometry.matrix_inner_width =
      std::max(1, geometry.matrix_width - panel_chrome);
  geometry.detailed_columns =
      !layout.compact() && geometry.matrix_inner_width >= 70;
  geometry.name_width = std::max(
      8, geometry.matrix_inner_width -
             (geometry.detailed_columns ? kDetailedColumnsWidth
                                        : kCompactColumnsWidth));
  return geometry;
}

std::vector<std::string> TopicAliases(const config::TopicRule& topic) {
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

ftxui::Element TopicRowElement(const config::TopicRule& topic,
                               const swarm_ros_bridge::NetworkInfo* live_info,
                               bool highlighted,
                               int name_width,
                               bool detailed) {
  using namespace ftxui;
  const float bandwidth =
      live_info != nullptr ? live_info->bandwidth_kbps : 0.0F;
  const auto pressure_color = PressureColor(bandwidth);
  const std::string dot = PressureDot(bandwidth);
  const std::string detected_type =
      live_info != nullptr && !live_info->msg_type.empty()
          ? live_info->msg_type
          : "auto";

  const std::string name =
      MiddleEllipsis(topic.topic_name, static_cast<std::size_t>(
                                          std::max(8, name_width)));
  auto name_part =
      hbox({
          text(highlighted ? "▸ " : "  "),
          text(name) | size(WIDTH, EQUAL, std::max(8, name_width)),
      }) |
      (highlighted ? bold | color(theme::Text()) : color(theme::TextMuted()));

  ftxui::Element row;
  if (!detailed) {
    row = hbox({
        text(dot + " ") | color(pressure_color),
        name_part | flex,
        text(live_info != nullptr ? FormatMetric(bandwidth, " kbps") : "--") |
            color(pressure_color),
        text(" "),
    });
  } else {
    const float rate =
        live_info != nullptr
            ? (live_info->direction == "send" ? live_info->send_rate_hz
                                              : live_info->recv_rate_hz)
            : -1.0F;
    const float stability =
        live_info != nullptr ? live_info->stability_score : -1.0F;
    const std::uint32_t drops =
        live_info != nullptr ? live_info->dropped_messages : 0;
    row = hbox({
        text(dot + " ") | color(pressure_color),
        name_part | flex,
        text(ShortType(detected_type)) | color(theme::Info()) |
            size(WIDTH, EQUAL, 6),
        text(FormatMetric(rate, " Hz")) | color(theme::Text()) |
            size(WIDTH, EQUAL, 9) | align_right,
        Bar(live_info != nullptr ? bandwidth / 2048.0F : 0.0F,
            pressure_color) |
            size(WIDTH, EQUAL, 8),
        text(FormatMetric(bandwidth, " kbps")) | color(pressure_color) |
            size(WIDTH, EQUAL, 11) | align_right,
        text(std::to_string(drops)) |
            color(drops > 0 ? theme::Warning() : theme::TextDim()) |
            size(WIDTH, EQUAL, 5) | align_right,
        text(FormatMetric(stability, "%")) |
            color(stability >= 0.0F && stability < 80.0F ? theme::Error()
                                                         : theme::Success()) |
            size(WIDTH, EQUAL, 5) | align_right,
        text(" "),
    });
  }
  if (highlighted) {
    return row | bgcolor(theme::BackgroundSelected());
  }
  return row;
}

ftxui::Element RenderTopicsScreen(
    const config::BridgeConfig& config,
    const ViewState& state,
    const std::shared_ptr<diagnostics::DiagnosticsCache>& diagnostics_cache,
    const HistoryStore& history,
    ftxui::Component topic_list,
    const LayoutContext& layout) {
  using namespace ftxui;

  Element inspector =
      vbox({text("Select a topic to inspect send/recv state.") |
            color(theme::TextDim())});
  Element transfer_summary = inspector;
  Element short_inspector = inspector;
  const auto geometry = MakeTopicPaneGeometry(layout);
  const int inspector_body_width = std::max(
      20, geometry.inspector_width -
              (layout.compact() && layout.short_height ? 0 : 2));

  if (!config.topics.empty()) {
    const int selected_index = std::min<int>(
        state.selected_topic, static_cast<int>(config.topics.size()) - 1);
    const auto& topic = config.topics[selected_index];
    const auto aliases = TopicAliases(topic);
    swarm_ros_bridge::NetworkInfo live_info;
    swarm_ros_bridge::NetworkInfo send_info;
    swarm_ros_bridge::NetworkInfo recv_info;
    const bool has_live_info =
        diagnostics_cache != nullptr &&
        diagnostics_cache->LookupAny(aliases, &live_info);
    const bool has_send_info =
        diagnostics_cache != nullptr &&
        diagnostics_cache->LookupDirected(aliases, "send", &send_info);
    const bool has_recv_info =
        diagnostics_cache != nullptr &&
        diagnostics_cache->LookupDirected(aliases, "recv", &recv_info);
    const float pressure_bandwidth =
        has_live_info ? live_info.bandwidth_kbps : 0.0F;
    const std::string detected_type =
        has_live_info && !live_info.msg_type.empty() ? live_info.msg_type
                                                     : "auto";
    const auto pressure_color = PressureColor(pressure_bandwidth);

    // -- header ---------------------------------------------------------------
    auto header = vbox({
        WrappedPath(topic.topic_name, inspector_body_width),
        hbox({
            BadgeOutline(ShortType(detected_type), theme::Info()),
            text(" "),
            BadgeOutline(EndEllipsis(topic.qos_class, 12),
                         theme::TextMuted()),
            text(" "),
            Badge(has_live_info ? PressureLabel(pressure_bandwidth) : "WAIT",
                  has_live_info ? pressure_color : theme::Warning()),
        }),
        separatorStyled(BorderStyle::LIGHT) | color(theme::BorderSubtle()),
    });

    // -- routing / transport --------------------------------------------------
    std::vector<Element> routing_rows = {
        InspectorField("Type", detected_type, theme::Info(),
                       inspector_body_width),
        InspectorField("Schema",
                       has_live_info && !live_info.schema_state.empty()
                           ? live_info.schema_state
                           : "discovering",
                       has_live_info && live_info.schema_state == "conflict"
                           ? theme::Error()
                           : theme::Info(),
                       inspector_body_width),
        InspectorField("Schema MD5",
                       has_live_info && !live_info.schema_md5.empty()
                           ? live_info.schema_md5
                           : "--",
                       theme::TextMuted(), inspector_body_width),
        InspectorField("Wire codec",
                       has_live_info && !live_info.codec.empty()
                           ? live_info.codec
                           : "pending",
                       theme::Text(), inspector_body_width),
        InspectorField("Sources", JoinHosts(topic.src_hosts), theme::Text(),
                       inspector_body_width),
        InspectorField("Targets", JoinHosts(topic.dst_hosts), theme::Text(),
                       inspector_body_width),
        InspectorField("Transport",
                       TransportLabel(config,
                                      has_live_info ? &live_info : nullptr),
                       theme::Text(), inspector_body_width),
        InspectorField("Prefix", topic.prefix ? "on" : "off",
                       theme::TextMuted(), inspector_body_width),
        InspectorField("Same prefix", topic.same_prefix ? "on" : "off",
                       theme::TextMuted(), inspector_body_width),
    };
    if (has_live_info && !live_info.schema_error.empty()) {
      routing_rows.push_back(
          InspectorField("Schema error", live_info.schema_error,
                         theme::Error(), inspector_body_width));
    }
    if (detected_type == "sensor_msgs/PointCloud2") {
      routing_rows.push_back(
          InspectorField("Cloud codec", CodecLabel(topic), theme::Text(),
                         inspector_body_width));
      if (topic.cloud_downsample > 0.0) {
        std::ostringstream stream;
        stream.setf(std::ios::fixed);
        stream.precision(topic.cloud_downsample >= 1.0 ? 2 : 4);
        stream << topic.cloud_downsample;
        routing_rows.push_back(InspectorField(
            "Downsample", stream.str(), theme::Text(), inspector_body_width));
      }
    }

    // -- image quality --------------------------------------------------------
    std::vector<Element> image_rows;
    if (detected_type == "sensor_msgs/Image") {
      std::ostringstream resize_stream;
      resize_stream.setf(std::ios::fixed);
      resize_stream.precision(2);
      resize_stream << topic.image_resize_rate << "x";
      image_rows.push_back(
          InspectorField("Resize", resize_stream.str(), theme::Text(),
                         inspector_body_width));
      image_rows.push_back(InspectorField(
          "JPEG quality", std::to_string(topic.image_jpeg_quality),
          theme::Text(), inspector_body_width));
      image_rows.push_back(
          InspectorField("Adaptive JPEG",
                         topic.image_adaptive_quality ? "on" : "off",
                         topic.image_adaptive_quality ? theme::Success()
                                                      : theme::TextMuted(),
                         inspector_body_width));
      if (topic.image_adaptive_quality) {
        image_rows.push_back(InspectorField(
            "Quality range", std::to_string(topic.image_min_jpeg_quality) +
                                 " - " +
                                 std::to_string(topic.image_max_jpeg_quality),
            theme::Text(), inspector_body_width));
        std::ostringstream bw_stream;
        bw_stream.setf(std::ios::fixed);
        bw_stream.precision(topic.image_target_bandwidth_kbps >= 100.0 ? 0
                                                                       : 1);
        bw_stream << topic.image_target_bandwidth_kbps << " kbps";
        image_rows.push_back(
            InspectorField("Target BW", bw_stream.str(), theme::Text(),
                           inspector_body_width));
        image_rows.push_back(InspectorField(
            "Current JPEG",
            has_live_info && live_info.current_jpeg_quality > 0
                ? std::to_string(live_info.current_jpeg_quality)
                : "-",
            theme::Text(), inspector_body_width));
      }
      image_rows.push_back(InspectorField(
          "Receiver loss",
          has_recv_info ? FormatMetric(recv_info.image_loss_rate_pct, "%")
                        : "--",
          has_recv_info && recv_info.image_loss_rate_pct > 5.0F
              ? theme::Warning()
              : theme::Text(),
          inspector_body_width));
      image_rows.push_back(InspectorField(
          "Complete frames",
          has_recv_info
              ? FormatMetric(recv_info.complete_frame_success_rate_pct, "%")
              : "--",
          theme::Text(), inspector_body_width));
      image_rows.push_back(InspectorField(
          "Effective RX",
          has_recv_info
              ? FormatMetric(recv_info.effective_recv_bandwidth_kbps, " kbps")
              : "--",
          theme::Text(), inspector_body_width));
      image_rows.push_back(InspectorField(
          "Decoded / expected",
          has_recv_info ? std::to_string(recv_info.decoded_frames) + " / " +
                              std::to_string(recv_info.expected_frames)
                        : "--",
          theme::Text(), inspector_body_width));
    }

    // -- live metrics ---------------------------------------------------------
    std::vector<Element> live_rows = {
        InspectorField("Latency",
                       has_live_info
                           ? FormatMetric(live_info.avg_latency_ms, " ms")
                           : "--",
                       theme::Accent(), inspector_body_width),
        InspectorField("Jitter",
                       has_live_info ? FormatMetric(live_info.jitter_ms, " ms")
                                     : "--",
                       theme::Text(), inspector_body_width),
        InspectorField("Stability",
                       has_live_info
                           ? FormatMetric(live_info.stability_score, "%")
                           : "--",
                       theme::Success(), inspector_body_width),
        InspectorField("Last recv age",
                       has_live_info
                           ? FormatMetric(live_info.last_recv_age_ms, " ms")
                           : "--",
                       theme::Text(), inspector_body_width),
    };

    // -- transfer + trend -----------------------------------------------------
    transfer_summary = vbox({
        SectionLabel("Live Transfer"),
        DirectionStrip("SEND", theme::Accent(), has_send_info, send_info),
        DirectionStrip("RECV", theme::Info(), has_recv_info, recv_info),
    });
    const auto topic_history = history.BandwidthKbps(aliases);
    auto trend = vbox({
        SectionLabel("Bandwidth Trend"),
        Sparkline(topic_history, theme::Primary()) |
            size(HEIGHT, EQUAL, 3),
    });

    std::vector<Element> inspector_parts{header, SectionLabel("Routing")};
    inspector_parts.insert(inspector_parts.end(), routing_rows.begin(),
                           routing_rows.end());
    if (!image_rows.empty()) {
      inspector_parts.push_back(SectionLabel("Image Quality"));
      inspector_parts.insert(inspector_parts.end(), image_rows.begin(),
                             image_rows.end());
    }
    inspector_parts.push_back(SectionLabel("Live Metrics"));
    inspector_parts.insert(inspector_parts.end(), live_rows.begin(),
                           live_rows.end());
    inspector_parts.push_back(trend);
    inspector = vbox(std::move(inspector_parts));
    short_inspector = hbox({
        text(" " + EndEllipsis(topic.topic_name,
                               std::max(8, layout.content_width - 12))) |
            bold | color(theme::Text()),
        filler(),
        text(has_live_info ? PressureLabel(pressure_bandwidth) : "WAIT") |
            bold |
            color(has_live_info ? pressure_color : theme::Warning()),
        text(" "),
    });
  }

  // -- topic matrix ----------------------------------------------------------
  auto matrix_header = hbox({
      text("    topic") | color(theme::TextDim()) |
          size(WIDTH, EQUAL, geometry.name_width + 4),
      text("type") | color(theme::TextDim()) | size(WIDTH, EQUAL, 6),
      text("rate") | color(theme::TextDim()) | size(WIDTH, EQUAL, 9) |
          align_right,
      text("bandwidth") | color(theme::TextDim()) |
          size(WIDTH, EQUAL, 19) | align_right,
      text("drop") | color(theme::TextDim()) | size(WIDTH, EQUAL, 5) |
          align_right,
      text("stab") | color(theme::TextDim()) | size(WIDTH, EQUAL, 5) |
          align_right,
      text(" "),
  });

  if (layout.compact()) {
    if (layout.short_height) {
      return vbox({
          topic_list->Render() | frame | vscroll_indicator | flex,
          separatorStyled(BorderStyle::LIGHT) | color(theme::BorderSubtle()),
          short_inspector,
      });
    }
    return vbox({
        Panel("Topics", topic_list->Render() | frame | vscroll_indicator) |
            flex,
        Panel("Inspector", inspector | frame | vscroll_indicator) |
            size(HEIGHT, EQUAL, std::max(6, layout.content_height / 2)),
    });
  }

  const auto matrix_body = vbox({
      matrix_header,
      separatorStyled(BorderStyle::LIGHT) | color(theme::BorderSubtle()),
      topic_list->Render() | frame | vscroll_indicator | flex,
      separatorStyled(BorderStyle::LIGHT) | color(theme::BorderSubtle()),
      transfer_summary,
  });

  if (geometry.split) {
    return hbox({
        Panel("Topic Matrix", matrix_body) | flex,
        Panel("Inspector", inspector | frame | vscroll_indicator) |
            size(WIDTH, EQUAL, geometry.inspector_width),
    });
  }

  return vbox({
      Panel("Topic Matrix", matrix_body) | flex,
      Panel("Inspector", inspector | frame | vscroll_indicator) |
          size(HEIGHT, EQUAL,
               std::max(6, std::min(12, layout.content_height / 3))),
  });
}

}  // namespace tui
}  // namespace swarm_ros_bridge
