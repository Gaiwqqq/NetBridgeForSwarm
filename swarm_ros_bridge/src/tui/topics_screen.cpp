#include "tui/topics_screen.hpp"

#include "tui/screen_common.hpp"

#include <ftxui/dom/elements.hpp>

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

namespace swarm_ros_bridge {
namespace tui {

namespace {

std::string JoinHosts(const std::vector<std::string>& hosts) {
  if (hosts.empty()) {
    return "-";
  }
  std::string joined = hosts.front();
  for (std::size_t i = 1; i < hosts.size(); ++i) {
    joined += ", " + hosts[i];
  }
  return joined;
}

std::string CodecLabel(const config::TopicRule& topic) {
  if (topic.msg_type != "sensor_msgs/PointCloud2") {
    return "-";
  }
  return topic.cloud_codec;
}

std::string ImageResizeLabel(const config::TopicRule& topic) {
  if (topic.msg_type != "sensor_msgs/Image") {
    return "-";
  }
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(2);
  stream << topic.image_resize_rate << "x";
  return stream.str();
}

std::string ImageJpegQualityLabel(const config::TopicRule& topic) {
  if (topic.msg_type != "sensor_msgs/Image") {
    return "-";
  }
  return std::to_string(topic.image_jpeg_quality);
}

std::string ImageAdaptiveQualityLabel(const config::TopicRule& topic) {
  if (topic.msg_type != "sensor_msgs/Image") {
    return "-";
  }
  return topic.image_adaptive_quality ? "on" : "off";
}

std::string ImageQualityRangeLabel(const config::TopicRule& topic) {
  if (topic.msg_type != "sensor_msgs/Image" || !topic.image_adaptive_quality) {
    return "-";
  }
  return std::to_string(topic.image_min_jpeg_quality) + " - " +
         std::to_string(topic.image_max_jpeg_quality);
}

std::string ImageTargetBandwidthLabel(const config::TopicRule& topic) {
  if (topic.msg_type != "sensor_msgs/Image" || !topic.image_adaptive_quality) {
    return "-";
  }
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(topic.image_target_bandwidth_kbps >= 100.0 ? 0 : 1);
  stream << topic.image_target_bandwidth_kbps << " kbps";
  return stream.str();
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

std::string FormatMetric(double value, const std::string& suffix) {
  if (value < 0.0) {
    return "--";
  }
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(value >= 100.0 ? 0 : 1);
  stream << value << suffix;
  return stream.str();
}

std::string BandwidthPressure(float bandwidth_kbps) {
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

ftxui::Color BandwidthPressureColor(float bandwidth_kbps) {
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

ftxui::Element MetricLine(const std::string& label, const std::string& value) {
  using namespace ftxui;
  return hbox({
      text(label) | color(Color::GrayLight),
      filler(),
      text(value) | color(Color::White),
  });
}

void AppendLine(std::vector<ftxui::Element>* lines,
                const std::string& label,
                const std::string& value) {
  if (lines == nullptr) {
    return;
  }
  lines->push_back(ftxui::text(label + value));
}

ftxui::Element DirectionSummary(const std::string& title,
                                const ftxui::Color& accent,
                                bool has_info,
                                const swarm_ros_bridge::NetworkInfo& info) {
  using namespace ftxui;
  const std::string rate =
      has_info ? FormatMetric(info.direction == "send" ? info.send_rate_hz : info.recv_rate_hz,
                              " Hz")
               : "--";
  const std::string bandwidth = has_info ? FormatMetric(info.bandwidth_kbps, " kbps") : "--";
  const std::string packet =
      has_info ? std::to_string(info.packet_size) + " B" : "--";
  const std::string drops =
      has_info ? std::to_string(info.dropped_messages) : "--";
  return hbox({
             text(title) | bold | color(accent),
             text("  ") ,
             text("rate ") | color(Color::GrayLight),
             text(rate) | color(Color::White),
             text("  bw ") | color(Color::GrayLight),
             text(bandwidth) | color(Color::White),
             text("  pkt ") | color(Color::GrayLight),
             text(packet) | color(Color::White),
             text("  drop ") | color(Color::GrayLight),
             text(drops) | color(Color::White),
         }) |
         borderRounded | bgcolor(Color::RGB(18, 23, 36));
}

}  // namespace

ftxui::Element RenderTopicsScreen(
    const config::BridgeConfig& config,
    const ViewState& state,
    const std::shared_ptr<diagnostics::DiagnosticsCache>& diagnostics_cache,
    ftxui::Component topic_list) {
  using namespace ftxui;

  Element details = text("No topics configured.") | color(Color::GrayLight);
  Element transfer_summary = text("Select a topic to inspect send/recv state.") |
                             color(Color::GrayLight);
  if (!config.topics.empty()) {
    const int selected_index =
        std::min<int>(state.selected_topic, static_cast<int>(config.topics.size()) - 1);
    const auto& topic = config.topics[selected_index];
    const auto aliases = TopicAliases(topic);
    swarm_ros_bridge::NetworkInfo live_info;
    swarm_ros_bridge::NetworkInfo send_info;
    swarm_ros_bridge::NetworkInfo recv_info;
    const bool has_live_info =
        diagnostics_cache != nullptr && diagnostics_cache->LookupAny(aliases, &live_info);
    const bool has_send_info =
        diagnostics_cache != nullptr &&
        diagnostics_cache->LookupDirected(aliases, "send", &send_info);
    const bool has_recv_info =
        diagnostics_cache != nullptr &&
        diagnostics_cache->LookupDirected(aliases, "recv", &recv_info);
    const float pressure_bandwidth =
        has_send_info && has_recv_info
            ? std::max(send_info.bandwidth_kbps, recv_info.bandwidth_kbps)
            : (has_send_info ? send_info.bandwidth_kbps
                             : (has_recv_info ? recv_info.bandwidth_kbps
                                              : (has_live_info ? live_info.bandwidth_kbps : 0.0F)));
    const auto pressure_color = BandwidthPressureColor(pressure_bandwidth);
    std::vector<Element> config_lines = {
        text("Sources: " + JoinHosts(topic.src_hosts)),
        text("Targets: " + JoinHosts(topic.dst_hosts)),
        text("Port: " + std::to_string(topic.src_port)),
        text("Prefix: " + std::string(topic.prefix ? "on" : "off")),
        text("Same prefix: " + std::string(topic.same_prefix ? "on" : "off")),
    };

    if (topic.msg_type == "sensor_msgs/PointCloud2") {
      AppendLine(&config_lines, "Cloud codec: ", CodecLabel(topic));
      if (topic.cloud_downsample > 0.0) {
        std::ostringstream stream;
        stream.setf(std::ios::fixed);
        stream.precision(topic.cloud_downsample >= 1.0 ? 2 : 4);
        stream << topic.cloud_downsample;
        AppendLine(&config_lines, "Downsample: ", stream.str());
      }
    }

    if (topic.msg_type == "sensor_msgs/Image") {
      AppendLine(&config_lines, "Image resize: ", ImageResizeLabel(topic));
      AppendLine(&config_lines, "JPEG quality: ", ImageJpegQualityLabel(topic));
      AppendLine(&config_lines, "Adaptive JPEG: ", ImageAdaptiveQualityLabel(topic));
      if (topic.image_adaptive_quality) {
        AppendLine(&config_lines, "Quality range: ", ImageQualityRangeLabel(topic));
        AppendLine(&config_lines, "Target BW: ", ImageTargetBandwidthLabel(topic));
        AppendLine(&config_lines,
                   "Current JPEG: ",
                   has_live_info && live_info.current_jpeg_quality > 0
                       ? std::to_string(live_info.current_jpeg_quality)
                       : "-");
      }
    }

    transfer_summary = vbox({
                           text("Live Transfer") | bold | color(Color::White),
                           DirectionSummary("SEND", Color::Magenta1, has_send_info, send_info),
                           DirectionSummary("RECV", Color::CyanLight, has_recv_info, recv_info),
                       });

    details = vbox({
                  text(topic.topic_name) | bold | color(Color::White),
                  text(topic.msg_type) | color(Color::CyanLight),
                  separator(),
                  hbox({
                      text("Pressure: ") | color(Color::GrayLight),
                      text(has_live_info ? BandwidthPressure(live_info.bandwidth_kbps) : "WAIT") |
                          bold | color(pressure_color),
                  }),
                  vbox(std::move(config_lines)),
                  separator(),
                  MetricLine("Latency",
                             has_live_info ? FormatMetric(live_info.avg_latency_ms, " ms") : "--"),
                  MetricLine("Jitter",
                             has_live_info ? FormatMetric(live_info.jitter_ms, " ms") : "--"),
                  MetricLine("Stability",
                             has_live_info ? FormatMetric(live_info.stability_score, "%") : "--"),
                  MetricLine("Last recv age",
                             has_live_info ? FormatMetric(live_info.last_recv_age_ms, " ms")
                                           : "--"),
              }) |
              color(Color::GrayLight);
  }

  auto legend = hbox({
                   text(" LOW ") | color(Color::Black) | bgcolor(Color::GreenLight),
                   text(" "),
                   text(" MID ") | color(Color::Black) | bgcolor(Color::YellowLight),
                   text(" "),
                   text(" HIGH ") | color(Color::Black) | bgcolor(Color::RedLight),
                   text("  bandwidth pressure coloring") | color(Color::GrayLight),
               });

  auto topic_panel_body = vbox({
      legend,
      separator(),
      topic_list->Render() | frame | vscroll_indicator | flex,
      separator(),
      transfer_summary,
  });

  return hbox({
             Panel("Topic Matrix", topic_panel_body) | flex,
             Panel("Inspector", details) | size(WIDTH, EQUAL, 38),
         }) |
         flex;
}

}  // namespace tui
}  // namespace swarm_ros_bridge
