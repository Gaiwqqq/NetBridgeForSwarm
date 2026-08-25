#include "diagnostics/topic_metrics.hpp"

#include <algorithm>

namespace swarm_ros_bridge {
namespace diagnostics {

namespace {

double ComputeObservedWindow(const ros::Time& now,
                             const ros::Time& window_start,
                             double configured_window_sec) {
  if (window_start.isZero()) {
    return configured_window_sec;
  }
  const double observed = std::max(0.001, (now - window_start).toSec());
  return std::min(std::max(0.001, configured_window_sec), observed);
}

double ComputeStability(const TopicRuntimeState& state,
                        double send_rate_hz,
                        double recv_rate_hz) {
  if (state.configured_rate_hz <= 0.0) {
    return state.dropped_messages == 0 ? 100.0 : 85.0;
  }

  const double reference_rate =
      state.direction == "send" ? send_rate_hz : recv_rate_hz;
  const double rate_ratio =
      std::min(1.0, reference_rate / std::max(0.001, state.configured_rate_hz));
  const double drop_penalty =
      std::min(40.0, static_cast<double>(state.dropped_messages) * 2.0);
  const double latency_penalty =
      state.avg_latency_ms > 0.0 ? std::min(25.0, state.avg_latency_ms / 8.0) : 0.0;
  return std::max(0.0, std::min(100.0, rate_ratio * 100.0 - drop_penalty - latency_penalty));
}

}  // namespace

void ObserveCompleteFrameSequence(std::uint64_t sequence,
                                  TopicRuntimeState* state) {
  if (state == nullptr) {
    return;
  }
  if (!state->frame_sequence_initialized) {
    state->frame_sequence_initialized = true;
    state->last_frame_sequence = sequence;
    state->expected_frames++;
    state->transport_complete_frames++;
    return;
  }
  if (sequence == state->last_frame_sequence) {
    return;
  }
  if (sequence < state->last_frame_sequence) {
    state->sequence_resets++;
    state->last_frame_sequence = sequence;
    state->expected_frames++;
    state->transport_complete_frames++;
    return;
  }

  const std::uint64_t sequence_delta = sequence - state->last_frame_sequence;
  state->expected_frames += sequence_delta;
  state->inferred_lost_frames += sequence_delta - 1U;
  state->transport_complete_frames++;
  state->last_frame_sequence = sequence;
}

TopicMetrics MakeTopicMetrics(const TopicRuntimeState& state, const ros::Time& now) {
  TopicMetrics metrics;
  metrics.topic_name = state.topic_name;
  metrics.msg_type = state.msg_type;
  metrics.direction = state.direction;
  metrics.codec = state.codec;
  metrics.transport = state.transport;
  metrics.qos_class = state.qos_class;
  metrics.configured_rate_hz = state.configured_rate_hz;

  const ros::Time send_window_start =
      state.recent_send_times.empty() ? state.window_start : state.recent_send_times.front();
  const ros::Time recv_window_start =
      state.recent_recv_times.empty() ? state.window_start : state.recent_recv_times.front();
  const double send_elapsed = ComputeObservedWindow(
      now, send_window_start, state.rate_window_sec);
  const double recv_elapsed = ComputeObservedWindow(
      now, recv_window_start, state.rate_window_sec);
  metrics.send_rate_hz =
      state.recent_send_times.empty() ? 0.0
                                      : static_cast<double>(state.recent_send_times.size()) /
                                            send_elapsed;
  metrics.recv_rate_hz =
      state.recent_recv_times.empty() ? 0.0
                                      : static_cast<double>(state.recent_recv_times.size()) /
                                            recv_elapsed;

  std::size_t recent_bytes = 0;
  if (state.direction == "send") {
    for (const auto& item : state.recent_send_bytes) {
      recent_bytes += item.second;
    }
    metrics.bandwidth_kbps = recent_bytes == 0
                                 ? 0.0
                                 : static_cast<double>(recent_bytes) * 8.0 / 1000.0 / send_elapsed;
  } else {
    for (const auto& item : state.recent_recv_bytes) {
      recent_bytes += item.second;
    }
    metrics.bandwidth_kbps = recent_bytes == 0
                                 ? 0.0
                                 : static_cast<double>(recent_bytes) * 8.0 / 1000.0 / recv_elapsed;
    metrics.effective_recv_bandwidth_kbps = metrics.bandwidth_kbps;
  }
  metrics.avg_latency_ms = state.avg_latency_ms;
  metrics.jitter_ms = state.jitter_ms;
  metrics.adaptive_quality_enabled = state.adaptive_quality_enabled;
  metrics.configured_jpeg_quality = state.configured_jpeg_quality;
  metrics.current_jpeg_quality = state.current_jpeg_quality;
  metrics.target_bandwidth_kbps = state.target_bandwidth_kbps;
  metrics.packet_size = state.packet_size;
  metrics.total_messages = state.total_sent + state.total_received;
  metrics.dropped_messages = state.dropped_messages;
  metrics.transport_queue_drops = state.transport_queue_drops;
  metrics.expected_frames = state.expected_frames;
  metrics.transport_complete_frames = state.transport_complete_frames;
  metrics.decoded_frames = state.direction == "recv" ? state.total_received : 0U;
  metrics.inferred_lost_frames = state.inferred_lost_frames;
  metrics.sequence_resets = state.sequence_resets;
  if (state.msg_type == "sensor_msgs/Image" &&
      state.direction == "recv" && state.expected_frames > 0U) {
    metrics.image_loss_rate_pct =
        std::min(100.0, static_cast<double>(state.inferred_lost_frames) * 100.0 /
                            static_cast<double>(state.expected_frames));
    metrics.complete_frame_success_rate_pct =
        std::min(100.0, static_cast<double>(state.total_received) * 100.0 /
                            static_cast<double>(state.expected_frames));
  }
  metrics.last_recv_age_ms =
      state.last_recv_time.isZero() ? -1.0 : (now - state.last_recv_time).toSec() * 1000.0;
  metrics.stability_score = ComputeStability(state, metrics.send_rate_hz, metrics.recv_rate_hz);
  return metrics;
}

}  // namespace diagnostics
}  // namespace swarm_ros_bridge
