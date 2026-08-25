#include "diagnostics/topic_metrics.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

namespace diagnostics = swarm_ros_bridge::diagnostics;

namespace {

bool Near(double lhs, double rhs, double tolerance = 1e-6) {
  return std::fabs(lhs - rhs) <= tolerance;
}

}  // namespace

int main() {
  diagnostics::TopicRuntimeState state;
  state.topic_name = "/camera";
  state.msg_type = "sensor_msgs/Image";
  state.direction = "recv";
  state.rate_window_sec = 3.0;

  diagnostics::ObserveCompleteFrameSequence(10, &state);
  diagnostics::ObserveCompleteFrameSequence(11, &state);
  diagnostics::ObserveCompleteFrameSequence(14, &state);
  diagnostics::ObserveCompleteFrameSequence(14, &state);  // duplicate
  assert(state.expected_frames == 5U);
  assert(state.transport_complete_frames == 3U);
  assert(state.inferred_lost_frames == 2U);

  state.total_received = 2U;  // one complete frame was replaced/failed decode
  const ros::Time now(100.0);
  state.window_start = ros::Time(99.0);
  state.last_recv_time = ros::Time(99.5);
  state.recent_recv_times = {ros::Time(99.0), ros::Time(99.5)};
  state.recent_recv_bytes = {{ros::Time(99.0), 1000U},
                             {ros::Time(99.5), 2000U}};
  const auto metrics = diagnostics::MakeTopicMetrics(state, now);
  assert(Near(metrics.image_loss_rate_pct, 40.0));
  assert(Near(metrics.complete_frame_success_rate_pct, 40.0));
  assert(Near(metrics.effective_recv_bandwidth_kbps, 24.0));
  assert(metrics.decoded_frames == 2U);

  diagnostics::ObserveCompleteFrameSequence(1, &state);  // sender restart
  assert(state.sequence_resets == 1U);
  assert(state.expected_frames == 6U);
  assert(state.transport_complete_frames == 4U);
  assert(state.inferred_lost_frames == 2U);

  std::cout << "image receive metric tests passed\n";
  return 0;
}
