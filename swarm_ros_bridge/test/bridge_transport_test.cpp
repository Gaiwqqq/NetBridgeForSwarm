#include "transport/bridge_transport.hpp"

#include <cassert>
#include <cstdint>
#include <deque>
#include <iostream>
#include <string>
#include <vector>

using swarm_ros_bridge::transport::DecodeEnvelope;
using swarm_ros_bridge::transport::EncodeEnvelope;
using swarm_ros_bridge::transport::BoundedQueuePushResult;
using swarm_ros_bridge::transport::GetQosPolicy;
using swarm_ros_bridge::transport::PayloadKind;
using swarm_ros_bridge::transport::ParseAliveKey;
using swarm_ros_bridge::transport::PushToBoundedEnvelopeQueue;
using swarm_ros_bridge::transport::QosClass;
using swarm_ros_bridge::transport::ServiceKey;
using swarm_ros_bridge::transport::TopicDirectedKey;
using swarm_ros_bridge::transport::TopicFanoutKey;
using swarm_ros_bridge::transport::TransportEnvelope;
using swarm_ros_bridge::transport::ValidateEnvelopeMetadata;

int main() {
  TransportEnvelope input;
  input.payload_kind = PayloadKind::kJpeg;
  input.sequence = 0x0102030405060708ULL;
  input.source_time_ns = 1699999999123456789LL;
  input.source_header_sequence = 42;
  input.source_host = "drone2";
  input.ros_type = "sensor_msgs/Image";
  input.ros_md5 = "060021388200f6f0f447d0fcd9c64743";
  input.frame_id = "camera_optical";
  input.payload = {0xff, 0xd8, 0x00, 0x7f, 0xff, 0xd9};

  std::vector<std::uint8_t> wire;
  std::string error;
  assert(EncodeEnvelope(input, &wire, &error));
  assert(wire.size() > input.payload.size());
  assert(wire[0] == 'N' && wire[1] == 'B' && wire[2] == 'Z' && wire[3] == '1');
  // Sequence is encoded in network byte order, independent of x86/ARM64 endianness.
  assert(wire[8] == 0x01 && wire[9] == 0x02 && wire[14] == 0x07 &&
         wire[15] == 0x08);

  TransportEnvelope output;
  assert(DecodeEnvelope(wire.data(), wire.size(), &output, &error));
  assert(output.payload_kind == input.payload_kind);
  assert(output.sequence == input.sequence);
  assert(output.source_time_ns == input.source_time_ns);
  assert(output.source_header_sequence == input.source_header_sequence);
  assert(output.source_host == input.source_host);
  assert(output.ros_type == input.ros_type);
  assert(output.ros_md5 == input.ros_md5);
  assert(output.frame_id == input.frame_id);
  assert(output.payload == input.payload);
  assert(ValidateEnvelopeMetadata(output, input.ros_type, input.ros_md5, &error));
  assert(!ValidateEnvelopeMetadata(output, "nav_msgs/Odometry", input.ros_md5,
                                   &error));
  assert(!ValidateEnvelopeMetadata(output, input.ros_type, "bad-md5", &error));

  std::vector<std::uint8_t> truncated = wire;
  truncated.pop_back();
  assert(!DecodeEnvelope(truncated.data(), truncated.size(), &output, &error));
  std::vector<std::uint8_t> trailing = wire;
  trailing.push_back(0);
  assert(!DecodeEnvelope(trailing.data(), trailing.size(), &output, &error));
  std::vector<std::uint8_t> bad_version = wire;
  bad_version[5] = 2;
  assert(!DecodeEnvelope(bad_version.data(), bad_version.size(), &output, &error));

  assert(TopicFanoutKey("drone1", "/odom") ==
         "netbridge/v1/topic/drone1/fanout/odom");
  assert(TopicDirectedKey("*", "drone3", "//goal/") ==
         "netbridge/v1/topic/*/dst/drone3/goal");
  assert(ServiceKey("drone1", "/add_two_ints") ==
         "netbridge/v1/service/drone1/add_two_ints");
  std::string alive_hostname;
  assert(ParseAliveKey("netbridge/v1/alive/drone7", &alive_hostname));
  assert(alive_hostname == "drone7");
  assert(!ParseAliveKey("netbridge/v1/alive/", &alive_hostname));
  assert(!ParseAliveKey("netbridge/v1/alive/drone7/extra", &alive_hostname));
  assert(!ParseAliveKey("netbridge/v2/alive/drone7", &alive_hostname));
  assert(!ParseAliveKey("netbridge/v1/alive/drone7", nullptr));

  const auto command = GetQosPolicy(QosClass::kCommand);
  const auto state = GetQosPolicy(QosClass::kState);
  const auto bulk = GetQosPolicy(QosClass::kBulk);
  const auto service = GetQosPolicy(QosClass::kService);
  assert(command.reliable && command.block_on_congestion && command.express);
  assert(!state.reliable && state.keep_latest && state.queue_capacity == 1);
  assert(!bulk.reliable && bulk.keep_latest && bulk.priority == "data_low");
  assert(service.reliable && service.block_on_congestion && service.express);

  std::deque<TransportEnvelope> latest_queue;
  TransportEnvelope first;
  first.sequence = 1;
  assert(PushToBoundedEnvelopeQueue(QosClass::kState, std::move(first),
                                    &latest_queue) ==
         BoundedQueuePushResult::kEnqueued);
  TransportEnvelope second;
  second.sequence = 2;
  assert(PushToBoundedEnvelopeQueue(QosClass::kState, std::move(second),
                                    &latest_queue) ==
         BoundedQueuePushResult::kReplacedOldest);
  assert(latest_queue.size() == 1 && latest_queue.front().sequence == 2);

  std::deque<TransportEnvelope> command_queue(command.queue_capacity);
  TransportEnvelope overflow;
  overflow.sequence = 99;
  assert(PushToBoundedEnvelopeQueue(QosClass::kCommand, std::move(overflow),
                                    &command_queue) ==
         BoundedQueuePushResult::kRejected);
  assert(command_queue.size() == command.queue_capacity);

  std::cout << "bridge transport wire/QoS tests passed\n";
  return 0;
}
