#ifndef SWARM_ROS_BRIDGE_TRANSPORT_BRIDGE_TRANSPORT_HPP_
#define SWARM_ROS_BRIDGE_TRANSPORT_BRIDGE_TRANSPORT_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace swarm_ros_bridge {
namespace transport {

constexpr std::uint16_t kBridgeProtocolVersion = 2;
constexpr std::size_t kMaxEnvelopePayloadBytes = 512U * 1024U * 1024U;
constexpr std::size_t kMaxRosMessageDefinitionBytes = 16U * 1024U * 1024U;

enum class PayloadKind : std::uint8_t {
  kRosSerialized = 0,
  kJpeg = 1,
  kPointCloudCompressed = 2,
  kServiceRequest = 3,
  kServiceResponse = 4,
  kTopicSchemaRequest = 5,
  kTopicSchemaResponse = 6,
};

enum class QosClass : std::uint8_t {
  kCommand,
  kState,
  kBulk,
  kService,
};

enum class BoundedQueuePushResult : std::uint8_t {
  kEnqueued,
  kReplacedOldest,
  kRejected,
};

struct QosPolicy {
  bool reliable{false};
  bool block_on_congestion{false};
  bool express{false};
  bool keep_latest{false};
  std::size_t queue_capacity{1};
  std::string priority;
};

struct TransportEnvelope {
  std::uint16_t protocol_version{kBridgeProtocolVersion};
  PayloadKind payload_kind{PayloadKind::kRosSerialized};
  std::uint64_t sequence{0};
  std::int64_t source_time_ns{0};
  std::uint32_t source_header_sequence{0};
  std::string source_host;
  std::string ros_type;
  std::string ros_md5;
  std::string ros_definition;
  std::string wire_codec;
  std::string routing_mode;
  bool latching{false};
  std::string frame_id;
  std::vector<std::uint8_t> payload;
};

struct ZenohTransportConfig {
  std::string mode{"peer"};
  bool multicast_scouting{true};
  bool gossip_scouting{true};
  bool compression_enabled{false};
  bool enable_liveliness{true};
  std::string multicast_scouting_address;
  std::vector<std::string> listen_endpoints;
  std::vector<std::string> connect_endpoints;
  std::vector<std::string> allowed_link_protocols;
  std::uint64_t service_timeout_ms{1000};
  std::size_t service_worker_threads{2};
  std::size_t service_queue_capacity{64};
};

struct TransportStats {
  bool session_open{false};
  bool link_connected{false};
  std::uint64_t connected_peer_count{0};
  std::uint64_t connected_router_count{0};
  std::uint64_t reconnect_count{0};
  std::uint64_t receive_decode_errors{0};
  std::uint64_t callback_drops{0};
  std::uint64_t service_timeouts{0};
  std::uint64_t service_queue_drops{0};
};

struct NodePresence {
  std::string hostname;
  bool online{false};
  std::uint64_t state_age_ms{0};
  std::uint64_t online_transitions{0};
};

bool EncodeEnvelope(const TransportEnvelope& envelope,
                    std::vector<std::uint8_t>* output,
                    std::string* error = nullptr);
bool DecodeEnvelope(const std::uint8_t* data,
                    std::size_t size,
                    TransportEnvelope* envelope,
                    std::string* error = nullptr);
bool ValidateEnvelopeMetadata(const TransportEnvelope& envelope,
                              const std::string& expected_ros_type,
                              const std::string& expected_ros_md5,
                              std::string* error = nullptr);

struct TopicSchema {
  std::string ros_type;
  std::string ros_md5;
  std::string ros_definition;
  std::string wire_codec{"ros1"};
  std::string routing_mode{"fanout"};
  bool latching{false};
};

bool ValidateTopicSchema(const TopicSchema& schema,
                         std::string* error = nullptr);
bool TopicSchemasCompatible(const TopicSchema& lhs,
                            const TopicSchema& rhs,
                            std::string* error = nullptr);
TopicSchema TopicSchemaFromEnvelope(const TransportEnvelope& envelope);
void SetEnvelopeTopicSchema(const TopicSchema& schema,
                            bool include_definition,
                            TransportEnvelope* envelope);

bool ParseQosClass(const std::string& value, QosClass* qos_class);
const char* QosClassName(QosClass qos_class);
QosPolicy GetQosPolicy(QosClass qos_class);
BoundedQueuePushResult PushToBoundedEnvelopeQueue(
    QosClass qos_class,
    TransportEnvelope&& envelope,
    std::deque<TransportEnvelope>* queue);

std::string TopicFanoutKey(const std::string& source_host,
                           const std::string& ros_topic);
std::string TopicDirectedKey(const std::string& source_host,
                             const std::string& destination_host,
                             const std::string& ros_topic);
std::string ServiceKey(const std::string& server_host,
                       const std::string& ros_service);
std::string TopicSchemaKey(const std::string& source_host,
                           const std::string& ros_topic);
std::string AliveKey(const std::string& hostname);
bool ParseAliveKey(const std::string& key, std::string* hostname);

class TransportPublisher {
 public:
  virtual ~TransportPublisher() = default;
  virtual bool Publish(const TransportEnvelope& envelope,
                       std::string* error = nullptr) = 0;
};

class TransportSubscription {
 public:
  virtual ~TransportSubscription() = default;
};

class TransportQueryable {
 public:
  virtual ~TransportQueryable() = default;
};

class BridgeTransport {
 public:
  using EnvelopeCallback = std::function<void(TransportEnvelope&&)>;
  using QueryHandler = std::function<bool(const TransportEnvelope&,
                                          TransportEnvelope*,
                                          std::string*)>;

  virtual ~BridgeTransport() = default;
  virtual std::shared_ptr<TransportPublisher> DeclarePublisher(
      const std::string& key, QosClass qos_class) = 0;
  virtual std::shared_ptr<TransportSubscription> Subscribe(
      const std::string& key_expression, EnvelopeCallback callback) = 0;
  virtual std::shared_ptr<TransportQueryable> DeclareQueryable(
      const std::string& key, QueryHandler handler) = 0;
  virtual bool Query(const std::string& key,
                     const TransportEnvelope& request,
                     std::uint64_t timeout_ms,
                     TransportEnvelope* response,
                     std::string* error = nullptr) = 0;
  virtual TransportStats GetStats() const = 0;
  virtual std::vector<NodePresence> GetNodePresence() const = 0;
  virtual void Close() = 0;
};

}  // namespace transport
}  // namespace swarm_ros_bridge

#endif  // SWARM_ROS_BRIDGE_TRANSPORT_BRIDGE_TRANSPORT_HPP_
