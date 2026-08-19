#include "transport/bridge_transport.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <sstream>

namespace swarm_ros_bridge {
namespace transport {
namespace {

constexpr std::array<std::uint8_t, 4> kMagic{{'N', 'B', 'Z', '1'}};

void SetError(const std::string& value, std::string* error) {
  if (error != nullptr) {
    *error = value;
  }
}

template <typename T>
void AppendUnsigned(T value, std::vector<std::uint8_t>* output) {
  for (std::size_t i = 0; i < sizeof(T); ++i) {
    const std::size_t shift = (sizeof(T) - i - 1U) * 8U;
    output->push_back(static_cast<std::uint8_t>((value >> shift) & 0xffU));
  }
}

template <typename T>
bool ReadUnsigned(const std::uint8_t** cursor, const std::uint8_t* end, T* value) {
  if (cursor == nullptr || *cursor == nullptr || value == nullptr ||
      static_cast<std::size_t>(end - *cursor) < sizeof(T)) {
    return false;
  }
  T result = 0;
  for (std::size_t i = 0; i < sizeof(T); ++i) {
    result = static_cast<T>((result << 8U) | *(*cursor)++);
  }
  *value = result;
  return true;
}

bool AppendString(const std::string& value,
                  std::vector<std::uint8_t>* output,
                  std::string* error) {
  if (value.size() > std::numeric_limits<std::uint16_t>::max()) {
    SetError("envelope string exceeds uint16 length", error);
    return false;
  }
  AppendUnsigned<std::uint16_t>(static_cast<std::uint16_t>(value.size()), output);
  output->insert(output->end(), value.begin(), value.end());
  return true;
}

bool ReadString(const std::uint8_t** cursor,
                const std::uint8_t* end,
                std::string* value) {
  std::uint16_t length = 0;
  if (!ReadUnsigned(cursor, end, &length) ||
      static_cast<std::size_t>(end - *cursor) < length) {
    return false;
  }
  value->assign(reinterpret_cast<const char*>(*cursor), length);
  *cursor += length;
  return true;
}

std::string NormalizeRosName(const std::string& name) {
  std::string normalized;
  normalized.reserve(name.size());
  bool previous_slash = true;
  for (const char ch : name) {
    if (ch == '/') {
      if (!previous_slash) {
        normalized.push_back('/');
      }
      previous_slash = true;
      continue;
    }
    normalized.push_back(ch);
    previous_slash = false;
  }
  while (!normalized.empty() && normalized.back() == '/') {
    normalized.pop_back();
  }
  return normalized;
}

}  // namespace

bool EncodeEnvelope(const TransportEnvelope& envelope,
                    std::vector<std::uint8_t>* output,
                    std::string* error) {
  if (output == nullptr) {
    SetError("output is null", error);
    return false;
  }
  if (envelope.protocol_version != kBridgeProtocolVersion) {
    SetError("unsupported envelope protocol version", error);
    return false;
  }
  if (envelope.payload.size() > kMaxEnvelopePayloadBytes ||
      envelope.payload.size() > std::numeric_limits<std::uint32_t>::max()) {
    SetError("envelope payload exceeds configured limit", error);
    return false;
  }

  output->clear();
  output->reserve(40U + envelope.source_host.size() + envelope.ros_type.size() +
                  envelope.ros_md5.size() + envelope.frame_id.size() +
                  envelope.payload.size());
  output->insert(output->end(), kMagic.begin(), kMagic.end());
  AppendUnsigned<std::uint16_t>(envelope.protocol_version, output);
  output->push_back(static_cast<std::uint8_t>(envelope.payload_kind));
  output->push_back(0U);  // Reserved flags.
  AppendUnsigned<std::uint64_t>(envelope.sequence, output);
  AppendUnsigned<std::uint64_t>(static_cast<std::uint64_t>(envelope.source_time_ns), output);
  AppendUnsigned<std::uint32_t>(envelope.source_header_sequence, output);
  if (!AppendString(envelope.source_host, output, error) ||
      !AppendString(envelope.ros_type, output, error) ||
      !AppendString(envelope.ros_md5, output, error) ||
      !AppendString(envelope.frame_id, output, error)) {
    output->clear();
    return false;
  }
  AppendUnsigned<std::uint32_t>(static_cast<std::uint32_t>(envelope.payload.size()), output);
  output->insert(output->end(), envelope.payload.begin(), envelope.payload.end());
  return true;
}

bool DecodeEnvelope(const std::uint8_t* data,
                    std::size_t size,
                    TransportEnvelope* envelope,
                    std::string* error) {
  if (data == nullptr || envelope == nullptr) {
    SetError("input or envelope is null", error);
    return false;
  }
  if (size < kMagic.size() || !std::equal(kMagic.begin(), kMagic.end(), data)) {
    SetError("invalid envelope magic", error);
    return false;
  }

  const std::uint8_t* cursor = data + kMagic.size();
  const std::uint8_t* const end = data + size;
  std::uint16_t version = 0;
  std::uint8_t payload_kind = 0;
  std::uint8_t flags = 0;
  std::uint64_t sequence = 0;
  std::uint64_t source_time_ns = 0;
  std::uint32_t source_header_sequence = 0;
  if (!ReadUnsigned(&cursor, end, &version) ||
      !ReadUnsigned(&cursor, end, &payload_kind) ||
      !ReadUnsigned(&cursor, end, &flags) ||
      !ReadUnsigned(&cursor, end, &sequence) ||
      !ReadUnsigned(&cursor, end, &source_time_ns) ||
      !ReadUnsigned(&cursor, end, &source_header_sequence)) {
    SetError("truncated envelope header", error);
    return false;
  }
  (void)flags;
  if (version != kBridgeProtocolVersion) {
    SetError("unsupported envelope protocol version", error);
    return false;
  }
  if (payload_kind > static_cast<std::uint8_t>(PayloadKind::kServiceResponse)) {
    SetError("invalid envelope payload kind", error);
    return false;
  }

  TransportEnvelope decoded;
  decoded.protocol_version = version;
  decoded.payload_kind = static_cast<PayloadKind>(payload_kind);
  decoded.sequence = sequence;
  decoded.source_time_ns = static_cast<std::int64_t>(source_time_ns);
  decoded.source_header_sequence = source_header_sequence;
  if (!ReadString(&cursor, end, &decoded.source_host) ||
      !ReadString(&cursor, end, &decoded.ros_type) ||
      !ReadString(&cursor, end, &decoded.ros_md5) ||
      !ReadString(&cursor, end, &decoded.frame_id)) {
    SetError("truncated envelope metadata", error);
    return false;
  }

  std::uint32_t payload_length = 0;
  if (!ReadUnsigned(&cursor, end, &payload_length) ||
      payload_length > kMaxEnvelopePayloadBytes ||
      static_cast<std::size_t>(end - cursor) != payload_length) {
    SetError("invalid or truncated envelope payload", error);
    return false;
  }
  decoded.payload.assign(cursor, end);
  *envelope = std::move(decoded);
  return true;
}

bool ValidateEnvelopeMetadata(const TransportEnvelope& envelope,
                              const std::string& expected_ros_type,
                              const std::string& expected_ros_md5,
                              std::string* error) {
  if (envelope.protocol_version != kBridgeProtocolVersion) {
    SetError("envelope protocol version mismatch", error);
    return false;
  }
  if (envelope.ros_type != expected_ros_type) {
    SetError("ROS type mismatch: expected " + expected_ros_type + ", got " +
                 envelope.ros_type,
             error);
    return false;
  }
  if (envelope.ros_md5 != expected_ros_md5) {
    SetError("ROS MD5 mismatch for " + expected_ros_type, error);
    return false;
  }
  return true;
}

bool ParseQosClass(const std::string& value, QosClass* qos_class) {
  if (qos_class == nullptr) {
    return false;
  }
  if (value == "command") {
    *qos_class = QosClass::kCommand;
  } else if (value == "state") {
    *qos_class = QosClass::kState;
  } else if (value == "bulk") {
    *qos_class = QosClass::kBulk;
  } else if (value == "service") {
    *qos_class = QosClass::kService;
  } else {
    return false;
  }
  return true;
}

const char* QosClassName(QosClass qos_class) {
  switch (qos_class) {
    case QosClass::kCommand:
      return "command";
    case QosClass::kState:
      return "state";
    case QosClass::kBulk:
      return "bulk";
    case QosClass::kService:
      return "service";
  }
  return "unknown";
}

QosPolicy GetQosPolicy(QosClass qos_class) {
  switch (qos_class) {
    case QosClass::kCommand:
      return {true, true, true, false, 64U, "real_time"};
    case QosClass::kState:
      return {false, false, true, true, 1U, "data_high"};
    case QosClass::kBulk:
      return {false, false, false, true, 1U, "data_low"};
    case QosClass::kService:
      return {true, true, true, false, 64U, "interactive_high"};
  }
  return {};
}

BoundedQueuePushResult PushToBoundedEnvelopeQueue(
    QosClass qos_class,
    TransportEnvelope&& envelope,
    std::deque<TransportEnvelope>* queue) {
  if (queue == nullptr) {
    return BoundedQueuePushResult::kRejected;
  }
  const QosPolicy policy = GetQosPolicy(qos_class);
  const std::size_t capacity = std::max<std::size_t>(1U, policy.queue_capacity);
  BoundedQueuePushResult result = BoundedQueuePushResult::kEnqueued;
  if (queue->size() >= capacity) {
    if (!policy.keep_latest) {
      return BoundedQueuePushResult::kRejected;
    }
    queue->pop_front();
    result = BoundedQueuePushResult::kReplacedOldest;
  }
  queue->push_back(std::move(envelope));
  return result;
}

std::string TopicFanoutKey(const std::string& source_host,
                           const std::string& ros_topic) {
  return "netbridge/v1/topic/" + source_host + "/fanout/" + NormalizeRosName(ros_topic);
}

std::string TopicDirectedKey(const std::string& source_host,
                             const std::string& destination_host,
                             const std::string& ros_topic) {
  return "netbridge/v1/topic/" + source_host + "/dst/" + destination_host + "/" +
         NormalizeRosName(ros_topic);
}

std::string ServiceKey(const std::string& server_host,
                       const std::string& ros_service) {
  return "netbridge/v1/service/" + server_host + "/" + NormalizeRosName(ros_service);
}

std::string AliveKey(const std::string& hostname) {
  return "netbridge/v1/alive/" + hostname;
}

bool ParseAliveKey(const std::string& key, std::string* hostname) {
  static const std::string kPrefix = "netbridge/v1/alive/";
  if (hostname == nullptr || key.rfind(kPrefix, 0) != 0) {
    return false;
  }
  const std::string parsed = key.substr(kPrefix.size());
  if (parsed.empty() || parsed.find('/') != std::string::npos) {
    return false;
  }
  *hostname = parsed;
  return true;
}

}  // namespace transport
}  // namespace swarm_ros_bridge
