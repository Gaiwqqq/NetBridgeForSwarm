#ifndef SWARM_ROS_BRIDGE_TRANSPORT_TOPIC_SCHEMA_REGISTRY_HPP_
#define SWARM_ROS_BRIDGE_TRANSPORT_TOPIC_SCHEMA_REGISTRY_HPP_

#include "transport/bridge_transport.hpp"

#include <map>
#include <mutex>
#include <string>

namespace swarm_ros_bridge {
namespace transport {

enum class TopicSchemaState {
  kDiscovering,
  kReady,
  kConflict,
};

struct TopicSchemaStatus {
  TopicSchemaState state{TopicSchemaState::kDiscovering};
  TopicSchema schema;
  std::string error;
  std::size_t source_count{0};
};

const char* TopicSchemaStateName(TopicSchemaState state);

// Holds the process-lifetime schema decision for every configured topic rule.
// A conflict is deliberately sticky: there is no API to replace or clear it.
class TopicSchemaRegistry {
 public:
  bool Register(const std::string& rule_id,
                const std::string& source_host,
                const TopicSchema& schema,
                std::string* error = nullptr);
  void MarkConflict(const std::string& rule_id, const std::string& error);
  TopicSchemaStatus GetStatus(const std::string& rule_id) const;

 private:
  struct Entry {
    TopicSchemaState state{TopicSchemaState::kDiscovering};
    TopicSchema reference;
    std::map<std::string, TopicSchema> sources;
    std::string error;
  };

  mutable std::mutex mutex_;
  std::map<std::string, Entry> entries_;
};

}  // namespace transport
}  // namespace swarm_ros_bridge

#endif  // SWARM_ROS_BRIDGE_TRANSPORT_TOPIC_SCHEMA_REGISTRY_HPP_
