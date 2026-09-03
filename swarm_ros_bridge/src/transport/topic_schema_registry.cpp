#include "transport/topic_schema_registry.hpp"

namespace swarm_ros_bridge {
namespace transport {

namespace {

void SetError(const std::string& value, std::string* error) {
  if (error != nullptr) {
    *error = value;
  }
}

}  // namespace

const char* TopicSchemaStateName(TopicSchemaState state) {
  switch (state) {
    case TopicSchemaState::kDiscovering:
      return "discovering";
    case TopicSchemaState::kReady:
      return "ready";
    case TopicSchemaState::kConflict:
      return "conflict";
  }
  return "conflict";
}

bool TopicSchemaRegistry::Register(const std::string& rule_id,
                                   const std::string& source_host,
                                   const TopicSchema& schema,
                                   std::string* error) {
  std::string validation_error;
  if (rule_id.empty() || source_host.empty()) {
    validation_error = "schema rule id and source host must not be empty";
  } else if (!ValidateTopicSchema(schema, &validation_error)) {
    // Keep the detailed validation error.
  }

  std::lock_guard<std::mutex> lock(mutex_);
  Entry& entry = entries_[rule_id];
  if (entry.state == TopicSchemaState::kConflict) {
    SetError(entry.error, error);
    return false;
  }
  if (!validation_error.empty()) {
    entry.state = TopicSchemaState::kConflict;
    entry.error = source_host + ": " + validation_error;
    SetError(entry.error, error);
    return false;
  }

  const auto existing_source = entry.sources.find(source_host);
  if (existing_source != entry.sources.end()) {
    std::string compatibility_error;
    if (!TopicSchemasCompatible(existing_source->second, schema,
                                &compatibility_error)) {
      entry.state = TopicSchemaState::kConflict;
      entry.error = source_host + " changed schema: " + compatibility_error;
      SetError(entry.error, error);
      return false;
    }
    return true;
  }

  if (entry.sources.empty()) {
    entry.reference = schema;
  } else {
    std::string compatibility_error;
    if (!TopicSchemasCompatible(entry.reference, schema, &compatibility_error)) {
      entry.state = TopicSchemaState::kConflict;
      entry.error = "schema conflict at " + source_host + ": " +
                    compatibility_error;
      SetError(entry.error, error);
      return false;
    }
  }
  entry.sources.emplace(source_host, schema);
  entry.state = TopicSchemaState::kReady;
  return true;
}

void TopicSchemaRegistry::MarkConflict(const std::string& rule_id,
                                       const std::string& error) {
  std::lock_guard<std::mutex> lock(mutex_);
  Entry& entry = entries_[rule_id];
  if (entry.state != TopicSchemaState::kConflict) {
    entry.state = TopicSchemaState::kConflict;
    entry.error = error.empty() ? "topic schema conflict" : error;
  }
}

TopicSchemaStatus TopicSchemaRegistry::GetStatus(
    const std::string& rule_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto item = entries_.find(rule_id);
  if (item == entries_.end()) {
    return {};
  }
  TopicSchemaStatus status;
  status.state = item->second.state;
  status.schema = item->second.reference;
  status.error = item->second.error;
  status.source_count = item->second.sources.size();
  return status;
}

}  // namespace transport
}  // namespace swarm_ros_bridge
