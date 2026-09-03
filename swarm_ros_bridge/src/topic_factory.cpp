#include "topic_factory.h"

#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/PoseStamped.h"
#include "opencv2/opencv.hpp"
#include "pcl/compression/octree_pointcloud_compression.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/message_traits.h"
#include "ros/serialization.h"
#include "sensor_msgs/image_encodings.h"
#include "swarm_ros_bridge/PtCloudCompress.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace {

std::string RosTypeMd5(const std::string& msg_type) {
#define X(type, classname)                         \
  if (msg_type == type) {                         \
    return ros::message_traits::MD5Sum<classname>::value(); \
  }
  SPECIALIZED_MSGS_MACRO
  MSGS_MACRO
#undef X
  return {};
}

template <typename T>
std::vector<std::uint8_t> SerializeRosMessage(const T& message) {
  const std::size_t size = ros::serialization::serializationLength(message);
  std::vector<std::uint8_t> output(size);
  ros::serialization::OStream stream(output.data(), output.size());
  ros::serialization::serialize(stream, message);
  return output;
}

template <typename T>
ros::Publisher AdvertisePreservingHeader(
    const std::string& topic_name,
    const std::shared_ptr<ros::NodeHandle>& node,
    bool latching) {
  ros::AdvertiseOptions options;
  options.init<T>(topic_name, PUB_QUEUE_SIZE);
  // roscpp normally overwrites Header.seq in the serialized output. The bridge
  // must publish the sequence received from the source host unchanged.
  options.has_header = false;
  options.latch = latching;
  return node->advertise(options);
}

std::int64_t RosTimeToNs(const ros::Time& stamp) {
  return stamp.isZero() ? 0 : static_cast<std::int64_t>(stamp.toNSec());
}

std::string WireCodecForTopic(const TopicCfg& topic,
                              const std::string& ros_type) {
  if (ros_type == "sensor_msgs/Image") {
    return "jpeg";
  }
  if (ros_type == "sensor_msgs/PointCloud2") {
    if (!topic.cloud_compress_) {
      return "pointcloud_raw";
    }
    return topic.cloud_codec_ == "draco" ? "pointcloud_draco"
                                           : "pointcloud_pcl_octree";
  }
  if (ros_type == "nav_msgs/Odometry") {
    return "odom_pose";
  }
  return "ros1";
}

swarm_ros_bridge::transport::PayloadKind ExpectedPayloadKind(
    const std::string& wire_codec) {
  if (wire_codec == "jpeg") {
    return swarm_ros_bridge::transport::PayloadKind::kJpeg;
  }
  if (wire_codec == "pointcloud_draco" ||
      wire_codec == "pointcloud_pcl_octree") {
    return swarm_ros_bridge::transport::PayloadKind::kPointCloudCompressed;
  }
  return swarm_ros_bridge::transport::PayloadKind::kRosSerialized;
}

template <typename T>
bool ExtractRegisteredDestinations(
    const topic_tools::ShapeShifter& message,
    std::vector<std::string>* destinations) {
  if constexpr (has_data_to_drone_ids<T, std::vector<uint8_t>>::value) {
    const auto typed = message.instantiate<T>();
    for (const std::uint8_t id : typed->to_drone_ids) {
      destinations->push_back("drone" + std::to_string(id));
    }
    return true;
  }
  return false;
}

}  // namespace

TopicFactory::TopicFactory(
    const TopicCfg& topic_cfg,
    const TopicTransports& transports,
    std::shared_ptr<swarm_ros_bridge::transport::TopicSchemaRegistry>
        schema_registry,
    SEND_OR_RECV send_or_recv,
    const std::shared_ptr<ros::NodeHandle>& nh_public)
    : topic_cfg_(topic_cfg),
      send_or_recv_(send_or_recv),
      transports_(transports),
      schema_registry_(std::move(schema_registry)),
      nh_public_(nh_public) {
  if (transports_.control == nullptr || schema_registry_ == nullptr ||
      nh_public_ == nullptr) {
    throw std::invalid_argument(
        "TopicFactory requires control transport, schema registry and ROS node");
  }

  metrics_state_.topic_name = topic_cfg_.name_;
  metrics_state_.msg_type = "auto";
  metrics_state_.direction = send_or_recv_ == SEND ? "send" : "recv";
  metrics_state_.codec = "pending";
  metrics_state_.transport = "pending";
  metrics_state_.schema_state = "discovering";
  metrics_state_.qos_class =
      swarm_ros_bridge::transport::QosClassName(topic_cfg_.qos_class_);
  metrics_state_.configured_rate_hz = topic_cfg_.max_freq_;
  metrics_state_.window_start = ros::Time::now();
  metrics_state_.adaptive_quality_enabled = topic_cfg_.img_adaptive_quality_;
  metrics_state_.configured_jpeg_quality =
      static_cast<std::uint32_t>(topic_cfg_.img_jpeg_quality_);
  metrics_state_.current_jpeg_quality =
      static_cast<std::uint32_t>(topic_cfg_.img_jpeg_quality_);
  metrics_state_.target_bandwidth_kbps = topic_cfg_.img_target_bandwidth_kbps_;
  metrics_state_.quality_step = static_cast<std::uint32_t>(topic_cfg_.img_quality_step_);
  metrics_state_.adapt_cooldown_frames =
      static_cast<std::uint32_t>(topic_cfg_.img_adapt_cooldown_frames_);
  if (send_or_recv_ == SEND) {
    sub_last_time_ = ros::Time(0);
    {
      std::lock_guard<std::mutex> lock(sender_subscription_mutex_);
      subscribeSenderLocked();
    }
    sender_watch_timer_ = nh_public_->createWallTimer(
        ros::WallDuration(0.2), &TopicFactory::watchSenderPublishers, this);
    INFO_MSG(" DISCOVER " << topic_cfg_.name_ << " | " << topic_cfg_.max_freq_
                           << "Hz");
  }
}

TopicFactory::~TopicFactory() { stopThread(); }

void TopicFactory::subscribeSenderLocked() {
  sub_ = nh_public_->subscribe<
      const ros::MessageEvent<topic_tools::ShapeShifter const>&>(
      topic_cfg_.name_, SUB_QUEUE_SIZE, &TopicFactory::shapeShifterCallback,
      this, ros::TransportHints().tcpNoDelay());
}

void TopicFactory::watchSenderPublishers(const ros::WallTimerEvent&) {
  if (send_or_recv_ != SEND || schemaConflict()) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(schema_mutex_);
    if (!schema_ready_) {
      return;
    }
  }

  std::lock_guard<std::mutex> lock(sender_subscription_mutex_);
  if (sub_.getNumPublishers() > 0U) {
    sender_publisher_seen_ = true;
    sender_rearmed_after_disconnect_ = false;
    return;
  }
  if (!sender_publisher_seen_ || sender_rearmed_after_disconnect_) {
    return;
  }

  // roscpp specializes a wildcard ShapeShifter subscription after its first
  // connection. Recreate it once the publisher disconnects so that a later
  // publisher with a changed MD5 reaches configureSenderSchema(), where the
  // rule is permanently quarantined instead of being silently rejected by the
  // TCPROS handshake.
  sub_.shutdown();
  subscribeSenderLocked();
  sender_publisher_seen_ = false;
  sender_rearmed_after_disconnect_ = true;
  ROS_INFO_STREAM("[TopicFactory] rearmed type discovery for "
                  << topic_cfg_.name_ << " after publisher disconnect");
}

void TopicFactory::shapeShifterCallback(
    const ros::MessageEvent<const topic_tools::ShapeShifter>& event) {
  if (sendFreqControl()) {
    return;
  }

  const topic_tools::ShapeShifter& shape = *event.getConstMessage();
  bool latching = false;
  const auto connection_header = event.getConnectionHeaderPtr();
  if (connection_header != nullptr) {
    const auto item = connection_header->find("latching");
    latching = item != connection_header->end() && item->second == "1";
  }
  std::string schema_error;
  if (!configureSenderSchema(shape, latching, &schema_error)) {
    ROS_ERROR_STREAM_THROTTLE(
        1.0, "[TopicFactory] schema rejected for " << topic_cfg_.name_ << ": "
                                                     << schema_error);
    if (schemaConflict()) {
      applySchemaQuarantine(schema_error);
    }
    recordDrop();
    return;
  }
  if (schemaConflict(&schema_error)) {
    applySchemaQuarantine(schema_error);
    ROS_ERROR_STREAM_THROTTLE(
        1.0, "[TopicFactory] quarantined " << topic_cfg_.wire_name_ << ": "
                                            << schema_error);
    recordDrop();
    return;
  }

  swarm_ros_bridge::transport::TopicSchema schema;
  {
    std::lock_guard<std::mutex> lock(schema_mutex_);
    schema = schema_;
  }
  swarm_ros_bridge::transport::TransportEnvelope envelope;
  envelope.payload_kind = swarm_ros_bridge::transport::PayloadKind::kRosSerialized;
  envelope.sequence = sequence_.fetch_add(1U) + 1U;
  envelope.source_host = topic_cfg_.my_hostname_;
  swarm_ros_bridge::transport::SetEnvelopeTopicSchema(schema, false, &envelope);
  envelope.source_time_ns = RosTimeToNs(ros::Time::now());

  try {
    if (schema.ros_type == "nav_msgs/Odometry") {
      const auto typed = shape.instantiate<nav_msgs::Odometry>();
      envelope.source_time_ns = RosTimeToNs(typed->header.stamp);
      envelope.source_header_sequence = typed->header.seq;
      envelope.frame_id = typed->header.frame_id;
      geometry_msgs::PoseStamped pose;
      pose.header = typed->header;
      pose.header.frame_id = typed->child_frame_id;
      pose.pose = typed->pose.pose;
      envelope.payload = SerializeRosMessage(pose);
    } else if (schema.ros_type == "sensor_msgs/Image") {
      const auto image_msg = shape.instantiate<sensor_msgs::Image>();
      envelope.source_time_ns = RosTimeToNs(image_msg->header.stamp);
      envelope.source_header_sequence = image_msg->header.seq;
      envelope.frame_id = image_msg->header.frame_id;
      cv_bridge::CvImageConstPtr cv_ptr;
      if (image_msg->encoding == sensor_msgs::image_encodings::BGR8) {
        cv_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
      } else {
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
      }
      cv::Mat resized;
      if (std::fabs(topic_cfg_.img_resize_rate_ - 1.0) > 1e-6) {
        const cv::Size target_size(
            std::max(1, static_cast<int>(std::floor(cv_ptr->image.cols *
                                                   topic_cfg_.img_resize_rate_))),
            std::max(1, static_cast<int>(std::floor(cv_ptr->image.rows *
                                                   topic_cfg_.img_resize_rate_))));
        cv::resize(cv_ptr->image, resized, target_size);
      } else {
        resized = cv_ptr->image;
      }
      const std::vector<int> parameters{
          cv::IMWRITE_JPEG_QUALITY, currentImageJpegQuality()};
      if (!cv::imencode(".jpg", resized, envelope.payload, parameters)) {
        throw std::runtime_error("JPEG encode failed");
      }
      envelope.payload_kind = swarm_ros_bridge::transport::PayloadKind::kJpeg;
    } else if (schema.ros_type == "sensor_msgs/PointCloud2") {
      const auto typed = shape.instantiate<sensor_msgs::PointCloud2>();
      envelope.source_time_ns = RosTimeToNs(typed->header.stamp);
      envelope.source_header_sequence = typed->header.seq;
      envelope.frame_id = typed->header.frame_id;
      ptCloudProcess(*typed, &envelope.payload);
      if (topic_cfg_.cloud_compress_) {
        envelope.payload_kind =
            swarm_ros_bridge::transport::PayloadKind::kPointCloudCompressed;
      }
    } else {
      envelope.payload = SerializeRosMessage(shape);
    }
  } catch (const std::exception& exception) {
    ROS_ERROR_STREAM("[TopicFactory] failed to encode " << topic_cfg_.name_ << ": "
                                                         << exception.what());
    recordDrop();
    return;
  }

  std::vector<std::string> destinations;
  const bool dynamic = extractDynamicDestinations(shape, &destinations);
  if (dynamic) {
    for (const std::string& target : destinations) {
      const auto publisher = dynamic_senders_.find(target);
      if (publisher == dynamic_senders_.end()) {
        INFO_MSG_RED("[TopicFactory] to_drone_ids not configured: " << target);
        recordDrop();
        continue;
      }
      std::string error;
      if (publisher->second->Publish(envelope, &error)) {
        recordSend(envelope.payload.size());
      } else {
        ROS_ERROR_STREAM("[TopicFactory] Zenoh publish failed: " << error);
        recordDrop();
      }
    }
  } else {
    std::string error;
    if (sender_ != nullptr && sender_->Publish(envelope, &error)) {
      recordSend(envelope.payload.size());
    } else {
      ROS_ERROR_STREAM_THROTTLE(1.0, "[TopicFactory] Zenoh publish failed: " << error);
      recordDrop();
    }
  }
}

bool TopicFactory::configureSenderSchema(
    const topic_tools::ShapeShifter& message,
    bool latching,
    std::string* error) {
  swarm_ros_bridge::transport::TopicSchema candidate;
  candidate.ros_type = message.getDataType();
  candidate.ros_md5 = message.getMD5Sum();
  candidate.ros_definition = message.getMessageDefinition();
  candidate.latching = latching;
  candidate.wire_codec = WireCodecForTopic(topic_cfg_, candidate.ros_type);
  std::vector<std::string> unused_destinations;
  candidate.routing_mode =
      extractDynamicDestinations(message, &unused_destinations)
          ? "to_drone_ids"
          : "fanout";

  const std::string compiled_md5 = RosTypeMd5(candidate.ros_type);
  if (!compiled_md5.empty() && compiled_md5 != candidate.ros_md5) {
    const std::string value = "local compiled MD5 does not match publisher for " +
                              candidate.ros_type;
    schema_registry_->MarkConflict(topic_cfg_.rule_id_, value);
    if (error != nullptr) {
      *error = value;
    }
    return false;
  }

  std::string registry_error;
  if (!schema_registry_->Register(topic_cfg_.rule_id_, topic_cfg_.my_hostname_,
                                  candidate, &registry_error)) {
    if (error != nullptr) {
      *error = registry_error;
    }
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(schema_mutex_);
    if (schema_ready_) {
      std::string compatibility_error;
      if (!swarm_ros_bridge::transport::TopicSchemasCompatible(
              schema_, candidate, &compatibility_error)) {
        schema_registry_->MarkConflict(topic_cfg_.rule_id_, compatibility_error);
        if (error != nullptr) {
          *error = compatibility_error;
        }
        return false;
      }
      return true;
    }
    schema_ = candidate;
    schema_ready_ = true;
  }

  try {
    active_transport_ = transportForSchema(candidate);
    if (candidate.routing_mode == "to_drone_ids") {
      for (const auto& destination : topic_cfg_.dst_hostname_map_) {
        if (!destination.second || destination.first == topic_cfg_.my_hostname_) {
          continue;
        }
        const std::string key = swarm_ros_bridge::transport::TopicDirectedKey(
            topic_cfg_.src_hostname_, destination.first, topic_cfg_.wire_name_);
        dynamic_senders_[destination.first] =
            active_transport_->DeclarePublisher(key, topic_cfg_.qos_class_);
      }
    } else {
      const std::string key = swarm_ros_bridge::transport::TopicFanoutKey(
          topic_cfg_.src_hostname_, topic_cfg_.wire_name_);
      sender_ = active_transport_->DeclarePublisher(key, topic_cfg_.qos_class_);
    }

    const std::string schema_key = swarm_ros_bridge::transport::TopicSchemaKey(
        topic_cfg_.src_hostname_, topic_cfg_.wire_name_);
    schema_queryable_ = transports_.control->DeclareQueryable(
        schema_key,
        [this](const swarm_ros_bridge::transport::TransportEnvelope& request,
               swarm_ros_bridge::transport::TransportEnvelope* response,
               std::string* query_error) {
          return handleSchemaQuery(request, response, query_error);
        });
    schema_publisher_ = transports_.control->DeclarePublisher(
        schema_key, swarm_ros_bridge::transport::QosClass::kCommand);
    swarm_ros_bridge::transport::TransportEnvelope announcement;
    announcement.payload_kind =
        swarm_ros_bridge::transport::PayloadKind::kTopicSchemaResponse;
    announcement.source_host = topic_cfg_.my_hostname_;
    swarm_ros_bridge::transport::SetEnvelopeTopicSchema(candidate, true,
                                                         &announcement);
    std::string publish_error;
    if (!schema_publisher_->Publish(announcement, &publish_error)) {
      ROS_WARN_STREAM("[TopicFactory] schema announcement failed for "
                      << topic_cfg_.wire_name_ << ": " << publish_error);
    }
  } catch (const std::exception& exception) {
    schema_registry_->MarkConflict(topic_cfg_.rule_id_, exception.what());
    if (error != nullptr) {
      *error = exception.what();
    }
    return false;
  }

  updateSchemaMetrics(candidate);
  INFO_MSG(" SCHEMA " << candidate.ros_type << " | " << candidate.ros_md5
                       << " | " << candidate.wire_codec << " | "
                       << candidate.routing_mode);
  return true;
}

bool TopicFactory::configureReceiverSchema(
    const swarm_ros_bridge::transport::TopicSchema& candidate,
    std::string* error) {
  if (candidate.routing_mode == "to_drone_ids" &&
      topic_cfg_.directed_name_.empty()) {
    const std::string value =
        "directed {id} topic requires a drone destination hostname";
    schema_registry_->MarkConflict(topic_cfg_.rule_id_, value);
    if (error != nullptr) {
      *error = value;
    }
    return false;
  }
  const std::string compiled_md5 = RosTypeMd5(candidate.ros_type);
  if (!compiled_md5.empty() && compiled_md5 != candidate.ros_md5) {
    const std::string value = "local compiled MD5 does not match remote schema for " +
                              candidate.ros_type;
    schema_registry_->MarkConflict(topic_cfg_.rule_id_, value);
    if (error != nullptr) {
      *error = value;
    }
    return false;
  }
  std::string registry_error;
  if (!schema_registry_->Register(topic_cfg_.rule_id_, topic_cfg_.src_hostname_,
                                  candidate, &registry_error)) {
    if (error != nullptr) {
      *error = registry_error;
    }
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(schema_mutex_);
    if (schema_ready_) {
      std::string compatibility_error;
      if (!swarm_ros_bridge::transport::TopicSchemasCompatible(
              schema_, candidate, &compatibility_error)) {
        schema_registry_->MarkConflict(topic_cfg_.rule_id_, compatibility_error);
        if (error != nullptr) {
          *error = compatibility_error;
        }
        return false;
      }
      return true;
    }
  }

  const std::string output_name =
      candidate.routing_mode == "to_drone_ids" &&
              !topic_cfg_.directed_name_.empty()
          ? topic_cfg_.directed_name_
          : topic_cfg_.fanout_name_;
  try {
    ros::Publisher publisher = advertiseForSchema(candidate, output_name);
    {
      std::lock_guard<std::mutex> lock(schema_mutex_);
      schema_ = candidate;
      schema_ready_ = true;
      topic_cfg_.name_ = output_name;
      pub_ = std::move(publisher);
    }
  } catch (const std::exception& exception) {
    schema_registry_->MarkConflict(topic_cfg_.rule_id_, exception.what());
    if (error != nullptr) {
      *error = exception.what();
    }
    return false;
  }
  active_transport_ = transportForSchema(candidate);
  updateSchemaMetrics(candidate);
  recv_condition_.notify_all();
  INFO_MSG(" READY " << candidate.ros_type << " | " << topic_cfg_.src_hostname_
                      << " -> " << output_name);
  return true;
}

bool TopicFactory::handleSchemaQuery(
    const swarm_ros_bridge::transport::TransportEnvelope& request,
    swarm_ros_bridge::transport::TransportEnvelope* response,
    std::string* error) {
  if (response == nullptr ||
      request.payload_kind !=
          swarm_ros_bridge::transport::PayloadKind::kTopicSchemaRequest) {
    if (error != nullptr) {
      *error = "invalid topic schema request";
    }
    return false;
  }
  const auto destination = topic_cfg_.dst_hostname_map_.find(request.source_host);
  if (destination == topic_cfg_.dst_hostname_map_.end() || !destination->second) {
    if (error != nullptr) {
      *error = "schema requester is not a configured destination";
    }
    return false;
  }
  std::lock_guard<std::mutex> lock(schema_mutex_);
  if (!schema_ready_) {
    if (error != nullptr) {
      *error = "topic schema has not been discovered";
    }
    return false;
  }
  response->payload_kind =
      swarm_ros_bridge::transport::PayloadKind::kTopicSchemaResponse;
  response->sequence = request.sequence;
  response->source_host = topic_cfg_.my_hostname_;
  swarm_ros_bridge::transport::SetEnvelopeTopicSchema(schema_, true, response);
  return true;
}

void TopicFactory::handleSchemaAnnouncement(
    swarm_ros_bridge::transport::TransportEnvelope&& envelope) {
  if (envelope.payload_kind !=
          swarm_ros_bridge::transport::PayloadKind::kTopicSchemaResponse ||
      envelope.source_host != topic_cfg_.src_hostname_) {
    return;
  }
  std::string error;
  if (!configureReceiverSchema(
          swarm_ros_bridge::transport::TopicSchemaFromEnvelope(envelope),
          &error)) {
    if (schemaConflict()) {
      applySchemaQuarantine(error);
    }
    ROS_ERROR_STREAM_THROTTLE(1.0, "[TopicFactory] schema announcement rejected: "
                                      << error);
  }
}

void TopicFactory::schemaResolveFunction() {
  const std::string key = swarm_ros_bridge::transport::TopicSchemaKey(
      topic_cfg_.src_hostname_, topic_cfg_.wire_name_);
  while (recv_thread_flag_.load()) {
    std::string conflict_error;
    if (schemaConflict(&conflict_error)) {
      applySchemaQuarantine(conflict_error);
    } else {
      swarm_ros_bridge::transport::TransportEnvelope request;
      request.payload_kind =
          swarm_ros_bridge::transport::PayloadKind::kTopicSchemaRequest;
      request.sequence = sequence_.fetch_add(1U) + 1U;
      request.source_host = topic_cfg_.my_hostname_;
      swarm_ros_bridge::transport::TransportEnvelope response;
      std::string query_error;
      if (transports_.control->Query(key, request, 500U, &response,
                                     &query_error) &&
          response.payload_kind ==
              swarm_ros_bridge::transport::PayloadKind::kTopicSchemaResponse &&
          response.source_host == topic_cfg_.src_hostname_) {
        std::string schema_error;
        if (!configureReceiverSchema(
                swarm_ros_bridge::transport::TopicSchemaFromEnvelope(response),
                &schema_error)) {
          if (schemaConflict()) {
            applySchemaQuarantine(schema_error);
          }
          ROS_ERROR_STREAM_THROTTLE(
              1.0, "[TopicFactory] schema query rejected for "
                       << topic_cfg_.wire_name_ << ": " << schema_error);
        }
      }
    }
    std::unique_lock<std::mutex> lock(schema_wait_mutex_);
    schema_condition_.wait_for(lock, std::chrono::milliseconds(500), [this]() {
      return !recv_thread_flag_.load();
    });
  }
}

bool TopicFactory::schemaConflict(std::string* error) const {
  const auto status = schema_registry_->GetStatus(topic_cfg_.rule_id_);
  if (status.state !=
      swarm_ros_bridge::transport::TopicSchemaState::kConflict) {
    return false;
  }
  if (error != nullptr) {
    *error = status.error;
  }
  return true;
}

void TopicFactory::applySchemaQuarantine(const std::string& error) {
  std::size_t queued = 0U;
  {
    std::lock_guard<std::mutex> lock(recv_mutex_);
    queued = recv_queue_.size();
    recv_queue_.clear();
  }
  {
    std::lock_guard<std::mutex> lock(schema_mutex_);
    schema_ready_ = false;
  }
  if (send_or_recv_ == SEND) {
    sender_watch_timer_.stop();
    {
      std::lock_guard<std::mutex> lock(sender_subscription_mutex_);
      sub_.shutdown();
    }
    sender_.reset();
    dynamic_senders_.clear();
    schema_publisher_.reset();
    schema_queryable_.reset();
  } else {
    pub_.shutdown();
  }
  {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    metrics_state_.schema_state = "conflict";
    metrics_state_.schema_error = error;
    metrics_state_.dropped_messages += queued;
    metrics_state_.transport_queue_drops += queued;
  }
  recv_condition_.notify_all();
}

std::shared_ptr<swarm_ros_bridge::transport::BridgeTransport>
TopicFactory::transportForSchema(
    const swarm_ros_bridge::transport::TopicSchema& schema) const {
  if (schema.ros_type == "sensor_msgs/Image" && transports_.image != nullptr) {
    return transports_.image;
  }
  if (schema.ros_type == "sensor_msgs/PointCloud2" &&
      transports_.cloud != nullptr) {
    return transports_.cloud;
  }
  return transports_.control;
}

ros::Publisher TopicFactory::advertiseForSchema(
    const swarm_ros_bridge::transport::TopicSchema& schema,
    const std::string& topic_name) const {
#define X(type, classname)                                                   \
  if (schema.ros_type == type) {                                             \
    return AdvertisePreservingHeader<classname>(topic_name, nh_public_,      \
                                                 schema.latching);           \
  }
  SPECIALIZED_MSGS_MACRO
#undef X
  topic_tools::ShapeShifter prototype;
  prototype.morph(schema.ros_md5, schema.ros_type, schema.ros_definition,
                  schema.latching ? "1" : "0");
  return prototype.advertise(*nh_public_, topic_name, PUB_QUEUE_SIZE,
                             schema.latching);
}

bool TopicFactory::extractDynamicDestinations(
    const topic_tools::ShapeShifter& message,
    std::vector<std::string>* destinations) const {
  if (destinations == nullptr) {
    return false;
  }
  destinations->clear();
#define X(type, classname)                                                   \
  if (message.getDataType() == type) {                                       \
    return ExtractRegisteredDestinations<classname>(message, destinations);  \
  }
  MSGS_MACRO
#undef X
  return false;
}

void TopicFactory::updateSchemaMetrics(
    const swarm_ros_bridge::transport::TopicSchema& schema) {
  std::lock_guard<std::mutex> lock(metrics_mutex_);
  metrics_state_.topic_name = topic_cfg_.name_;
  metrics_state_.msg_type = schema.ros_type;
  metrics_state_.codec = schema.wire_codec;
  metrics_state_.transport =
      schema.ros_type == "sensor_msgs/Image" && transports_.image != nullptr
          ? "zenoh-udp"
          : (schema.ros_type == "sensor_msgs/PointCloud2" &&
                     transports_.cloud != nullptr
                 ? "zenoh-cloud-tcp"
                 : (transports_.image != nullptr || transports_.cloud != nullptr
                        ? "zenoh-tcp"
                        : "zenoh"));
  metrics_state_.schema_state = "ready";
  metrics_state_.schema_md5 = schema.ros_md5;
  metrics_state_.schema_error.clear();
}

template <typename T>
void TopicFactory::ptCloudProcess(const T& msg, std::vector<std::uint8_t>* data) {
  if (data == nullptr) {
    throw std::invalid_argument("point cloud output is null");
  }
  sensor_msgs::PointCloud2 cloud_msg = msg;
  if (topic_cfg_.cloud_downsample_ > 0) {
    pcl::PCLPointCloud2::Ptr cloud_in(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2 cloud_downsampled;
    pcl_conversions::toPCL(cloud_msg, *cloud_in);
    pcl::VoxelGrid<pcl::PCLPointCloud2> filter;
    filter.setInputCloud(cloud_in);
    const float leaf = static_cast<float>(topic_cfg_.cloud_downsample_);
    filter.setLeafSize(leaf, leaf, leaf);
    filter.setDownsampleAllData(true);
    filter.filter(cloud_downsampled);
    pcl_conversions::fromPCL(cloud_downsampled, cloud_msg);
    cloud_msg.header = msg.header;
  }

  if (!topic_cfg_.cloud_compress_) {
    *data = SerializeRosMessage(cloud_msg);
    return;
  }

  swarm_ros_bridge::PtCloudCompress compressed;
  compressed.original_header = msg.header;
  compressed.sender_stamp = msg.header.stamp;
  compressed.original_width = cloud_msg.width;
  compressed.original_height = cloud_msg.height;
  compressed.original_frame_id = msg.header.frame_id;
  compressed.voxel_leaf_size = static_cast<float>(topic_cfg_.cloud_downsample_);
  compressed.receiver_stamp = true;
  compressed.codec = topic_cfg_.cloud_codec_;

  if (topic_cfg_.cloud_codec_ == "draco") {
    swarm_ros_bridge::transport::EncodedPointCloud encoded;
    if (!draco_codec_.Encode(cloud_msg, &encoded)) {
      throw std::runtime_error("Draco encode failed");
    }
    compressed.compressed_bytes = std::move(encoded.payload);
    compressed.format_version = encoded.format_version;
    compressed.point_type = encoded.point_type;
    compressed.original_fields = std::move(encoded.original_fields);
    compressed.original_is_bigendian = encoded.original_is_bigendian;
    compressed.original_point_step = encoded.original_point_step;
    compressed.original_row_step = encoded.original_row_step;
    compressed.original_is_dense = encoded.original_is_dense;
    compressed.sidecar_bytes = std::move(encoded.sidecar_bytes);
  } else {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(cloud_msg, *cloud_in);
    std::stringstream stream;
    pcl::io::OctreePointCloudCompression<pcl::PointXYZ> codec(
        pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR, false, 1e-3, 1e-3,
        false);
    codec.encodePointCloud(cloud_in, stream);
    const std::string bytes = stream.str();
    compressed.compressed_bytes.assign(bytes.begin(), bytes.end());
    compressed.codec = "pcl_octree";
  }
  *data = SerializeRosMessage(compressed);
}

bool TopicFactory::sendFreqControl() {
  const ros::Time now = ros::Time::now();
  if (std::fabs(topic_cfg_.max_freq_ - (-1.0)) <
      std::numeric_limits<double>::epsilon()) {
    return false;
  }
  const double minimum_interval = 1.0 / std::max(0.001, topic_cfg_.max_freq_);
  if (!sub_last_time_.isZero() &&
      (now - sub_last_time_).toSec() < minimum_interval) {
    recordDrop();
    return true;
  }
  sub_last_time_ = now;
  return false;
}

void TopicFactory::deserializePublish(
    const swarm_ros_bridge::transport::TransportEnvelope& envelope) {
  swarm_ros_bridge::transport::TopicSchema schema;
  {
    std::lock_guard<std::mutex> lock(schema_mutex_);
    if (!schema_ready_) {
      throw std::runtime_error("topic schema is not ready");
    }
    schema = schema_;
  }
  std::string metadata_error;
  if (!swarm_ros_bridge::transport::ValidateEnvelopeMetadata(
          envelope, schema.ros_type, schema.ros_md5, &metadata_error)) {
    throw std::runtime_error(metadata_error);
  }
  if (envelope.wire_codec != schema.wire_codec ||
      envelope.routing_mode != schema.routing_mode) {
    throw std::runtime_error("topic wire contract mismatch");
  }
  if (envelope.payload_kind != ExpectedPayloadKind(schema.wire_codec)) {
    throw std::runtime_error("topic payload kind does not match wire codec");
  }
#define X(type, classname)                 \
  if (schema.ros_type == type) {            \
    return deserializePub<classname>(envelope); \
  }
  SPECIALIZED_MSGS_MACRO
#undef X
  topic_tools::ShapeShifter message;
  message.morph(schema.ros_md5, schema.ros_type, schema.ros_definition,
                schema.latching ? "1" : "0");
  ros::serialization::IStream stream(
      const_cast<std::uint8_t*>(envelope.payload.data()),
      envelope.payload.size());
  ros::serialization::deserialize(stream, message);
  pub_.publish(message);
  recordReceive(envelope.payload.size(), inferLatencyMs(envelope.source_time_ns));
}

template <typename T>
void TopicFactory::deserializePub(
    const swarm_ros_bridge::transport::TransportEnvelope& envelope) {
  T msg;
  if constexpr (std::is_same<T, sensor_msgs::Image>::value) {
    if (envelope.payload_kind != swarm_ros_bridge::transport::PayloadKind::kJpeg) {
      throw std::runtime_error("image topic received a non-JPEG payload");
    }
    const cv::Mat encoded(1, static_cast<int>(envelope.payload.size()), CV_8UC1,
                          const_cast<std::uint8_t*>(envelope.payload.data()));
    const cv::Mat frame = cv::imdecode(encoded, cv::IMREAD_COLOR);
    if (frame.empty()) {
      throw std::runtime_error("JPEG decode failed");
    }
    std_msgs::Header header;
    header.seq = envelope.source_header_sequence;
    if (envelope.source_time_ns > 0) {
      header.stamp.fromNSec(static_cast<std::uint64_t>(envelope.source_time_ns));
    }
    header.frame_id = envelope.frame_id;
    msg = *cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frame)
               .toImageMsg();
  } else {
    ros::serialization::IStream stream(
        const_cast<std::uint8_t*>(envelope.payload.data()), envelope.payload.size());
    if constexpr (std::is_same<T, nav_msgs::Odometry>::value) {
      geometry_msgs::PoseStamped pose;
      ros::serialization::deserialize(stream, pose);
      msg.header = pose.header;
      msg.header.frame_id = "world";
      msg.child_frame_id = pose.header.frame_id;
      msg.pose.pose = pose.pose;
    } else if constexpr (std::is_same<T, sensor_msgs::PointCloud2>::value) {
      if (envelope.wire_codec == "pointcloud_draco" ||
          envelope.wire_codec == "pointcloud_pcl_octree") {
        swarm_ros_bridge::PtCloudCompress compressed;
        ros::serialization::deserialize(stream, compressed);
        if (compressed.codec == "draco") {
          swarm_ros_bridge::transport::EncodedPointCloud encoded_cloud;
          encoded_cloud.codec = compressed.codec;
          encoded_cloud.format_version = compressed.format_version;
          encoded_cloud.point_type = compressed.point_type;
          encoded_cloud.original_header = compressed.original_header;
          encoded_cloud.original_width = compressed.original_width;
          encoded_cloud.original_height = compressed.original_height;
          encoded_cloud.original_fields = std::move(compressed.original_fields);
          encoded_cloud.original_is_bigendian = compressed.original_is_bigendian;
          encoded_cloud.original_point_step = compressed.original_point_step;
          encoded_cloud.original_row_step = compressed.original_row_step;
          encoded_cloud.original_is_dense = compressed.original_is_dense;
          encoded_cloud.source_stamp = compressed.sender_stamp;
          encoded_cloud.receive_stamp = ros::Time::now();
          encoded_cloud.frame_id = compressed.original_frame_id;
          encoded_cloud.payload = std::move(compressed.compressed_bytes);
          encoded_cloud.sidecar_bytes = std::move(compressed.sidecar_bytes);
          if (!draco_codec_.Decode(encoded_cloud, &msg)) {
            throw std::runtime_error("Draco decode failed");
          }
          msg.header = compressed.original_header;
        } else {
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
          std::stringstream compressed_stream;
          compressed_stream.write(
              reinterpret_cast<const char*>(compressed.compressed_bytes.data()),
              static_cast<std::streamsize>(compressed.compressed_bytes.size()));
          pcl::io::OctreePointCloudCompression<pcl::PointXYZ> codec(
              pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR, false, 1e-3,
              1e-3, false);
          codec.decodePointCloud(compressed_stream, cloud);
          cloud->width = compressed.original_width;
          cloud->height = compressed.original_height;
          pcl::toROSMsg(*cloud, msg);
          msg.header = compressed.original_header;
        }
      } else {
        ros::serialization::deserialize(stream, msg);
      }
    } else {
      ros::serialization::deserialize(stream, msg);
    }
  }

  pub_.publish(msg);
  recordReceive(envelope.payload.size(), inferLatencyMs(envelope.source_time_ns));
}

void TopicFactory::enqueueEnvelope(
    swarm_ros_bridge::transport::TransportEnvelope&& envelope) {
  if (envelope.source_host != topic_cfg_.src_hostname_) {
    recordDrop();
    return;
  }
  const auto source = topic_cfg_.src_hostname_map_.find(envelope.source_host);
  if (source == topic_cfg_.src_hostname_map_.end() || !source->second) {
    recordDrop();
    return;
  }

  std::string conflict_error;
  if (schemaConflict(&conflict_error)) {
    applySchemaQuarantine(conflict_error);
    ROS_ERROR_STREAM_THROTTLE(1.0, "[TopicFactory] dropped quarantined topic "
                                      << topic_cfg_.wire_name_ << ": "
                                      << conflict_error);
    recordDrop();
    return;
  }

  swarm_ros_bridge::transport::TopicSchema schema;
  bool ready = false;
  {
    std::lock_guard<std::mutex> lock(schema_mutex_);
    ready = schema_ready_;
    if (ready) {
      schema = schema_;
    }
  }
  if (ready) {
    std::string metadata_error;
    if (!swarm_ros_bridge::transport::ValidateEnvelopeMetadata(
            envelope, schema.ros_type, schema.ros_md5, &metadata_error) ||
        envelope.wire_codec != schema.wire_codec ||
        envelope.routing_mode != schema.routing_mode ||
        envelope.payload_kind != ExpectedPayloadKind(schema.wire_codec)) {
      ROS_ERROR_STREAM_THROTTLE(
          1.0, "[TopicFactory] rejected envelope for " << topic_cfg_.name_
                                                        << ": "
                                                        << (metadata_error.empty()
                                                                ? "wire contract mismatch"
                                                                : metadata_error));
      recordDrop();
      return;
    }
  }

  if (ready && schema.ros_type == "sensor_msgs/Image") {
    recordCompleteImageEnvelope(envelope.sequence);
  }

  {
    std::lock_guard<std::mutex> lock(recv_mutex_);
    if (!recv_thread_flag_.load()) {
      return;
    }
    const auto result =
        swarm_ros_bridge::transport::PushToBoundedEnvelopeQueue(
            topic_cfg_.qos_class_, std::move(envelope), &recv_queue_);
    if (result ==
        swarm_ros_bridge::transport::BoundedQueuePushResult::kRejected) {
      recordTransportQueueDrop();
      return;
    }
    if (result == swarm_ros_bridge::transport::BoundedQueuePushResult::
                      kReplacedOldest) {
      recordTransportQueueDrop();
    }
  }
  recv_condition_.notify_one();
}

void TopicFactory::recvFunction() {
  while (true) {
    swarm_ros_bridge::transport::TransportEnvelope envelope;
    {
      std::unique_lock<std::mutex> lock(recv_mutex_);
      recv_condition_.wait(lock, [this]() {
        bool ready = false;
        {
          std::lock_guard<std::mutex> schema_lock(schema_mutex_);
          ready = schema_ready_;
        }
        return !recv_thread_flag_.load() ||
               (!recv_queue_.empty() && (ready || schemaConflict()));
      });
      if (!recv_thread_flag_.load() && recv_queue_.empty()) {
        return;
      }
      envelope = std::move(recv_queue_.front());
      recv_queue_.pop_front();
    }
    if (schemaConflict()) {
      recordDrop();
      continue;
    }
    try {
      deserializePublish(envelope);
    } catch (const std::exception& exception) {
      ROS_ERROR_STREAM("[TopicFactory] failed to decode " << topic_cfg_.name_ << ": "
                                                           << exception.what());
      recordDrop();
    }
  }
}

swarm_ros_bridge::diagnostics::TopicMetrics TopicFactory::GetMetricsSnapshot() {
  const auto schema_status = schema_registry_->GetStatus(topic_cfg_.rule_id_);
  if (schema_status.state ==
      swarm_ros_bridge::transport::TopicSchemaState::kConflict) {
    applySchemaQuarantine(schema_status.error);
  }
  std::lock_guard<std::mutex> lock(metrics_mutex_);
  const ros::Time now = ros::Time::now();
  pruneMetricsWindowLocked(now);
  auto state = metrics_state_;
  if (schema_status.state ==
      swarm_ros_bridge::transport::TopicSchemaState::kConflict) {
    state.schema_state = "conflict";
    state.schema_error = schema_status.error;
  }
  return swarm_ros_bridge::diagnostics::MakeTopicMetrics(state, now);
}

int TopicFactory::currentImageJpegQuality() const {
  std::lock_guard<std::mutex> lock(metrics_mutex_);
  return static_cast<int>(metrics_state_.current_jpeg_quality);
}

void TopicFactory::recordSend(std::size_t bytes) {
  std::lock_guard<std::mutex> lock(metrics_mutex_);
  const ros::Time now = ros::Time::now();
  pruneMetricsWindowLocked(now);
  metrics_state_.total_sent++;
  metrics_state_.total_send_bytes += bytes;
  metrics_state_.packet_size = static_cast<std::uint32_t>(bytes);
  metrics_state_.last_send_time = now;
  metrics_state_.recent_send_times.push_back(now);
  metrics_state_.recent_send_bytes.emplace_back(now, bytes);
  maybeAdaptImageQualityLocked(now);
}

void TopicFactory::recordDrop() {
  std::lock_guard<std::mutex> lock(metrics_mutex_);
  metrics_state_.dropped_messages++;
}

void TopicFactory::recordTransportQueueDrop() {
  std::lock_guard<std::mutex> lock(metrics_mutex_);
  metrics_state_.dropped_messages++;
  metrics_state_.transport_queue_drops++;
}

void TopicFactory::recordCompleteImageEnvelope(std::uint64_t sequence) {
  std::lock_guard<std::mutex> lock(metrics_mutex_);
  swarm_ros_bridge::diagnostics::ObserveCompleteFrameSequence(
      sequence, &metrics_state_);
}

void TopicFactory::recordReceive(std::size_t bytes, double latency_ms) {
  std::lock_guard<std::mutex> lock(metrics_mutex_);
  const ros::Time now = ros::Time::now();
  pruneMetricsWindowLocked(now);
  metrics_state_.total_received++;
  metrics_state_.total_recv_bytes += bytes;
  metrics_state_.packet_size = static_cast<std::uint32_t>(bytes);
  metrics_state_.last_recv_time = now;
  metrics_state_.recent_recv_times.push_back(now);
  metrics_state_.recent_recv_bytes.emplace_back(now, bytes);
  computeLatencyStats(latency_ms, &metrics_state_);
}

void TopicFactory::pruneMetricsWindowLocked(const ros::Time& now) {
  const ros::Duration max_age(metrics_state_.rate_window_sec);
  while (!metrics_state_.recent_send_times.empty() &&
         now - metrics_state_.recent_send_times.front() > max_age) {
    metrics_state_.recent_send_times.pop_front();
  }
  while (!metrics_state_.recent_recv_times.empty() &&
         now - metrics_state_.recent_recv_times.front() > max_age) {
    metrics_state_.recent_recv_times.pop_front();
  }
  while (!metrics_state_.recent_send_bytes.empty() &&
         now - metrics_state_.recent_send_bytes.front().first > max_age) {
    metrics_state_.recent_send_bytes.pop_front();
  }
  while (!metrics_state_.recent_recv_bytes.empty() &&
         now - metrics_state_.recent_recv_bytes.front().first > max_age) {
    metrics_state_.recent_recv_bytes.pop_front();
  }
}

void TopicFactory::maybeAdaptImageQualityLocked(const ros::Time& now) {
  if (metrics_state_.msg_type != "sensor_msgs/Image" ||
      !metrics_state_.adaptive_quality_enabled ||
      metrics_state_.target_bandwidth_kbps <= 0.0 ||
      metrics_state_.recent_send_bytes.size() < 2 ||
      metrics_state_.total_sent <
          metrics_state_.last_adapt_total_sent + metrics_state_.adapt_cooldown_frames) {
    return;
  }

  std::size_t recent_bytes = 0;
  for (const auto& item : metrics_state_.recent_send_bytes) {
    recent_bytes += item.second;
  }
  const double window = std::max(
      0.001, (now - metrics_state_.recent_send_bytes.front().first).toSec());
  const double bandwidth_kbps = recent_bytes * 8.0 / 1000.0 / window;
  const double upper = metrics_state_.target_bandwidth_kbps * 1.15;
  const double lower = metrics_state_.target_bandwidth_kbps * 0.85;
  int quality = static_cast<int>(metrics_state_.current_jpeg_quality);
  if (bandwidth_kbps > upper && quality > topic_cfg_.img_min_jpeg_quality_) {
    quality = std::max(topic_cfg_.img_min_jpeg_quality_,
                       quality - topic_cfg_.img_quality_step_);
  } else if (bandwidth_kbps < lower && quality < topic_cfg_.img_max_jpeg_quality_) {
    quality = std::min(topic_cfg_.img_max_jpeg_quality_,
                       quality + topic_cfg_.img_quality_step_);
  } else {
    return;
  }
  metrics_state_.current_jpeg_quality = static_cast<std::uint32_t>(quality);
  metrics_state_.last_adapt_total_sent = metrics_state_.total_sent;
}

double TopicFactory::inferLatencyMs(std::int64_t source_time_ns) {
  if (source_time_ns <= 0) {
    return -1.0;
  }
  const std::int64_t now_ns = static_cast<std::int64_t>(ros::Time::now().toNSec());
  if (now_ns < source_time_ns) {
    return -1.0;
  }
  return static_cast<double>(now_ns - source_time_ns) / 1e6;
}

void TopicFactory::computeLatencyStats(
    double latency_ms,
    swarm_ros_bridge::diagnostics::TopicRuntimeState* state) {
  if (state == nullptr || latency_ms < 0.0) {
    return;
  }
  if (state->total_received <= 1 || state->avg_latency_ms <= 0.0) {
    state->avg_latency_ms = latency_ms;
    state->jitter_ms = 0.0;
    state->last_latency_ms = latency_ms;
    return;
  }
  constexpr double kAlpha = 0.2;
  state->avg_latency_ms = state->avg_latency_ms * (1.0 - kAlpha) +
                          latency_ms * kAlpha;
  if (state->last_latency_ms >= 0.0) {
    const double delta = std::fabs(latency_ms - state->last_latency_ms);
    state->jitter_ms = state->jitter_ms * (1.0 - kAlpha) + delta * kAlpha;
  }
  state->last_latency_ms = latency_ms;
}

void TopicFactory::createThread() {
  if (send_or_recv_ != RECV || recv_thread_flag_.exchange(true)) {
    return;
  }
  recv_thread_ = std::thread(&TopicFactory::recvFunction, this);
  try {
    std::vector<std::shared_ptr<swarm_ros_bridge::transport::BridgeTransport>>
        candidate_transports;
    candidate_transports.push_back(transports_.control);
    if (transports_.image != nullptr &&
        transports_.image != transports_.control) {
      candidate_transports.push_back(transports_.image);
    }
    if (transports_.cloud != nullptr && transports_.cloud != transports_.control &&
        transports_.cloud != transports_.image) {
      candidate_transports.push_back(transports_.cloud);
    }
    const std::vector<std::string> keys{
        swarm_ros_bridge::transport::TopicFanoutKey(
            topic_cfg_.src_hostname_, topic_cfg_.wire_name_),
        swarm_ros_bridge::transport::TopicDirectedKey(
            topic_cfg_.src_hostname_, topic_cfg_.my_hostname_,
            topic_cfg_.wire_name_)};
    for (const auto& transport : candidate_transports) {
      for (const std::string& key : keys) {
        subscriptions_.push_back(transport->Subscribe(
            key,
            [this](swarm_ros_bridge::transport::TransportEnvelope&& envelope) {
              enqueueEnvelope(std::move(envelope));
            }));
      }
    }
    const std::string schema_key = swarm_ros_bridge::transport::TopicSchemaKey(
        topic_cfg_.src_hostname_, topic_cfg_.wire_name_);
    schema_subscription_ = transports_.control->Subscribe(
        schema_key,
        [this](swarm_ros_bridge::transport::TransportEnvelope&& envelope) {
          handleSchemaAnnouncement(std::move(envelope));
        });
    schema_thread_ = std::thread(&TopicFactory::schemaResolveFunction, this);
    INFO_MSG(" WAIT SCHEMA | " << schema_key << " -> "
                                << topic_cfg_.fanout_name_);
  } catch (...) {
    recv_thread_flag_.store(false);
    recv_condition_.notify_all();
    schema_condition_.notify_all();
    if (schema_thread_.joinable()) {
      schema_thread_.join();
    }
    if (recv_thread_.joinable()) {
      recv_thread_.join();
    }
    throw;
  }
}

void TopicFactory::stopThread() {
  if (send_or_recv_ == SEND) {
    sender_watch_timer_.stop();
    {
      std::lock_guard<std::mutex> lock(sender_subscription_mutex_);
      sub_.shutdown();
    }
    dynamic_senders_.clear();
    sender_.reset();
    schema_publisher_.reset();
    schema_queryable_.reset();
    return;
  }
  schema_subscription_.reset();
  subscriptions_.clear();
  if (recv_thread_flag_.exchange(false)) {
    recv_condition_.notify_all();
    schema_condition_.notify_all();
    if (schema_thread_.joinable()) {
      schema_thread_.join();
    }
    if (recv_thread_.joinable()) {
      recv_thread_.join();
    }
  }
  pub_.shutdown();
}
