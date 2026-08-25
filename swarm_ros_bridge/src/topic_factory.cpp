#include "topic_factory.h"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "pcl/compression/octree_pointcloud_compression.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/message_traits.h"
#include "ros/serialization.h"
#include "sensor_msgs/image_encodings.h"
#include "swarm_ros_bridge/PtCloudCompress.h"

#include <algorithm>
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
    const std::shared_ptr<ros::NodeHandle>& node) {
  ros::AdvertiseOptions options;
  options.init<T>(topic_name, PUB_QUEUE_SIZE);
  // roscpp normally overwrites Header.seq in the serialized output. The bridge
  // must publish the sequence received from the source host unchanged.
  options.has_header = false;
  return node->advertise(options);
}

std::int64_t RosTimeToNs(const ros::Time& stamp) {
  return stamp.isZero() ? 0 : static_cast<std::int64_t>(stamp.toNSec());
}

}  // namespace

TopicFactory::TopicFactory(
    const TopicCfg& topic_cfg,
    std::shared_ptr<swarm_ros_bridge::transport::BridgeTransport> transport,
    SEND_OR_RECV send_or_recv,
    const std::shared_ptr<ros::NodeHandle>& nh_public)
    : topic_cfg_(topic_cfg),
      send_or_recv_(send_or_recv),
      transport_(std::move(transport)) {
  if (transport_ == nullptr) {
    throw std::invalid_argument("TopicFactory requires a shared bridge transport");
  }

  metrics_state_.topic_name = topic_cfg_.name_;
  metrics_state_.msg_type = topic_cfg_.type_;
  metrics_state_.direction = send_or_recv_ == SEND ? "send" : "recv";
  metrics_state_.codec = topic_cfg_.type_ == "sensor_msgs/PointCloud2"
                             ? (topic_cfg_.cloud_compress_ ? topic_cfg_.cloud_codec_ : "raw")
                             : (topic_cfg_.type_ == "sensor_msgs/Image" ? "jpeg" : "raw");
  metrics_state_.transport = topic_cfg_.transport_name_;
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
    if (!topic_cfg_.dynamic_dst_) {
      const std::string key = swarm_ros_bridge::transport::TopicFanoutKey(
          topic_cfg_.src_hostname_, topic_cfg_.wire_name_);
      sender_ = transport_->DeclarePublisher(key, topic_cfg_.qos_class_);
      INFO_MSG(" SEND " << topic_cfg_.transport_name_ << " | " << key
                         << " | " << topic_cfg_.max_freq_ << "Hz");
    } else {
      for (const auto& destination : topic_cfg_.dst_hostname_map_) {
        if (!destination.second || destination.first == topic_cfg_.my_hostname_) {
          continue;
        }
        const std::string key = swarm_ros_bridge::transport::TopicDirectedKey(
            topic_cfg_.src_hostname_, destination.first, topic_cfg_.wire_name_);
        dynamic_senders_[destination.first] =
            transport_->DeclarePublisher(key, topic_cfg_.qos_class_);
      }
      INFO_MSG(" SEND " << topic_cfg_.transport_name_ << " DYNAMIC | "
                         << topic_cfg_.wire_name_ << " | "
                         << topic_cfg_.max_freq_ << "Hz");
    }
    sub_last_time_ = ros::Time(0);
    sub_ = topicSubscriber(topic_cfg_.name_, topic_cfg_.type_, nh_public);
  } else {
    pub_ = topicPublisher(topic_cfg_.name_, topic_cfg_.type_, nh_public);
  }
}

TopicFactory::~TopicFactory() { stopThread(); }

ros::Subscriber TopicFactory::topicSubscriber(
    const std::string& topic_name,
    const std::string& msg_type,
    const std::shared_ptr<ros::NodeHandle>& nh) {
#define X(type, classname)                 \
  if (msg_type == type) {                 \
    return nh_sub<classname>(topic_name, nh); \
  }
  MSGS_MACRO
#undef X
  throw std::invalid_argument("Invalid ROS message type: " + msg_type);
}

template <typename T>
void TopicFactory::subCallback(const ros::MessageEvent<const T>& event) {
  if (sendFreqControl()) {
    return;
  }

  const T& msg = *event.getConstMessage();
  swarm_ros_bridge::transport::TransportEnvelope envelope;
  envelope.payload_kind = swarm_ros_bridge::transport::PayloadKind::kRosSerialized;
  envelope.sequence = sequence_.fetch_add(1U) + 1U;
  envelope.source_host = topic_cfg_.my_hostname_;
  envelope.ros_type = topic_cfg_.type_;
  envelope.ros_md5 = RosTypeMd5(topic_cfg_.type_);

  if constexpr (has_data_header<T, std_msgs::Header>::value) {
    envelope.source_time_ns = RosTimeToNs(msg.header.stamp);
    envelope.source_header_sequence = msg.header.seq;
    envelope.frame_id = msg.header.frame_id;
  } else {
    envelope.source_time_ns = RosTimeToNs(ros::Time::now());
  }

  try {
    if constexpr (std::is_same<T, nav_msgs::Odometry>::value) {
      geometry_msgs::PoseStamped pose;
      pose.header = msg.header;
      pose.header.frame_id = msg.child_frame_id;
      pose.pose = msg.pose.pose;
      envelope.payload = SerializeRosMessage(pose);
    } else if constexpr (std::is_same<T, sensor_msgs::Image>::value) {
      const sensor_msgs::ImageConstPtr image_msg = event.getConstMessage();
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
    } else if constexpr (std::is_same<T, sensor_msgs::PointCloud2>::value) {
      ptCloudProcess(msg, &envelope.payload);
      if (topic_cfg_.cloud_compress_) {
        envelope.payload_kind =
            swarm_ros_bridge::transport::PayloadKind::kPointCloudCompressed;
      }
    } else {
      envelope.payload = SerializeRosMessage(msg);
    }
  } catch (const std::exception& exception) {
    ROS_ERROR_STREAM("[TopicFactory] failed to encode " << topic_cfg_.name_ << ": "
                                                         << exception.what());
    recordDrop();
    return;
  }

  if constexpr (has_data_to_drone_ids<T, std::vector<uint8_t>>::value) {
    for (const std::uint8_t id : msg.to_drone_ids) {
      const std::string target = "drone" + std::to_string(id);
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

template <typename T>
ros::Subscriber TopicFactory::nh_sub(
    const std::string& topic_name,
    const std::shared_ptr<ros::NodeHandle>& nh) {
  return nh->subscribe<const ros::MessageEvent<T const>&>(
      topic_name, SUB_QUEUE_SIZE, &TopicFactory::subCallback<T>, this,
      ros::TransportHints().tcpNoDelay());
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

ros::Publisher TopicFactory::topicPublisher(
    const std::string& topic_name,
    const std::string& msg_type,
    const std::shared_ptr<ros::NodeHandle>& nh) {
#define X(type, classname)                     \
  if (msg_type == type) {                     \
    return AdvertisePreservingHeader<classname>(topic_name, nh); \
  }
  MSGS_MACRO
#undef X
  throw std::invalid_argument("Invalid ROS message type: " + msg_type);
}

void TopicFactory::deserializePublish(
    const swarm_ros_bridge::transport::TransportEnvelope& envelope) {
#define X(type, classname)                 \
  if (topic_cfg_.type_ == type) {          \
    return deserializePub<classname>(envelope); \
  }
  MSGS_MACRO
#undef X
  throw std::invalid_argument("Invalid ROS message type: " + topic_cfg_.type_);
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
      if (topic_cfg_.cloud_compress_) {
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
  const auto expected_payload_kind =
      topic_cfg_.type_ == "sensor_msgs/Image"
          ? swarm_ros_bridge::transport::PayloadKind::kJpeg
          : (topic_cfg_.type_ == "sensor_msgs/PointCloud2" &&
                     topic_cfg_.cloud_compress_
                 ? swarm_ros_bridge::transport::PayloadKind::kPointCloudCompressed
                 : swarm_ros_bridge::transport::PayloadKind::kRosSerialized);
  if (envelope.payload_kind != expected_payload_kind) {
    ROS_ERROR_STREAM_THROTTLE(
        1.0, "[TopicFactory] rejected payload kind for " << topic_cfg_.name_);
    recordDrop();
    return;
  }
  std::string metadata_error;
  if (!swarm_ros_bridge::transport::ValidateEnvelopeMetadata(
          envelope, topic_cfg_.type_, RosTypeMd5(topic_cfg_.type_),
          &metadata_error)) {
    ROS_ERROR_STREAM_THROTTLE(
        1.0, "[TopicFactory] rejected envelope for " << topic_cfg_.name_ << ": "
                                                       << metadata_error);
    recordDrop();
    return;
  }
  if (!topic_cfg_.dynamic_dst_ && envelope.source_host != topic_cfg_.src_hostname_) {
    recordDrop();
    return;
  }
  const auto source = topic_cfg_.src_hostname_map_.find(envelope.source_host);
  if (source == topic_cfg_.src_hostname_map_.end() || !source->second) {
    recordDrop();
    return;
  }

  if (topic_cfg_.type_ == "sensor_msgs/Image") {
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
        return !recv_thread_flag_.load() || !recv_queue_.empty();
      });
      if (!recv_thread_flag_.load() && recv_queue_.empty()) {
        return;
      }
      envelope = std::move(recv_queue_.front());
      recv_queue_.pop_front();
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

swarm_ros_bridge::diagnostics::TopicMetrics TopicFactory::GetMetricsSnapshot() const {
  std::lock_guard<std::mutex> lock(metrics_mutex_);
  const ros::Time now = ros::Time::now();
  const_cast<TopicFactory*>(this)->pruneMetricsWindowLocked(now);
  return swarm_ros_bridge::diagnostics::MakeTopicMetrics(metrics_state_, now);
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
  if (topic_cfg_.type_ != "sensor_msgs/Image" ||
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
  const std::string key = topic_cfg_.dynamic_dst_
                              ? swarm_ros_bridge::transport::TopicDirectedKey(
                                    "*", topic_cfg_.my_hostname_, topic_cfg_.wire_name_)
                              : swarm_ros_bridge::transport::TopicFanoutKey(
                                    topic_cfg_.src_hostname_, topic_cfg_.wire_name_);
  try {
    subscription_ = transport_->Subscribe(
        key, [this](swarm_ros_bridge::transport::TransportEnvelope&& envelope) {
          enqueueEnvelope(std::move(envelope));
        });
    INFO_MSG(" RECV " << topic_cfg_.transport_name_ << " | " << key
                       << " -> " << topic_cfg_.name_);
  } catch (...) {
    recv_thread_flag_.store(false);
    recv_condition_.notify_all();
    if (recv_thread_.joinable()) {
      recv_thread_.join();
    }
    throw;
  }
}

void TopicFactory::stopThread() {
  if (send_or_recv_ == SEND) {
    sub_.shutdown();
    dynamic_senders_.clear();
    sender_.reset();
    return;
  }
  subscription_.reset();
  if (recv_thread_flag_.exchange(false)) {
    recv_condition_.notify_all();
    if (recv_thread_.joinable()) {
      recv_thread_.join();
    }
  }
  pub_.shutdown();
}
