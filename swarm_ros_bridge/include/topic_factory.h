#ifndef SRC_TOPIC_FACTORY_H
#define SRC_TOPIC_FACTORY_H

#include "diagnostics/topic_metrics.hpp"
#include "msgs_macro.hpp"
#include "transport/bridge_transport.hpp"
#include "transport/draco_pointcloud_codec.hpp"
#include "transport/topic_schema_registry.hpp"

#include <boost/tti/has_data.hpp>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#define SUB_QUEUE_SIZE 10
#define PUB_QUEUE_SIZE 20
BOOST_TTI_HAS_DATA(to_drone_ids);

struct TopicCfg {
  std::string name_;
  std::string fanout_name_;
  std::string directed_name_;
  std::string wire_name_;
  std::string rule_id_;
  std::string src_hostname_;
  std::string my_hostname_;
  XmlRpc::XmlRpcValue src_hostnames_xml;
  XmlRpc::XmlRpcValue dst_hostnames_xml;
  std::map<std::string, bool> dst_hostname_map_;
  std::map<std::string, bool> src_hostname_map_;
  double max_freq_{10.0};
  double img_resize_rate_{1.0};
  int img_jpeg_quality_{80};
  bool img_adaptive_quality_{false};
  int img_min_jpeg_quality_{45};
  int img_max_jpeg_quality_{90};
  double img_target_bandwidth_kbps_{1200.0};
  int img_quality_step_{5};
  int img_adapt_cooldown_frames_{8};
  bool cloud_compress_{false};
  double cloud_downsample_{-1};
  std::string cloud_codec_{"raw"};
  bool has_prefix_{true};
  bool same_prefix_{false};
  swarm_ros_bridge::transport::QosClass qos_class_{
      swarm_ros_bridge::transport::QosClass::kState};
};

struct TopicTransports {
  std::shared_ptr<swarm_ros_bridge::transport::BridgeTransport> control;
  std::shared_ptr<swarm_ros_bridge::transport::BridgeTransport> image;
  std::shared_ptr<swarm_ros_bridge::transport::BridgeTransport> cloud;
};

class TopicFactory {
 public:
  enum SEND_OR_RECV { SEND, RECV };
  using Ptr = std::shared_ptr<TopicFactory>;

  TopicFactory(const TopicCfg& topic_cfg,
               const TopicTransports& transports,
               std::shared_ptr<swarm_ros_bridge::transport::TopicSchemaRegistry>
                   schema_registry,
               SEND_OR_RECV send_or_recv,
               const std::shared_ptr<ros::NodeHandle>& nh_public);
  ~TopicFactory();

  void createThread();
  void stopThread();
  swarm_ros_bridge::diagnostics::TopicMetrics GetMetricsSnapshot();

 private:
  TopicCfg topic_cfg_;
  SEND_OR_RECV send_or_recv_;
  TopicTransports transports_;
  std::shared_ptr<swarm_ros_bridge::transport::TopicSchemaRegistry>
      schema_registry_;
  std::shared_ptr<ros::NodeHandle> nh_public_;
  std::shared_ptr<swarm_ros_bridge::transport::BridgeTransport> active_transport_;
  std::shared_ptr<swarm_ros_bridge::transport::TransportPublisher> sender_;
  std::map<std::string,
           std::shared_ptr<swarm_ros_bridge::transport::TransportPublisher>>
      dynamic_senders_;
  std::vector<std::shared_ptr<swarm_ros_bridge::transport::TransportSubscription>>
      subscriptions_;
  std::shared_ptr<swarm_ros_bridge::transport::TransportSubscription>
      schema_subscription_;
  std::shared_ptr<swarm_ros_bridge::transport::TransportPublisher>
      schema_publisher_;
  std::shared_ptr<swarm_ros_bridge::transport::TransportQueryable>
      schema_queryable_;

  ros::Time sub_last_time_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  std::atomic<bool> recv_thread_flag_{false};
  std::thread recv_thread_;
  std::thread schema_thread_;
  std::mutex recv_mutex_;
  std::condition_variable recv_condition_;
  std::mutex schema_wait_mutex_;
  std::condition_variable schema_condition_;
  std::deque<swarm_ros_bridge::transport::TransportEnvelope> recv_queue_;
  mutable std::mutex schema_mutex_;
  swarm_ros_bridge::transport::TopicSchema schema_;
  bool schema_ready_{false};
  std::atomic<std::uint64_t> sequence_{0};
  swarm_ros_bridge::transport::DracoPointCloudCodec draco_codec_;
  mutable std::mutex metrics_mutex_;
  swarm_ros_bridge::diagnostics::TopicRuntimeState metrics_state_;

  void enqueueEnvelope(swarm_ros_bridge::transport::TransportEnvelope&& envelope);
  void recvFunction();
  bool sendFreqControl();
  int currentImageJpegQuality() const;
  void recordSend(std::size_t bytes);
  void recordDrop();
  void recordTransportQueueDrop();
  void recordCompleteImageEnvelope(std::uint64_t sequence);
  void recordReceive(std::size_t bytes, double latency_ms);
  void pruneMetricsWindowLocked(const ros::Time& now);
  void maybeAdaptImageQualityLocked(const ros::Time& now);
  static double inferLatencyMs(std::int64_t source_time_ns);
  static void computeLatencyStats(
      double latency_ms,
      swarm_ros_bridge::diagnostics::TopicRuntimeState* state);

  void shapeShifterCallback(
      const ros::MessageEvent<const topic_tools::ShapeShifter>& event);
  bool configureSenderSchema(const topic_tools::ShapeShifter& message,
                             bool latching,
                             std::string* error);
  bool configureReceiverSchema(
      const swarm_ros_bridge::transport::TopicSchema& schema,
      std::string* error);
  bool handleSchemaQuery(
      const swarm_ros_bridge::transport::TransportEnvelope& request,
      swarm_ros_bridge::transport::TransportEnvelope* response,
      std::string* error);
  void handleSchemaAnnouncement(
      swarm_ros_bridge::transport::TransportEnvelope&& envelope);
  void schemaResolveFunction();
  bool schemaConflict(std::string* error = nullptr) const;
  void applySchemaQuarantine(const std::string& error);
  std::shared_ptr<swarm_ros_bridge::transport::BridgeTransport>
  transportForSchema(
      const swarm_ros_bridge::transport::TopicSchema& schema) const;
  ros::Publisher advertiseForSchema(
      const swarm_ros_bridge::transport::TopicSchema& schema,
      const std::string& topic_name) const;
  bool extractDynamicDestinations(const topic_tools::ShapeShifter& message,
                                  std::vector<std::string>* destinations) const;
  void updateSchemaMetrics(
      const swarm_ros_bridge::transport::TopicSchema& schema);
  void deserializePublish(
      const swarm_ros_bridge::transport::TransportEnvelope& envelope);

  template <typename T>
  void deserializePub(
      const swarm_ros_bridge::transport::TransportEnvelope& envelope);

  template <typename T>
  void ptCloudProcess(const T& msg, std::vector<std::uint8_t>* data);
};

#endif  // SRC_TOPIC_FACTORY_H
