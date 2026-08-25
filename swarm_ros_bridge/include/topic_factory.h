#ifndef SRC_TOPIC_FACTORY_H
#define SRC_TOPIC_FACTORY_H

#include "diagnostics/topic_metrics.hpp"
#include "msgs_macro.hpp"
#include "transport/bridge_transport.hpp"
#include "transport/draco_pointcloud_codec.hpp"

#include <boost/tti/has_data.hpp>
#include <ros/ros.h>

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
BOOST_TTI_HAS_DATA(header);

struct TopicCfg {
  std::string name_;
  std::string raw_name_;
  std::string wire_name_;
  std::string type_;
  std::string src_hostname_;
  std::string my_hostname_;
  std::string transport_name_{"zenoh"};
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
  bool dynamic_dst_{false};
  swarm_ros_bridge::transport::QosClass qos_class_{
      swarm_ros_bridge::transport::QosClass::kState};
};

class TopicFactory {
 public:
  enum SEND_OR_RECV { SEND, RECV };
  using Ptr = std::shared_ptr<TopicFactory>;

  TopicFactory(const TopicCfg& topic_cfg,
               std::shared_ptr<swarm_ros_bridge::transport::BridgeTransport> transport,
               SEND_OR_RECV send_or_recv,
               const std::shared_ptr<ros::NodeHandle>& nh_public);
  ~TopicFactory();

  void createThread();
  void stopThread();
  swarm_ros_bridge::diagnostics::TopicMetrics GetMetricsSnapshot() const;

 private:
  TopicCfg topic_cfg_;
  SEND_OR_RECV send_or_recv_;
  std::shared_ptr<swarm_ros_bridge::transport::BridgeTransport> transport_;
  std::shared_ptr<swarm_ros_bridge::transport::TransportPublisher> sender_;
  std::map<std::string,
           std::shared_ptr<swarm_ros_bridge::transport::TransportPublisher>>
      dynamic_senders_;
  std::shared_ptr<swarm_ros_bridge::transport::TransportSubscription> subscription_;

  ros::Time sub_last_time_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  std::atomic<bool> recv_thread_flag_{false};
  std::thread recv_thread_;
  std::mutex recv_mutex_;
  std::condition_variable recv_condition_;
  std::deque<swarm_ros_bridge::transport::TransportEnvelope> recv_queue_;
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

  template <typename T>
  void subCallback(const ros::MessageEvent<const T>& event);

  template <typename T>
  ros::Subscriber nh_sub(const std::string& topic_name,
                         const std::shared_ptr<ros::NodeHandle>& nh);

  ros::Subscriber topicSubscriber(const std::string& topic_name,
                                  const std::string& msg_type,
                                  const std::shared_ptr<ros::NodeHandle>& nh);
  static ros::Publisher topicPublisher(const std::string& topic_name,
                                       const std::string& msg_type,
                                       const std::shared_ptr<ros::NodeHandle>& nh);
  void deserializePublish(
      const swarm_ros_bridge::transport::TransportEnvelope& envelope);

  template <typename T>
  void deserializePub(
      const swarm_ros_bridge::transport::TransportEnvelope& envelope);

  template <typename T>
  void ptCloudProcess(const T& msg, std::vector<std::uint8_t>* data);
};

#endif  // SRC_TOPIC_FACTORY_H
