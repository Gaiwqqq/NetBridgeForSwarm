#ifndef SRC_BRIDGE_FACTORY_H
#define SRC_BRIDGE_FACTORY_H

#include "service_factory.h"
#include "swarm_ros_bridge/NetworkArray.h"
#include "topic_factory.h"
#include "transport/bridge_transport.hpp"

#include <ros/ros.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

class BridgeFactory {
 public:
  static constexpr int DRONE_ID_NULL = -9999;

  explicit BridgeFactory(ros::NodeHandle& node, ros::NodeHandle& node_public);
  ~BridgeFactory();
  void startBridge();
  void stopBridge();

 private:
  std::shared_ptr<ros::NodeHandle> nh_;
  std::shared_ptr<ros::NodeHandle> nh_public_;
  std::string my_hostname_;
  bool do_odom_convert_{true};
  bool is_debug_{false};
  int my_drone_id_{DRONE_ID_NULL};
  XmlRpc::XmlRpcValue topics_xml_;
  XmlRpc::XmlRpcValue services_xml_;
  std::vector<TopicCfg> topic_cfgs_;
  std::map<std::string, TopicFactory::Ptr> send_topics_;
  std::map<std::string, TopicFactory::Ptr> recv_topics_;
  std::map<std::string, ServiceFactory::Ptr> service_servers_;
  std::map<std::string, ServiceFactory::Ptr> service_clients_;
  std::map<std::string, bool> dst_hostname_map_;
  std::map<std::string, bool> src_hostname_map_;
  std::map<std::string, std::string> host_map_;
  swarm_ros_bridge::transport::ZenohTransportConfig zenoh_config_;
  std::shared_ptr<swarm_ros_bridge::transport::BridgeTransport> transport_;
  ros::Publisher diagnostics_pub_;
  ros::Timer diagnostics_timer_;
  bool stopped_{false};

  void getMyHostName();
  void getHostTopicAndTransportConfig();
  void getServiceConfigAndInit();
  void topicOperatorInit();
  void publishDiagnostics(const ros::TimerEvent& event);
  void expandHostSelection(const XmlRpc::XmlRpcValue& selection,
                           std::map<std::string, bool>* output) const;
};

#endif  // SRC_BRIDGE_FACTORY_H
