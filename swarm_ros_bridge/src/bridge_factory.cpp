/**
 * @file bridge_factory.cpp
 * @author      Weiqi Gai   (github: Gaiwqqq) 2024.12
 * @contributor KengHou Hoi  (github: James Hoi) 2024.8
 * @date 2024.12.13
 * @brief This file contains the implementation of the BridgeFactory class.
 * @version 0.1 beta
 * @license BSD 3-Clause License
 * @copyright (c) 2024, Weiqi Gai
 * All rights reserved.
 */

#include "bridge_factory.hpp"
#include <memory>

BridgeFactory::BridgeFactory(ros::NodeHandle& node, ros::NodeHandle& node_public) {
  nh_        = std::make_shared<ros::NodeHandle>(node);
  nh_public_ = std::make_shared<ros::NodeHandle>(node_public);
  XmlRpc::XmlRpcValue config;
  if (!nh_->getParam("config", config)) {
    ROS_FATAL("[Bridge]: cannot find bridge_config parameter, quit program.");
    return;
  }
  do_odom_convert_ = config["odom_convert"];
  is_debug_        = config["debug"];
  my_drone_id_     = DRONE_ID_NULL;
  INFO_MSG_GREEN(">>>>>>>>>>>>>>>>>>>>> Bridge Node >>>>>>>>>>>>>>>>>>>>>");
  getMyHostName();
  getIpAndTopicConfig();
  INFO_MSG_GREEN(">>>>>>>>>>>>>>>>>>>>> Topic List >>>>>>>>>>>>>>>>>>>>>>");
  topicOperatorInit();
  INFO_MSG_GREEN(">>>>>>>>>>>>>>>>>>>>> Service List >>>>>>>>>>>>>>>>>>>>>");
  getServiceConfigAndInit();
}

BridgeFactory::~BridgeFactory() = default;

void BridgeFactory::getMyHostName() {
  // get my hostname
  if (nh_->getParam("hostname", my_hostname_)){
    if (my_hostname_.rfind("drone") == 0)
      my_drone_id_ = std::stoi(my_hostname_.substr(5));
  }else if (const char* env_ptr = std::getenv("DRONE_ID")){
    my_drone_id_ = std::atoi(env_ptr);
    my_hostname_ = "drone" + std::to_string(my_drone_id_);
  }else{
    ROS_FATAL("[Bridge]: cannot find hostname parameter, quit program.");
    return ;
  }
  INFO_MSG_BLUE("[Bridge]: my_hostname_ -> " << my_hostname_);
  if (my_drone_id_ >= 0){
    INFO_MSG_BLUE("[Bridge]: my_drone_id_ -> " << my_drone_id_);
  }
  else if (my_drone_id_ != DRONE_ID_NULL)
    ROS_FATAL("[Bridge]: invalid my_drone_id_ value!");
}



void BridgeFactory::getIpAndTopicConfig() {
  // explain bridge topic configuration
  if (!nh_->getParam("IP", ip_xml_)){
    ROS_ERROR("[bridge node] No IP found in the configuration!");
    return ;
  }
  if (nh_->getParam("topics", topics_xml_))
    ROS_ASSERT(topics_xml_.getType() == XmlRpc::XmlRpcValue::TypeArray);
  else
    ROS_WARN("[bridge node] No topics found in the configuration!");

  for (auto & iter : ip_xml_){
    std::string host_name = iter.first;
    std::string ip_addr   = iter.second;
    if (ip_map_.find(host_name) != ip_map_.end())
      INFO_MSG_YELLOW("[Bridge] IPs with the same name in configuration -> " <<  host_name.c_str());
    if (host_name.find("all") != std::string::npos || host_name.find("all_drone") != std::string::npos) continue;
    ip_map_[host_name]           = ip_addr;
    dst_hostname_map_[host_name] = false;
    src_hostname_map_[host_name] = false;
  }
  my_ip_ = ip_map_[my_hostname_];

  for (const auto& iter : ip_map_)
    if (iter.first != my_hostname_)
      INFO_MSG("[Bridge]: host name -> " << iter.first << " | ip -> " << iter.second);
    else
      INFO_MSG("[Bridge]: host name -> (ME)" << iter.first << " | ip -> (ME)" << iter.second);

  std::map<int, bool> port_used_map_;
  // get all topics from yaml
  for (int i = 0; i < topics_xml_.size(); ++i){
    ROS_ASSERT(topics_xml_[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    XmlRpc::XmlRpcValue topic_xml     = topics_xml_[i];
    std::string topic_name            = topic_xml["topic_name"];
    std::string topic_type            = topic_xml["msg_type"];
    XmlRpc::XmlRpcValue src_hostnames = topic_xml["srcIP"];
    XmlRpc::XmlRpcValue dst_hostnames = topic_xml["dstIP"];
    XmlRpc::XmlRpcValue frequency     = topic_xml["max_freq"];
    int src_port                      = topic_xml["srcPort"];
    bool has_prefix                   = topic_xml["prefix"];
    bool same_prefix                  = topic_xml["same_prefix"];
    bool cloud_compress                  = false;
    double cloud_downsample              = -1;
    double img_resize_rate               = 1.0;
    if (topic_type == "sensor_msgs/Image") {
      if (topic_xml.hasMember("imgResizeRate")) {
        XmlRpc::XmlRpcValue img_resize_rate_xml = topic_xml["imgResizeRate"];
        img_resize_rate = (double)(img_resize_rate_xml);
        INFO_MSG_GREEN("   ** this img will be resized [imgResizeRate -> " << img_resize_rate << "]");
      }else
        INFO_MSG_YELLOW("[Bridge]: topic does not have imgResizeRate, use default value 1.0");
    }
    if (topic_type == "sensor_msgs/PointCloud2") {
      if (topic_xml.hasMember("cloudCompress")) {
        cloud_compress = topic_xml["cloudCompress"];
      }else
        INFO_MSG_YELLOW("[Bridge]: topic does not have cloudCompress, use default value false");

      if (topic_xml.hasMember("cloudDownsample")) {
        XmlRpc::XmlRpcValue cloud_downsample_xml = topic_xml["cloudDownsample"];
        cloud_downsample = static_cast<double>(cloud_downsample_xml);
        if (cloud_downsample < 1e-4 || cloud_downsample > 1e4) {
          INFO_MSG_RED("[Bridge]: cloudDownsample value is out of range [0.0001, 10000], reset to 0.1");
          cloud_downsample = -1;
        }else
          INFO_MSG_GREEN("   ** this cloud will be downsampled [cloudDownsample -> " << cloud_downsample << "]");
      }else
        INFO_MSG_YELLOW("[Bridge]: topic does not have cloudDownsample, use default value -1");
    }

    ROS_ASSERT(src_hostnames.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(dst_hostnames.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT_MSG(!topic_name.empty(), "[Bridge] yaml topic_name cannot be none");
    ROS_ASSERT_MSG(!topic_type.empty(), "[Bridge] yaml msg_type cannot be none");

    topic_cfgs_.emplace_back();
    auto & topic              = topic_cfgs_.back();
    topic.my_ip_              = my_ip_;
    topic.my_hostname_        = my_hostname_;
    topic.name_               = topic_name;
    if (topic.name_.at(0) != '/'){
      ROS_FATAL("[Bridge]: topic name must start with '/', please check the configuration!");
      return;
    }
    topic.type_               = topic_type;
    topic.max_freq_           = (XmlRpc::XmlRpcValue::TypeInt == frequency.getType() ? (int)frequency : (double)frequency);
    topic.img_resize_rate_    = img_resize_rate;
    topic.cloud_compress_     = cloud_compress;
    topic.cloud_downsample_   = cloud_downsample;
    if (cloud_compress)
      INFO_MSG_GREEN("   ** this cloud will be compressed");
    topic.port_               = src_port;
    if (port_used_map_.find(topic.port_) == port_used_map_.end())
      port_used_map_[topic.port_] = true;
    else{
      ROS_FATAL("[Bridge]: port %d is used by other topic, please check the configuration!", topic.port_);
      return;
    }
    topic.has_prefix_       = has_prefix;
    topic.same_prefix_      = same_prefix;
    topic.dst_hostnames_xml = dst_hostnames;
    topic.src_hostnames_xml = src_hostnames;
    topic.src_hostname_map_ = src_hostname_map_;
    topic.dst_hostname_map_ = dst_hostname_map_;
    topic.only1_dst_hostname_ = checkIfPointToPointPipeline(topic);
    if (topic.type_ == "sensor_msgs/Image" && topic.only1_dst_hostname_ == NOT_ONLY_ONE_DST)
      ROS_FATAL("[Bridge]: Image topic must be point-to-point pipeline, please check the configuration!");

    for (int j = 0; j < src_hostnames.size(); ++j){
      if (src_hostnames[j] == "all"){
        for (auto & item : topic.src_hostname_map_) {
          item.second = true;
          topic.src_ip_map_[ip_map_[item.first]] = true;
        }
        break;
      }
      else if (src_hostnames[j] == "all_drone"){
        for (auto & item : topic.src_hostname_map_)
          if (item.first.find("drone") != std::string::npos){
            item.second = true;
            topic.src_ip_map_[ip_map_[item.first]] = true;
          }
      }else{
        topic.src_hostname_map_[src_hostnames[j]]    = true;
        topic.src_ip_map_[ip_map_[src_hostnames[j]]] = true;
      }
    }
    for (int j = 0; j < dst_hostnames.size(); ++j){
      if (dst_hostnames[j] == "all"){
        for (auto & item : topic.dst_hostname_map_) {
          item.second = true;
          topic.dst_ip_map_[ip_map_[item.first]] = true;
        }
        break;
      }
      else if (dst_hostnames[j] == "all_drone"){
        for (auto & item : topic.dst_hostname_map_)
          if (item.first.find("drone") != std::string::npos){
            item.second = true;
            topic.dst_ip_map_[ip_map_[item.first]] = true;
          }
      }else{
        topic.dst_hostname_map_[dst_hostnames[j]]    = true;
        topic.dst_ip_map_[ip_map_[dst_hostnames[j]]] = true;
      }
    }

    INFO_MSG_BLUE("[Bridge]: ** topic analyse : " << topic.name_);
    // check if msg has "to_drone_ids"
    bool has_ids_member = false;
#define X(type, classname) \
    if constexpr (has_data_to_drone_ids<classname, std::vector<uint8_t>>::value) \
      has_ids_member |= (topic.type_ == type);
    MSGS_MACRO
#undef X
    topic.dynamic_dst_ = has_ids_member;
  }
}

void BridgeFactory::getServiceConfigAndInit() {
  if (nh_->getParam("services", services_xml_))
    ROS_ASSERT(services_xml_.getType() == XmlRpc::XmlRpcValue::TypeArray);
  else
    ROS_WARN("[bridge node] No service found in the configuration!");

  // get all service config from yaml
  for (int i = 0 ; i < services_xml_.size(); ++i){

    ROS_ASSERT(services_xml_[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    bool i_am_server = false;
    bool i_am_client = false;
    XmlRpc::XmlRpcValue service_xml      = services_xml_[i];
    std::string service_name             = service_xml["srv_name"];
    std::string service_type             = service_xml["srv_type"];
    if (service_xml["serverIp"].getType() != XmlRpc::XmlRpcValue::TypeString){
      ROS_FATAL("[Bridge]: server only support one ip, please check the configuration!");
      return;
    }
    std::string server_hostname          = service_xml["serverIp"];
    XmlRpc::XmlRpcValue client_hostnames = service_xml["clientIp"];
    int         service_port             = service_xml["srcPort"];
    bool        if_prefix                = service_xml["prefix"];

    if (server_hostname == my_hostname_) i_am_server = true;
    for (int j = 0; j < client_hostnames.size(); ++j){
      std::string hostname_temp = client_hostnames[j];
      if (hostname_temp == my_hostname_){
        i_am_client = true;
        break;
      }
    }
    if (service_name[0] != '/'){
      ROS_ERROR_STREAM("[Bridge]: Service name must start with '/', please check the configuration!");
      continue;
    }
    if (service_servers_.find(service_name) != service_servers_.end() ||
        service_clients_.find(service_name) != service_clients_.end()){
      ROS_ERROR_STREAM("[Bridge]: Service name already exists, please check the configuration!");
      continue;
    }
    if (i_am_server && i_am_client){
      ROS_ERROR_STREAM("[Bridge]: service config error! You can't be client and server at the same time!");
      continue;
    }

    ServiceConfig service_cfg;
    service_cfg.service_name  = service_name;
    service_cfg.service_type  = service_type;
    service_cfg.port          = std::to_string(service_port);
    service_cfg.server_ip     = ip_map_[server_hostname];
    service_cfg.if_prefix     = if_prefix;
    service_cfg.my_hostname   = my_hostname_;
    if (service_cfg.if_prefix){
      service_cfg.service_prefix_name = "/" + server_hostname;
      service_cfg.service_prefix_name.append(service_name);
    }
    if (i_am_server){
      service_servers_[service_cfg.service_name] =
              std::make_shared<ServiceFactory>(ServiceFactory::SERVER, nh_public_, service_cfg);
    }else if(i_am_client){
      service_clients_[service_cfg.service_name] =
              std::make_shared<ServiceFactory>(ServiceFactory::CLIENT, nh_public_, service_cfg);
    }
    INFO_MSG_BLUE("[Bridge]: ** service config analyse : " << service_name);
  }
}

std::string BridgeFactory::checkIfPointToPointPipeline(TopicCfg& topic_cfg) {
  if (topic_cfg.src_hostnames_xml.size() == 1 || topic_cfg.dst_hostnames_xml.size() == 1){
    if (topic_cfg.src_hostnames_xml[0] == "all" || topic_cfg.src_hostnames_xml[0] == "all_drone") return NOT_ONLY_ONE_DST;
    if (topic_cfg.dst_hostnames_xml[0] == "all" || topic_cfg.dst_hostnames_xml[0] == "all_drone") return NOT_ONLY_ONE_DST;
    std::string res = topic_cfg.dst_hostnames_xml[0];
    return res;
  } else
    return NOT_ONLY_ONE_DST;
}

void BridgeFactory::topicOperatorInit() {
  // process send topics
  for (const auto& item : topic_cfgs_){
    TopicCfg t = item;
    if (t.src_hostname_map_[my_hostname_]){
      const std::string drone_prefix = "/drone_{id}";
      if (t.name_.rfind(drone_prefix) == 0 && my_drone_id_ != DRONE_ID_NULL){
        std::string topic_prefix = "/drone_" + std::to_string(my_drone_id_);
        t.name_.replace(0, drone_prefix.size(), topic_prefix);
      }

      t.raw_name_ = nh_->resolveName(t.name_);
      t.src_ip_       = my_ip_;
      t.src_hostname_ = my_hostname_;

      if (send_topics_.find(t.name_) == send_topics_.end())
        send_topics_[t.name_] = std::make_shared<TopicFactory>(t, ip_map_, TopicFactory::SEND_OR_RECV::SEND, nh_public_);
    }
  }
  INFO_MSG_GREEN("[Bridge]: send_topics_ SIZE -> " << send_topics_.size());

  // process recv topics
  for (const auto& item : topic_cfgs_){
    TopicCfg topic = item;
    for (auto &ip : ip_map_){
      if (!topic.dst_hostname_map_[topic.my_hostname_]) break;  // dst is not me
      if (ip.first == my_hostname_ && !is_debug_) continue;     // topic I send
      if (!topic.src_hostname_map_[ip.first]) continue;         // source name error
      TopicCfg t = topic;
      std::string src_hostname_tmp = ip.first;

      const std::string drone_prefix = "/drone_{id}";
      if (t.name_.rfind(drone_prefix) == 0){
        ROS_ASSERT(src_hostname_tmp.rfind("drone") == 0);
        int drone_id = t.dynamic_dst_ ? my_drone_id_ : std::stoi(src_hostname_tmp.substr(5, 5)) ;
        std::string  topic_prefix = "/drone_" + std::to_string(drone_id);
        t.name_.replace(0, drone_prefix.size(), topic_prefix);
      }
      t.raw_name_ = nh_->resolveName(t.name_);
      if (t.has_prefix_ && !t.same_prefix_)
        t.name_ = "/" + src_hostname_tmp + t.name_;
      else if (t.same_prefix_)
        t.name_ = "/bridge" + t.name_;

      t.src_hostname_ = src_hostname_tmp;
      t.src_ip_       = ip.second;
      if (recv_topics_.find(t.name_) == recv_topics_.end())
        recv_topics_[t.name_] = std::make_shared<TopicFactory>(t, ip_map_, TopicFactory::SEND_OR_RECV::RECV, nh_public_);
      else{
        ROS_FATAL("[Bridge]: topic name %s already exists, please check the configuration!",  t.name_.c_str());
        return;
      }
      // if dynamic dst, only bind once, one topicinfo is enough
      if (t.dynamic_dst_) break;
    }
  }
  INFO_MSG_GREEN("[Bridge]: recv_topics_ SIZE -> " << recv_topics_.size());
}

bool BridgeFactory::xmlContain(XmlRpc::XmlRpcValue array, std::string value)
{
  for (int32_t i = 0; i < array.size(); ++i)
  {
    if (array[i] == value)
      return true;
  }
  return false;
}

void BridgeFactory::startBridge() {
  for (auto& recv_factory : recv_topics_){
    recv_factory.second->createThread();
  }
  INFO_MSG_GREEN("===========================================");
  INFO_MSG_GREEN("[Bridge]: all recv_topics_ threads started!");
  INFO_MSG_GREEN("===========================================");
}

void BridgeFactory::stopBridge() {
  for (auto& recv_factory : recv_topics_){
    recv_factory.second->stopThread();
  }
  for (auto& send_factory : send_topics_){
    send_factory.second->stopThread();
  }
  for (auto& service_server : service_servers_){
    service_server.second->stopServerThread();
  }
  INFO_MSG_RED("===========================================");
  INFO_MSG_RED("[Bridge]: all recv_topics_ threads Stopped!");
  INFO_MSG_RED("===========================================");
}