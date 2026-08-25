#include "bridge_factory.hpp"

#include "msgs_macro.hpp"
#include "transport/zenoh_transport.hpp"

#include <algorithm>
#include <cstdlib>
#include <stdexcept>
#include <utility>

namespace {

double XmlRpcToDouble(const XmlRpc::XmlRpcValue& value, double fallback) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(value);
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<double>(value);
  }
  return fallback;
}

int XmlRpcToInt(const XmlRpc::XmlRpcValue& value, int fallback) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(value);
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<int>(static_cast<double>(value));
  }
  return fallback;
}

bool XmlRpcToBool(const XmlRpc::XmlRpcValue& value, bool fallback) {
  return value.getType() == XmlRpc::XmlRpcValue::TypeBoolean
             ? static_cast<bool>(value)
             : fallback;
}

std::string XmlRpcToString(const XmlRpc::XmlRpcValue& value,
                           const std::string& fallback) {
  return value.getType() == XmlRpc::XmlRpcValue::TypeString
             ? static_cast<std::string>(value)
             : fallback;
}

std::vector<std::string> XmlRpcToStringVector(
    const XmlRpc::XmlRpcValue& value) {
  std::vector<std::string> output;
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return output;
  }
  output.reserve(value.size());
  for (int i = 0; i < value.size(); ++i) {
    if (value[i].getType() == XmlRpc::XmlRpcValue::TypeString) {
      output.push_back(static_cast<std::string>(value[i]));
    }
  }
  return output;
}

bool UsesEndpointProtocol(const std::string& endpoint,
                          const std::string& protocol) {
  return endpoint.rfind(protocol + "/", 0) == 0;
}

std::string TopicTransportName(bool image_session_enabled,
                               bool cloud_session_enabled,
                               const std::string& ros_type) {
  if (image_session_enabled && ros_type == "sensor_msgs/Image") {
    return "zenoh-udp";
  }
  if (cloud_session_enabled && ros_type == "sensor_msgs/PointCloud2") {
    return "zenoh-cloud-tcp";
  }
  return image_session_enabled || cloud_session_enabled ? "zenoh-tcp" : "zenoh";
}

void ValidateEndpointProtocols(const std::vector<std::string>& endpoints,
                               const std::string& protocol,
                               const std::string& config_name) {
  for (const std::string& endpoint : endpoints) {
    if (!UsesEndpointProtocol(endpoint, protocol)) {
      throw std::invalid_argument(config_name + " endpoint must use " +
                                  protocol + "/: " + endpoint);
    }
  }
}

}  // namespace

BridgeFactory::BridgeFactory(ros::NodeHandle& node,
                             ros::NodeHandle& node_public)
    : nh_(std::make_shared<ros::NodeHandle>(node)),
      nh_public_(std::make_shared<ros::NodeHandle>(node_public)) {
  XmlRpc::XmlRpcValue config;
  if (!nh_->getParam("config", config) ||
      config.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    throw std::runtime_error("cannot find valid private ~config parameter");
  }
  if (config.hasMember("odom_convert")) {
    do_odom_convert_ = XmlRpcToBool(config["odom_convert"], true);
  }
  if (config.hasMember("debug")) {
    is_debug_ = XmlRpcToBool(config["debug"], false);
  }

  INFO_MSG_GREEN(">>>>>>>>>>>>>>>>>>>>> NetBridge Zenoh >>>>>>>>>>>>>>>>>>>>>");
  getMyHostName();
  getHostTopicAndTransportConfig();
  control_transport_ = std::make_shared<swarm_ros_bridge::transport::ZenohTransport>(
      zenoh_config_, my_hostname_);
  if (image_session_enabled_) {
    image_transport_ =
        std::make_shared<swarm_ros_bridge::transport::ZenohTransport>(
            image_zenoh_config_, my_hostname_);
  }
  if (cloud_session_enabled_) {
    cloud_transport_ =
        std::make_shared<swarm_ros_bridge::transport::ZenohTransport>(
            cloud_zenoh_config_, my_hostname_);
  }

  INFO_MSG_GREEN(">>>>>>>>>>>>>>>>>>>>> Topic List >>>>>>>>>>>>>>>>>>>>>>");
  topicOperatorInit();
  diagnostics_pub_ = nh_public_->advertise<swarm_ros_bridge::NetworkArray>(
      "/swarm_bridge/diagnostics", 10);
  diagnostics_timer_ = nh_public_->createTimer(
      ros::Duration(1.0), &BridgeFactory::publishDiagnostics, this);
  INFO_MSG_GREEN(">>>>>>>>>>>>>>>>>>>>> Service List >>>>>>>>>>>>>>>>>>>>>");
  getServiceConfigAndInit();
}

BridgeFactory::~BridgeFactory() { stopBridge(); }

void BridgeFactory::getMyHostName() {
  if (nh_->getParam("hostname", my_hostname_)) {
    if (my_hostname_.rfind("drone", 0) == 0) {
      my_drone_id_ = std::stoi(my_hostname_.substr(5));
    }
  } else if (const char* drone_id = std::getenv("DRONE_ID")) {
    my_drone_id_ = std::atoi(drone_id);
    my_hostname_ = "drone" + std::to_string(my_drone_id_);
  } else {
    throw std::runtime_error("cannot find hostname parameter or DRONE_ID");
  }
  if (my_hostname_.empty()) {
    throw std::runtime_error("hostname cannot be empty");
  }
  INFO_MSG_BLUE("[Bridge] hostname -> " << my_hostname_);
}

void BridgeFactory::expandHostSelection(
    const XmlRpc::XmlRpcValue& selection,
    std::map<std::string, bool>* output) const {
  if (output == nullptr ||
      selection.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    throw std::invalid_argument("srcIP/dstIP/clientIp must be an array");
  }
  for (int i = 0; i < selection.size(); ++i) {
    const std::string selected = static_cast<std::string>(selection[i]);
    if (selected == "all") {
      for (auto& host : *output) {
        host.second = true;
      }
    } else if (selected == "all_drone") {
      for (auto& host : *output) {
        if (host.first.rfind("drone", 0) == 0) {
          host.second = true;
        }
      }
    } else {
      const auto host = output->find(selected);
      if (host == output->end()) {
        throw std::invalid_argument("unknown host in route: " + selected);
      }
      host->second = true;
    }
  }
}

void BridgeFactory::getHostTopicAndTransportConfig() {
  XmlRpc::XmlRpcValue hosts_xml;
  if (nh_->getParam("hosts", hosts_xml)) {
    if (hosts_xml.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      throw std::invalid_argument("hosts must be an array");
    }
    for (int i = 0; i < hosts_xml.size(); ++i) {
      host_map_[static_cast<std::string>(hosts_xml[i])] = {};
    }
  } else {
    XmlRpc::XmlRpcValue legacy_ip_xml;
    if (!nh_->getParam("IP", legacy_ip_xml) ||
        legacy_ip_xml.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      throw std::runtime_error("hosts is required (legacy IP map is also accepted)");
    }
    ROS_WARN("[Bridge] parameter 'IP' is deprecated; use a 'hosts' array and Zenoh endpoints");
    for (auto iterator = legacy_ip_xml.begin(); iterator != legacy_ip_xml.end();
         ++iterator) {
      if (iterator->first == "all" || iterator->first == "all_drone") {
        continue;
      }
      host_map_[iterator->first] = static_cast<std::string>(iterator->second);
    }
  }
  if (host_map_.find(my_hostname_) == host_map_.end()) {
    throw std::invalid_argument("local hostname is missing from hosts/IP configuration");
  }
  for (const auto& host : host_map_) {
    src_hostname_map_[host.first] = false;
    dst_hostname_map_[host.first] = false;
  }

  XmlRpc::XmlRpcValue zenoh_xml;
  if (nh_->getParam("zenoh", zenoh_xml)) {
    if (zenoh_xml.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      throw std::invalid_argument("zenoh must be a struct");
    }
    if (zenoh_xml.hasMember("mode")) {
      zenoh_config_.mode = XmlRpcToString(zenoh_xml["mode"], "peer");
    }
    if (zenoh_xml.hasMember("multicast_scouting")) {
      zenoh_config_.multicast_scouting =
          XmlRpcToBool(zenoh_xml["multicast_scouting"], true);
    }
    if (zenoh_xml.hasMember("gossip_scouting")) {
      zenoh_config_.gossip_scouting =
          XmlRpcToBool(zenoh_xml["gossip_scouting"], true);
    }
    if (zenoh_xml.hasMember("compression_enabled")) {
      zenoh_config_.compression_enabled =
          XmlRpcToBool(zenoh_xml["compression_enabled"], false);
    }
    if (zenoh_xml.hasMember("multicast_address")) {
      zenoh_config_.multicast_scouting_address =
          XmlRpcToString(zenoh_xml["multicast_address"], "");
    }
    if (zenoh_xml.hasMember("listen_endpoints")) {
      zenoh_config_.listen_endpoints =
          XmlRpcToStringVector(zenoh_xml["listen_endpoints"]);
    }
    if (zenoh_xml.hasMember("connect_endpoints")) {
      zenoh_config_.connect_endpoints =
          XmlRpcToStringVector(zenoh_xml["connect_endpoints"]);
    }
    if (zenoh_xml.hasMember("service_timeout_ms")) {
      zenoh_config_.service_timeout_ms = static_cast<std::uint64_t>(std::max(
          1, XmlRpcToInt(zenoh_xml["service_timeout_ms"], 1000)));
    }
    if (zenoh_xml.hasMember("service_worker_threads")) {
      zenoh_config_.service_worker_threads = static_cast<std::size_t>(std::max(
          1, XmlRpcToInt(zenoh_xml["service_worker_threads"], 2)));
    }
    if (zenoh_xml.hasMember("service_queue_capacity")) {
      zenoh_config_.service_queue_capacity = static_cast<std::size_t>(std::max(
          1, XmlRpcToInt(zenoh_xml["service_queue_capacity"], 64)));
    }
    const bool seed_from_ip =
        zenoh_xml.hasMember("seed_from_ip_table") &&
        XmlRpcToBool(zenoh_xml["seed_from_ip_table"], false);
    const int seed_port = zenoh_xml.hasMember("seed_port")
                              ? XmlRpcToInt(zenoh_xml["seed_port"], 7447)
                              : 7447;
    if (seed_from_ip) {
      for (const auto& host : host_map_) {
        if (host.first != my_hostname_ && !host.second.empty()) {
          zenoh_config_.connect_endpoints.push_back(
              "tcp/" + host.second + ":" + std::to_string(seed_port));
        }
      }
    }

    if (zenoh_xml.hasMember("image_session")) {
      const XmlRpc::XmlRpcValue& image_xml = zenoh_xml["image_session"];
      if (image_xml.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        throw std::invalid_argument("zenoh.image_session must be a struct");
      }
      image_session_enabled_ = image_xml.hasMember("enabled")
                                   ? XmlRpcToBool(image_xml["enabled"], false)
                                   : false;
      if (image_session_enabled_) {
        image_zenoh_config_ = zenoh_config_;
        image_zenoh_config_.enable_liveliness = false;
        image_zenoh_config_.service_worker_threads = 1U;
        image_zenoh_config_.service_queue_capacity = 1U;
        if (image_xml.hasMember("mode")) {
          image_zenoh_config_.mode =
              XmlRpcToString(image_xml["mode"], zenoh_config_.mode);
        }
        if (image_xml.hasMember("multicast_scouting")) {
          image_zenoh_config_.multicast_scouting = XmlRpcToBool(
              image_xml["multicast_scouting"],
              zenoh_config_.multicast_scouting);
        }
        if (image_xml.hasMember("gossip_scouting")) {
          image_zenoh_config_.gossip_scouting = XmlRpcToBool(
              image_xml["gossip_scouting"], zenoh_config_.gossip_scouting);
        }
        if (image_xml.hasMember("multicast_address")) {
          image_zenoh_config_.multicast_scouting_address =
              XmlRpcToString(image_xml["multicast_address"], "");
        }
        if (image_xml.hasMember("compression_enabled")) {
          image_zenoh_config_.compression_enabled = XmlRpcToBool(
              image_xml["compression_enabled"], false);
        }
        image_zenoh_config_.listen_endpoints =
            image_xml.hasMember("listen_endpoints")
                ? XmlRpcToStringVector(image_xml["listen_endpoints"])
                : std::vector<std::string>{};
        image_zenoh_config_.connect_endpoints =
            image_xml.hasMember("connect_endpoints")
                ? XmlRpcToStringVector(image_xml["connect_endpoints"])
                : std::vector<std::string>{};

        const bool image_seed_from_ip =
            image_xml.hasMember("seed_from_ip_table") &&
            XmlRpcToBool(image_xml["seed_from_ip_table"], false);
        const int image_seed_port = image_xml.hasMember("seed_port")
                                        ? XmlRpcToInt(image_xml["seed_port"], 7448)
                                        : 7448;
        if (image_seed_from_ip) {
          for (const auto& host : host_map_) {
            if (host.first != my_hostname_ && !host.second.empty()) {
              image_zenoh_config_.connect_endpoints.push_back(
                  "udp/" + host.second + ":" +
                  std::to_string(image_seed_port) +
                  "?rel=1;mixed_rel=1;multistream=1");
            }
          }
        }

        zenoh_config_.allowed_link_protocols = {"tcp"};
        image_zenoh_config_.allowed_link_protocols = {"udp"};
        ValidateEndpointProtocols(zenoh_config_.listen_endpoints, "tcp",
                                  "zenoh control session listen");
        ValidateEndpointProtocols(zenoh_config_.connect_endpoints, "tcp",
                                  "zenoh control session connect");
        ValidateEndpointProtocols(image_zenoh_config_.listen_endpoints, "udp",
                                  "zenoh image session listen");
        ValidateEndpointProtocols(image_zenoh_config_.connect_endpoints, "udp",
                                  "zenoh image session connect");
        if (image_zenoh_config_.listen_endpoints.empty() &&
            image_zenoh_config_.connect_endpoints.empty()) {
          throw std::invalid_argument(
              "zenoh.image_session requires at least one UDP listen or connect endpoint");
        }
      }
    }

    if (zenoh_xml.hasMember("cloud_session")) {
      const XmlRpc::XmlRpcValue& cloud_xml = zenoh_xml["cloud_session"];
      if (cloud_xml.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        throw std::invalid_argument("zenoh.cloud_session must be a struct");
      }
      cloud_session_enabled_ = cloud_xml.hasMember("enabled")
                                   ? XmlRpcToBool(cloud_xml["enabled"], false)
                                   : false;
      if (cloud_session_enabled_) {
        cloud_zenoh_config_ = zenoh_config_;
        cloud_zenoh_config_.enable_liveliness = false;
        cloud_zenoh_config_.service_worker_threads = 1U;
        cloud_zenoh_config_.service_queue_capacity = 1U;
        cloud_zenoh_config_.mode = cloud_xml.hasMember("mode")
                                       ? XmlRpcToString(cloud_xml["mode"], zenoh_config_.mode)
                                       : zenoh_config_.mode;
        cloud_zenoh_config_.multicast_scouting =
            cloud_xml.hasMember("multicast_scouting")
                ? XmlRpcToBool(cloud_xml["multicast_scouting"],
                               zenoh_config_.multicast_scouting)
                : zenoh_config_.multicast_scouting;
        cloud_zenoh_config_.gossip_scouting =
            cloud_xml.hasMember("gossip_scouting")
                ? XmlRpcToBool(cloud_xml["gossip_scouting"],
                               zenoh_config_.gossip_scouting)
                : zenoh_config_.gossip_scouting;
        cloud_zenoh_config_.compression_enabled =
            cloud_xml.hasMember("compression_enabled")
                ? XmlRpcToBool(cloud_xml["compression_enabled"], false)
                : false;
        cloud_zenoh_config_.multicast_scouting_address =
            cloud_xml.hasMember("multicast_address")
                ? XmlRpcToString(cloud_xml["multicast_address"], "")
                : std::string("224.0.0.225:7446");
        cloud_zenoh_config_.listen_endpoints =
            cloud_xml.hasMember("listen_endpoints")
                ? XmlRpcToStringVector(cloud_xml["listen_endpoints"])
                : std::vector<std::string>{};
        cloud_zenoh_config_.connect_endpoints =
            cloud_xml.hasMember("connect_endpoints")
                ? XmlRpcToStringVector(cloud_xml["connect_endpoints"])
                : std::vector<std::string>{};

        const bool cloud_seed_from_ip =
            cloud_xml.hasMember("seed_from_ip_table") &&
            XmlRpcToBool(cloud_xml["seed_from_ip_table"], false);
        const int cloud_seed_port = cloud_xml.hasMember("seed_port")
                                        ? XmlRpcToInt(cloud_xml["seed_port"], 7449)
                                        : 7449;
        if (cloud_seed_from_ip) {
          for (const auto& host : host_map_) {
            if (host.first != my_hostname_ && !host.second.empty()) {
              cloud_zenoh_config_.connect_endpoints.push_back(
                  "tcp/" + host.second + ":" +
                  std::to_string(cloud_seed_port));
            }
          }
        }

        zenoh_config_.allowed_link_protocols = {"tcp"};
        cloud_zenoh_config_.allowed_link_protocols = {"tcp"};
        ValidateEndpointProtocols(zenoh_config_.listen_endpoints, "tcp",
                                  "zenoh control session listen");
        ValidateEndpointProtocols(zenoh_config_.connect_endpoints, "tcp",
                                  "zenoh control session connect");
        ValidateEndpointProtocols(cloud_zenoh_config_.listen_endpoints, "tcp",
                                  "zenoh cloud session listen");
        ValidateEndpointProtocols(cloud_zenoh_config_.connect_endpoints, "tcp",
                                  "zenoh cloud session connect");
        if (cloud_zenoh_config_.listen_endpoints.empty() &&
            cloud_zenoh_config_.connect_endpoints.empty()) {
          throw std::invalid_argument(
              "zenoh.cloud_session requires at least one TCP listen or connect endpoint");
        }
      }
    }
  }
  if (zenoh_config_.mode != "peer" && zenoh_config_.mode != "client" &&
      zenoh_config_.mode != "router") {
    throw std::invalid_argument("zenoh.mode must be peer, client, or router");
  }
  if (zenoh_config_.compression_enabled) {
    ROS_WARN("[Bridge] Zenoh generic compression is enabled; disable it for JPEG/Draco workloads");
  }
  if (image_session_enabled_ &&
      image_zenoh_config_.mode != "peer" &&
      image_zenoh_config_.mode != "client" &&
      image_zenoh_config_.mode != "router") {
    throw std::invalid_argument(
        "zenoh.image_session.mode must be peer, client, or router");
  }
  if (cloud_session_enabled_ &&
      cloud_zenoh_config_.mode != "peer" &&
      cloud_zenoh_config_.mode != "client" &&
      cloud_zenoh_config_.mode != "router") {
    throw std::invalid_argument(
        "zenoh.cloud_session.mode must be peer, client, or router");
  }

  if (!nh_->getParam("topics", topics_xml_)) {
    topics_xml_.setSize(0);
  }
  if (topics_xml_.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    throw std::invalid_argument("topics must be an array");
  }

  for (int i = 0; i < topics_xml_.size(); ++i) {
    XmlRpc::XmlRpcValue topic_xml = topics_xml_[i];
    if (topic_xml.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      throw std::invalid_argument("each topic entry must be a struct");
    }
    const std::string topic_name = static_cast<std::string>(topic_xml["topic_name"]);
    const std::string topic_type = static_cast<std::string>(topic_xml["msg_type"]);
    if (topic_name.empty() || topic_name.front() != '/') {
      throw std::invalid_argument("topic_name must start with '/': " + topic_name);
    }
    if (!topic_xml.hasMember("qos_class")) {
      throw std::invalid_argument("qos_class is required for topic " + topic_name);
    }
    swarm_ros_bridge::transport::QosClass qos_class;
    const std::string qos_name = static_cast<std::string>(topic_xml["qos_class"]);
    if (!swarm_ros_bridge::transport::ParseQosClass(qos_name, &qos_class) ||
        qos_class == swarm_ros_bridge::transport::QosClass::kService) {
      throw std::invalid_argument("invalid topic qos_class for " + topic_name +
                                  ": " + qos_name);
    }

    TopicCfg topic;
    topic.my_hostname_ = my_hostname_;
    topic.name_ = topic_name;
    topic.wire_name_ = topic_name;
    topic.type_ = topic_type;
    topic.qos_class_ = qos_class;
    topic.max_freq_ = XmlRpcToDouble(topic_xml["max_freq"], 10.0);
    topic.has_prefix_ = topic_xml.hasMember("prefix")
                            ? XmlRpcToBool(topic_xml["prefix"], true)
                            : true;
    topic.same_prefix_ = topic_xml.hasMember("same_prefix")
                             ? XmlRpcToBool(topic_xml["same_prefix"], false)
                             : false;
    topic.src_hostnames_xml = topic_xml["srcIP"];
    topic.dst_hostnames_xml = topic_xml["dstIP"];
    topic.src_hostname_map_ = src_hostname_map_;
    topic.dst_hostname_map_ = dst_hostname_map_;
    expandHostSelection(topic.src_hostnames_xml, &topic.src_hostname_map_);
    expandHostSelection(topic.dst_hostnames_xml, &topic.dst_hostname_map_);

    if (topic_type == "sensor_msgs/Image") {
      if (topic_xml.hasMember("imgResizeRate")) {
        topic.img_resize_rate_ =
            XmlRpcToDouble(topic_xml["imgResizeRate"], topic.img_resize_rate_);
      }
      if (topic_xml.hasMember("imgJpegQuality")) {
        topic.img_jpeg_quality_ =
            std::max(10, std::min(100, XmlRpcToInt(topic_xml["imgJpegQuality"], 80)));
      }
      if (topic_xml.hasMember("imgAdaptiveQuality")) {
        topic.img_adaptive_quality_ =
            XmlRpcToBool(topic_xml["imgAdaptiveQuality"], false);
      }
      if (topic_xml.hasMember("imgMinJpegQuality")) {
        topic.img_min_jpeg_quality_ = XmlRpcToInt(topic_xml["imgMinJpegQuality"], 45);
      }
      if (topic_xml.hasMember("imgMaxJpegQuality")) {
        topic.img_max_jpeg_quality_ = XmlRpcToInt(topic_xml["imgMaxJpegQuality"], 90);
      }
      if (topic_xml.hasMember("imgTargetBandwidthKbps")) {
        topic.img_target_bandwidth_kbps_ =
            XmlRpcToDouble(topic_xml["imgTargetBandwidthKbps"], 1200.0);
      }
      if (topic_xml.hasMember("imgQualityStep")) {
        topic.img_quality_step_ =
            std::max(1, XmlRpcToInt(topic_xml["imgQualityStep"], 5));
      }
      if (topic_xml.hasMember("imgAdaptCooldownFrames")) {
        topic.img_adapt_cooldown_frames_ =
            std::max(1, XmlRpcToInt(topic_xml["imgAdaptCooldownFrames"], 8));
      }
      topic.img_min_jpeg_quality_ =
          std::max(10, std::min(100, topic.img_min_jpeg_quality_));
      topic.img_max_jpeg_quality_ = std::max(
          topic.img_min_jpeg_quality_, std::min(100, topic.img_max_jpeg_quality_));
    }
    if (topic_type == "sensor_msgs/PointCloud2") {
      if (topic_xml.hasMember("cloudCompress")) {
        topic.cloud_compress_ = XmlRpcToBool(topic_xml["cloudCompress"], false);
      }
      if (topic_xml.hasMember("cloudDownsample")) {
        topic.cloud_downsample_ =
            XmlRpcToDouble(topic_xml["cloudDownsample"], -1.0);
        if (topic.cloud_downsample_ < 1e-4 || topic.cloud_downsample_ > 1e4) {
          topic.cloud_downsample_ = -1.0;
        }
      }
      topic.cloud_codec_ = topic_xml.hasMember("cloudCodec")
                               ? XmlRpcToString(topic_xml["cloudCodec"], "raw")
                               : (topic.cloud_compress_ ? "pcl_octree" : "raw");
      if (topic.cloud_compress_ && topic.cloud_codec_ != "draco" &&
          topic.cloud_codec_ != "pcl_octree") {
        throw std::invalid_argument("cloudCodec must be draco or pcl_octree");
      }
    }

    bool has_ids_member = false;
#define X(type, classname)                                                   \
  if constexpr (has_data_to_drone_ids<classname, std::vector<uint8_t>>::value) { \
    has_ids_member = has_ids_member || topic.type_ == type;                  \
  }
    MSGS_MACRO
#undef X
    topic.dynamic_dst_ = has_ids_member;
    topic_cfgs_.push_back(std::move(topic));
  }
}

void BridgeFactory::topicOperatorInit() {
  for (const TopicCfg& configured : topic_cfgs_) {
    TopicCfg topic = configured;
    if (!topic.src_hostname_map_[my_hostname_]) {
      continue;
    }
    const std::string drone_prefix = "/drone_{id}";
    if (topic.name_.rfind(drone_prefix, 0) == 0 &&
        my_drone_id_ != DRONE_ID_NULL) {
      topic.name_.replace(0, drone_prefix.size(),
                          "/drone_" + std::to_string(my_drone_id_));
    }
    topic.raw_name_ = nh_->resolveName(topic.name_);
    topic.src_hostname_ = my_hostname_;
    topic.transport_name_ = TopicTransportName(
        image_session_enabled_, cloud_session_enabled_, topic.type_);
    send_topics_[topic.name_] = std::make_shared<TopicFactory>(
        topic, transportForTopic(topic), TopicFactory::SEND, nh_public_);
  }

  for (const TopicCfg& configured : topic_cfgs_) {
    if (!configured.dst_hostname_map_.at(my_hostname_)) {
      continue;
    }
    for (const auto& source : host_map_) {
      if (source.first == my_hostname_ && !is_debug_) {
        continue;
      }
      if (!configured.src_hostname_map_.at(source.first)) {
        continue;
      }
      TopicCfg topic = configured;
      const std::string drone_prefix = "/drone_{id}";
      if (topic.name_.rfind(drone_prefix, 0) == 0) {
        if (source.first.rfind("drone", 0) != 0) {
          throw std::invalid_argument("{id} topic source must be a drone host");
        }
        const int drone_id = topic.dynamic_dst_
                                 ? my_drone_id_
                                 : std::stoi(source.first.substr(5));
        topic.name_.replace(0, drone_prefix.size(),
                            "/drone_" + std::to_string(drone_id));
      }
      topic.raw_name_ = nh_->resolveName(topic.name_);
      if (topic.has_prefix_ && !topic.same_prefix_) {
        topic.name_ = "/" + source.first + topic.name_;
      } else if (topic.same_prefix_) {
        topic.name_ = "/bridge" + topic.name_;
      }
      topic.src_hostname_ = source.first;
      topic.transport_name_ = TopicTransportName(
          image_session_enabled_, cloud_session_enabled_, topic.type_);
      if (recv_topics_.find(topic.name_) != recv_topics_.end()) {
        throw std::invalid_argument("duplicate receive ROS topic: " + topic.name_);
      }
      recv_topics_[topic.name_] = std::make_shared<TopicFactory>(
          topic, transportForTopic(topic), TopicFactory::RECV, nh_public_);
      if (topic.dynamic_dst_) {
        break;
      }
    }
  }
  INFO_MSG_GREEN("[Bridge] send topics: " << send_topics_.size()
                                           << ", receive topics: "
                                           << recv_topics_.size());
}

std::shared_ptr<swarm_ros_bridge::transport::BridgeTransport>
BridgeFactory::transportForTopic(const TopicCfg& topic) const {
  if (image_session_enabled_ && topic.type_ == "sensor_msgs/Image") {
    if (image_transport_ == nullptr) {
      throw std::runtime_error("image Zenoh session is not initialized");
    }
    return image_transport_;
  }
  if (cloud_session_enabled_ && topic.type_ == "sensor_msgs/PointCloud2") {
    if (cloud_transport_ == nullptr) {
      throw std::runtime_error("cloud Zenoh session is not initialized");
    }
    return cloud_transport_;
  }
  if (control_transport_ == nullptr) {
    throw std::runtime_error("control Zenoh session is not initialized");
  }
  return control_transport_;
}

void BridgeFactory::getServiceConfigAndInit() {
  if (!nh_->getParam("services", services_xml_)) {
    services_xml_.setSize(0);
  }
  if (services_xml_.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    throw std::invalid_argument("services must be an array");
  }

  for (int i = 0; i < services_xml_.size(); ++i) {
    XmlRpc::XmlRpcValue service_xml = services_xml_[i];
    const std::string service_name =
        static_cast<std::string>(service_xml["srv_name"]);
    const std::string service_type =
        static_cast<std::string>(service_xml["srv_type"]);
    const std::string server_hostname =
        static_cast<std::string>(service_xml["serverIp"]);
    if (service_name.empty() || service_name.front() != '/') {
      throw std::invalid_argument("srv_name must start with '/': " + service_name);
    }
    if (host_map_.find(server_hostname) == host_map_.end()) {
      throw std::invalid_argument("unknown service server host: " + server_hostname);
    }

    std::map<std::string, bool> client_hosts = dst_hostname_map_;
    expandHostSelection(service_xml["clientIp"], &client_hosts);
    const bool i_am_server = server_hostname == my_hostname_;
    const bool i_am_client = client_hosts[my_hostname_];
    if (i_am_server && i_am_client) {
      throw std::invalid_argument("a host cannot be both client and server for " +
                                  service_name);
    }

    ServiceConfig config;
    config.service_name = service_name;
    config.service_type = service_type;
    config.server_hostname = server_hostname;
    config.my_hostname = my_hostname_;
    config.client_hostnames = std::move(client_hosts);
    config.timeout_ms = zenoh_config_.service_timeout_ms;
    config.if_prefix = service_xml.hasMember("prefix")
                           ? XmlRpcToBool(service_xml["prefix"], true)
                           : true;
    config.service_prefix_name = config.if_prefix
                                     ? "/" + server_hostname + service_name
                                     : service_name;
    if (i_am_server) {
      service_servers_[service_name] = std::make_shared<ServiceFactory>(
          ServiceFactory::SERVER, nh_public_, config, control_transport_);
    } else if (i_am_client) {
      service_clients_[service_name] = std::make_shared<ServiceFactory>(
          ServiceFactory::CLIENT, nh_public_, config, control_transport_);
    }
  }
}

void BridgeFactory::startBridge() {
  for (auto& receiver : recv_topics_) {
    receiver.second->createThread();
  }
  INFO_MSG_GREEN("[Bridge] all Zenoh subscriptions started");
}

void BridgeFactory::stopBridge() {
  if (stopped_) {
    return;
  }
  stopped_ = true;
  diagnostics_timer_.stop();
  for (auto& receiver : recv_topics_) {
    receiver.second->stopThread();
  }
  for (auto& sender : send_topics_) {
    sender.second->stopThread();
  }
  for (auto& service : service_clients_) {
    service.second->stopClientThread();
  }
  for (auto& service : service_servers_) {
    service.second->stopServerThread();
  }
  service_clients_.clear();
  service_servers_.clear();
  recv_topics_.clear();
  send_topics_.clear();
  if (image_transport_ != nullptr) {
    image_transport_->Close();
  }
  if (cloud_transport_ != nullptr) {
    cloud_transport_->Close();
  }
  if (control_transport_ != nullptr) {
    control_transport_->Close();
  }
  INFO_MSG_RED("[Bridge] Zenoh bridge stopped cleanly");
}

void BridgeFactory::publishDiagnostics(const ros::TimerEvent&) {
  swarm_ros_bridge::NetworkArray message;
  message.header.stamp = ros::Time::now();
  message.header.frame_id = my_hostname_;

  const auto control_stats =
      control_transport_ == nullptr
          ? swarm_ros_bridge::transport::TransportStats{}
          : control_transport_->GetStats();
  const auto image_stats =
      image_transport_ == nullptr
          ? swarm_ros_bridge::transport::TransportStats{}
          : image_transport_->GetStats();
  const auto cloud_stats =
      cloud_transport_ == nullptr
          ? swarm_ros_bridge::transport::TransportStats{}
          : cloud_transport_->GetStats();
  const auto append_metrics = [&message, &control_stats, &image_stats, &cloud_stats](
                                  const TopicFactory::Ptr& factory) {
    const auto metrics = factory->GetMetricsSnapshot();
    const auto& transport_stats =
        metrics.transport == "zenoh-udp"
            ? image_stats
            : (metrics.transport == "zenoh-cloud-tcp" ? cloud_stats
                                                        : control_stats);
    swarm_ros_bridge::NetworkInfo info;
    info.name = metrics.topic_name;
    info.msg_type = metrics.msg_type;
    info.direction = metrics.direction;
    info.codec = metrics.codec;
    info.transport = metrics.transport;
    info.qos_class = metrics.qos_class;
    info.configured_rate_hz = static_cast<float>(metrics.configured_rate_hz);
    info.send_rate_hz = static_cast<float>(metrics.send_rate_hz);
    info.recv_rate_hz = static_cast<float>(metrics.recv_rate_hz);
    info.bandwidth_kbps = static_cast<float>(metrics.bandwidth_kbps);
    info.effective_recv_bandwidth_kbps =
        static_cast<float>(metrics.effective_recv_bandwidth_kbps);
    info.image_loss_rate_pct =
        static_cast<float>(metrics.image_loss_rate_pct);
    info.complete_frame_success_rate_pct =
        static_cast<float>(metrics.complete_frame_success_rate_pct);
    info.avg_latency_ms = static_cast<float>(metrics.avg_latency_ms);
    info.jitter_ms = static_cast<float>(metrics.jitter_ms);
    info.stability_score = static_cast<float>(metrics.stability_score);
    info.last_recv_age_ms = static_cast<float>(metrics.last_recv_age_ms);
    info.adaptive_quality_enabled = metrics.adaptive_quality_enabled;
    info.configured_jpeg_quality = metrics.configured_jpeg_quality;
    info.current_jpeg_quality = metrics.current_jpeg_quality;
    info.target_bandwidth_kbps = static_cast<float>(metrics.target_bandwidth_kbps);
    info.packet_size = metrics.packet_size;
    info.total_messages = static_cast<std::uint32_t>(metrics.total_messages);
    info.dropped_messages = static_cast<std::uint32_t>(metrics.dropped_messages);
    info.transport_queue_drops = metrics.transport_queue_drops;
    info.expected_frames = metrics.expected_frames;
    info.transport_complete_frames = metrics.transport_complete_frames;
    info.decoded_frames = metrics.decoded_frames;
    info.inferred_lost_frames = metrics.inferred_lost_frames;
    info.sequence_resets = metrics.sequence_resets;
    info.session_connected = transport_stats.session_open;
    info.link_connected = transport_stats.link_connected;
    info.connected_peer_count = transport_stats.connected_peer_count;
    info.connected_router_count = transport_stats.connected_router_count;
    message.info.push_back(std::move(info));
  };
  for (const auto& sender : send_topics_) {
    append_metrics(sender.second);
  }
  for (const auto& receiver : recv_topics_) {
    append_metrics(receiver.second);
  }

  const auto append_session = [&message](
                                  const std::string& name,
                                  const std::string& direction,
                                  const std::string& transport_name,
                                  const swarm_ros_bridge::transport::TransportStats& stats) {
    swarm_ros_bridge::NetworkInfo session;
    session.name = name;
    session.msg_type = "transport";
    session.direction = direction;
    session.codec = "none";
    session.transport = transport_name;
    session.qos_class =
        direction == "image" || direction == "cloud" ? "bulk" : "service";
    session.session_connected = stats.session_open;
    session.link_connected = stats.link_connected;
    session.connected_peer_count = stats.connected_peer_count;
    session.connected_router_count = stats.connected_router_count;
    session.reconnect_count = stats.reconnect_count;
    session.transport_queue_drops =
        stats.callback_drops + stats.service_queue_drops;
    session.service_timeouts = stats.service_timeouts;
    session.decode_errors = stats.receive_decode_errors;
    message.info.push_back(std::move(session));
  };

  if (control_transport_ != nullptr) {
    append_session("@zenoh/session/control", "control",
                   image_session_enabled_ || cloud_session_enabled_
                       ? "zenoh-tcp"
                       : "zenoh",
                   control_stats);
  }
  if (image_transport_ != nullptr) {
    append_session("@zenoh/session/image", "image", "zenoh-udp",
                   image_stats);
  }
  if (cloud_transport_ != nullptr) {
    append_session("@zenoh/session/cloud", "cloud", "zenoh-cloud-tcp",
                   cloud_stats);
  }

  if (control_transport_ != nullptr) {
    for (const auto& node : control_transport_->GetNodePresence()) {
      swarm_ros_bridge::NetworkInfo presence;
      presence.name = "@zenoh/node/" + node.hostname;
      presence.msg_type = "presence";
      presence.direction = node.online ? "online" : "offline";
      presence.codec = "none";
      presence.transport = "zenoh-liveliness";
      presence.node_hostname = node.hostname;
      presence.node_online = node.online;
      presence.node_state_age_ms = node.state_age_ms;
      presence.node_online_transitions = node.online_transitions;
      presence.session_connected = control_stats.session_open;
      presence.link_connected = control_stats.link_connected;
      message.info.push_back(std::move(presence));
    }
  }
  diagnostics_pub_.publish(message);
}
