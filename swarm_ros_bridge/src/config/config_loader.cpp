#include "config/config_loader.hpp"

#include <xmlrpcpp/XmlRpcValue.h>

namespace swarm_ros_bridge {
namespace config {

namespace {

std::string GetString(const XmlRpc::XmlRpcValue& value) {
  return static_cast<std::string>(value);
}

double GetDouble(const XmlRpc::XmlRpcValue& value, double fallback) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(value);
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<double>(value);
  }
  return fallback;
}

bool GetBoolMember(const XmlRpc::XmlRpcValue& value,
                   const std::string& key,
                   bool fallback) {
  if (!value.hasMember(key)) {
    return fallback;
  }
  return static_cast<bool>(value[key]);
}

int GetIntMember(const XmlRpc::XmlRpcValue& value,
                 const std::string& key,
                 int fallback) {
  if (!value.hasMember(key)) {
    return fallback;
  }
  if (value[key].getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(value[key]);
  }
  return fallback;
}

void LoadHosts(const XmlRpc::XmlRpcValue& list, std::vector<std::string>* out) {
  if (out == nullptr || list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return;
  }
  out->clear();
  for (int i = 0; i < list.size(); ++i) {
    out->push_back(GetString(list[i]));
  }
}

void LoadTopicRules(const XmlRpc::XmlRpcValue& topics, BridgeConfig* config) {
  if (config == nullptr || topics.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return;
  }

  config->topics.clear();
  for (int i = 0; i < topics.size(); ++i) {
    const XmlRpc::XmlRpcValue& topic_xml = topics[i];
    TopicRule rule;
    rule.topic_name = GetString(topic_xml["topic_name"]);
    rule.msg_type = GetString(topic_xml["msg_type"]);
    LoadHosts(topic_xml["srcIP"], &rule.src_hosts);
    LoadHosts(topic_xml["dstIP"], &rule.dst_hosts);
    rule.src_port = static_cast<int>(topic_xml["srcPort"]);
    rule.max_freq_hz = GetDouble(topic_xml["max_freq"], rule.max_freq_hz);
    rule.prefix = GetBoolMember(topic_xml, "prefix", rule.prefix);
    rule.same_prefix = GetBoolMember(topic_xml, "same_prefix", rule.same_prefix);
    rule.cloud_compress = GetBoolMember(topic_xml, "cloudCompress", rule.cloud_compress);
    rule.cloud_downsample = topic_xml.hasMember("cloudDownsample")
                                ? GetDouble(topic_xml["cloudDownsample"], rule.cloud_downsample)
                                : rule.cloud_downsample;
    rule.image_resize_rate = topic_xml.hasMember("imgResizeRate")
                                 ? GetDouble(topic_xml["imgResizeRate"], rule.image_resize_rate)
                                 : rule.image_resize_rate;
    rule.image_jpeg_quality = GetIntMember(topic_xml, "imgJpegQuality", rule.image_jpeg_quality);
    rule.image_adaptive_quality =
        GetBoolMember(topic_xml, "imgAdaptiveQuality", rule.image_adaptive_quality);
    rule.image_min_jpeg_quality =
        GetIntMember(topic_xml, "imgMinJpegQuality", rule.image_min_jpeg_quality);
    rule.image_max_jpeg_quality =
        GetIntMember(topic_xml, "imgMaxJpegQuality", rule.image_max_jpeg_quality);
    rule.image_target_bandwidth_kbps = topic_xml.hasMember("imgTargetBandwidthKbps")
                                           ? GetDouble(topic_xml["imgTargetBandwidthKbps"],
                                                       rule.image_target_bandwidth_kbps)
                                           : rule.image_target_bandwidth_kbps;
    rule.image_quality_step =
        GetIntMember(topic_xml, "imgQualityStep", rule.image_quality_step);
    rule.image_adapt_cooldown_frames =
        GetIntMember(topic_xml, "imgAdaptCooldownFrames", rule.image_adapt_cooldown_frames);
    if (topic_xml.hasMember("cloudCodec")) {
      rule.cloud_codec = GetString(topic_xml["cloudCodec"]);
    } else if (rule.cloud_compress) {
      rule.cloud_codec = "pcl_octree";
    }
    config->topics.push_back(rule);
  }
}

void LoadServiceRules(const XmlRpc::XmlRpcValue& services, BridgeConfig* config) {
  if (config == nullptr || services.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return;
  }

  config->services.clear();
  for (int i = 0; i < services.size(); ++i) {
    const XmlRpc::XmlRpcValue& service_xml = services[i];
    ServiceRule rule;
    rule.service_name = GetString(service_xml["srv_name"]);
    rule.service_type = GetString(service_xml["srv_type"]);
    rule.server_host = GetString(service_xml["serverIp"]);
    LoadHosts(service_xml["clientIp"], &rule.client_hosts);
    rule.src_port = static_cast<int>(service_xml["srcPort"]);
    rule.prefix = GetBoolMember(service_xml, "prefix", rule.prefix);
    config->services.push_back(rule);
  }
}

}  // namespace

bool ConfigLoader::LoadFromRosParams(const ros::NodeHandle& nh, BridgeConfig* config) {
  if (config == nullptr) {
    return false;
  }

  *config = MakeDefaultConfig();
  nh.getParam("hostname", config->hostname);

  XmlRpc::XmlRpcValue runtime_xml;
  if (nh.getParam("config", runtime_xml) &&
      runtime_xml.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
    config->runtime.debug = GetBoolMember(runtime_xml, "debug", config->runtime.debug);
    config->runtime.odom_convert =
        GetBoolMember(runtime_xml, "odom_convert", config->runtime.odom_convert);
    config->runtime.monitor_node =
        GetBoolMember(runtime_xml, "monitor_node", config->runtime.monitor_node);
    config->runtime.warn_threshold =
        GetIntMember(runtime_xml, "warn_threshold", config->runtime.warn_threshold);
    config->runtime.monitor_rate_hz =
        GetIntMember(runtime_xml, "monitor_rate_hz", config->runtime.monitor_rate_hz);
  }

  XmlRpc::XmlRpcValue ip_xml;
  if (nh.getParam("IP", ip_xml) && ip_xml.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
    config->ip_map.clear();
    for (auto it = ip_xml.begin(); it != ip_xml.end(); ++it) {
      config->ip_map[it->first] = GetString(it->second);
    }
  }

  XmlRpc::XmlRpcValue topics_xml;
  if (nh.getParam("topics", topics_xml)) {
    LoadTopicRules(topics_xml, config);
  }

  XmlRpc::XmlRpcValue services_xml;
  if (nh.getParam("services", services_xml)) {
    LoadServiceRules(services_xml, config);
  }

  return true;
}

}  // namespace config
}  // namespace swarm_ros_bridge
