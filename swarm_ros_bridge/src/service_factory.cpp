#include "service_factory.h"

#include <ros/message.h>
#include <ros/names.h>
#include <ros/serialization.h>
#include <ros/service_traits.h>

#include <boost/make_shared.hpp>

#include <cstring>
#include <exception>
#include <utility>

ServiceFactory::ServiceFactory(
    ServiceFactory::ClientOrServer client_or_server,
    std::shared_ptr<ros::NodeHandle> nh,
    const ServiceConfig& config,
    std::shared_ptr<swarm_ros_bridge::transport::BridgeTransport> transport)
    : nh_(std::move(nh)),
      client_or_server_(client_or_server),
      config_(config),
      transport_(std::move(transport)) {
  if (transport_ == nullptr) {
    throw std::invalid_argument("ServiceFactory requires a shared bridge transport");
  }
  if (!typeExists(config_.service_type)) {
    throw std::invalid_argument("Invalid ROS service type: " + config_.service_type);
  }

  if (client_or_server_ == CLIENT) {
    ros::AdvertiseServiceOptions options;
    options.service = config_.service_prefix_name;
    options.callback_queue = nullptr;
    options.datatype = config_.service_type;
    options.md5sum = getServiceMd5(config_.service_type);
    options.helper = boost::make_shared<ServiceCallbackHelper>(
        config_.service_prefix_name, config_.service_name, this);
    options.req_datatype = config_.service_type + "Request";
    options.res_datatype = config_.service_type + "Response";
    server_handle_client_msg_ = nh_->advertiseService(options);
    INFO_MSG_YELLOW("[SrvFactory] Zenoh proxy: " << config_.service_prefix_name
                                                  << " type: " << config_.service_type);
  } else {
    const std::string key = swarm_ros_bridge::transport::ServiceKey(
        config_.server_hostname, config_.service_name);
    queryable_ = transport_->DeclareQueryable(
        key,
        [this](const swarm_ros_bridge::transport::TransportEnvelope& request,
               swarm_ros_bridge::transport::TransportEnvelope* response,
               std::string* error) {
          return handleRemoteRequest(request, response, error);
        });
    INFO_MSG_GREEN("[SrvFactory] Zenoh queryable: " << key);
  }
}

ServiceFactory::~ServiceFactory() {
  stopClientThread();
  stopServerThread();
}

bool ServiceFactory::call(
    const std::string& service_name,
    ros::ServiceCallbackHelperCallParams& params) const {
  swarm_ros_bridge::transport::TransportEnvelope request;
  request.payload_kind =
      swarm_ros_bridge::transport::PayloadKind::kServiceRequest;
  request.sequence = sequence_.fetch_add(1U) + 1U;
  request.source_time_ns = static_cast<std::int64_t>(ros::Time::now().toNSec());
  request.source_host = config_.my_hostname;
  request.ros_type = config_.service_type;
  request.ros_md5 = getServiceMd5(config_.service_type);
  if (params.request.num_bytes > 0U) {
    request.payload.assign(params.request.buf.get(),
                           params.request.buf.get() + params.request.num_bytes);
  }

  swarm_ros_bridge::transport::TransportEnvelope response;
  std::string error;
  const std::string key = swarm_ros_bridge::transport::ServiceKey(
      config_.server_hostname, service_name);
  if (!transport_->Query(key, request, config_.timeout_ms, &response, &error)) {
    if (error.find("timed out") != std::string::npos) {
      timeout_count_.fetch_add(1U);
    }
    ROS_ERROR_STREAM("[SrvFactory] " << service_name << " failed: " << error);
    return false;
  }
  std::string metadata_error;
  if (response.payload_kind !=
          swarm_ros_bridge::transport::PayloadKind::kServiceResponse ||
      !swarm_ros_bridge::transport::ValidateEnvelopeMetadata(
          response, config_.service_type, getServiceMd5(config_.service_type),
          &metadata_error)) {
    ROS_ERROR_STREAM("[SrvFactory] rejected invalid response envelope for "
                     << service_name << ": " << metadata_error);
    return false;
  }

  boost::shared_array<std::uint8_t> data(new std::uint8_t[response.payload.size()]);
  if (!response.payload.empty()) {
    std::memcpy(data.get(), response.payload.data(), response.payload.size());
  }
  params.response = ros::SerializedMessage(data, response.payload.size());
  return true;
}

bool ServiceFactory::handleRemoteRequest(
    const swarm_ros_bridge::transport::TransportEnvelope& request,
    swarm_ros_bridge::transport::TransportEnvelope* response,
    std::string* error) const {
  if (response == nullptr) {
    if (error != nullptr) {
      *error = "service response output is null";
    }
    return false;
  }
  std::string metadata_error;
  if (request.payload_kind !=
          swarm_ros_bridge::transport::PayloadKind::kServiceRequest ||
      !swarm_ros_bridge::transport::ValidateEnvelopeMetadata(
          request, config_.service_type, getServiceMd5(config_.service_type),
          &metadata_error)) {
    if (error != nullptr) {
      *error = metadata_error.empty() ? "invalid service request payload kind"
                                      : metadata_error;
    }
    return false;
  }
  const auto allowed_client = config_.client_hostnames.find(request.source_host);
  if (allowed_client == config_.client_hostnames.end() || !allowed_client->second) {
    if (error != nullptr) {
      *error = "service client is not allowed by bridge configuration";
    }
    return false;
  }

  try {
    boost::shared_array<std::uint8_t> request_data(
        new std::uint8_t[request.payload.size()]);
    if (!request.payload.empty()) {
      std::memcpy(request_data.get(), request.payload.data(), request.payload.size());
    }
    ros::SerializedMessage request_message(request_data, request.payload.size());
    topic_tools::ShapeShifter request_shifter;
    topic_tools::ShapeShifter response_shifter;
    ros::serialization::deserializeMessage(request_message, request_shifter);

    ros::ServiceClientOptions options(config_.service_name, "*", false,
                                      ros::M_string());
    ros::ServiceClient client = nh_->serviceClient(options);
    if (!client.isValid()) {
      if (error != nullptr) {
        *error = "failed to create local ROS service client";
      }
      return false;
    }
    const bool call_succeeded =
        client.call(request_shifter, response_shifter, "*");
    ros::SerializedMessage serialized =
        ros::serialization::serializeServiceResponse(call_succeeded,
                                                     response_shifter);

    response->payload_kind =
        swarm_ros_bridge::transport::PayloadKind::kServiceResponse;
    response->sequence = request.sequence;
    response->source_time_ns =
        static_cast<std::int64_t>(ros::Time::now().toNSec());
    response->source_host = config_.my_hostname;
    response->ros_type = config_.service_type;
    response->ros_md5 = getServiceMd5(config_.service_type);
    response->payload.clear();
    if (serialized.num_bytes > 0U) {
      response->payload.assign(serialized.buf.get(),
                               serialized.buf.get() + serialized.num_bytes);
    }
    return true;
  } catch (const std::exception& exception) {
    if (error != nullptr) {
      *error = exception.what();
    }
    return false;
  }
}

std::string ServiceFactory::getServiceMd5(const std::string& type_in) {
#define X(type, classname)              \
  if (type_in == type) {                \
    return getServiceMsgMd5<classname>(type_in); \
  }
  SRVS_MACRO
#undef X
  return {};
}

bool ServiceFactory::typeExists(const std::string& type_in) {
#define X(type, classname) \
  if (type_in == type) {   \
    return true;           \
  }
  SRVS_MACRO
#undef X
  return false;
}

template <typename T>
std::string ServiceFactory::getServiceMsgMd5(const std::string& type_in) {
  std::string error_message;
  if (!ros::names::validate(type_in, error_message)) {
    ROS_ERROR_STREAM("[SrvFactory] invalid service type name: " << error_message);
    return {};
  }
  T service_message;
  return ros::service_traits::md5sum(service_message);
}

void ServiceFactory::stopServerThread() { queryable_.reset(); }

void ServiceFactory::stopClientThread() { server_handle_client_msg_.shutdown(); }

bool ServiceCallbackHelper::call(ros::ServiceCallbackHelperCallParams& params) {
  return client_->call(name_raw_, params);
}
