#ifndef SRC_SERVICE_FACTORY_H
#define SRC_SERVICE_FACTORY_H

#include "msgs_macro.hpp"
#include "transport/bridge_transport.hpp"

#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <string>

class ServiceFactory;

struct ServiceConfig {
  std::string service_name;
  std::string service_prefix_name;
  std::string service_type;
  std::string server_hostname;
  std::string my_hostname;
  std::map<std::string, bool> client_hostnames;
  std::uint64_t timeout_ms{1000};
  bool if_prefix{false};
};

class ServiceCallbackHelper : public ros::ServiceCallbackHelper {
 public:
  ServiceCallbackHelper(std::string service_name,
                        std::string service_name_raw,
                        ServiceFactory* client)
      : name_(std::move(service_name)),
        name_raw_(std::move(service_name_raw)),
        client_(client) {}
  bool call(ros::ServiceCallbackHelperCallParams& params) override;

 private:
  std::string name_;
  std::string name_raw_;
  ServiceFactory* client_;
};

class ServiceFactory {
 public:
  enum ClientOrServer { CLIENT, SERVER };
  using Ptr = std::shared_ptr<ServiceFactory>;

  ServiceFactory(
      ClientOrServer client_or_server,
      std::shared_ptr<ros::NodeHandle> nh,
      const ServiceConfig& config,
      std::shared_ptr<swarm_ros_bridge::transport::BridgeTransport> transport);
  ~ServiceFactory();

  bool call(const std::string& service_name,
            ros::ServiceCallbackHelperCallParams& params) const;
  void stopServerThread();
  void stopClientThread();
  std::uint64_t timeoutCount() const { return timeout_count_.load(); }

 private:
  std::shared_ptr<ros::NodeHandle> nh_;
  ClientOrServer client_or_server_;
  ServiceConfig config_;
  std::shared_ptr<swarm_ros_bridge::transport::BridgeTransport> transport_;
  ros::ServiceServer server_handle_client_msg_;
  std::shared_ptr<swarm_ros_bridge::transport::TransportQueryable> queryable_;
  mutable std::atomic<std::uint64_t> sequence_{0};
  mutable std::atomic<std::uint64_t> timeout_count_{0};

  static bool typeExists(const std::string& type_in);
  static std::string getServiceMd5(const std::string& type_in);
  template <typename T>
  static std::string getServiceMsgMd5(const std::string& type_in);

  bool handleRemoteRequest(
      const swarm_ros_bridge::transport::TransportEnvelope& request,
      swarm_ros_bridge::transport::TransportEnvelope* response,
      std::string* error) const;
};

#endif  // SRC_SERVICE_FACTORY_H
