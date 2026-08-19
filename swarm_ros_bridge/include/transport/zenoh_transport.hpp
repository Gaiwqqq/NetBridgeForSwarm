#ifndef SWARM_ROS_BRIDGE_TRANSPORT_ZENOH_TRANSPORT_HPP_
#define SWARM_ROS_BRIDGE_TRANSPORT_ZENOH_TRANSPORT_HPP_

#include "transport/bridge_transport.hpp"

#include <memory>
#include <string>

namespace swarm_ros_bridge {
namespace transport {

class ZenohTransport final : public BridgeTransport {
 public:
  ZenohTransport(const ZenohTransportConfig& config, std::string hostname);
  ~ZenohTransport() override;

  std::shared_ptr<TransportPublisher> DeclarePublisher(
      const std::string& key, QosClass qos_class) override;
  std::shared_ptr<TransportSubscription> Subscribe(
      const std::string& key_expression, EnvelopeCallback callback) override;
  std::shared_ptr<TransportQueryable> DeclareQueryable(
      const std::string& key, QueryHandler handler) override;
  bool Query(const std::string& key,
             const TransportEnvelope& request,
             std::uint64_t timeout_ms,
             TransportEnvelope* response,
             std::string* error = nullptr) override;
  TransportStats GetStats() const override;
  std::vector<NodePresence> GetNodePresence() const override;
  void Close() override;

 private:
  class Impl;
  std::shared_ptr<Impl> impl_;
};

}  // namespace transport
}  // namespace swarm_ros_bridge

#endif  // SWARM_ROS_BRIDGE_TRANSPORT_ZENOH_TRANSPORT_HPP_
