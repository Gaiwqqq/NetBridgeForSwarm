#include "transport/bridge_transport.hpp"
#include "transport/zenoh_transport.hpp"

#include <cassert>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace transport = swarm_ros_bridge::transport;

namespace {

bool HasPresence(const std::shared_ptr<transport::ZenohTransport>& bridge,
                 const std::string& hostname,
                 bool online) {
  for (const auto& node : bridge->GetNodePresence()) {
    if (node.hostname == hostname && node.online == online) {
      return true;
    }
  }
  return false;
}

bool WaitForPresence(const std::shared_ptr<transport::ZenohTransport>& bridge,
                     const std::string& hostname,
                     bool online,
                     std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (HasPresence(bridge, hostname, online)) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return HasPresence(bridge, hostname, online);
}

}  // namespace

int main() {
  transport::ZenohTransportConfig server_config;
  server_config.multicast_scouting = false;
  server_config.gossip_scouting = false;
  server_config.listen_endpoints = {"tcp/127.0.0.1:17447"};

  transport::ZenohTransportConfig client_config;
  client_config.multicast_scouting = false;
  client_config.gossip_scouting = false;
  client_config.listen_endpoints = {"tcp/127.0.0.1:0"};
  client_config.connect_endpoints = {"tcp/127.0.0.1:17447"};

  auto server = std::make_shared<transport::ZenohTransport>(server_config, "server");
  auto client = std::make_shared<transport::ZenohTransport>(client_config, "client");

  assert(WaitForPresence(server, "server", true, std::chrono::seconds(1)));
  assert(WaitForPresence(client, "client", true, std::chrono::seconds(1)));
  assert(WaitForPresence(server, "client", true, std::chrono::seconds(3)));
  assert(WaitForPresence(client, "server", true, std::chrono::seconds(3)));

  std::mutex mutex;
  std::condition_variable condition;
  bool received = false;
  transport::TransportEnvelope received_envelope;
  auto subscription = server->Subscribe(
      transport::TopicFanoutKey("client", "/smoke"),
      [&](transport::TransportEnvelope&& envelope) {
        std::lock_guard<std::mutex> lock(mutex);
        received_envelope = std::move(envelope);
        received = true;
        condition.notify_all();
      });
  auto publisher = client->DeclarePublisher(
      transport::TopicFanoutKey("client", "/smoke"),
      transport::QosClass::kCommand);

  auto queryable = server->DeclareQueryable(
      transport::ServiceKey("server", "/echo"),
      [](const transport::TransportEnvelope& request,
         transport::TransportEnvelope* response,
         std::string*) {
        *response = request;
        response->payload_kind = transport::PayloadKind::kServiceResponse;
        response->source_host = "server";
        return true;
      });

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  transport::TransportEnvelope publication;
  publication.sequence = 7;
  publication.source_host = "client";
  publication.ros_type = "std_msgs/String";
  publication.ros_md5 = "992ce8a1687cec8c8bd883ec73ca41d1";
  publication.payload = {'z', 'e', 'n', 'o', 'h'};
  std::string error;
  assert(publisher->Publish(publication, &error));
  {
    std::unique_lock<std::mutex> lock(mutex);
    assert(condition.wait_for(lock, std::chrono::seconds(3), [&]() {
      return received;
    }));
  }
  assert(received_envelope.sequence == publication.sequence);
  assert(received_envelope.payload == publication.payload);

  transport::TransportEnvelope request;
  request.payload_kind = transport::PayloadKind::kServiceRequest;
  request.sequence = 9;
  request.source_host = "client";
  request.ros_type = "test/Echo";
  request.ros_md5 = "test-md5";
  request.payload = {1, 2, 3, 4};
  transport::TransportEnvelope response;
  assert(client->Query(transport::ServiceKey("server", "/echo"), request,
                       1000, &response, &error));
  assert(response.payload_kind == transport::PayloadKind::kServiceResponse);
  assert(response.source_host == "server");
  assert(response.sequence == request.sequence);
  assert(response.payload == request.payload);

  auto failing_queryable = server->DeclareQueryable(
      transport::ServiceKey("server", "/failure"),
      [](const transport::TransportEnvelope&, transport::TransportEnvelope*,
         std::string* handler_error) {
        *handler_error = "intentional service error";
        return false;
      });
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  error.clear();
  assert(!client->Query(transport::ServiceKey("server", "/failure"), request,
                        1000, &response, &error));
  assert(error.find("intentional service error") != std::string::npos);

  auto slow_queryable = server->DeclareQueryable(
      transport::ServiceKey("server", "/slow"),
      [](const transport::TransportEnvelope& slow_request,
         transport::TransportEnvelope* slow_response, std::string*) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        *slow_response = slow_request;
        slow_response->payload_kind = transport::PayloadKind::kServiceResponse;
        return true;
      });
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  error.clear();
  assert(!client->Query(transport::ServiceKey("server", "/slow"), request, 50,
                        &response, &error));
  if (error.find("timed out") == std::string::npos) {
    std::cerr << "unexpected slow-query error: " << error << '\n';
    return 2;
  }
  assert(client->GetStats().service_timeouts >= 1U);

  slow_queryable.reset();
  failing_queryable.reset();
  queryable.reset();
  publisher.reset();
  subscription.reset();
  client->Close();
  assert(WaitForPresence(server, "client", false, std::chrono::seconds(3)));
  server->Close();
  assert(!client->GetStats().session_open);
  assert(!server->GetStats().session_open);
  std::cout << "Zenoh pub/sub, query/reply, and liveliness smoke test passed\n";
  return 0;
}
