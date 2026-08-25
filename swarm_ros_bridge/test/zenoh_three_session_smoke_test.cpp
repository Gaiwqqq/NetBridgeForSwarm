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
#include <vector>

namespace transport = swarm_ros_bridge::transport;

namespace {

bool WaitForLink(const std::shared_ptr<transport::ZenohTransport>& bridge,
                 std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (bridge->GetStats().link_connected) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return bridge->GetStats().link_connected;
}

transport::ZenohTransportConfig MakeConfig(
    const std::string& protocol,
    const std::string& listen_endpoint,
    const std::string& connect_endpoint,
    bool enable_liveliness) {
  transport::ZenohTransportConfig config;
  config.multicast_scouting = false;
  config.gossip_scouting = false;
  config.enable_liveliness = enable_liveliness;
  config.listen_endpoints = {listen_endpoint};
  if (!connect_endpoint.empty()) {
    config.connect_endpoints = {connect_endpoint};
  }
  config.allowed_link_protocols = {protocol};
  return config;
}

}  // namespace

int main() {
  const std::string udp_options = "?rel=1;mixed_rel=1;multistream=1";
  auto control_server = std::make_shared<transport::ZenohTransport>(
      MakeConfig("tcp", "tcp/127.0.0.1:18447", "", true),
      "control-server");
  auto control_client = std::make_shared<transport::ZenohTransport>(
      MakeConfig("tcp", "tcp/127.0.0.1:18446",
                 "tcp/127.0.0.1:18447", true),
      "control-client");
  auto image_server = std::make_shared<transport::ZenohTransport>(
      MakeConfig("udp", "udp/127.0.0.1:18448" + udp_options, "", false),
      "image-server");
  auto image_client = std::make_shared<transport::ZenohTransport>(
      MakeConfig("udp", "udp/127.0.0.1:18449" + udp_options,
                 "udp/127.0.0.1:18448" + udp_options, false),
      "image-client");
  auto cloud_server = std::make_shared<transport::ZenohTransport>(
      MakeConfig("tcp", "tcp/127.0.0.1:18450", "", false),
      "cloud-server");
  auto cloud_client = std::make_shared<transport::ZenohTransport>(
      MakeConfig("tcp", "tcp/127.0.0.1:18451",
                 "tcp/127.0.0.1:18450", false),
      "cloud-client");

  assert(WaitForLink(control_server, std::chrono::seconds(3)));
  assert(WaitForLink(control_client, std::chrono::seconds(3)));
  assert(WaitForLink(image_server, std::chrono::seconds(3)));
  assert(WaitForLink(image_client, std::chrono::seconds(3)));
  assert(WaitForLink(cloud_server, std::chrono::seconds(3)));
  assert(WaitForLink(cloud_client, std::chrono::seconds(3)));
  assert(image_server->GetNodePresence().empty());
  assert(image_client->GetNodePresence().empty());
  assert(cloud_server->GetNodePresence().empty());
  assert(cloud_client->GetNodePresence().empty());

  const std::string key = transport::TopicFanoutKey("triple-client", "/route-check");
  std::mutex mutex;
  std::condition_variable condition;
  int control_messages = 0;
  int image_messages = 0;
  int cloud_messages = 0;
  std::vector<std::uint8_t> control_payload;
  std::vector<std::uint8_t> image_payload;
  std::vector<std::uint8_t> cloud_payload;

  auto control_subscription = control_server->Subscribe(
      key, [&](transport::TransportEnvelope&& envelope) {
        std::lock_guard<std::mutex> lock(mutex);
        ++control_messages;
        control_payload = std::move(envelope.payload);
        condition.notify_all();
      });
  auto image_subscription = image_server->Subscribe(
      key, [&](transport::TransportEnvelope&& envelope) {
        std::lock_guard<std::mutex> lock(mutex);
        ++image_messages;
        image_payload = std::move(envelope.payload);
        condition.notify_all();
      });
  auto cloud_subscription = cloud_server->Subscribe(
      key, [&](transport::TransportEnvelope&& envelope) {
        std::lock_guard<std::mutex> lock(mutex);
        ++cloud_messages;
        cloud_payload = std::move(envelope.payload);
        condition.notify_all();
      });
  auto control_publisher =
      control_client->DeclarePublisher(key, transport::QosClass::kCommand);
  auto image_publisher =
      image_client->DeclarePublisher(key, transport::QosClass::kBulk);
  auto cloud_publisher =
      cloud_client->DeclarePublisher(key, transport::QosClass::kBulk);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  transport::TransportEnvelope control;
  control.sequence = 1;
  control.source_host = "triple-client";
  control.ros_type = "std_msgs/String";
  control.ros_md5 = "992ce8a1687cec8c8bd883ec73ca41d1";
  control.payload = {'t', 'c', 'p'};

  transport::TransportEnvelope image;
  image.payload_kind = transport::PayloadKind::kJpeg;
  image.sequence = 2;
  image.source_host = "triple-client";
  image.ros_type = "sensor_msgs/Image";
  image.ros_md5 = "060021388200f6f0f447d0fcd9c64743";
  image.payload.resize(100U * 1024U);
  for (std::size_t index = 0; index < image.payload.size(); ++index) {
    image.payload[index] = static_cast<std::uint8_t>(index % 251U);
  }
  const auto expected_image_payload = image.payload;

  transport::TransportEnvelope cloud;
  cloud.payload_kind = transport::PayloadKind::kPointCloudCompressed;
  cloud.sequence = 3;
  cloud.source_host = "triple-client";
  cloud.ros_type = "sensor_msgs/PointCloud2";
  cloud.ros_md5 = "1158d486dd51d683ce2f1be655c3c181";
  cloud.payload.resize(16U * 1024U, 0x5aU);
  const auto expected_cloud_payload = cloud.payload;

  std::string error;
  assert(control_publisher->Publish(control, &error));
  assert(image_publisher->Publish(image, &error));
  assert(cloud_publisher->Publish(cloud, &error));
  {
    std::unique_lock<std::mutex> lock(mutex);
    assert(condition.wait_for(lock, std::chrono::seconds(5), [&]() {
      return control_messages >= 1 && image_messages >= 1 &&
             cloud_messages >= 1;
    }));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  {
    std::lock_guard<std::mutex> lock(mutex);
    assert(control_messages == 1);
    assert(image_messages == 1);
    assert(cloud_messages == 1);
    assert(control_payload == control.payload);
    assert(image_payload == expected_image_payload);
    assert(cloud_payload == expected_cloud_payload);
  }

  cloud_publisher.reset();
  image_publisher.reset();
  control_publisher.reset();
  cloud_subscription.reset();
  image_subscription.reset();
  control_subscription.reset();
  cloud_client->Close();
  cloud_server->Close();
  image_client->Close();
  image_server->Close();
  control_client->Close();
  control_server->Close();
  std::cout << "Zenoh TCP control + UDP image + TCP cloud three-session smoke test passed\n";
  return 0;
}
