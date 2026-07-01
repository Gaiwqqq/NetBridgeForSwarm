#include "config/bridge_config.hpp"

namespace swarm_ros_bridge {
namespace config {

BridgeConfig MakeDefaultConfig() {
  BridgeConfig config;
  config.hostname = "groundStation0";
  return config;
}

}  // namespace config
}  // namespace swarm_ros_bridge
