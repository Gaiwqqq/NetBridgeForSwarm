#include "config/bridge_config.hpp"
#include "config/config_loader.hpp"
#include "diagnostics/diagnostics_cache.hpp"
#include "swarm_ros_bridge/NetworkArray.h"
#include "tui/app.hpp"

#include <ros/ros.h>
#include <ros/spinner.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "swarm_bridge_tui");
  ros::NodeHandle nh("~");
  auto config = swarm_ros_bridge::config::MakeDefaultConfig();
  swarm_ros_bridge::config::ConfigLoader::LoadFromRosParams(nh, &config);
  auto diagnostics_cache =
      std::make_shared<swarm_ros_bridge::diagnostics::DiagnosticsCache>();
  ros::NodeHandle public_nh;
  auto callback = [diagnostics_cache](const swarm_ros_bridge::NetworkArray::ConstPtr& message) {
    diagnostics_cache->Update(*message);
  };
  ros::Subscriber diagnostics_sub =
      public_nh.subscribe<swarm_ros_bridge::NetworkArray>(
          "/swarm_bridge/diagnostics", 10, callback);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  swarm_ros_bridge::tui::App app(config, diagnostics_cache);
  return app.Run();
}
