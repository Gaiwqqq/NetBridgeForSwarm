#include "config/bridge_config.hpp"
#include "config/config_loader.hpp"
#include "diagnostics/diagnostics_cache.hpp"
#include "swarm_ros_bridge/NetworkArray.h"
#include "tui/app.hpp"
#include "tui/log_store.hpp"

#include <ros/ros.h>
#include <ros/spinner.h>
#include <rosgraph_msgs/Log.h>

#include <iomanip>
#include <cstdint>
#include <ctime>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

namespace {

std::string LevelToString(uint8_t level) {
  switch (level) {
    case rosgraph_msgs::Log::DEBUG:
      return "DEBUG";
    case rosgraph_msgs::Log::INFO:
      return "INFO";
    case rosgraph_msgs::Log::WARN:
      return "WARN";
    case rosgraph_msgs::Log::ERROR:
      return "ERROR";
    case rosgraph_msgs::Log::FATAL:
      return "FATAL";
    default:
      return "DEBUG";
  }
}

std::string StampToString(const rosgraph_msgs::Log& message) {
  const ros::Time stamp = message.header.stamp.isZero()
                              ? ros::Time::now()
                              : message.header.stamp;
  const std::time_t seconds = static_cast<std::time_t>(stamp.sec);
  std::tm local{};
  localtime_r(&seconds, &local);
  std::ostringstream stream;
  stream << std::setfill('0') << std::setw(2) << local.tm_hour << ":"
         << std::setw(2) << local.tm_min << ":" << std::setw(2)
         << local.tm_sec;
  return stream.str();
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "swarm_bridge_tui");
  ros::NodeHandle nh("~");
  auto config = swarm_ros_bridge::config::MakeDefaultConfig();
  swarm_ros_bridge::config::ConfigLoader::LoadFromRosParams(nh, &config);
  auto diagnostics_cache =
      std::make_shared<swarm_ros_bridge::diagnostics::DiagnosticsCache>();
  auto log_store = std::make_shared<swarm_ros_bridge::tui::LogStore>();

  ros::NodeHandle public_nh;
  auto diagnostics_callback =
      [diagnostics_cache](const swarm_ros_bridge::NetworkArray::ConstPtr& message) {
        diagnostics_cache->Update(*message);
      };
  ros::Subscriber diagnostics_sub =
      public_nh.subscribe<swarm_ros_bridge::NetworkArray>(
          "/swarm_bridge/diagnostics", 10, diagnostics_callback);

  auto rosout_callback = [log_store](const rosgraph_msgs::Log::ConstPtr& message) {
    swarm_ros_bridge::tui::LogRecord record;
    record.stamp = StampToString(*message);
    record.level = LevelToString(message->level);
    record.node = message->name;
    record.message = message->msg;
    log_store->Append(std::move(record));
  };
  ros::Subscriber rosout_sub = public_nh.subscribe<rosgraph_msgs::Log>(
      "/rosout", 100, rosout_callback);

  ros::AsyncSpinner spinner(2);
  spinner.start();
  swarm_ros_bridge::tui::App app(config, diagnostics_cache, log_store);
  return app.Run();
}
