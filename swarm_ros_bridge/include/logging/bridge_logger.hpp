#ifndef SWARM_ROS_BRIDGE_LOGGING_BRIDGE_LOGGER_HPP_
#define SWARM_ROS_BRIDGE_LOGGING_BRIDGE_LOGGER_HPP_

#include <ros/ros.h>
#include <sstream>
#include <string>

namespace swarm_ros_bridge {
namespace logging {

enum class Level {
  kDebug,
  kInfo,
  kWarn,
  kError,
};

class BridgeLogger {
 public:
  static void Log(Level level,
                  const std::string& module,
                  const std::string& topic,
                  const std::string& message);

  static const char* LevelName(Level level);
  static const char* Emoji(Level level);
};

}  // namespace logging
}  // namespace swarm_ros_bridge

#define BRIDGE_LOG_STREAM(level, module, topic, expr)                                \
  do {                                                                               \
    std::ostringstream _bridge_log_stream;                                           \
    _bridge_log_stream << expr;                                                      \
    ::swarm_ros_bridge::logging::BridgeLogger::Log(level, module, topic,             \
                                                   _bridge_log_stream.str());         \
  } while (false)

#define BRIDGE_LOG_DEBUG(module, topic, expr)                                         \
  BRIDGE_LOG_STREAM(::swarm_ros_bridge::logging::Level::kDebug, module, topic, expr)
#define BRIDGE_LOG_INFO(module, topic, expr)                                          \
  BRIDGE_LOG_STREAM(::swarm_ros_bridge::logging::Level::kInfo, module, topic, expr)
#define BRIDGE_LOG_WARN(module, topic, expr)                                          \
  BRIDGE_LOG_STREAM(::swarm_ros_bridge::logging::Level::kWarn, module, topic, expr)
#define BRIDGE_LOG_ERROR(module, topic, expr)                                         \
  BRIDGE_LOG_STREAM(::swarm_ros_bridge::logging::Level::kError, module, topic, expr)

#endif  // SWARM_ROS_BRIDGE_LOGGING_BRIDGE_LOGGER_HPP_
