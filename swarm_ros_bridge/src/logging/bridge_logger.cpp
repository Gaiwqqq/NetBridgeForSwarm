#include "logging/bridge_logger.hpp"

namespace swarm_ros_bridge {
namespace logging {

namespace {

std::string FormatLine(Level level,
                       const std::string& module,
                       const std::string& topic,
                       const std::string& message) {
  std::ostringstream oss;
  oss << BridgeLogger::Emoji(level) << " "
      << "[" << BridgeLogger::LevelName(level) << "]"
      << "[" << module << "]";
  if (!topic.empty()) {
    oss << "[" << topic << "]";
  }
  oss << " " << message;
  return oss.str();
}

}  // namespace

void BridgeLogger::Log(Level level,
                       const std::string& module,
                       const std::string& topic,
                       const std::string& message) {
  const std::string line = FormatLine(level, module, topic, message);
  switch (level) {
    case Level::kDebug:
      ROS_DEBUG_STREAM(line);
      break;
    case Level::kInfo:
      ROS_INFO_STREAM(line);
      break;
    case Level::kWarn:
      ROS_WARN_STREAM(line);
      break;
    case Level::kError:
      ROS_ERROR_STREAM(line);
      break;
  }
}

const char* BridgeLogger::LevelName(Level level) {
  switch (level) {
    case Level::kDebug:
      return "DEBUG";
    case Level::kInfo:
      return "INFO";
    case Level::kWarn:
      return "WARN";
    case Level::kError:
      return "ERROR";
  }
  return "INFO";
}

const char* BridgeLogger::Emoji(Level level) {
  switch (level) {
    case Level::kDebug:
      return "[..]";
    case Level::kInfo:
      return "[OK]";
    case Level::kWarn:
      return "[!]";
    case Level::kError:
      return "[X]";
  }
  return "[OK]";
}

}  // namespace logging
}  // namespace swarm_ros_bridge
