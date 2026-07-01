#ifndef SWARM_ROS_BRIDGE_TRANSPORT_POINTCLOUD_CODEC_HPP_
#define SWARM_ROS_BRIDGE_TRANSPORT_POINTCLOUD_CODEC_HPP_

#include <sensor_msgs/PointCloud2.h>

#include <cstdint>
#include <string>
#include <vector>

namespace swarm_ros_bridge {
namespace transport {

struct EncodedPointCloud {
  std::string codec;
  ros::Time source_stamp;
  ros::Time receive_stamp;
  std::string frame_id;
  std::vector<std::uint8_t> payload;
};

class PointCloudCodec {
 public:
  virtual ~PointCloudCodec() = default;

  virtual std::string Name() const = 0;
  virtual bool Encode(const sensor_msgs::PointCloud2& message,
                      EncodedPointCloud* output) const = 0;
  virtual bool Decode(const EncodedPointCloud& input,
                      sensor_msgs::PointCloud2* message) const = 0;
};

}  // namespace transport
}  // namespace swarm_ros_bridge

#endif  // SWARM_ROS_BRIDGE_TRANSPORT_POINTCLOUD_CODEC_HPP_
