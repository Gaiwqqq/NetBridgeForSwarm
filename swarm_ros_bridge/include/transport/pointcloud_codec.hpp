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
  std::uint8_t format_version{0};
  std::string point_type;
  std_msgs::Header original_header;
  std::uint32_t original_width{0};
  std::uint32_t original_height{0};
  std::vector<sensor_msgs::PointField> original_fields;
  bool original_is_bigendian{false};
  std::uint32_t original_point_step{0};
  std::uint32_t original_row_step{0};
  bool original_is_dense{false};
  ros::Time source_stamp;
  ros::Time receive_stamp;
  std::string frame_id;
  std::vector<std::uint8_t> payload;
  std::vector<std::uint8_t> sidecar_bytes;
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
