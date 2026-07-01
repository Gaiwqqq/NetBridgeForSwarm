#ifndef SWARM_ROS_BRIDGE_TRANSPORT_DRACO_POINTCLOUD_CODEC_HPP_
#define SWARM_ROS_BRIDGE_TRANSPORT_DRACO_POINTCLOUD_CODEC_HPP_

#include "transport/pointcloud_codec.hpp"

namespace swarm_ros_bridge {
namespace transport {

class DracoPointCloudCodec : public PointCloudCodec {
 public:
  std::string Name() const override;
  bool Encode(const sensor_msgs::PointCloud2& message,
              EncodedPointCloud* output) const override;
  bool Decode(const EncodedPointCloud& input,
              sensor_msgs::PointCloud2* message) const override;
};

}  // namespace transport
}  // namespace swarm_ros_bridge

#endif  // SWARM_ROS_BRIDGE_TRANSPORT_DRACO_POINTCLOUD_CODEC_HPP_
