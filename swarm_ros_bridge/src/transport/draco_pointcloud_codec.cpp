#include "transport/draco_pointcloud_codec.hpp"

#include <draco/attributes/geometry_attribute.h>
#include <draco/attributes/point_attribute.h>
#include <draco/compression/decode.h>
#include <draco/compression/encode.h>
#include <draco/core/decoder_buffer.h>
#include <draco/core/encoder_buffer.h>
#include <draco/point_cloud/point_cloud.h>
#include <draco/point_cloud/point_cloud_builder.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <array>

namespace swarm_ros_bridge {
namespace transport {

std::string DracoPointCloudCodec::Name() const {
  return "draco";
}

bool DracoPointCloudCodec::Encode(const sensor_msgs::PointCloud2& message,
                                  EncodedPointCloud* output) const {
  if (output == nullptr) {
    return false;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(message, cloud);
  if (cloud.empty()) {
    output->codec = Name();
    output->source_stamp = message.header.stamp;
    output->receive_stamp = ros::Time(0);
    output->frame_id = message.header.frame_id;
    output->payload.clear();
    return true;
  }

  draco::PointCloudBuilder builder;
  builder.Start(static_cast<int>(cloud.size()));
  const int position_att_id = builder.AddAttribute(
      draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32, false);
  if (position_att_id < 0) {
    return false;
  }

  std::array<float, 3> point{};
  for (draco::PointIndex i(0); i < static_cast<uint32_t>(cloud.size()); ++i) {
    point = {cloud[static_cast<std::size_t>(i.value())].x,
             cloud[static_cast<std::size_t>(i.value())].y,
             cloud[static_cast<std::size_t>(i.value())].z};
    builder.SetAttributeValueForPoint(position_att_id, i, point.data());
  }

  std::unique_ptr<draco::PointCloud> draco_cloud = builder.Finalize(false);
  if (draco_cloud == nullptr) {
    return false;
  }

  draco::Encoder encoder;
  encoder.SetSpeedOptions(5, 5);
  encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, 14);
  draco::EncoderBuffer buffer;
  const auto status = encoder.EncodePointCloudToBuffer(*draco_cloud, &buffer);
  if (!status.ok()) {
    return false;
  }

  output->codec = Name();
  output->source_stamp = message.header.stamp;
  output->receive_stamp = ros::Time(0);
  output->frame_id = message.header.frame_id;
  output->payload.assign(buffer.data(), buffer.data() + buffer.size());
  return true;
}

bool DracoPointCloudCodec::Decode(const EncodedPointCloud& input,
                                  sensor_msgs::PointCloud2* message) const {
  if (message == nullptr) {
    return false;
  }

  draco::DecoderBuffer buffer;
  if (!input.payload.empty()) {
    buffer.Init(reinterpret_cast<const char*>(input.payload.data()), input.payload.size());
  }

  if (input.payload.empty()) {
    pcl::PointCloud<pcl::PointXYZ> empty_cloud;
    pcl::toROSMsg(empty_cloud, *message);
    message->header.frame_id = input.frame_id;
    message->header.stamp = input.receive_stamp.isZero() ? ros::Time::now() : input.receive_stamp;
    return true;
  }

  draco::Decoder decoder;
  auto decoded = decoder.DecodePointCloudFromBuffer(&buffer);
  if (!decoded.ok()) {
    return false;
  }

  std::unique_ptr<draco::PointCloud> point_cloud = std::move(decoded).value();
  if (point_cloud == nullptr) {
    return false;
  }

  const draco::PointAttribute* position =
      point_cloud->GetNamedAttribute(draco::GeometryAttribute::POSITION);
  if (position == nullptr) {
    return false;
  }

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl_cloud.reserve(point_cloud->num_points());
  std::array<float, 3> point{};
  for (draco::PointIndex i(0); i < point_cloud->num_points(); ++i) {
    position->GetMappedValue(i, point.data());
    pcl_cloud.emplace_back(point[0], point[1], point[2]);
  }

  pcl_cloud.width = static_cast<std::uint32_t>(pcl_cloud.size());
  pcl_cloud.height = 1;
  pcl_cloud.is_dense = false;
  pcl::toROSMsg(pcl_cloud, *message);
  message->header.frame_id = input.frame_id;
  message->header.stamp = input.receive_stamp.isZero() ? ros::Time::now() : input.receive_stamp;
  return true;
}

}  // namespace transport
}  // namespace swarm_ros_bridge
