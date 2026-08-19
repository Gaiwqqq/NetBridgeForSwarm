#include "transport/draco_pointcloud_codec.hpp"

#include <sensor_msgs/PointField.h>

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

namespace {

sensor_msgs::PointField Field(const std::string& name,
                              std::uint32_t offset,
                              std::uint8_t datatype) {
  sensor_msgs::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1U;
  return field;
}

void WriteUint32(std::uint32_t value, bool bigendian, std::uint8_t* data) {
  if (bigendian) {
    data[0] = static_cast<std::uint8_t>(value >> 24U);
    data[1] = static_cast<std::uint8_t>(value >> 16U);
    data[2] = static_cast<std::uint8_t>(value >> 8U);
    data[3] = static_cast<std::uint8_t>(value);
  } else {
    data[0] = static_cast<std::uint8_t>(value);
    data[1] = static_cast<std::uint8_t>(value >> 8U);
    data[2] = static_cast<std::uint8_t>(value >> 16U);
    data[3] = static_cast<std::uint8_t>(value >> 24U);
  }
}

std::uint32_t ReadUint32(const std::uint8_t* data, bool bigendian) {
  if (bigendian) {
    return (static_cast<std::uint32_t>(data[0]) << 24U) |
           (static_cast<std::uint32_t>(data[1]) << 16U) |
           (static_cast<std::uint32_t>(data[2]) << 8U) |
           static_cast<std::uint32_t>(data[3]);
  }
  return static_cast<std::uint32_t>(data[0]) |
         (static_cast<std::uint32_t>(data[1]) << 8U) |
         (static_cast<std::uint32_t>(data[2]) << 16U) |
         (static_cast<std::uint32_t>(data[3]) << 24U);
}

void WriteFloat(float value, bool bigendian, std::uint8_t* data) {
  std::uint32_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  WriteUint32(bits, bigendian, data);
}

float ReadFloat(const std::uint8_t* data, bool bigendian) {
  const std::uint32_t bits = ReadUint32(data, bigendian);
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

std::size_t PointOffset(const sensor_msgs::PointCloud2& message,
                        std::uint32_t index) {
  const std::uint32_t row = index / message.width;
  const std::uint32_t column = index % message.width;
  return static_cast<std::size_t>(row) * message.row_step +
         static_cast<std::size_t>(column) * message.point_step;
}

void AssertMetadataEqual(const sensor_msgs::PointCloud2& expected,
                         const sensor_msgs::PointCloud2& actual) {
  assert(actual.header.seq == expected.header.seq);
  assert(actual.header.stamp == expected.header.stamp);
  assert(actual.header.frame_id == expected.header.frame_id);
  assert(actual.width == expected.width);
  assert(actual.height == expected.height);
  assert(actual.is_bigendian == expected.is_bigendian);
  assert(actual.point_step == expected.point_step);
  assert(actual.row_step == expected.row_step);
  assert(actual.is_dense == expected.is_dense);
  assert(actual.data.size() == expected.data.size());
  assert(actual.fields.size() == expected.fields.size());
  for (std::size_t i = 0; i < expected.fields.size(); ++i) {
    assert(actual.fields[i].name == expected.fields[i].name);
    assert(actual.fields[i].offset == expected.fields[i].offset);
    assert(actual.fields[i].datatype == expected.fields[i].datatype);
    assert(actual.fields[i].count == expected.fields[i].count);
  }
}

void AssertBytesOutsideFieldsEqual(
    const sensor_msgs::PointCloud2& expected,
    const sensor_msgs::PointCloud2& actual,
    const std::vector<std::pair<std::uint32_t, std::uint32_t>>& excluded) {
  for (std::uint32_t row = 0; row < expected.height; ++row) {
    const std::size_t row_offset = static_cast<std::size_t>(row) * expected.row_step;
    for (std::uint32_t column = 0; column < expected.width; ++column) {
      const std::size_t point_offset =
          row_offset + static_cast<std::size_t>(column) * expected.point_step;
      for (std::uint32_t byte = 0; byte < expected.point_step; ++byte) {
        bool skip = false;
        for (const auto& range : excluded) {
          skip = skip || (byte >= range.first && byte < range.second);
        }
        if (!skip) {
          assert(actual.data[point_offset + byte] ==
                 expected.data[point_offset + byte]);
        }
      }
    }
    for (std::size_t byte = expected.width * expected.point_step;
         byte < expected.row_step; ++byte) {
      assert(actual.data[row_offset + byte] == expected.data[row_offset + byte]);
    }
  }
}

void AssertPositionsNear(const sensor_msgs::PointCloud2& expected,
                         const sensor_msgs::PointCloud2& actual,
                         float tolerance) {
  const std::uint32_t count = expected.width * expected.height;
  for (std::uint32_t index = 0; index < count; ++index) {
    const std::size_t expected_offset = PointOffset(expected, index);
    const std::size_t actual_offset = PointOffset(actual, index);
    for (std::uint32_t field_offset : {0U, 4U, 8U}) {
      const float source = ReadFloat(expected.data.data() + expected_offset +
                                         field_offset,
                                     expected.is_bigendian);
      const float decoded = ReadFloat(actual.data.data() + actual_offset +
                                          field_offset,
                                      actual.is_bigendian);
      assert(std::fabs(source - decoded) <= tolerance);
    }
  }
}

sensor_msgs::PointCloud2 MakeOrganizedXyzWithIntensity() {
  sensor_msgs::PointCloud2 message;
  message.header.seq = 4242U;
  message.header.stamp = ros::Time(123U, 456U);
  message.header.frame_id = "organized_xyz";
  message.width = 3U;
  message.height = 2U;
  message.fields = {
      Field("x", 0U, sensor_msgs::PointField::FLOAT32),
      Field("y", 4U, sensor_msgs::PointField::FLOAT32),
      Field("z", 8U, sensor_msgs::PointField::FLOAT32),
      Field("intensity", 12U, sensor_msgs::PointField::FLOAT32),
  };
  message.is_bigendian = false;
  message.point_step = 16U;
  message.row_step = 52U;
  message.is_dense = true;
  message.data.resize(message.row_step * message.height);
  for (std::size_t i = 0; i < message.data.size(); ++i) {
    message.data[i] = static_cast<std::uint8_t>((i * 37U + 11U) & 0xffU);
  }
  for (std::uint32_t index = 0; index < message.width * message.height; ++index) {
    const std::size_t offset = PointOffset(message, index);
    WriteFloat(4.0F - 1.3F * index, false, message.data.data() + offset);
    WriteFloat(-2.0F + 0.7F * index, false, message.data.data() + offset + 4U);
    WriteFloat(0.5F + 1.1F * index, false, message.data.data() + offset + 8U);
    WriteFloat(100.0F + index, false, message.data.data() + offset + 12U);
  }
  return message;
}

sensor_msgs::PointCloud2 MakeOrganizedXyzRgbWithRing() {
  sensor_msgs::PointCloud2 message;
  message.header.seq = 777U;
  message.header.stamp = ros::Time(321U, 654U);
  message.header.frame_id = "organized_xyzrgb";
  message.width = 2U;
  message.height = 3U;
  message.fields = {
      Field("x", 0U, sensor_msgs::PointField::FLOAT32),
      Field("y", 4U, sensor_msgs::PointField::FLOAT32),
      Field("z", 8U, sensor_msgs::PointField::FLOAT32),
      Field("rgb", 12U, sensor_msgs::PointField::FLOAT32),
      Field("ring", 16U, sensor_msgs::PointField::UINT16),
  };
  message.is_bigendian = true;
  message.point_step = 20U;
  message.row_step = 44U;
  message.is_dense = false;
  message.data.resize(message.row_step * message.height);
  for (std::size_t i = 0; i < message.data.size(); ++i) {
    message.data[i] = static_cast<std::uint8_t>((i * 19U + 7U) & 0xffU);
  }
  for (std::uint32_t index = 0; index < message.width * message.height; ++index) {
    const std::size_t offset = PointOffset(message, index);
    WriteFloat(-5.0F + 1.7F * index, true, message.data.data() + offset);
    WriteFloat(3.0F - 0.9F * index, true, message.data.data() + offset + 4U);
    WriteFloat(8.0F + 0.2F * index, true, message.data.data() + offset + 8U);
    const std::uint32_t packed =
        ((0x40U + index) << 24U) | ((0x10U + index) << 16U) |
        ((0x20U + index) << 8U) | (0x30U + index);
    WriteUint32(packed, true, message.data.data() + offset + 12U);
    message.data[offset + 16U] = static_cast<std::uint8_t>(index + 20U);
    message.data[offset + 17U] = static_cast<std::uint8_t>(index + 40U);
  }
  return message;
}

void TestXyzAndAdditionalFieldRoundTrip() {
  const sensor_msgs::PointCloud2 source = MakeOrganizedXyzWithIntensity();
  swarm_ros_bridge::transport::DracoPointCloudCodec codec;
  swarm_ros_bridge::transport::EncodedPointCloud encoded;
  assert(codec.Encode(source, &encoded));
  assert(encoded.format_version == 2U);
  assert(encoded.point_type == "xyz");
  assert(encoded.sidecar_bytes.size() == 32U);
  assert(!encoded.payload.empty());

  sensor_msgs::PointCloud2 decoded;
  assert(codec.Decode(encoded, &decoded));
  AssertMetadataEqual(source, decoded);
  AssertPositionsNear(source, decoded, 0.002F);
  AssertBytesOutsideFieldsEqual(source, decoded,
                                {{0U, 4U}, {4U, 8U}, {8U, 12U}});

  auto bad_sidecar = encoded;
  bad_sidecar.sidecar_bytes.pop_back();
  assert(!codec.Decode(bad_sidecar, &decoded));
  auto bad_version = encoded;
  bad_version.format_version = 1U;
  assert(!codec.Decode(bad_version, &decoded));
}

void TestXyzRgbAndAdditionalFieldRoundTrip() {
  const sensor_msgs::PointCloud2 source = MakeOrganizedXyzRgbWithRing();
  swarm_ros_bridge::transport::DracoPointCloudCodec codec;
  swarm_ros_bridge::transport::EncodedPointCloud encoded;
  assert(codec.Encode(source, &encoded));
  assert(encoded.point_type == "xyzrgb");
  assert(encoded.sidecar_bytes.size() == 36U);

  sensor_msgs::PointCloud2 decoded;
  assert(codec.Decode(encoded, &decoded));
  AssertMetadataEqual(source, decoded);
  AssertPositionsNear(source, decoded, 0.002F);
  AssertBytesOutsideFieldsEqual(
      source, decoded, {{0U, 4U}, {4U, 8U}, {8U, 12U}, {12U, 16U}});
  for (std::uint32_t index = 0; index < source.width * source.height; ++index) {
    const std::size_t source_offset = PointOffset(source, index);
    const std::size_t decoded_offset = PointOffset(decoded, index);
    assert(ReadUint32(source.data.data() + source_offset + 12U, true) ==
           ReadUint32(decoded.data.data() + decoded_offset + 12U, true));
  }
}

void TestMalformedCloudsAreRejected() {
  swarm_ros_bridge::transport::DracoPointCloudCodec codec;
  swarm_ros_bridge::transport::EncodedPointCloud encoded;

  auto truncated = MakeOrganizedXyzWithIntensity();
  truncated.data.pop_back();
  assert(!codec.Encode(truncated, &encoded));

  auto missing_z = MakeOrganizedXyzWithIntensity();
  missing_z.fields.erase(missing_z.fields.begin() + 2);
  assert(!codec.Encode(missing_z, &encoded));
}

}  // namespace

int main() {
  TestXyzAndAdditionalFieldRoundTrip();
  TestXyzRgbAndAdditionalFieldRoundTrip();
  TestMalformedCloudsAreRejected();
  std::cout << "draco point cloud codec tests passed" << std::endl;
  return 0;
}
