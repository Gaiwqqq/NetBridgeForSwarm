#include "transport/draco_pointcloud_codec.hpp"

#include <draco/attributes/geometry_attribute.h>
#include <draco/attributes/point_attribute.h>
#include <draco/compression/decode.h>
#include <draco/compression/encode.h>
#include <draco/core/decoder_buffer.h>
#include <draco/core/encoder_buffer.h>
#include <draco/point_cloud/point_cloud.h>
#include <draco/point_cloud/point_cloud_builder.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace swarm_ros_bridge {
namespace transport {
namespace {

constexpr std::uint8_t kLayoutFormatVersion = 2;
constexpr char kPointTypeXyz[] = "xyz";
constexpr char kPointTypeXyzRgb[] = "xyzrgb";

enum class ColorEncoding {
  kNone,
  kPacked,
  kSeparate,
};

struct PointLayout {
  const sensor_msgs::PointField* x{nullptr};
  const sensor_msgs::PointField* y{nullptr};
  const sensor_msgs::PointField* z{nullptr};
  const sensor_msgs::PointField* packed_color{nullptr};
  const sensor_msgs::PointField* red{nullptr};
  const sensor_msgs::PointField* green{nullptr};
  const sensor_msgs::PointField* blue{nullptr};
  const sensor_msgs::PointField* alpha{nullptr};
  ColorEncoding color_encoding{ColorEncoding::kNone};
  std::vector<bool> encoded_byte_mask;
};

std::size_t DatatypeSize(std::uint8_t datatype) {
  switch (datatype) {
    case sensor_msgs::PointField::INT8:
    case sensor_msgs::PointField::UINT8:
      return 1U;
    case sensor_msgs::PointField::INT16:
    case sensor_msgs::PointField::UINT16:
      return 2U;
    case sensor_msgs::PointField::INT32:
    case sensor_msgs::PointField::UINT32:
    case sensor_msgs::PointField::FLOAT32:
      return 4U;
    case sensor_msgs::PointField::FLOAT64:
      return 8U;
    default:
      return 0U;
  }
}

bool FieldFits(const sensor_msgs::PointField& field, std::uint32_t point_step) {
  const std::size_t datatype_size = DatatypeSize(field.datatype);
  if (datatype_size == 0U || field.count == 0U) {
    return false;
  }
  const std::uint64_t size =
      static_cast<std::uint64_t>(datatype_size) * field.count;
  return field.offset <= point_step && size <= point_step - field.offset;
}

const sensor_msgs::PointField* FindField(
    const std::vector<sensor_msgs::PointField>& fields,
    const std::string& name) {
  const auto found = std::find_if(
      fields.begin(), fields.end(), [&name](const sensor_msgs::PointField& field) {
        return field.name == name;
      });
  return found == fields.end() ? nullptr : &*found;
}

bool IsFloat32Scalar(const sensor_msgs::PointField* field,
                     std::uint32_t point_step) {
  return field != nullptr && field->datatype == sensor_msgs::PointField::FLOAT32 &&
         field->count == 1U && FieldFits(*field, point_step);
}

bool IsUint8Scalar(const sensor_msgs::PointField* field,
                   std::uint32_t point_step) {
  return field != nullptr && field->datatype == sensor_msgs::PointField::UINT8 &&
         field->count == 1U && FieldFits(*field, point_step);
}

bool IsPackedColor(const sensor_msgs::PointField* field,
                   std::uint32_t point_step) {
  return field != nullptr && field->count == 1U &&
         (field->datatype == sensor_msgs::PointField::FLOAT32 ||
          field->datatype == sensor_msgs::PointField::UINT32) &&
         FieldFits(*field, point_step);
}

bool SpansOverlap(const sensor_msgs::PointField& lhs,
                  const sensor_msgs::PointField& rhs) {
  const std::uint64_t lhs_end =
      lhs.offset + DatatypeSize(lhs.datatype) * lhs.count;
  const std::uint64_t rhs_end =
      rhs.offset + DatatypeSize(rhs.datatype) * rhs.count;
  return lhs.offset < rhs_end && rhs.offset < lhs_end;
}

bool MarkField(const sensor_msgs::PointField& field,
               std::vector<bool>* mask) {
  if (mask == nullptr || !FieldFits(field, mask->size())) {
    return false;
  }
  const std::size_t end =
      field.offset + DatatypeSize(field.datatype) * field.count;
  std::fill(mask->begin() + field.offset, mask->begin() + end, true);
  return true;
}

bool ParseLayout(const std::vector<sensor_msgs::PointField>& fields,
                 std::uint32_t point_step,
                 PointLayout* layout) {
  if (layout == nullptr || point_step == 0U) {
    return false;
  }
  for (const auto& field : fields) {
    if (!FieldFits(field, point_step)) {
      return false;
    }
  }

  PointLayout parsed;
  parsed.x = FindField(fields, "x");
  parsed.y = FindField(fields, "y");
  parsed.z = FindField(fields, "z");
  if (!IsFloat32Scalar(parsed.x, point_step) ||
      !IsFloat32Scalar(parsed.y, point_step) ||
      !IsFloat32Scalar(parsed.z, point_step) ||
      SpansOverlap(*parsed.x, *parsed.y) ||
      SpansOverlap(*parsed.x, *parsed.z) ||
      SpansOverlap(*parsed.y, *parsed.z)) {
    return false;
  }

  parsed.packed_color = FindField(fields, "rgba");
  if (!IsPackedColor(parsed.packed_color, point_step)) {
    parsed.packed_color = FindField(fields, "rgb");
  }
  if (IsPackedColor(parsed.packed_color, point_step)) {
    parsed.color_encoding = ColorEncoding::kPacked;
  } else {
    parsed.packed_color = nullptr;
    parsed.red = FindField(fields, "r");
    parsed.green = FindField(fields, "g");
    parsed.blue = FindField(fields, "b");
    parsed.alpha = FindField(fields, "a");
    if (IsUint8Scalar(parsed.red, point_step) &&
        IsUint8Scalar(parsed.green, point_step) &&
        IsUint8Scalar(parsed.blue, point_step) &&
        (parsed.alpha == nullptr || IsUint8Scalar(parsed.alpha, point_step))) {
      parsed.color_encoding = ColorEncoding::kSeparate;
    } else {
      parsed.red = nullptr;
      parsed.green = nullptr;
      parsed.blue = nullptr;
      parsed.alpha = nullptr;
    }
  }

  std::vector<const sensor_msgs::PointField*> encoded_fields{
      parsed.x, parsed.y, parsed.z};
  if (parsed.color_encoding == ColorEncoding::kPacked) {
    encoded_fields.push_back(parsed.packed_color);
  } else if (parsed.color_encoding == ColorEncoding::kSeparate) {
    encoded_fields.push_back(parsed.red);
    encoded_fields.push_back(parsed.green);
    encoded_fields.push_back(parsed.blue);
    if (parsed.alpha != nullptr) {
      encoded_fields.push_back(parsed.alpha);
    }
  }
  for (std::size_t i = 0; i < encoded_fields.size(); ++i) {
    for (std::size_t j = i + 1; j < encoded_fields.size(); ++j) {
      if (SpansOverlap(*encoded_fields[i], *encoded_fields[j])) {
        return false;
      }
    }
  }
  for (const auto& field : fields) {
    const bool is_encoded = std::find(encoded_fields.begin(),
                                      encoded_fields.end(), &field) !=
                            encoded_fields.end();
    if (is_encoded) {
      continue;
    }
    for (const auto* encoded_field : encoded_fields) {
      if (SpansOverlap(field, *encoded_field)) {
        return false;
      }
    }
  }

  parsed.encoded_byte_mask.assign(point_step, false);
  for (const auto* field : encoded_fields) {
    if (!MarkField(*field, &parsed.encoded_byte_mask)) {
      return false;
    }
  }
  *layout = std::move(parsed);
  return true;
}

bool ValidateMessageStorage(const sensor_msgs::PointCloud2& message,
                            std::uint64_t* point_count) {
  if (point_count == nullptr ||
      (message.height == 0U && message.width != 0U)) {
    return false;
  }
  const std::uint64_t row_data_size =
      static_cast<std::uint64_t>(message.width) * message.point_step;
  const std::uint64_t expected_data_size =
      static_cast<std::uint64_t>(message.row_step) * message.height;
  const std::uint64_t count =
      static_cast<std::uint64_t>(message.width) * message.height;
  if (row_data_size > message.row_step ||
      expected_data_size != message.data.size() ||
      count > static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max())) {
    return false;
  }
  if (count > 0U && message.point_step == 0U) {
    return false;
  }
  *point_count = count;
  return true;
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

void WriteUint32(std::uint32_t value, bool bigendian, std::uint8_t* data) {
  if (bigendian) {
    data[0] = static_cast<std::uint8_t>(value >> 24U);
    data[1] = static_cast<std::uint8_t>(value >> 16U);
    data[2] = static_cast<std::uint8_t>(value >> 8U);
    data[3] = static_cast<std::uint8_t>(value);
    return;
  }
  data[0] = static_cast<std::uint8_t>(value);
  data[1] = static_cast<std::uint8_t>(value >> 8U);
  data[2] = static_cast<std::uint8_t>(value >> 16U);
  data[3] = static_cast<std::uint8_t>(value >> 24U);
}

float ReadFloat32(const std::uint8_t* data, bool bigendian) {
  const std::uint32_t bits = ReadUint32(data, bigendian);
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

void WriteFloat32(float value, bool bigendian, std::uint8_t* data) {
  std::uint32_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  WriteUint32(bits, bigendian, data);
}

std::array<std::uint8_t, 4> ReadColor(const std::uint8_t* point,
                                      bool bigendian,
                                      const PointLayout& layout) {
  if (layout.color_encoding == ColorEncoding::kPacked) {
    const std::uint32_t packed =
        ReadUint32(point + layout.packed_color->offset, bigendian);
    return {{static_cast<std::uint8_t>(packed >> 16U),
             static_cast<std::uint8_t>(packed >> 8U),
             static_cast<std::uint8_t>(packed),
             static_cast<std::uint8_t>(packed >> 24U)}};
  }
  return {{point[layout.red->offset], point[layout.green->offset],
           point[layout.blue->offset],
           layout.alpha == nullptr ? std::uint8_t{255}
                                   : point[layout.alpha->offset]}};
}

void WriteColor(const std::array<std::uint8_t, 4>& color,
                bool bigendian,
                const PointLayout& layout,
                std::uint8_t* point) {
  if (layout.color_encoding == ColorEncoding::kPacked) {
    const std::uint32_t packed =
        (static_cast<std::uint32_t>(color[3]) << 24U) |
        (static_cast<std::uint32_t>(color[0]) << 16U) |
        (static_cast<std::uint32_t>(color[1]) << 8U) |
        static_cast<std::uint32_t>(color[2]);
    WriteUint32(packed, bigendian, point + layout.packed_color->offset);
    return;
  }
  point[layout.red->offset] = color[0];
  point[layout.green->offset] = color[1];
  point[layout.blue->offset] = color[2];
  if (layout.alpha != nullptr) {
    point[layout.alpha->offset] = color[3];
  }
}

std::size_t UnencodedBytesPerPoint(const PointLayout& layout) {
  return static_cast<std::size_t>(std::count(
      layout.encoded_byte_mask.begin(), layout.encoded_byte_mask.end(), false));
}

bool ExtractSidecar(const sensor_msgs::PointCloud2& message,
                    const PointLayout& layout,
                    std::vector<std::uint8_t>* sidecar) {
  if (sidecar == nullptr) {
    return false;
  }
  const std::size_t per_point = UnencodedBytesPerPoint(layout);
  const std::size_t row_padding =
      message.row_step - message.width * message.point_step;
  const std::uint64_t expected =
      static_cast<std::uint64_t>(per_point) * message.width * message.height +
      static_cast<std::uint64_t>(row_padding) * message.height;
  if (expected > std::numeric_limits<std::size_t>::max()) {
    return false;
  }
  sidecar->clear();
  sidecar->reserve(static_cast<std::size_t>(expected));
  for (std::uint32_t row = 0; row < message.height; ++row) {
    const std::size_t row_offset = static_cast<std::size_t>(row) * message.row_step;
    for (std::uint32_t column = 0; column < message.width; ++column) {
      const std::size_t point_offset =
          row_offset + static_cast<std::size_t>(column) * message.point_step;
      for (std::size_t byte = 0; byte < message.point_step; ++byte) {
        if (!layout.encoded_byte_mask[byte]) {
          sidecar->push_back(message.data[point_offset + byte]);
        }
      }
    }
    sidecar->insert(
        sidecar->end(),
        message.data.begin() + row_offset + message.width * message.point_step,
        message.data.begin() + row_offset + message.row_step);
  }
  return sidecar->size() == expected;
}

bool RestoreSidecar(const std::vector<std::uint8_t>& sidecar,
                    const PointLayout& layout,
                    sensor_msgs::PointCloud2* message) {
  if (message == nullptr) {
    return false;
  }
  const std::size_t per_point = UnencodedBytesPerPoint(layout);
  const std::size_t row_padding =
      message->row_step - message->width * message->point_step;
  const std::uint64_t expected =
      static_cast<std::uint64_t>(per_point) * message->width * message->height +
      static_cast<std::uint64_t>(row_padding) * message->height;
  if (expected != sidecar.size()) {
    return false;
  }

  std::size_t cursor = 0U;
  for (std::uint32_t row = 0; row < message->height; ++row) {
    const std::size_t row_offset = static_cast<std::size_t>(row) * message->row_step;
    for (std::uint32_t column = 0; column < message->width; ++column) {
      const std::size_t point_offset =
          row_offset + static_cast<std::size_t>(column) * message->point_step;
      for (std::size_t byte = 0; byte < message->point_step; ++byte) {
        if (!layout.encoded_byte_mask[byte]) {
          message->data[point_offset + byte] = sidecar[cursor++];
        }
      }
    }
    std::copy(sidecar.begin() + cursor,
              sidecar.begin() + cursor + row_padding,
              message->data.begin() + row_offset +
                  message->width * message->point_step);
    cursor += row_padding;
  }
  return cursor == sidecar.size();
}

std::size_t PointDataOffset(const sensor_msgs::PointCloud2& message,
                            std::uint32_t point_index) {
  const std::uint32_t row = point_index / message.width;
  const std::uint32_t column = point_index % message.width;
  return static_cast<std::size_t>(row) * message.row_step +
         static_cast<std::size_t>(column) * message.point_step;
}

}  // namespace

std::string DracoPointCloudCodec::Name() const { return "draco"; }

bool DracoPointCloudCodec::Encode(const sensor_msgs::PointCloud2& message,
                                  EncodedPointCloud* output) const {
  if (output == nullptr) {
    return false;
  }
  std::uint64_t point_count = 0U;
  PointLayout layout;
  if (!ValidateMessageStorage(message, &point_count) ||
      (point_count > 0U &&
       !ParseLayout(message.fields, message.point_step, &layout))) {
    return false;
  }

  EncodedPointCloud encoded;
  encoded.codec = Name();
  encoded.format_version = kLayoutFormatVersion;
  encoded.point_type = layout.color_encoding == ColorEncoding::kNone
                           ? kPointTypeXyz
                           : kPointTypeXyzRgb;
  encoded.original_header = message.header;
  encoded.original_width = message.width;
  encoded.original_height = message.height;
  encoded.original_fields = message.fields;
  encoded.original_is_bigendian = message.is_bigendian;
  encoded.original_point_step = message.point_step;
  encoded.original_row_step = message.row_step;
  encoded.original_is_dense = message.is_dense;
  encoded.source_stamp = message.header.stamp;
  encoded.frame_id = message.header.frame_id;

  if (point_count == 0U) {
    encoded.sidecar_bytes = message.data;
    *output = std::move(encoded);
    return true;
  }
  if (!ExtractSidecar(message, layout, &encoded.sidecar_bytes)) {
    return false;
  }

  draco::PointCloudBuilder builder;
  builder.Start(static_cast<std::uint32_t>(point_count));
  const int position_id = builder.AddAttribute(
      draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32, false);
  const int point_index_id = builder.AddAttribute(
      draco::GeometryAttribute::GENERIC, 1, draco::DT_UINT32, false);
  const int color_id = layout.color_encoding == ColorEncoding::kNone
                           ? -1
                           : builder.AddAttribute(
                                 draco::GeometryAttribute::COLOR, 4,
                                 draco::DT_UINT8, false);
  if (position_id < 0 || point_index_id < 0 ||
      (layout.color_encoding != ColorEncoding::kNone && color_id < 0)) {
    return false;
  }

  std::array<float, 3> position{};
  for (std::uint32_t index = 0; index < point_count; ++index) {
    const std::size_t offset = PointDataOffset(message, index);
    const std::uint8_t* point = message.data.data() + offset;
    position = {{ReadFloat32(point + layout.x->offset, message.is_bigendian),
                 ReadFloat32(point + layout.y->offset, message.is_bigendian),
                 ReadFloat32(point + layout.z->offset, message.is_bigendian)}};
    const draco::PointIndex draco_index(index);
    builder.SetAttributeValueForPoint(position_id, draco_index,
                                      position.data());
    builder.SetAttributeValueForPoint(point_index_id, draco_index, &index);
    if (color_id >= 0) {
      const auto color = ReadColor(point, message.is_bigendian, layout);
      builder.SetAttributeValueForPoint(color_id, draco_index, color.data());
    }
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
  encoded.payload.assign(buffer.data(), buffer.data() + buffer.size());
  *output = std::move(encoded);
  return true;
}

bool DracoPointCloudCodec::Decode(const EncodedPointCloud& input,
                                  sensor_msgs::PointCloud2* message) const {
  if (message == nullptr || input.format_version != kLayoutFormatVersion) {
    return false;
  }

  sensor_msgs::PointCloud2 decoded;
  decoded.header = input.original_header;
  decoded.width = input.original_width;
  decoded.height = input.original_height;
  decoded.fields = input.original_fields;
  decoded.is_bigendian = input.original_is_bigendian;
  decoded.point_step = input.original_point_step;
  decoded.row_step = input.original_row_step;
  decoded.is_dense = input.original_is_dense;
  const std::uint64_t data_size =
      static_cast<std::uint64_t>(decoded.row_step) * decoded.height;
  if (data_size > std::numeric_limits<std::size_t>::max()) {
    return false;
  }
  decoded.data.assign(static_cast<std::size_t>(data_size), 0U);

  std::uint64_t point_count = 0U;
  if (!ValidateMessageStorage(decoded, &point_count)) {
    return false;
  }
  if (point_count == 0U) {
    if (!input.payload.empty() || input.sidecar_bytes.size() != decoded.data.size()) {
      return false;
    }
    decoded.data = input.sidecar_bytes;
    *message = std::move(decoded);
    return true;
  }

  PointLayout layout;
  if (!ParseLayout(decoded.fields, decoded.point_step, &layout) ||
      (input.point_type != kPointTypeXyz &&
       input.point_type != kPointTypeXyzRgb) ||
      (input.point_type == kPointTypeXyzRgb) !=
          (layout.color_encoding != ColorEncoding::kNone) ||
      !RestoreSidecar(input.sidecar_bytes, layout, &decoded) ||
      input.payload.empty()) {
    return false;
  }

  draco::DecoderBuffer buffer;
  buffer.Init(reinterpret_cast<const char*>(input.payload.data()),
              input.payload.size());
  draco::Decoder decoder;
  auto decoded_result = decoder.DecodePointCloudFromBuffer(&buffer);
  if (!decoded_result.ok()) {
    return false;
  }
  std::unique_ptr<draco::PointCloud> draco_cloud =
      std::move(decoded_result).value();
  if (draco_cloud == nullptr || draco_cloud->num_points() != point_count) {
    return false;
  }

  const draco::PointAttribute* position =
      draco_cloud->GetNamedAttribute(draco::GeometryAttribute::POSITION);
  const draco::PointAttribute* point_index =
      draco_cloud->GetNamedAttribute(draco::GeometryAttribute::GENERIC);
  const draco::PointAttribute* color =
      draco_cloud->GetNamedAttribute(draco::GeometryAttribute::COLOR);
  if (position == nullptr || point_index == nullptr ||
      (layout.color_encoding != ColorEncoding::kNone && color == nullptr)) {
    return false;
  }

  std::vector<bool> restored(point_count, false);
  std::array<float, 3> position_value{};
  std::array<std::uint8_t, 4> color_value{};
  for (draco::PointIndex index(0); index < draco_cloud->num_points(); ++index) {
    std::uint32_t original_index = 0U;
    point_index->GetMappedValue(index, &original_index);
    if (original_index >= point_count || restored[original_index]) {
      return false;
    }
    restored[original_index] = true;
    position->GetMappedValue(index, position_value.data());
    const std::size_t offset = PointDataOffset(decoded, original_index);
    std::uint8_t* point = decoded.data.data() + offset;
    WriteFloat32(position_value[0], decoded.is_bigendian,
                 point + layout.x->offset);
    WriteFloat32(position_value[1], decoded.is_bigendian,
                 point + layout.y->offset);
    WriteFloat32(position_value[2], decoded.is_bigendian,
                 point + layout.z->offset);
    if (color != nullptr) {
      color->GetMappedValue(index, color_value.data());
      WriteColor(color_value, decoded.is_bigendian, layout, point);
    }
  }
  if (std::find(restored.begin(), restored.end(), false) != restored.end()) {
    return false;
  }
  *message = std::move(decoded);
  return true;
}

}  // namespace transport
}  // namespace swarm_ros_bridge
