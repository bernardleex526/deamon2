#include "m20_follow_control/pointcloud_wire.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <type_traits>

namespace m20::follow
{
namespace
{

template <typename T>
bool read(const std::vector<std::uint8_t> &input, std::size_t &offset, T &value)
{
  static_assert(std::is_trivially_copyable_v<T>);
  if (offset > input.size() || input.size() - offset < sizeof(T))
  {
    return false;
  }
  std::memcpy(&value, input.data() + offset, sizeof(T));
  offset += sizeof(T);
  return true;
}

bool readString(const std::vector<std::uint8_t> &input, std::size_t &offset, std::string &value)
{
  std::uint32_t size = 0;
  if (!read(input, offset, size) || offset > input.size() || input.size() - offset < size)
  {
    return false;
  }
  value.assign(reinterpret_cast<const char *>(input.data() + offset), size);
  offset += size;
  return true;
}

const WirePointField *findField(const WirePointCloud &cloud, const std::string &name)
{
  const auto field =
    std::find_if(cloud.fields.begin(), cloud.fields.end(),
                 [&name](const auto &candidate) { return candidate.name == name; });
  return field == cloud.fields.end() ? nullptr : &*field;
}

float readFloat(const std::uint8_t *point, std::uint32_t offset)
{
  float value = 0.0F;
  std::memcpy(&value, point + offset, sizeof(value));
  return value;
}

}  // namespace

bool deserializePointCloudWire(const std::vector<std::uint8_t> &bytes, WirePointCloud &cloud,
                               std::string &error)
{
  std::size_t offset = 0;
  std::uint32_t version = 0;
  std::uint32_t field_count = 0;
  if (!read(bytes, offset, version) || version != kPointCloudWireVersion ||
      !read(bytes, offset, cloud.stamp_sec) || !read(bytes, offset, cloud.stamp_nanosec) ||
      !readString(bytes, offset, cloud.frame_id) || !read(bytes, offset, cloud.height) ||
      !read(bytes, offset, cloud.width) || !read(bytes, offset, field_count) || field_count > 128U)
  {
    error = "invalid point-cloud wire header";
    return false;
  }

  cloud.fields.clear();
  cloud.fields.reserve(field_count);
  for (std::uint32_t index = 0; index < field_count; ++index)
  {
    WirePointField field;
    if (!readString(bytes, offset, field.name) || !read(bytes, offset, field.offset) ||
        !read(bytes, offset, field.datatype) || !read(bytes, offset, field.count))
    {
      error = "truncated point-cloud field table";
      return false;
    }
    cloud.fields.push_back(std::move(field));
  }

  std::uint8_t flag = 0;
  std::uint32_t data_size = 0;
  if (!read(bytes, offset, flag))
  {
    error = "truncated point-cloud endian flag";
    return false;
  }
  cloud.is_bigendian = flag != 0;
  if (!read(bytes, offset, cloud.point_step) || !read(bytes, offset, cloud.row_step) ||
      !read(bytes, offset, flag) || !read(bytes, offset, data_size) ||
      data_size > kMaxPointCloudWireBytes || offset > bytes.size() ||
      bytes.size() - offset != data_size)
  {
    error = "invalid point-cloud wire data length";
    return false;
  }
  cloud.is_dense = flag != 0;
  cloud.data.assign(bytes.begin() + static_cast<std::ptrdiff_t>(offset), bytes.end());
  error.clear();
  return true;
}

bool extractXyzPoints(const WirePointCloud &cloud, std::vector<Point3D> &points, std::string &error)
{
  constexpr std::uint8_t kFloat32 = 7U;
  const auto *x = findField(cloud, "x");
  const auto *y = findField(cloud, "y");
  const auto *z = findField(cloud, "z");
  if (!(x && y && z) || x->datatype != kFloat32 || y->datatype != kFloat32 ||
      z->datatype != kFloat32)
  {
    error = "point cloud requires float32 x, y and z fields";
    return false;
  }
  if (cloud.is_bigendian)
  {
    error = "big-endian point clouds are not supported";
    return false;
  }
  const auto field_fits = [&cloud](const WirePointField *field)
  { return static_cast<std::size_t>(field->offset) + sizeof(float) <= cloud.point_step; };
  const std::size_t row_payload = static_cast<std::size_t>(cloud.point_step) * cloud.width;
  const std::size_t total_payload = static_cast<std::size_t>(cloud.row_step) * cloud.height;
  if (!field_fits(x) || !field_fits(y) || !field_fits(z) || cloud.point_step == 0U ||
      row_payload > cloud.row_step || total_payload > cloud.data.size())
  {
    error = "invalid point-cloud layout";
    return false;
  }

  points.clear();
  points.reserve(static_cast<std::size_t>(cloud.width) * cloud.height);
  for (std::uint32_t row = 0; row < cloud.height; ++row)
  {
    for (std::uint32_t column = 0; column < cloud.width; ++column)
    {
      const auto offset = static_cast<std::size_t>(row) * cloud.row_step +
                          static_cast<std::size_t>(column) * cloud.point_step;
      const auto *point = cloud.data.data() + offset;
      Point3D output;
      output.x = readFloat(point, x->offset);
      output.y = readFloat(point, y->offset);
      output.z = readFloat(point, z->offset);
      if (std::isfinite(output.x) && std::isfinite(output.y) && std::isfinite(output.z))
      {
        points.push_back(output);
      }
    }
  }
  error.clear();
  return true;
}

}  // namespace m20::follow
