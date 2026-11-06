#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace m20::follow
{

inline constexpr std::uint32_t kPointCloudWireMagic = 0x4D323043U;
inline constexpr std::uint32_t kPointCloudWireVersion = 1U;
inline constexpr std::uint32_t kMaxPointCloudWireBytes = 64U * 1024U * 1024U;

struct Point3D
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct WirePointField
{
  std::string name;
  std::uint32_t offset{0};
  std::uint8_t datatype{0};
  std::uint32_t count{0};
};

struct WirePointCloud
{
  std::int32_t stamp_sec{0};
  std::uint32_t stamp_nanosec{0};
  std::string frame_id;
  std::uint32_t height{0};
  std::uint32_t width{0};
  std::vector<WirePointField> fields;
  bool is_bigendian{false};
  std::uint32_t point_step{0};
  std::uint32_t row_step{0};
  bool is_dense{false};
  std::vector<std::uint8_t> data;
};

bool deserializePointCloudWire(const std::vector<std::uint8_t> &bytes, WirePointCloud &cloud,
                               std::string &error);
bool extractXyzPoints(const WirePointCloud &cloud, std::vector<Point3D> &points,
                      std::string &error);

}  // namespace m20::follow
