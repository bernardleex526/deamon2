#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <type_traits>
#include <vector>

#include "m20_follow_control/pointcloud_wire.hpp"

namespace
{

template <typename T>
void append(std::vector<std::uint8_t> &output, const T &value)
{
  static_assert(std::is_trivially_copyable_v<T>);
  const auto *begin = reinterpret_cast<const std::uint8_t *>(&value);
  output.insert(output.end(), begin, begin + sizeof(T));
}

void appendString(std::vector<std::uint8_t> &output, const std::string &value)
{
  append(output, static_cast<std::uint32_t>(value.size()));
  output.insert(output.end(), value.begin(), value.end());
}

std::vector<std::uint8_t> makePayload()
{
  std::vector<std::uint8_t> output;
  append(output, m20::follow::kPointCloudWireVersion);
  append(output, std::int32_t{10});
  append(output, std::uint32_t{20});
  appendString(output, "base_link");
  append(output, std::uint32_t{1});
  append(output, std::uint32_t{2});
  append(output, std::uint32_t{3});
  for (const auto &name_offset :
       {std::pair<std::string, std::uint32_t>{"x", 0}, {"y", 4}, {"z", 8}})
  {
    appendString(output, name_offset.first);
    append(output, name_offset.second);
    append(output, std::uint8_t{7});
    append(output, std::uint32_t{1});
  }
  append(output, std::uint8_t{0});
  append(output, std::uint32_t{12});
  append(output, std::uint32_t{24});
  append(output, std::uint8_t{1});
  append(output, std::uint32_t{24});
  for (const float value : {1.0F, 2.0F, 3.0F, -1.0F, -2.0F, -3.0F})
  {
    append(output, value);
  }
  return output;
}

TEST(PointCloudWire, DecodesReferenceGatewayPayload)
{
  m20::follow::WirePointCloud cloud;
  std::string error;
  ASSERT_TRUE(m20::follow::deserializePointCloudWire(makePayload(), cloud, error)) << error;
  EXPECT_EQ(cloud.frame_id, "base_link");
  EXPECT_EQ(cloud.width, 2U);

  std::vector<m20::follow::Point3D> points;
  ASSERT_TRUE(m20::follow::extractXyzPoints(cloud, points, error)) << error;
  ASSERT_EQ(points.size(), 2U);
  EXPECT_DOUBLE_EQ(points[0].x, 1.0);
  EXPECT_DOUBLE_EQ(points[0].y, 2.0);
  EXPECT_DOUBLE_EQ(points[0].z, 3.0);
  EXPECT_DOUBLE_EQ(points[1].x, -1.0);
}

TEST(PointCloudWire, RejectsTruncatedPayload)
{
  auto payload = makePayload();
  payload.pop_back();
  m20::follow::WirePointCloud cloud;
  std::string error;
  EXPECT_FALSE(m20::follow::deserializePointCloudWire(payload, cloud, error));
}

}  // namespace
