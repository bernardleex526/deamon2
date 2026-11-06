#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "m20_follow_control/m20_protocol.hpp"

namespace
{

using m20::follow::AxisParameters;
using m20::follow::axisParametersValid;
using m20::follow::M20Protocol;
using m20::follow::normalizeAxisCommand;
using m20::follow::RobotStatus;
using m20::follow::statusAllowsMotion;
using m20::follow::VelocityCommand;

std::string payload(const std::vector<std::uint8_t> &frame)
{
  return std::string(frame.begin() + 16, frame.end());
}

TEST(M20Protocol, BuildsLittleEndianJsonApduHeader)
{
  const auto frame = M20Protocol::heartbeat(0x1234, "2026-09-04 10:00:00");

  ASSERT_GE(frame.size(), 16U);
  EXPECT_EQ(frame[0], 0xeb);
  EXPECT_EQ(frame[1], 0x91);
  EXPECT_EQ(frame[2], 0xeb);
  EXPECT_EQ(frame[3], 0x90);
  EXPECT_EQ(frame[6], 0x34);
  EXPECT_EQ(frame[7], 0x12);
  EXPECT_EQ(frame[8], 0x01);
  const std::size_t encoded_length = frame[4] | (static_cast<std::size_t>(frame[5]) << 8U);
  EXPECT_EQ(encoded_length, frame.size() - 16U);
  EXPECT_NE(payload(frame).find("\"Type\":100"), std::string::npos);
  EXPECT_NE(payload(frame).find("\"Command\":100"), std::string::npos);
}

TEST(M20Protocol, NormalizesOnlyAfterApplyingSafePhysicalLimits)
{
  AxisParameters parameters;
  const auto axis = normalizeAxisCommand(VelocityCommand{1.0, -1.0, 2.0}, parameters);

  EXPECT_NEAR(axis.x, 0.25 / 1.5, 1e-9);
  EXPECT_NEAR(axis.y, -0.15 / 0.6, 1e-9);
  EXPECT_NEAR(axis.yaw, 0.3 / 1.0, 1e-9);
}

TEST(M20Protocol, RejectsUnsafeAxisScaleConfiguration)
{
  AxisParameters parameters;
  EXPECT_TRUE(axisParametersValid(parameters));
  parameters.safe_max_x = 2.0;
  EXPECT_FALSE(axisParametersValid(parameters));
}

TEST(M20Protocol, ParsesBasicStatusAndResponseErrorCode)
{
  const std::string json =
    "{\"PatrolDevice\":{\"Type\":1002,\"Command\":6,\"Items\":{"
    "\"ErrorCode\":0,\"BasicStatus\":{\"MotionState\":17,\"Gait\":12290,"
    "\"HES\":0,\"ControlUsageMode\":0,\"Charge\":0,\"Direction\":0,"
    "\"Version\":\"PRO\"}}}}";
  std::vector<std::uint8_t> frame(16, 0);
  frame[0] = 0xeb;
  frame[1] = 0x91;
  frame[2] = 0xeb;
  frame[3] = 0x90;
  frame[4] = static_cast<std::uint8_t>(json.size() & 0xffU);
  frame[5] = static_cast<std::uint8_t>((json.size() >> 8U) & 0xffU);
  frame[6] = 9;
  frame[8] = 1;
  frame.insert(frame.end(), json.begin(), json.end());

  const auto parsed = M20Protocol::parse(frame);

  ASSERT_TRUE(parsed.valid) << parsed.error;
  EXPECT_EQ(parsed.message_id, 9);
  EXPECT_EQ(parsed.type, 1002);
  EXPECT_EQ(parsed.command, 6);
  ASSERT_TRUE(parsed.error_code.has_value());
  EXPECT_EQ(*parsed.error_code, 0);
  EXPECT_TRUE(parsed.status.received);
  EXPECT_EQ(parsed.status.motion_state, 17);
  EXPECT_EQ(parsed.status.gait, 0x3002);
  EXPECT_EQ(parsed.status.usage_mode, 0);
  EXPECT_EQ(parsed.status.hard_estop, 0);
  EXPECT_EQ(parsed.status.charge_state, 0);
  EXPECT_EQ(parsed.status.direction, 0);
  EXPECT_EQ(parsed.status.version, "PRO");
}

TEST(M20Protocol, MotionGateRequiresAllBasicStatusSafetyConditions)
{
  RobotStatus status;
  status.received = true;
  status.motion_state = 17;
  status.usage_mode = 0;
  status.hard_estop = 0;
  status.charge_state = 0;
  status.direction = 0;
  EXPECT_TRUE(statusAllowsMotion(status));

  status.direction = 1;
  EXPECT_FALSE(statusAllowsMotion(status));
  status.direction = 0;
  status.hard_estop = 1;
  EXPECT_FALSE(statusAllowsMotion(status));
}

TEST(M20Protocol, RejectsTruncatedDatagram)
{
  auto frame = M20Protocol::heartbeat(1, "2026-09-04 10:00:00");
  frame.pop_back();

  const auto parsed = M20Protocol::parse(frame);

  EXPECT_FALSE(parsed.valid);
}

}  // namespace
