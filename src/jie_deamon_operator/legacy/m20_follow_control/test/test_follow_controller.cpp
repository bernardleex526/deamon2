#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "m20_follow_control/follow_controller.hpp"

namespace
{

using m20::follow::FollowController;
using m20::follow::FollowParameters;
using m20::follow::Point2D;

FollowParameters testParameters()
{
  FollowParameters parameters;
  parameters.follow_distance = 1.0;
  parameters.target_radius = 0.25;
  parameters.min_target_points = 2;
  parameters.target_smoothing = 1.0;
  parameters.emergency_distance = 0.5;
  parameters.slowdown_distance = 0.8;
  parameters.influence_distance = 0.8;
  parameters.repulsion_gain = 0.0;
  parameters.linear_gain = 1.0;
  parameters.max_linear_x = 0.4;
  parameters.max_linear_y = 0.2;
  parameters.max_yaw_rate = 0.5;
  parameters.max_linear_acceleration = 10.0;
  parameters.max_yaw_acceleration = 10.0;
  return parameters;
}

TEST(FollowController, MovesTowardVisibleTargetWithinConfiguredLimits)
{
  FollowController controller(testParameters());
  controller.setTarget(1.5, 0.0);

  const auto result = controller.update({{1.45, -0.03}, {1.55, 0.03}}, 0.1);

  EXPECT_TRUE(result.target_visible);
  EXPECT_FALSE(result.emergency_stop);
  EXPECT_GT(result.command.vx, 0.0);
  EXPECT_LE(result.command.vx, 0.4);
  EXPECT_NEAR(result.command.vy, 0.0, 1e-6);
}

TEST(FollowController, StopsImmediatelyWhenTargetIsLost)
{
  FollowController controller(testParameters());
  controller.setTarget(1.5, 0.0);
  ASSERT_TRUE(controller.update({{1.45, 0.0}, {1.55, 0.0}}, 0.1).target_visible);

  const auto result = controller.update({}, 0.1);

  EXPECT_FALSE(result.target_visible);
  EXPECT_DOUBLE_EQ(result.command.vx, 0.0);
  EXPECT_DOUBLE_EQ(result.command.vy, 0.0);
  EXPECT_DOUBLE_EQ(result.command.yaw_rate, 0.0);
}

TEST(FollowController, StopsForNonTargetObstacleInsideEmergencyDistance)
{
  FollowController controller(testParameters());
  controller.setTarget(1.5, 0.0);

  const auto result = controller.update({{1.45, -0.03}, {1.55, 0.03}, {0.48, 0.35}}, 0.1);

  EXPECT_TRUE(result.target_visible);
  EXPECT_TRUE(result.emergency_stop);
  EXPECT_DOUBLE_EQ(result.command.vx, 0.0);
}

TEST(FollowController, IgnoresPointsInsideM20BodyFootprint)
{
  FollowController controller(testParameters());
  controller.setTarget(1.5, 0.0);

  const auto result =
    controller.update({{1.45, -0.03}, {1.55, 0.03}, {0.2, 0.1}, {-0.2, -0.1}}, 0.1);

  EXPECT_TRUE(result.target_visible);
  EXPECT_FALSE(result.emergency_stop);
  EXPECT_GT(result.command.vx, 0.0);
}

TEST(FollowController, RejectsUnsafeDistanceOrdering)
{
  auto parameters = testParameters();
  parameters.emergency_distance = 1.0;
  parameters.slowdown_distance = 0.8;
  EXPECT_THROW(FollowController controller(parameters), std::invalid_argument);
}

}  // namespace
