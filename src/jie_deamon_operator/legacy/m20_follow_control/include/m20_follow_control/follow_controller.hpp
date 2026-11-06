#pragma once

#include <cstddef>
#include <vector>

namespace m20::follow
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct VelocityCommand
{
  double vx{0.0};
  double vy{0.0};
  double yaw_rate{0.0};
};

struct FollowParameters
{
  double follow_distance{1.2};
  double target_radius{0.4};
  std::size_t min_target_points{3};
  double target_smoothing{0.35};

  double body_min_x{-0.45};
  double body_max_x{0.45};
  double body_min_y{-0.25};
  double body_max_y{0.25};

  double influence_distance{1.0};
  double slowdown_distance{1.0};
  double emergency_distance{0.7};
  double repulsion_gain{0.05};

  double linear_gain{0.5};
  double lateral_gain{0.4};
  double yaw_gain{0.8};
  double linear_deadband{0.05};
  double lateral_deadband{0.05};
  double yaw_deadband{0.08};

  double max_linear_x{0.25};
  double max_linear_y{0.15};
  double max_yaw_rate{0.3};
  double max_linear_acceleration{0.3};
  double max_yaw_acceleration{0.5};
};

struct FollowResult
{
  VelocityCommand command;
  bool target_visible{false};
  bool emergency_stop{false};
  double target_x{0.0};
  double target_y{0.0};
  double min_obstacle_distance{0.0};
  std::size_t target_points{0};
};

class FollowController
{
 public:
  explicit FollowController(FollowParameters parameters = {});

  void setTarget(double x, double y);
  void reset();
  FollowResult update(const std::vector<Point2D> &points, double dt_seconds);

 private:
  VelocityCommand applyLimits(const VelocityCommand &desired, double dt_seconds);

  FollowParameters parameters_;
  double target_x_;
  double target_y_;
  bool target_acquired_{false};
  VelocityCommand last_command_;
};

}  // namespace m20::follow
