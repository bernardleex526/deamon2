#include "m20_follow_control/follow_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace m20::follow
{
namespace
{

double clampDelta(double value, double previous, double max_delta)
{
  return previous + std::clamp(value - previous, -max_delta, max_delta);
}

double distanceToBody(const Point2D &point, const FollowParameters &parameters)
{
  const double dx =
    std::max({parameters.body_min_x - point.x, 0.0, point.x - parameters.body_max_x});
  const double dy =
    std::max({parameters.body_min_y - point.y, 0.0, point.y - parameters.body_max_y});
  return std::hypot(dx, dy);
}

bool insideBody(const Point2D &point, const FollowParameters &parameters)
{
  return point.x >= parameters.body_min_x && point.x <= parameters.body_max_x &&
         point.y >= parameters.body_min_y && point.y <= parameters.body_max_y;
}

}  // namespace

FollowController::FollowController(FollowParameters parameters)
    : parameters_(parameters), target_x_(parameters.follow_distance), target_y_(0.0)
{
  if (parameters_.follow_distance <= 0.0 || parameters_.target_radius <= 0.0 ||
      parameters_.min_target_points == 0U || parameters_.body_min_x >= parameters_.body_max_x ||
      parameters_.body_min_y >= parameters_.body_max_y || parameters_.emergency_distance < 0.0 ||
      parameters_.slowdown_distance <= parameters_.emergency_distance ||
      parameters_.influence_distance < parameters_.slowdown_distance ||
      parameters_.max_linear_x <= 0.0 || parameters_.max_linear_y <= 0.0 ||
      parameters_.max_yaw_rate <= 0.0 || parameters_.max_linear_acceleration < 0.0 ||
      parameters_.max_yaw_acceleration < 0.0)
  {
    throw std::invalid_argument("invalid M20 follow safety parameters");
  }
}

void FollowController::setTarget(double x, double y)
{
  target_x_ = x;
  target_y_ = y;
  target_acquired_ = false;
  last_command_ = {};
}

void FollowController::reset()
{
  target_x_ = parameters_.follow_distance;
  target_y_ = 0.0;
  target_acquired_ = false;
  last_command_ = {};
}

FollowResult FollowController::update(const std::vector<Point2D> &points, double dt_seconds)
{
  FollowResult result;
  result.target_x = target_x_;
  result.target_y = target_y_;
  result.min_obstacle_distance = std::numeric_limits<double>::infinity();

  double centroid_x = 0.0;
  double centroid_y = 0.0;
  std::vector<bool> target_mask(points.size(), false);
  for (std::size_t index = 0; index < points.size(); ++index)
  {
    const auto &point = points[index];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || insideBody(point, parameters_))
    {
      continue;
    }
    if (std::hypot(point.x - target_x_, point.y - target_y_) <= parameters_.target_radius)
    {
      target_mask[index] = true;
      centroid_x += point.x;
      centroid_y += point.y;
      ++result.target_points;
    }
  }

  if (result.target_points < parameters_.min_target_points)
  {
    target_acquired_ = false;
    last_command_ = {};
    result.command = {};
    return result;
  }

  centroid_x /= static_cast<double>(result.target_points);
  centroid_y /= static_cast<double>(result.target_points);
  const double smoothing = std::clamp(parameters_.target_smoothing, 0.0, 1.0);
  if (target_acquired_)
  {
    target_x_ += smoothing * (centroid_x - target_x_);
    target_y_ += smoothing * (centroid_y - target_y_);
  }
  else
  {
    target_x_ = centroid_x;
    target_y_ = centroid_y;
  }
  target_acquired_ = true;
  result.target_visible = true;
  result.target_x = target_x_;
  result.target_y = target_y_;

  double repulsion_x = 0.0;
  double repulsion_y = 0.0;
  for (std::size_t index = 0; index < points.size(); ++index)
  {
    const auto &point = points[index];
    if (target_mask[index] || !std::isfinite(point.x) || !std::isfinite(point.y) ||
        insideBody(point, parameters_))
    {
      continue;
    }

    const double clearance = distanceToBody(point, parameters_);
    result.min_obstacle_distance = std::min(result.min_obstacle_distance, clearance);
    if (clearance <= 1e-6 || clearance >= parameters_.influence_distance || point.x < -0.1)
    {
      continue;
    }

    const double center_distance = std::hypot(point.x, point.y);
    if (center_distance <= 1e-6)
    {
      continue;
    }
    const double force = parameters_.repulsion_gain *
                         (1.0 / clearance - 1.0 / parameters_.influence_distance) /
                         (clearance * clearance);
    repulsion_x -= force * point.x / center_distance;
    repulsion_y -= force * point.y / center_distance;
  }

  if (result.min_obstacle_distance <= parameters_.emergency_distance)
  {
    result.emergency_stop = true;
    last_command_ = {};
    result.command = {};
    return result;
  }

  VelocityCommand desired;
  const double longitudinal_error = target_x_ - parameters_.follow_distance;
  if (std::abs(longitudinal_error) > parameters_.linear_deadband)
  {
    desired.vx = longitudinal_error * parameters_.linear_gain;
  }
  if (std::abs(target_y_) > parameters_.lateral_deadband)
  {
    desired.vy = target_y_ * parameters_.lateral_gain;
  }
  const double target_angle = std::atan2(target_y_, target_x_);
  if (std::abs(target_angle) > parameters_.yaw_deadband)
  {
    desired.yaw_rate = target_angle * parameters_.yaw_gain;
  }

  desired.vx += repulsion_x;
  desired.vy += repulsion_y;
  if (result.min_obstacle_distance < parameters_.slowdown_distance)
  {
    const double denominator = parameters_.slowdown_distance - parameters_.emergency_distance;
    const double factor =
      denominator > 1e-6
        ? std::clamp((result.min_obstacle_distance - parameters_.emergency_distance) / denominator,
                     0.0, 1.0)
        : 0.0;
    desired.vx *= factor;
    desired.vy *= factor;
  }

  result.command = applyLimits(desired, dt_seconds);
  return result;
}

VelocityCommand FollowController::applyLimits(const VelocityCommand &desired, double dt_seconds)
{
  const double dt = std::clamp(dt_seconds, 0.001, 0.5);
  VelocityCommand limited;
  limited.vx = std::clamp(desired.vx, -parameters_.max_linear_x, parameters_.max_linear_x);
  limited.vy = std::clamp(desired.vy, -parameters_.max_linear_y, parameters_.max_linear_y);
  limited.yaw_rate =
    std::clamp(desired.yaw_rate, -parameters_.max_yaw_rate, parameters_.max_yaw_rate);

  const double max_linear_delta = std::max(0.0, parameters_.max_linear_acceleration) * dt;
  const double max_yaw_delta = std::max(0.0, parameters_.max_yaw_acceleration) * dt;
  limited.vx = clampDelta(limited.vx, last_command_.vx, max_linear_delta);
  limited.vy = clampDelta(limited.vy, last_command_.vy, max_linear_delta);
  limited.yaw_rate = clampDelta(limited.yaw_rate, last_command_.yaw_rate, max_yaw_delta);
  last_command_ = limited;
  return limited;
}

}  // namespace m20::follow
