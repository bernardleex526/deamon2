#ifndef GAIT_FOLLOW_CONTROLLER_HPP
#define GAIT_FOLLOW_CONTROLLER_HPP

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <geometry_msgs/msg/twist.hpp>

// Intent generation, not the safety gate. Zero remains zero inside tolerance.
class GaitFollowController {
public:
    struct Config {
        double distance = 1.2;
        double distance_stop = .06;
        double distance_start = .10;
        double angle_stop = .07;
        double angle_start = .15;
        double min_vx = .22;
        double max_vx = .30;
        double min_wz = .52;
        double max_wz = .60;
        double acceleration = .30;
        double angular_acceleration = .80;
    };

    void configure(const Config& c) {
        for (double v : {c.distance, c.distance_stop, c.distance_start,
                c.angle_stop, c.angle_start, c.min_vx, c.max_vx, c.min_wz,
                c.max_wz, c.acceleration, c.angular_acceleration}) {
            if (!std::isfinite(v) || v <= 0) {
                throw std::invalid_argument("Gait controller parameters must be finite and positive");
            }
        }
        if (c.distance_stop >= c.distance_start || c.angle_stop >= c.angle_start ||
            c.angle_start >= 1.5707963267948966 || c.distance <= c.distance_start ||
            c.min_vx < .20 || c.min_wz < .50 || c.min_vx > c.max_vx ||
            c.min_wz > c.max_wz) {
            throw std::invalid_argument("Invalid basic-gait limits or hysteresis");
        }
        config_ = c;
        reset();
    }

    void reset() {
        approaching_ = turning_ = false;
        last_ = geometry_msgs::msg::Twist{};
        state_ = "holding";
    }

    const char* state() const { return state_; }

    geometry_msgs::msg::Twist step(double x, double y, double steering = 0,
                                   double speed_scale = 1, double dt = .1) {
        geometry_msgs::msg::Twist command;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(steering) ||
            !std::isfinite(speed_scale) || !std::isfinite(dt) || dt <= 0 ||
            speed_scale <= 0 || std::hypot(x, y) < 1e-6) {
            reset();
            state_ = "blocked";
            return command;
        }
        dt = std::min(dt, .2);
        const double pi = std::acos(-1.0);
        const double angle = std::remainder(std::atan2(y, x) +
            std::clamp(steering, -.25, .25), 2*pi);
        turning_ = std::abs(angle) > (turning_ ? config_.angle_stop : config_.angle_start);
        if (turning_) {
            approaching_ = false;
            double desired = std::clamp(std::abs(angle), config_.min_wz, config_.max_wz);
            desired = std::min(desired, std::max(config_.min_wz,
                std::abs(last_.angular.z) + config_.angular_acceleration*dt));
            // Insert a zero sample before a commanded direction reversal.
            if (last_.angular.z*angle >= 0) {
                command.angular.z = std::copysign(desired, angle);
            }
            state_ = "turning";
        } else {
            const double error = std::hypot(x, y) - config_.distance;
            approaching_ = error > (approaching_ ? config_.distance_stop : config_.distance_start);
            const double budget = config_.max_vx * std::min(1.0, speed_scale);
            if (approaching_ && budget >= config_.min_vx) {
                command.linear.x = std::min(budget, std::max(config_.min_vx, .5*error));
                command.linear.x = std::min(command.linear.x,
                    std::max(config_.min_vx, last_.linear.x + config_.acceleration*dt));
                state_ = "approaching";
            } else {
                state_ = approaching_ ? "blocked" : "holding";
            }
        }
        last_ = command;
        return command;
    }

private:
    Config config_;
    bool approaching_ = false;
    bool turning_ = false;
    geometry_msgs::msg::Twist last_;
    const char* state_ = "holding";
};

#endif
