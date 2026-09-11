/**
 * @file lidar_tracker.hpp
 * @brief 雷达跟随模块 - 处理激光雷达数据并计算跟随速度
 */

#ifndef LIDAR_TRACKER_HPP
#define LIDAR_TRACKER_HPP

#include "common_types.hpp"
#include "kalman_filter.hpp"
#include "gait_follow_controller.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <algorithm>
#include <functional>
#include <limits>

/**
 * @class LidarTracker
 * @brief 雷达跟随处理器
 */
class LidarTracker {
public:
    struct Config {
        double scan_yaw = 3.141592653589793;
        double follow_distance = FOLLOW_DIST;
        double corridor_width = RECTANGLE_WIDTH;
        double frame_front = ROBOT_FRAME_FRONT;
        double frame_back = ROBOT_FRAME_BACK;
        double frame_left = ROBOT_FRAME_LEFT;
        double frame_right = ROBOT_FRAME_RIGHT;
        bool gait_aware = false;
        GaitFollowController::Config gait;
        double apf_clearance = .60;
        double slowdown_clearance = .40;
        double emergency_clearance = .12;
        int minimum_target_points = 1;
        double target_cluster_gap = .12;
        double target_ambiguity_margin = .04;
    };
    struct Diagnostics {
        int target_points = 0;
        double target_x = 0, target_y = 0;
        double corridor = 0, repulse_x = 0, repulse_y = 0;
        geometry_msgs::msg::Twist before_limit = geometry_msgs::msg::Twist();
        bool tracking_valid = false;
        const char* tracking_state = "unselected";
        const char* controller_state = "holding";
    };
    const Diagnostics& diagnostics() const { return diagnostics_; }
    void configure(const Config& config) {
        config_ = config;
        if (config.minimum_target_points < 1) {
            throw std::invalid_argument("minimum_target_points must be positive");
        }
        for (double value : {config.target_cluster_gap, config.target_ambiguity_margin}) {
            if (!std::isfinite(value) || value <= 0) {
                throw std::invalid_argument("Target association thresholds must be positive");
            }
        }
        auto gait = config.gait;
        gait.distance = config.follow_distance;
        gait_controller_.configure(gait);
        if (config.gait_aware && (!std::isfinite(config.apf_clearance) ||
            !std::isfinite(config.slowdown_clearance) ||
            !std::isfinite(config.emergency_clearance) ||
            config.emergency_clearance <= 0 ||
            config.emergency_clearance >= config.slowdown_clearance ||
            config.slowdown_clearance > config.apf_clearance)) {
            throw std::invalid_argument("Invalid body-clearance APF distances");
        }
    }
    using VelocityCallback = std::function<void(const geometry_msgs::msg::Twist&)>;
    using DataBroadcastCallback = std::function<void()>;

    void invalidateTarget(const char* reason = "lost") {
        lost_latched_ = config_.gait_aware;
        double x, y;
        target_selection_ = state_.getTargetSelection(x, y);
        state_.is_moving_enabled.store(false);
        state_.setVelocity(0, 0, 0);
        state_.setPoints({});
        gait_controller_.reset();
        kalman_.reset();
        diagnostics_.tracking_valid = false;
        diagnostics_.target_points = 0;
        diagnostics_.before_limit = geometry_msgs::msg::Twist{};
        diagnostics_.tracking_state = reason;
        diagnostics_.controller_state = "blocked";
        if (velocity_callback_) velocity_callback_(geometry_msgs::msg::Twist{});
    }
    
    LidarTracker(SharedState& state) : state_(state) {
        // 计算机器人在窗口中的像素坐标
        robot_center_pixel_ = cv::Point(
            WINDOW_SIZE / 2,
            WINDOW_SIZE - static_cast<int>(ROBOT_Y_OFFSET_M * METERS_TO_PIXELS)
        );
    }
    
    // 设置速度发布回调
    void setVelocityCallback(VelocityCallback cb) {
        velocity_callback_ = std::move(cb);
    }
    
    // 设置数据广播回调
    void setDataBroadcastCallback(DataBroadcastCallback cb) {
        data_broadcast_callback_ = std::move(cb);
    }
    
    // 设置OpenCV可视化开关
    void setOpenCVEnabled(bool enabled) {
        enable_opencv_ = enabled;
        if (enabled) {
            cv::namedWindow("Follow", cv::WINDOW_AUTOSIZE);
        }
    }
    
    // 设置卡尔曼滤波开关
    void setKalmanEnabled(bool enabled) {
        enable_kalman_ = enabled;
        if (enabled) {
            kalman_.reset();
        }
    }
    
    // 设置卡尔曼滤波参数
    void setKalmanParams(double process_noise, double measurement_noise) {
        kalman_.setProcessNoise(process_noise);
        kalman_.setMeasurementNoise(measurement_noise);
    }
    
    // 销毁OpenCV窗口
    void destroyWindows() {
        if (enable_opencv_) {
            cv::destroyAllWindows();
        }
    }
    
    /**
     * @brief 处理激光扫描数据
     */
    void processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        diagnostics_ = Diagnostics{};
        if (!state_.active.load()) {
            invalidateTarget("inactive");
            return;
        }
        
        double target_x, target_y;
        const auto selection = state_.getTargetSelection(target_x, target_y);
        if (selection != target_selection_) {
            target_selection_ = selection;
            lost_latched_ = false;
            gait_controller_.reset();
            kalman_.reset();
            selected_point_budget_ = 0;
        }
        if (config_.gait_aware && (selection == 0 || lost_latched_)) {
            invalidateTarget(selection == 0 ? "unselected" : "lost");
            return;
        }
        
        cv::Mat image;
        if (enable_opencv_) {
            image = cv::Mat::zeros(TOTAL_WINDOW_HEIGHT, WINDOW_SIZE, CV_8UC3);
            drawBackground(image, target_x, target_y);
        }
        
        if (scan_msg->ranges.empty() || !std::isfinite(scan_msg->angle_min) ||
            !std::isfinite(scan_msg->angle_increment) || scan_msg->angle_increment <= 0 ||
            !std::isfinite(scan_msg->range_min) || !std::isfinite(scan_msg->range_max) ||
            scan_msg->range_min < 0 || scan_msg->range_max <= scan_msg->range_min) {
            invalidateTarget("invalid_scan");
            return;
        }
        const double stamp = scan_msg->header.stamp.sec + scan_msg->header.stamp.nanosec*1e-9;
        scan_dt_ = stamp > previous_stamp_ && previous_stamp_ > 0 ?
            std::min(.2, stamp-previous_stamp_) : .1;
        previous_stamp_ = stamp;
        
        // 处理扫描点
        double centroid_x = 0.0, centroid_y = 0.0;
        int points_in_target_count = 0;
        std::vector<std::pair<double, double>> target_candidates;
        
        double target_vec_x = target_x;
        double target_vec_y = target_y;
        double target_vec_len = std::sqrt(target_vec_x * target_vec_x + target_vec_y * target_vec_y);
        double left_y_min = config_.corridor_width / 2;
        double right_y_min = -config_.corridor_width / 2;
        
        // 势场法：排斥力累积和最近障碍距离
        double repulse_x = 0.0, repulse_y = 0.0;
        double min_obstacle_dist = 999.0;
        
        std::vector<std::pair<double, double>> points;
        points.reserve(scan_msg->ranges.size());
        
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
            float range = scan_msg->ranges[i];
            if (!std::isfinite(range) || range <= 0.0 ||
                range < scan_msg->range_min || range > scan_msg->range_max) continue;
            
            float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            double point_x = range * cos(angle + config_.scan_yaw);
            double point_y = range * sin(angle + config_.scan_yaw);
            
            // 计算到机器人的距离
            double dist_to_robot = std::sqrt(point_x * point_x + point_y * point_y);
            
            // 可视化：根据距离着色
            if (enable_opencv_) {
                cv::Scalar color;
                if (dist_to_robot < APF_EMERGENCY_DIST) {
                    color = cv::Scalar(0, 0, 255);      // 红色：危险
                } else if (dist_to_robot < APF_SLOWDOWN_DIST) {
                    color = cv::Scalar(0, 165, 255);    // 橙色：警告
                } else if (dist_to_robot < APF_INFLUENCE_DIST) {
                    color = cv::Scalar(0, 255, 255);    // 黄色：影响范围内
                } else {
                    color = cv::Scalar(100, 100, 100);  // 灰色：安全
                }
                cv::circle(image, toPixel(point_x, point_y), 2, color, -1, cv::LINE_AA);
            }
            
            // 更新最近障碍距离（排除机器人框架区域）
            bool in_robot_frame = (point_x > -config_.frame_back && point_x < config_.frame_front &&
                                   point_y > -config_.frame_right && point_y < config_.frame_left);
            
            const double dx = point_x - std::clamp(point_x, -config_.frame_back, config_.frame_front);
            const double dy = point_y - std::clamp(point_y, -config_.frame_right, config_.frame_left);
            const double clearance = std::hypot(dx, dy);
            const double obstacle_distance = config_.gait_aware ? clearance : dist_to_robot;
            if (!in_robot_frame && obstacle_distance < min_obstacle_dist) {
                min_obstacle_dist = obstacle_distance;
            }

            const bool target_point = std::hypot(point_x-target_x, point_y-target_y) < TARGET_RADIUS;
            
            // 势场法：计算排斥力（排除机器人框架区域，只考虑前方和侧方障碍）
            if (config_.gait_aware && !in_robot_frame && !target_point &&
                clearance > 1e-6 && clearance < config_.apf_clearance) {
                const double force = APF_REPULSE_GAIN *
                    (1.0/clearance - 1.0/config_.apf_clearance)/(clearance*clearance);
                repulse_x -= force * dx/clearance;
                repulse_y -= force * dy/clearance;
            } else if (!config_.gait_aware && !in_robot_frame &&
                       dist_to_robot < APF_INFLUENCE_DIST && point_x > -0.1) {
                double force = APF_REPULSE_GAIN * (1.0 / dist_to_robot - 1.0 / APF_INFLUENCE_DIST) 
                               / (dist_to_robot * dist_to_robot);
                repulse_x -= force * point_x / dist_to_robot;
                repulse_y -= force * point_y / dist_to_robot;
            }
            
            // 排除机器人框架内的点，不参与目标质心、路径障碍计算，也不广播到Web
            if (in_robot_frame) continue;
            
            // 只有非机器人框架内的点才加入广播列表
            points.emplace_back(point_x, point_y);
            
            double dist_to_target_center = std::sqrt(pow(point_x - target_x, 2) + pow(point_y - target_y, 2));
            
            if (dist_to_target_center < TARGET_RADIUS) {
                target_candidates.emplace_back(point_x, point_y);
                centroid_x += point_x;
                centroid_y += point_y;
                points_in_target_count++;
                continue;
            }
            
            if (target_vec_len > 1e-6) {
                double proj_x = (point_x * target_vec_x + point_y * target_vec_y) / target_vec_len;
                double proj_y = (point_x * -target_vec_y + point_y * target_vec_x) / target_vec_len;
                
                if (proj_x >= 0 && proj_x <= target_vec_len && std::abs(proj_y) <= config_.corridor_width / 2.0) {
                    if (enable_opencv_) {
                        if (proj_y > 0) {
                            cv::line(image, robot_center_pixel_, toPixel(point_x, point_y), cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
                        } else {
                            cv::line(image, robot_center_pixel_, toPixel(point_x, point_y), cv::Scalar(100, 100, 255), 1, cv::LINE_AA);
                        }
                    }
                    
                    if (proj_y > 0 && proj_y < left_y_min) {
                        left_y_min = proj_y;
                    } else if (proj_y <= 0 && proj_y > right_y_min) {
                        right_y_min = proj_y;
                    }
                }
            }
        }
        
        // 限制排斥力幅度
        double repulse_mag = std::sqrt(repulse_x * repulse_x + repulse_y * repulse_y);
        if (repulse_mag > 1.0) {
            repulse_x /= repulse_mag;
            repulse_y /= repulse_mag;
        }
        
        // 更新点云缓存
        state_.setPoints(std::move(points));
        
        if (config_.gait_aware) {
            // Segment only the selected window; never average unrelated objects.
            double best = std::numeric_limits<double>::infinity();
            double second = best;
            size_t best_begin = 0, best_end = 0;
            centroid_x = centroid_y = 0;
            points_in_target_count = 0;
            for (size_t begin = 0; begin < target_candidates.size();) {
                size_t end = begin + 1;
                double sx = target_candidates[begin].first;
                double sy = target_candidates[begin].second;
                while (end < target_candidates.size() && std::hypot(
                    target_candidates[end].first-target_candidates[end-1].first,
                    target_candidates[end].second-target_candidates[end-1].second) <=
                    config_.target_cluster_gap) {
                    sx += target_candidates[end].first;
                    sy += target_candidates[end].second;
                    ++end;
                }
                const int count = static_cast<int>(end-begin);
                if (count >= config_.minimum_target_points) {
                    const double score = std::hypot(sx/count-target_x, sy/count-target_y);
                    if (score < best) {
                        second = best;
                        best = score;
                        centroid_x = sx;
                        centroid_y = sy;
                        points_in_target_count = count;
                        best_begin = begin;
                        best_end = end;
                    } else {
                        second = std::min(second, score);
                    }
                }
                begin = end;
            }
            if (std::isfinite(second) && second-best < config_.target_ambiguity_margin) {
                invalidateTarget("ambiguous_target");
                return;
            }
            if (selected_point_budget_ > 0 && points_in_target_count > selected_point_budget_) {
                std::vector<std::pair<double, double>> selected(
                    target_candidates.begin()+best_begin, target_candidates.begin()+best_end);
                std::sort(selected.begin(), selected.end(), [=](const auto& a, const auto& b) {
                    return std::hypot(a.first-target_x, a.second-target_y) <
                           std::hypot(b.first-target_x, b.second-target_y);
                });
                centroid_x = centroid_y = 0;
                points_in_target_count = selected_point_budget_;
                for (int i = 0; i < points_in_target_count; ++i) {
                    centroid_x += selected[i].first;
                    centroid_y += selected[i].second;
                }
            }
            if (selected_point_budget_ == 0 && points_in_target_count > 0) {
                selected_point_budget_ = points_in_target_count;
            }
        }

        // 更新目标位置为质心（可选卡尔曼滤波平滑）
        if (points_in_target_count >= config_.minimum_target_points) {
            double raw_x = centroid_x / points_in_target_count;
            double raw_y = centroid_y / points_in_target_count;
            
            if (enable_kalman_) {
                double filtered_x, filtered_y;
                kalman_.update(raw_x, raw_y, filtered_x, filtered_y);
                state_.updateTrackedTarget(filtered_x, filtered_y);
            } else {
                state_.updateTrackedTarget(raw_x, raw_y);
            }
        }
        
        // 计算速度
        state_.getTarget(diagnostics_.target_x, diagnostics_.target_y);
        diagnostics_.target_points = points_in_target_count;
        diagnostics_.repulse_x = repulse_x;
        diagnostics_.repulse_y = repulse_y;
        diagnostics_.tracking_valid = points_in_target_count >= config_.minimum_target_points;
        diagnostics_.tracking_state = diagnostics_.tracking_valid ? "tracking" : "lost";
        if (config_.gait_aware && !diagnostics_.tracking_valid) {
            invalidateTarget();
            return;
        }
        geometry_msgs::msg::Twist cmd_vel_msg;
        int mode = state_.control_mode.load();
        
        if (mode == MODE_DIRECT) {
            double vx, vy, wz;
            state_.getDirectCmd(vx, vy, wz);
            cmd_vel_msg.linear.x = vx;
            cmd_vel_msg.linear.y = vy;
            cmd_vel_msg.angular.z = wz;
        }
        else if (mode == MODE_FOLLOW && diagnostics_.tracking_valid) {
            if (config_.gait_aware && !state_.is_moving_enabled.load()) {
                gait_controller_.reset();
            } else {
            state_.getTarget(target_x, target_y);
            calculateFollowVelocity(cmd_vel_msg, target_x, target_y, 
                                    left_y_min, right_y_min,
                                    repulse_x, repulse_y, min_obstacle_dist);
            }
        }
        diagnostics_.controller_state = config_.gait_aware ? gait_controller_.state() : "legacy";
        
        // 缓存速度
        state_.setVelocity(cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, cmd_vel_msg.angular.z);
        
        // 发布速度
        publishVelocity(cmd_vel_msg, mode);
        
        // OpenCV可视化
        if (enable_opencv_) {
            drawSpeedDisplay(image, cmd_vel_msg);
            std::string status = state_.is_moving_enabled.load() ? "MOVING" : "STOPPED";
            cv::putText(image, status, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                       state_.is_moving_enabled.load() ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
            cv::imshow("Follow", image);
            cv::waitKey(1);
        }
        
        // 广播数据
        if (data_broadcast_callback_) {
            data_broadcast_callback_();
        }
    }

private:
    SharedState& state_;
    Config config_;
    GaitFollowController gait_controller_;
    std::uint64_t target_selection_ = 0;
    bool lost_latched_ = false;
    int selected_point_budget_ = 0;
    double previous_stamp_ = 0;
    double scan_dt_ = .1;
    Diagnostics diagnostics_;
    cv::Point robot_center_pixel_;
    bool enable_opencv_ = false;
    bool enable_kalman_ = false;
    KalmanFilter2D kalman_;
    VelocityCallback velocity_callback_;
    DataBroadcastCallback data_broadcast_callback_;
    
    // 坐标转换
    cv::Point toPixel(double robot_x, double robot_y) const {
        int px = robot_center_pixel_.x - static_cast<int>(robot_y * METERS_TO_PIXELS);
        int py = robot_center_pixel_.y - static_cast<int>(robot_x * METERS_TO_PIXELS);
        return cv::Point(px, py);
    }
    
    // 绘制背景
    void drawBackground(cv::Mat& image, double target_x, double target_y) {
        double dist_to_target = std::sqrt(target_x * target_x + target_y * target_y);
        if (dist_to_target > 1e-6) {
            double angle_to_target = atan2(target_y, target_x);
            double half_width = config_.corridor_width / 2.0;
            
            cv::Point2f corners_robot[4];
            corners_robot[0] = cv::Point2f(0 - half_width * sin(angle_to_target), 0 + half_width * cos(angle_to_target));
            corners_robot[1] = cv::Point2f(0 + half_width * sin(angle_to_target), 0 - half_width * cos(angle_to_target));
            corners_robot[2] = cv::Point2f(target_x + half_width * sin(angle_to_target), target_y - half_width * cos(angle_to_target));
            corners_robot[3] = cv::Point2f(target_x - half_width * sin(angle_to_target), target_y + half_width * cos(angle_to_target));
            
            std::vector<cv::Point> corners_pixel;
            for (int i = 0; i < 4; ++i) {
                corners_pixel.push_back(toPixel(corners_robot[i].x, corners_robot[i].y));
            }
            cv::fillConvexPoly(image, corners_pixel, cv::Scalar(50, 50, 50), cv::LINE_AA);
        }
        
        cv::circle(image, robot_center_pixel_, 8, cv::Scalar(255, 200, 200), -1, cv::LINE_AA);
        const std::vector<double> scales = {1.0, 2.0, 3.0, 4.0};
        for (double dist : scales) {
            int radius_px = static_cast<int>(dist * METERS_TO_PIXELS);
            cv::circle(image, robot_center_pixel_, radius_px, cv::Scalar(128, 128, 128), 1, cv::LINE_AA);
        }
        
        cv::Point target_center_px = toPixel(target_x, target_y);
        int target_radius_px = static_cast<int>(TARGET_RADIUS * METERS_TO_PIXELS);
        cv::circle(image, target_center_px, target_radius_px, cv::Scalar(255, 0, 255), 1, cv::LINE_AA);
        cv::line(image, robot_center_pixel_, target_center_px, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    }
    
    // 计算跟随速度（带势场避障）
    void calculateFollowVelocity(geometry_msgs::msg::Twist& cmd, double target_x, double target_y,
                                  double left_y_min, double right_y_min,
                                  double repulse_x, double repulse_y, double min_obstacle_dist) {
        if (config_.gait_aware) {
            diagnostics_.corridor = (left_y_min + right_y_min) / 2.0;
            const double steering = std::atan2(diagnostics_.corridor + repulse_y,
                                               std::max(.5, std::hypot(target_x, target_y)));
            const double budget = std::clamp((min_obstacle_dist-config_.emergency_clearance) /
                (config_.slowdown_clearance-config_.emergency_clearance), 0.0, 1.0);
            cmd = gait_controller_.step(target_x, target_y, steering, budget, scan_dt_);
            diagnostics_.before_limit = cmd;
            return;
        }
        // 紧急停止检查
        if (min_obstacle_dist < APF_EMERGENCY_DIST) {
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.angular.z = 0.0;
            return;
        }
        
        // 前后运动控制（带死区）
        double dist_error = target_x - config_.follow_distance;
        if (std::abs(dist_error) < 0.05) {
            cmd.linear.x = 0.0;
        } else {
            cmd.linear.x = dist_error * LINEAR_SCALE_FACTOR;
            if (cmd.linear.x < 0) cmd.linear.x *= 0.8;
            
            double min_speed = 0.06;
            if (std::abs(cmd.linear.x) < min_speed) {
                cmd.linear.x = (cmd.linear.x > 0) ? min_speed : -min_speed;
            }
        }

        
        // 旋转运动控制（带死区）
        double angle_error = atan2(target_y, target_x);
        if (std::abs(angle_error) < 0.1) {
            cmd.angular.z = 0.0;
        } else {
            cmd.angular.z = angle_error * ANGULAR_SCALE_FACTOR;
        }
        
        // 横向运动控制（带死区）
        double lateral_error = (left_y_min + right_y_min) / 2.0;
        if (std::abs(lateral_error) > 1.0) lateral_error = 0.0;
        if (std::abs(lateral_error) < 0.03) {
            lateral_error = 0.0;
        } else {
            lateral_error *= LINEAR_Y_SCALE_FACTOR;
        }
        diagnostics_.corridor = lateral_error;
        cmd.linear.y = lateral_error;
        
        // 融合势场排斥力
        cmd.linear.x += repulse_x;
        cmd.linear.y += repulse_y;
        
        // 接近障碍时减速
        if (min_obstacle_dist < APF_SLOWDOWN_DIST) {
            double slowdown_factor = (min_obstacle_dist - APF_EMERGENCY_DIST) 
                                   / (APF_SLOWDOWN_DIST - APF_EMERGENCY_DIST);
            slowdown_factor = std::clamp(slowdown_factor, 0.1, 1.0);
            cmd.linear.x *= slowdown_factor;
        }
        
        diagnostics_.before_limit = cmd;
        // 限制速度
        cmd.linear.x = std::clamp(cmd.linear.x, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
        cmd.linear.y = std::clamp(cmd.linear.y, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
        cmd.angular.z = std::clamp(cmd.angular.z, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
    }
    
    // 发布速度
    void publishVelocity(const geometry_msgs::msg::Twist& cmd, int mode) {
        if (!velocity_callback_) return;
        
        if (state_.is_moving_enabled.load() || mode == MODE_DIRECT) {
            if (mode == MODE_DIRECT) {
                velocity_callback_(cmd);
            } else if (state_.is_moving_enabled.load()) {
                velocity_callback_(cmd);
            } else {
                geometry_msgs::msg::Twist zero_vel;
                velocity_callback_(zero_vel);
            }
        } else {
            geometry_msgs::msg::Twist zero_vel;
            velocity_callback_(zero_vel);
        }
    }
    
    // 绘制速度显示
    void drawSpeedDisplay(cv::Mat& image, const geometry_msgs::msg::Twist& cmd_vel) {
        int center_x = WINDOW_SIZE / 2;
        int center_y = WINDOW_SIZE + SPEED_DISPLAY_HEIGHT / 2;
        
        cv::line(image, cv::Point(0, WINDOW_SIZE), cv::Point(WINDOW_SIZE, WINDOW_SIZE),
                 cv::Scalar(80, 80, 80), 1);
        
        cv::line(image, cv::Point(center_x - SPEED_BAR_LENGTH - 10, center_y),
                 cv::Point(center_x + SPEED_BAR_LENGTH + 10, center_y),
                 cv::Scalar(60, 60, 60), 1);
        cv::line(image, cv::Point(center_x, center_y - SPEED_BAR_LENGTH - 10),
                 cv::Point(center_x, center_y + SPEED_BAR_LENGTH + 10),
                 cv::Scalar(60, 60, 60), 1);
        
        double vx = std::clamp(cmd_vel.linear.x, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
        double vy = std::clamp(cmd_vel.linear.y, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
        double wz = std::clamp(cmd_vel.angular.z, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
        
        int bar_x = static_cast<int>((vx / MAX_LINEAR_SPEED) * SPEED_BAR_LENGTH * 4);
        int bar_y = static_cast<int>((vy / MAX_LINEAR_SPEED) * SPEED_BAR_LENGTH * 4);
        
        if (std::abs(bar_x) > 1) {
            cv::line(image, cv::Point(center_x, center_y),
                     cv::Point(center_x, center_y - bar_x),
                     cv::Scalar(0, 0, 255), 4, cv::LINE_AA);
        }
        
        if (std::abs(bar_y) > 1) {
            cv::line(image, cv::Point(center_x, center_y),
                     cv::Point(center_x - bar_y, center_y),
                     cv::Scalar(0, 255, 0), 4, cv::LINE_AA);
        }
        
        cv::circle(image, cv::Point(center_x, center_y), 5, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
        
        int arc_center_x = center_x + 120;
        cv::circle(image, cv::Point(arc_center_x, center_y), SPEED_ARC_RADIUS,
                   cv::Scalar(60, 60, 60), 1, cv::LINE_AA);
        
        if (std::abs(wz) > 0.01) {
            double start_angle = -90;
            double arc_angle = -(wz / MAX_ANGULAR_SPEED) * 180;
            double end_angle = start_angle + arc_angle;
            
            double draw_start = start_angle;
            double draw_end = end_angle;
            if (arc_angle < 0) {
                std::swap(draw_start, draw_end);
            }
            
            cv::ellipse(image, cv::Point(arc_center_x, center_y), 
                       cv::Size(SPEED_ARC_RADIUS, SPEED_ARC_RADIUS),
                       0, draw_start, draw_end,
                       cv::Scalar(255, 150, 0), 4, cv::LINE_AA);
        }
        
        char buf[64];
        snprintf(buf, sizeof(buf), "Vx:%.2f", cmd_vel.linear.x);
        cv::putText(image, buf, cv::Point(10, WINDOW_SIZE + 25),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
        
        snprintf(buf, sizeof(buf), "Vy:%.2f", cmd_vel.linear.y);
        cv::putText(image, buf, cv::Point(10, WINDOW_SIZE + 50),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        
        snprintf(buf, sizeof(buf), "Wz:%.2f", cmd_vel.angular.z);
        cv::putText(image, buf, cv::Point(10, WINDOW_SIZE + 75),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 150, 0), 1);
    }
};

#endif // LIDAR_TRACKER_HPP
