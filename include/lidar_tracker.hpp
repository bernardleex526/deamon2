/**
 * @file lidar_tracker.hpp
 * @brief 雷达跟随模块 - 处理激光雷达数据并计算跟随速度
 */

#ifndef LIDAR_TRACKER_HPP
#define LIDAR_TRACKER_HPP

#include "common_types.hpp"
#include "kalman_filter.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <algorithm>
#include <functional>

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
    };
    void configure(const Config& config) { config_ = config; }
    using VelocityCallback = std::function<void(const geometry_msgs::msg::Twist&)>;
    using DataBroadcastCallback = std::function<void()>;
    
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
        if (!state_.active.load()) {
            return;
        }
        
        double target_x, target_y;
        state_.getTarget(target_x, target_y);
        
        cv::Mat image;
        if (enable_opencv_) {
            image = cv::Mat::zeros(TOTAL_WINDOW_HEIGHT, WINDOW_SIZE, CV_8UC3);
            drawBackground(image, target_x, target_y);
        }
        
        if (scan_msg->ranges.empty()) {
            return;
        }
        
        // 处理扫描点
        double centroid_x = 0.0, centroid_y = 0.0;
        int points_in_target_count = 0;
        
        double target_vec_x = target_x;
        double target_vec_y = target_y;
        double target_vec_len = std::sqrt(target_vec_x * target_vec_x + target_vec_y * target_vec_y);
        double left_y_min = -config_.corridor_width / 2;
        double right_y_min = config_.corridor_width / 2;
        
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
            
            if (!in_robot_frame && dist_to_robot < min_obstacle_dist) {
                min_obstacle_dist = dist_to_robot;
            }
            
            // 势场法：计算排斥力（排除机器人框架区域，只考虑前方和侧方障碍）
            if (!in_robot_frame && dist_to_robot < APF_INFLUENCE_DIST && point_x > -0.1) {
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
                    
                    if (proj_y > 0 && proj_y > left_y_min) {
                        left_y_min = proj_y;
                    } else if (proj_y <= 0 && proj_y < right_y_min) {
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
        
        // 更新目标位置为质心（可选卡尔曼滤波平滑）
        if (points_in_target_count > 0) {
            double raw_x = centroid_x / points_in_target_count;
            double raw_y = centroid_y / points_in_target_count;
            
            if (enable_kalman_) {
                double filtered_x, filtered_y;
                kalman_.update(raw_x, raw_y, filtered_x, filtered_y);
                state_.setTarget(filtered_x, filtered_y);
            } else {
                state_.setTarget(raw_x, raw_y);
            }
        }
        
        // 计算速度
        geometry_msgs::msg::Twist cmd_vel_msg;
        int mode = state_.control_mode.load();
        
        if (mode == MODE_DIRECT) {
            double vx, vy, wz;
            state_.getDirectCmd(vx, vy, wz);
            cmd_vel_msg.linear.x = vx;
            cmd_vel_msg.linear.y = vy;
            cmd_vel_msg.angular.z = wz;
        }
        else if (mode == MODE_FOLLOW && points_in_target_count > 0) {
            state_.getTarget(target_x, target_y);
            calculateFollowVelocity(cmd_vel_msg, target_x, target_y, 
                                    left_y_min, right_y_min,
                                    repulse_x, repulse_y, min_obstacle_dist);
        }
        
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
        double lateral_error = -(left_y_min + right_y_min);
        if (std::abs(lateral_error) > 1.0) lateral_error = 0.0;
        if (std::abs(lateral_error) < 0.03) {
            cmd.linear.y = 0.0;
        } else {
            cmd.linear.y = lateral_error * LINEAR_Y_SCALE_FACTOR;
        }
        
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
