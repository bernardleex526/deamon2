/**
 * @file direct_control.hpp
 * @brief 直接控制模块 - 处理直接速度指令
 */

#ifndef DIRECT_CONTROL_HPP
#define DIRECT_CONTROL_HPP

#include "common_types.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <functional>

/**
 * @class DirectController
 * @brief 直接控制处理器
 */
class DirectController {
public:
    using VelocityCallback = std::function<void(const geometry_msgs::msg::Twist&)>;
    
    DirectController(SharedState& state) : state_(state) {}
    
    // 设置速度发布回调
    void setVelocityCallback(VelocityCallback cb) {
        velocity_callback_ = std::move(cb);
    }
    
    /**
     * @brief 定时回调，发布直接控制速度
     */
    void timerCallback() {
        // 只在直接控制模式下处理
        if (state_.control_mode.load() != MODE_DIRECT) {
            return;
        }
        
        geometry_msgs::msg::Twist cmd_vel_msg;
        double vx, vy, wz;
        state_.getDirectCmd(vx, vy, wz);
        cmd_vel_msg.linear.x = vx;
        cmd_vel_msg.linear.y = vy;
        cmd_vel_msg.angular.z = wz;
        
        // 缓存速度
        state_.setVelocity(vx, vy, wz);
        
        // 发布速度
        if (velocity_callback_) {
            velocity_callback_(cmd_vel_msg);
        }
    }
    
    /**
     * @brief 处理直接控制指令
     */
    void processDirectCmd(double vx, double vy, double wz) {
        state_.setDirectCmd(vx, vy, wz);
        
        // 自动切换到直接控制模式
        if (state_.control_mode.load() != MODE_DIRECT) {
            state_.control_mode.store(MODE_DIRECT);
        }
    }

private:
    SharedState& state_;
    VelocityCallback velocity_callback_;
};

#endif // DIRECT_CONTROL_HPP
