#include <gtest/gtest.h>
#include <limits>
#include "lidar_tracker.hpp"
#include "direct_control.hpp"

TEST(M20Tracker, BodyForwardAndLostTargetStop) {
    SharedState state;
    state.active = true;
    state.is_moving_enabled = true;
    state.setTarget(1.8, 0.0);
    LidarTracker tracker(state);
    LidarTracker::Config config;
    config.scan_yaw = 0.0;
    config.follow_distance = 1.2;
    tracker.configure(config);
    geometry_msgs::msg::Twist output;
    tracker.setVelocityCallback([&](const geometry_msgs::msg::Twist& cmd) { output = cmd; });
    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    scan->range_min = 0.1;
    scan->range_max = 12.0;
    scan->angle_min = 0.0;
    scan->angle_increment = 0.01;
    scan->ranges = {1.8f};
    tracker.processScan(scan);
    EXPECT_NEAR(output.linear.x, 0.3, 1e-5);
    EXPECT_NEAR(output.angular.z, 0.0, 1e-5);
    scan->ranges = {5.0f};
    tracker.processScan(scan);
    EXPECT_DOUBLE_EQ(output.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(output.angular.z, 0.0);
}

TEST(M20Tracker, InvalidRangesDoNotBecomeTargets) {
    SharedState state;
    state.active = true;
    state.is_moving_enabled = true;
    LidarTracker tracker(state);
    geometry_msgs::msg::Twist output;
    output.linear.x = 1.0;
    tracker.setVelocityCallback([&](const geometry_msgs::msg::Twist& cmd) { output = cmd; });
    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    scan->range_min = 0.1;
    scan->range_max = 12.0;
    scan->angle_increment = 0.01;
    scan->ranges = {0.0f, -1.0f, std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::quiet_NaN(), 30.0f};
    tracker.processScan(scan);
    EXPECT_DOUBLE_EQ(output.linear.x, 0.0);
    EXPECT_TRUE(state.getPoints().empty());
}

TEST(M20Direct, ExpiredInputIsNotRefreshedByTimer) {
    SharedState state;
    state.control_mode = MODE_DIRECT;
    DirectController direct(state);
    geometry_msgs::msg::Twist output;
    direct.setVelocityCallback([&](const geometry_msgs::msg::Twist& cmd) { output = cmd; });
    direct.processDirectCmd(0.3, -0.2, 0.4);
    direct.timerCallback();
    EXPECT_DOUBLE_EQ(output.linear.x, 0.3);
    state.direct_received -= std::chrono::seconds(1);
    direct.timerCallback();
    EXPECT_DOUBLE_EQ(output.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(output.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(output.angular.z, 0.0);
    direct.timerCallback();
    EXPECT_DOUBLE_EQ(output.linear.x, 0.0);
}
