#include <gtest/gtest.h>
#include <limits>
#include "lidar_tracker.hpp"
#include "direct_control.hpp"

static sensor_msgs::msg::LaserScan::SharedPtr pointsScan(
        const std::vector<std::pair<double, double>>& points) {
    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    scan->range_min = 0.1;
    scan->range_max = 12.0;
    scan->angle_min = -3.141592653589793;
    scan->angle_increment = 2 * 3.141592653589793 / 7200;
    scan->ranges.assign(7200, std::numeric_limits<float>::infinity());
    for (auto p : points) {
        int i = std::lround((std::atan2(p.second, p.first) - scan->angle_min) /
                             scan->angle_increment);
        if (i >= 0 && i < 7200) scan->ranges[i] = std::hypot(p.first, p.second);
    }
    return scan;
}

TEST(M20Tracker, BasicGaitApproachesInsideOldDeadZone) {
    SharedState state;
    state.active = true;
    state.is_moving_enabled = true;
    state.setTarget(1.5, 0);
    LidarTracker tracker(state);
    LidarTracker::Config config;
    config.scan_yaw = 0;
    config.follow_distance = 1.2;
    config.gait_aware = true;
    config.frame_front = config.frame_back = .41;
    config.frame_left = config.frame_right = .253;
    tracker.configure(config);
    geometry_msgs::msg::Twist output;
    tracker.setVelocityCallback([&](const auto& v) { output = v; });
    tracker.processScan(pointsScan({{1.5, 0}}));
    EXPECT_GE(output.linear.x, .22);
    EXPECT_LE(output.linear.x, .30);
    EXPECT_DOUBLE_EQ(output.linear.y, 0);
}

TEST(M20Tracker, BasicGaitApfSeesObstacleOutsideBody) {
    SharedState state;
    state.active = true;
    state.is_moving_enabled = true;
    state.setTarget(2, 0);
    LidarTracker tracker(state);
    LidarTracker::Config c;
    c.scan_yaw = 0;
    c.gait_aware = true;
    c.frame_front = c.frame_back = .41;
    c.frame_left = c.frame_right = .253;
    tracker.configure(c);
    tracker.processScan(pointsScan({{2, 0}, {.7, .35}}));
    EXPECT_LT(tracker.diagnostics().repulse_y, 0);
}

TEST(M20Tracker, BasicGaitLossRequiresExplicitReselection) {
    SharedState state;
    state.active = true;
    state.is_moving_enabled = true;
    state.setTarget(1.8, 0);
    LidarTracker tracker(state);
    LidarTracker::Config c;
    c.scan_yaw = 0;
    c.gait_aware = true;
    c.follow_distance = 1.2;
    tracker.configure(c);
    geometry_msgs::msg::Twist cmd;
    tracker.setVelocityCallback([&](const auto& v) { cmd = v; });
    auto target = pointsScan({{1.8, 0}});
    tracker.processScan(target);
    ASSERT_GT(cmd.linear.x, 0);
    tracker.processScan(pointsScan({{5, 0}}));
    EXPECT_DOUBLE_EQ(cmd.linear.x, 0);
    state.is_moving_enabled = true;
    tracker.processScan(target);
    EXPECT_DOUBLE_EQ(cmd.linear.x, 0);
    state.setTarget(1.8, 0);
    state.is_moving_enabled = true;
    tracker.processScan(target);
    EXPECT_GT(cmd.linear.x, 0);
}

TEST(M20Tracker, BasicGaitRejectsUnselectedSparseAndInvalidScans) {
    SharedState state;
    state.active = true;
    state.is_moving_enabled = true;
    LidarTracker tracker(state);
    LidarTracker::Config c;
    c.scan_yaw = 0;
    c.gait_aware = true;
    c.minimum_target_points = 3;
    tracker.configure(c);
    tracker.processScan(pointsScan({{1.8, 0}}));
    EXPECT_FALSE(tracker.diagnostics().tracking_valid);
    state.setTarget(1.8, 0);
    tracker.processScan(pointsScan({{1.8, 0}}));
    EXPECT_FALSE(tracker.diagnostics().tracking_valid);
    state.setTarget(1.8, 0);
    auto scan = pointsScan({{1.8, -.04}, {1.8, 0}, {1.8, .04}});
    tracker.processScan(scan);
    EXPECT_TRUE(tracker.diagnostics().tracking_valid);
    scan->angle_increment = std::nan("");
    tracker.processScan(scan);
    EXPECT_FALSE(tracker.diagnostics().tracking_valid);
    EXPECT_FALSE(state.is_moving_enabled.load());
}

TEST(M20Tracker, BasicGaitDoesNotAverageTwoAmbiguousTargets) {
    SharedState state;
    state.active = true;
    state.is_moving_enabled = true;
    state.setTarget(1.8, 0);
    LidarTracker tracker(state);
    LidarTracker::Config c;
    c.scan_yaw = 0;
    c.gait_aware = true;
    c.minimum_target_points = 3;
    tracker.configure(c);
    tracker.processScan(pointsScan({{1.8, -.22}, {1.8, -.2}, {1.8, -.18},
                                   {1.8, .18}, {1.8, .2}, {1.8, .22}}));
    EXPECT_FALSE(tracker.diagnostics().tracking_valid);
}

TEST(M20Tracker, FaultAfterSelectionRequiresANewerSelection) {
    SharedState state;
    state.active = true;
    LidarTracker tracker(state);
    LidarTracker::Config c;
    c.scan_yaw = 0;
    c.gait_aware = true;
    tracker.configure(c);
    state.setTarget(1.8, 0);
    tracker.invalidateTarget("scan_stale");
    tracker.processScan(pointsScan({{1.8, 0}}));
    EXPECT_FALSE(tracker.diagnostics().tracking_valid);
    state.setTarget(1.8, 0);
    tracker.processScan(pointsScan({{1.8, 0}}));
    EXPECT_TRUE(tracker.diagnostics().tracking_valid);
}

TEST(M20Tracker, TemporaryClusterMergeDoesNotDragSelectedTarget) {
    SharedState state;
    state.active = true;
    state.setTarget(1.8, -.2);
    LidarTracker tracker(state);
    LidarTracker::Config c;
    c.scan_yaw = 0;
    c.gait_aware = true;
    c.minimum_target_points = 3;
    tracker.configure(c);
    tracker.processScan(pointsScan({{1.8, -.22}, {1.8, -.2}, {1.8, -.18}}));
    ASSERT_TRUE(tracker.diagnostics().tracking_valid);
    tracker.processScan(pointsScan({{1.8, -.22}, {1.8, -.2}, {1.8, -.18},
                                   {1.8, -.1}, {1.8, 0}, {1.8, .08}}));
    EXPECT_TRUE(tracker.diagnostics().tracking_valid);
    EXPECT_NEAR(tracker.diagnostics().target_y, -.2, .02);
}

TEST(M20Gait, StopsNearSetpointWithoutChatterOrBacking) {
    GaitFollowController controller;
    GaitFollowController::Config c;
    controller.configure(c);
    EXPECT_GE(controller.step(1.5, 0).linear.x, .22);
    EXPECT_GE(controller.step(1.28, 0).linear.x, .22);
    EXPECT_DOUBLE_EQ(controller.step(1.25, 0).linear.x, 0);
    EXPECT_DOUBLE_EQ(controller.step(1.28, 0).linear.x, 0);
    EXPECT_DOUBLE_EQ(controller.step(1.0, 0).linear.x, 0);
    EXPECT_GE(controller.step(1.32, 0).linear.x, .22);
}

TEST(M20Gait, TurnsBeforeAdvancingAndHonorsObstacleBudget) {
    GaitFollowController controller;
    controller.configure({});
    auto cmd = controller.step(1.5, .4);
    EXPECT_DOUBLE_EQ(cmd.linear.x, 0);
    EXPECT_DOUBLE_EQ(cmd.linear.y, 0);
    EXPECT_GE(cmd.angular.z, .52);
    EXPECT_LE(cmd.angular.z, .60);
    cmd = controller.step(1.5, .01);
    EXPECT_GE(cmd.linear.x, .22);
    EXPECT_DOUBLE_EQ(cmd.angular.z, 0);
    EXPECT_DOUBLE_EQ(controller.step(1.5, 0, 0, .5).linear.x, 0);
    cmd = controller.step(1.5, .4, 0, 0);
    EXPECT_DOUBLE_EQ(cmd.angular.z, 0);
    EXPECT_DOUBLE_EQ(cmd.linear.x, 0);
}

TEST(M20Gait, IdealKinematicsConvergesToTolerance) {
    GaitFollowController controller;
    controller.configure({});
    double distance = 1.82;
    for (int i = 0; i < 300; ++i) {
        const auto cmd = controller.step(distance, 0);
        EXPECT_TRUE(cmd.linear.x == 0 || (cmd.linear.x >= .22 && cmd.linear.x <= .30));
        distance -= cmd.linear.x * .1;
    }
    EXPECT_NEAR(distance, 1.2, .06);
    EXPECT_DOUBLE_EQ(controller.step(distance, 0).linear.x, 0);
}

TEST(M20Gait, InvalidConfigurationAndInputFailClosed) {
    GaitFollowController controller;
    auto c = GaitFollowController::Config{};
    c.min_vx = .1;
    EXPECT_THROW(controller.configure(c), std::invalid_argument);
    controller.configure({});
    EXPECT_DOUBLE_EQ(controller.step(std::nan(""), 0).linear.x, 0);
}

TEST(M20Tracker, OneSidedCorridorDoesNotReverseAvoidance) {
    SharedState state;
    state.active = true;
    state.is_moving_enabled = true;
    state.setTarget(2.0, 0.0);
    LidarTracker tracker(state);
    LidarTracker::Config config;
    config.scan_yaw = 0;
    config.follow_distance = 1.2;
    config.corridor_width = 0.8;
    tracker.configure(config);
    geometry_msgs::msg::Twist out;
    tracker.setVelocityCallback([&](const auto& v) { out = v; });
    tracker.processScan(pointsScan({{2.0, 0.0}, {1.0, -0.2}}));
    EXPECT_GE(out.linear.y, 0.0);  // Right obstacle must not push us right.
    EXPECT_LE(std::abs(out.linear.y), 0.11);
}

TEST(M20Tracker, EmptyScanPublishesStop) {
    SharedState state;
    state.active = true;
    state.is_moving_enabled = true;
    state.setTarget(2.0, 0.0);
    LidarTracker tracker(state);
    LidarTracker::Config config;
    config.scan_yaw = 0;
    tracker.configure(config);
    geometry_msgs::msg::Twist out;
    tracker.setVelocityCallback([&](const auto& v) { out = v; });
    tracker.processScan(pointsScan({{2.0, 0.0}}));
    ASSERT_GT(out.linear.x, 0);
    auto scan = pointsScan({});
    scan->ranges.clear();
    tracker.processScan(scan);
    EXPECT_DOUBLE_EQ(out.linear.x, 0);
}

TEST(M20Tracker, TwoOclockUsesYawNotTargetDirectedLateralVelocity) {
    SharedState state;
    state.active = true;
    state.is_moving_enabled = true;
    state.setTarget(1.5, -std::sqrt(6.75));
    LidarTracker tracker(state);
    LidarTracker::Config config;
    config.scan_yaw = 0;
    config.follow_distance = 1.2;
    tracker.configure(config);
    geometry_msgs::msg::Twist out;
    tracker.setVelocityCallback([&](const auto& v) { out = v; });
    tracker.processScan(pointsScan({{1.5, -std::sqrt(6.75)}}));
    EXPECT_GT(out.linear.x, 0);
    EXPECT_DOUBLE_EQ(out.linear.y, 0);
    EXPECT_LT(out.angular.z, 0);
    EXPECT_NEAR(out.linear.x, .15, .001);
}

TEST(M20Tracker, PreservesUpstreamFrontDistanceResponse) {
    SharedState state;
    state.active = true;
    state.is_moving_enabled = true;
    state.setTarget(1.5, 0);
    LidarTracker tracker(state);
    LidarTracker::Config config;
    config.scan_yaw = 0;
    config.follow_distance = 2;
    tracker.configure(config);
    geometry_msgs::msg::Twist out;
    tracker.setVelocityCallback([&](const auto& v) { out = v; });
    tracker.processScan(pointsScan({{1.5, 0}}));
    EXPECT_NEAR(out.linear.x, -.2, .001);
    EXPECT_DOUBLE_EQ(out.linear.y, 0);
}

TEST(M20Tracker, CorridorUsesNearestBoundaryAndIsSymmetric) {
    for (double side : {-1.0, 1.0}) {
        SharedState state;
        state.active = true;
        state.is_moving_enabled = true;
        state.setTarget(2, 0);
        LidarTracker tracker(state);
        LidarTracker::Config config;
        config.scan_yaw = 0;
        config.corridor_width = .8;
        tracker.configure(config);
        tracker.processScan(pointsScan({{2, 0}, {1, side*.2}, {1.2, side*.35}}));
        EXPECT_NEAR(tracker.diagnostics().corridor, -side*.1, .001);
    }
}

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
