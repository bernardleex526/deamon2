// Isolated callback ordering tests. No Bridge, UDP transport, or robot domain.
#include <gtest/gtest.h>
#include <cstdlib>
#define JIE_DEAMON_NO_MAIN
#include "../src/robot_nexus.cpp"

class RobotNexusTestPeer {
public:
    static void pauseTimers(RobotNexusNode& node) {
        node.tracking_timer_->cancel();
        node.direct_control_timer_->cancel();
    }
    static void ageInputs(RobotNexusNode& node) {
        node.last_scan_stamp_ -= 1.;
        node.last_scan_ -= std::chrono::seconds(1);
    }
    static void scan(RobotNexusNode& node, const sensor_msgs::msg::LaserScan::SharedPtr& msg) {
        node.scanCallback(msg);
    }
    static std::int64_t epoch(RobotNexusNode& node) { return node.fault_epoch_; }
    static std::int64_t selection(RobotNexusNode& node) {
        double x, y;
        return node.shared_state_.getTargetSelection(x, y);
    }
};

class M20Node : public testing::Test {
protected:
    void SetUp() override {
        setenv("ROS_DOMAIN_ID", "186", 1);
        setenv("ROS_LOCALHOST_ONLY", "1", 1);
        unsetenv("FASTRTPS_DEFAULT_PROFILES_FILE");
        unsetenv("FASTDDS_DEFAULT_PROFILES_FILE");
        rclcpp::init(0, nullptr);
        executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        rclcpp::NodeOptions options;
        options.parameter_overrides({
            {"active", true}, {"gait_aware_follow", true}, {"enable_web", false},
            {"enable_android", false}, {"enable_actions", false}, {"scan_yaw", 0.0},
            {"follow_distance", 1.2}, {"minimum_target_points", 3},
            {"scan_frame", "lidar_link"}, {"tracking_timeout", .5}});
        node = std::make_shared<RobotNexusNode>(options);
        RobotNexusTestPeer::pauseTimers(*node);
        observer = std::make_shared<rclcpp::Node>("node_contract_observer");
        raw_sub = observer->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
            [this](geometry_msgs::msg::Twist::SharedPtr msg) { ++raw_count; raw = *msg; });
        tracking_sub = observer->create_subscription<std_msgs::msg::String>(
            "/robot_nexus/tracking_status", 10,
            [this](std_msgs::msg::String::SharedPtr) { ++tracking_count; });
        target = observer->create_client<rcl_interfaces::srv::SetParametersAtomically>(
            "/robot_nexus/select_target");
        moving = observer->create_client<std_srvs::srv::SetBool>("/robot_nexus/set_moving");
        executor->add_node(node);
        executor->add_node(observer);
        ASSERT_TRUE(wait([&]() { return moving->service_is_ready() &&
            target->service_is_ready() && observer->count_publishers("/cmd_vel") == 1; }));
    }
    void TearDown() override {
        executor->remove_node(observer);
        executor->remove_node(node);
        target.reset(); moving.reset(); raw_sub.reset(); tracking_sub.reset();
        observer.reset(); node.reset();
        executor.reset();
        rclcpp::shutdown();
    }
    bool wait(const std::function<bool()>& predicate) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while (!predicate() && std::chrono::steady_clock::now() < deadline) {
            executor->spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return predicate();
    }
    bool enable(bool value) {
        if (value) return enableRequest(makeEnableRequest());
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = value;
        auto future = moving->async_send_request(req);
        if (executor->spin_until_future_complete(future, std::chrono::seconds(3)) !=
            rclcpp::FutureReturnCode::SUCCESS) throw std::runtime_error("service timed out");
        return future.get()->success;
    }
    std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Request> makeEnableRequest() {
        auto request = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();
        const auto stamp = node->now().nanoseconds();
        request->parameters = {
            rclcpp::Parameter("stamp_sec", static_cast<int64_t>(stamp/1000000000LL)).to_parameter_msg(),
            rclcpp::Parameter("stamp_nanosec", static_cast<int64_t>(stamp%1000000000LL)).to_parameter_msg(),
            rclcpp::Parameter("selection_id", RobotNexusTestPeer::selection(*node)).to_parameter_msg(),
            rclcpp::Parameter("fault_epoch", RobotNexusTestPeer::epoch(*node)).to_parameter_msg()};
        return request;
    }
    bool enableRequest(const std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Request>& request) {
        auto client = observer->create_client<rcl_interfaces::srv::SetParametersAtomically>(
            "/robot_nexus/enable_follow");
        if (!client->wait_for_service(std::chrono::seconds(3))) throw std::runtime_error("enable unavailable");
        auto future = client->async_send_request(request);
        if (executor->spin_until_future_complete(future, std::chrono::seconds(3)) !=
            rclcpp::FutureReturnCode::SUCCESS) throw std::runtime_error("enable timed out");
        return future.get()->result.successful;
    }
    sensor_msgs::msg::LaserScan::SharedPtr freshScan() {
        auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
        scan->header.frame_id = "lidar_link";
        scan->header.stamp = node->now();
        scan->angle_min = -.01; scan->angle_increment = .01;
        scan->range_min = .1; scan->range_max = 12.;
        scan->ranges = {1.8f, 1.8f, 1.8f};
        return scan;
    }
    void deliverScan(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
        const auto before = raw_count;
        RobotNexusTestPeer::scan(*node, scan);
        ASSERT_TRUE(wait([&]() { return raw_count > before; }));
    }
    void startFollowing() {
        deliverScan(freshScan());
        ASSERT_TRUE(select(selectionRequest()));
        deliverScan(freshScan());
        ASSERT_TRUE(enable(true));
        deliverScan(freshScan());
        ASSERT_TRUE(wait([&]() { return raw.linear.x > .2; }));
    }
    using Selection = rcl_interfaces::srv::SetParametersAtomically;
    std::shared_ptr<Selection::Request> selectionRequest() {
        auto request = std::make_shared<Selection::Request>();
        const auto stamp = node->now();
        request->parameters = {
            rclcpp::Parameter("target_x", 1.8).to_parameter_msg(),
            rclcpp::Parameter("target_y", 0.).to_parameter_msg(),
            rclcpp::Parameter("frame_id", std::string("lidar_link")).to_parameter_msg(),
            rclcpp::Parameter("stamp_sec", static_cast<int64_t>(stamp.nanoseconds()/1000000000LL)).to_parameter_msg(),
            rclcpp::Parameter("stamp_nanosec", static_cast<int64_t>(stamp.nanoseconds()%1000000000LL)).to_parameter_msg(),
            rclcpp::Parameter("fault_epoch", RobotNexusTestPeer::epoch(*node)).to_parameter_msg()};
        return request;
    }
    bool select(const std::shared_ptr<Selection::Request>& request) {
        auto future = target->async_send_request(request);
        if (executor->spin_until_future_complete(future, std::chrono::seconds(3)) !=
            rclcpp::FutureReturnCode::SUCCESS) throw std::runtime_error("selection timed out");
        return future.get()->result.successful;
    }
    std::shared_ptr<RobotNexusNode> node;
    rclcpp::Node::SharedPtr observer;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
    rclcpp::Client<Selection>::SharedPtr target;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr raw_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tracking_sub;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr moving;
    geometry_msgs::msg::Twist raw;
    size_t raw_count = 0, tracking_count = 0;
};

TEST_F(M20Node, RejectedStaleEnableLatchesBeforeFreshScan) {
    startFollowing();
    RobotNexusTestPeer::ageInputs(*node);
    EXPECT_FALSE(enable(true));
    deliverScan(freshScan());
    EXPECT_DOUBLE_EQ(raw.linear.x, 0);
    EXPECT_FALSE(enable(true));
}

TEST_F(M20Node, NegativeStampPublishesZeroInsteadOfThrowing) {
    startFollowing();
    auto scan = freshScan();
    scan->header.stamp.sec = -1;
    EXPECT_NO_THROW(deliverScan(scan));
    EXPECT_DOUBLE_EQ(raw.linear.x, 0);
    EXPECT_FALSE(enable(true));
}

TEST_F(M20Node, FreshScanCannotHideAnExpiredInterval) {
    startFollowing();
    RobotNexusTestPeer::ageInputs(*node);
    deliverScan(freshScan());
    EXPECT_DOUBLE_EQ(raw.linear.x, 0);
    EXPECT_FALSE(enable(true));
}

TEST_F(M20Node, InvalidNanosecondFieldLatches) {
    startFollowing();
    auto scan = freshScan();
    scan->header.stamp.nanosec = 1000000000u;
    EXPECT_NO_THROW(deliverScan(scan));
    EXPECT_DOUBLE_EQ(raw.linear.x, 0);
    deliverScan(freshScan());
    EXPECT_FALSE(enable(true));
}

TEST_F(M20Node, RequestIssuedBeforeFaultCannotReselectAfterFault) {
    startFollowing();
    auto queued = selectionRequest();
    RobotNexusTestPeer::ageInputs(*node);
    EXPECT_FALSE(enable(true));
    deliverScan(freshScan());
    EXPECT_FALSE(select(queued));
    deliverScan(freshScan());
    EXPECT_FALSE(enable(true));
    // Only a new request carrying the newly observed server epoch can recover.
    ASSERT_TRUE(select(selectionRequest()));
    deliverScan(freshScan());
    ASSERT_TRUE(enable(true));
    deliverScan(freshScan());
    EXPECT_TRUE(wait([&]() { return raw.linear.x > .2; }));
}

TEST_F(M20Node, StaleOrMalformedSelectionStopsAndLegacyTopicIsAbsent) {
    startFollowing();
    EXPECT_EQ(node->count_subscribers("/robot_nexus/target"), 0u);
    auto old = selectionRequest();
    old->parameters[3].value.integer_value -= 2;
    EXPECT_FALSE(select(old));
    deliverScan(freshScan());
    EXPECT_FALSE(enable(true));
    auto wrong = selectionRequest();
    wrong->parameters[2].value.string_value = "map";
    EXPECT_FALSE(select(wrong));
    auto duplicate = selectionRequest();
    duplicate->parameters.push_back(duplicate->parameters.front());
    EXPECT_FALSE(select(duplicate));
}

TEST_F(M20Node, QueuedEnableCannotEnableReselectedTarget) {
    startFollowing();
    auto queued = makeEnableRequest();
    RobotNexusTestPeer::ageInputs(*node);
    EXPECT_FALSE(enable(true));
    deliverScan(freshScan());
    ASSERT_TRUE(select(selectionRequest()));
    deliverScan(freshScan());
    EXPECT_FALSE(enableRequest(queued));
    deliverScan(freshScan());
    EXPECT_DOUBLE_EQ(raw.linear.x, 0);
    EXPECT_FALSE(enable(true));
}

TEST_F(M20Node, DisableInvalidatesQueuedEnableAndLegacyEnableIsRejected) {
    startFollowing();
    auto queued = makeEnableRequest();
    ASSERT_TRUE(enable(false));
    EXPECT_FALSE(enableRequest(queued));
    deliverScan(freshScan());
    EXPECT_DOUBLE_EQ(raw.linear.x, 0);
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = true;
    auto future = moving->async_send_request(req);
    ASSERT_EQ(executor->spin_until_future_complete(future, std::chrono::seconds(3)),
              rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_FALSE(future.get()->success);
}
