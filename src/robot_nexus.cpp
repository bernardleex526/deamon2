/**
 * @file robot_nexus.cpp
 * @brief ROS2机器人中枢控制节点
 * 
 * 功能：
 * - 订阅激光雷达数据，追踪目标
 * - 通过UDP发送雷达数据到Android App
 * - 接收Android App的控制指令
 * - Web可视化显示
 * - 可选的OpenCV可视化显示（调试用）
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <map>
#include <thread>
#include <chrono>
#include <stdexcept>
#include <sstream>
#include <iomanip>

#include "common_types.hpp"
#include "lidar_tracker.hpp"
#include "direct_control.hpp"
#include "web_comm.hpp"
#include "android_comm.hpp"

/**
 * @class RobotNexusNode
 * @brief 机器人中枢控制ROS2节点
 */
class RobotNexusNode : public rclcpp::Node
{
    friend class RobotNexusTestPeer;
public:
    explicit RobotNexusNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("robot_nexus", options),
          lidar_tracker_(shared_state_),
          direct_controller_(shared_state_),
          web_comm_(shared_state_),
          android_comm_(shared_state_)
    {
        // 声明参数
        this->declare_parameter<bool>("active", false);
        this->declare_parameter<bool>("enable_opencv", false);
        this->declare_parameter<bool>("enable_web", true);
        this->declare_parameter<bool>("enable_kalman", false);
        this->declare_parameter<std::string>("web_root", "");
        this->declare_parameter<bool>("enable_android", true);
        this->declare_parameter<bool>("enable_actions", true);
        this->declare_parameter<double>("scan_yaw", 3.141592653589793);
        this->declare_parameter<double>("follow_distance", FOLLOW_DIST);
        this->declare_parameter<double>("corridor_width", RECTANGLE_WIDTH);
        this->declare_parameter<double>("frame_front", ROBOT_FRAME_FRONT);
        this->declare_parameter<double>("frame_back", ROBOT_FRAME_BACK);
        this->declare_parameter<double>("frame_left", ROBOT_FRAME_LEFT);
        this->declare_parameter<double>("frame_right", ROBOT_FRAME_RIGHT);
        this->declare_parameter<double>("direct_timeout", 0.3);
        this->declare_parameter<bool>("gait_aware_follow", false);
        this->declare_parameter<int>("minimum_target_points", 1);
        this->declare_parameter<double>("target_cluster_gap", .12);
        this->declare_parameter<double>("target_ambiguity_margin", .04);
        this->declare_parameter<double>("tracking_timeout", .3);
        this->declare_parameter<std::string>("scan_frame", "");
        this->declare_parameter<double>("distance_stop", .06);
        this->declare_parameter<double>("distance_start", .10);
        this->declare_parameter<double>("angle_stop", .07);
        this->declare_parameter<double>("angle_start", .15);
        this->declare_parameter<double>("min_follow_vx", .22);
        this->declare_parameter<double>("max_follow_vx", .30);
        this->declare_parameter<double>("min_follow_wz", .52);
        this->declare_parameter<double>("max_follow_wz", .60);
        this->declare_parameter<double>("apf_clearance", .60);
        this->declare_parameter<double>("slowdown_clearance", .40);
        this->declare_parameter<double>("emergency_clearance", .12);
        LidarTracker::Config tracker_config;
        tracker_config.scan_yaw = get_parameter("scan_yaw").as_double();
        tracker_config.follow_distance = get_parameter("follow_distance").as_double();
        tracker_config.corridor_width = get_parameter("corridor_width").as_double();
        tracker_config.frame_front = get_parameter("frame_front").as_double();
        tracker_config.frame_back = get_parameter("frame_back").as_double();
        tracker_config.frame_left = get_parameter("frame_left").as_double();
        tracker_config.frame_right = get_parameter("frame_right").as_double();
        tracker_config.gait_aware = get_parameter("gait_aware_follow").as_bool();
        gait_aware_ = tracker_config.gait_aware;
        tracker_config.minimum_target_points = get_parameter("minimum_target_points").as_int();
        tracker_config.target_cluster_gap = get_parameter("target_cluster_gap").as_double();
        tracker_config.target_ambiguity_margin = get_parameter("target_ambiguity_margin").as_double();
        tracker_config.gait.distance_stop = get_parameter("distance_stop").as_double();
        tracker_config.gait.distance_start = get_parameter("distance_start").as_double();
        tracker_config.gait.angle_stop = get_parameter("angle_stop").as_double();
        tracker_config.gait.angle_start = get_parameter("angle_start").as_double();
        tracker_config.gait.min_vx = get_parameter("min_follow_vx").as_double();
        tracker_config.gait.max_vx = get_parameter("max_follow_vx").as_double();
        tracker_config.gait.min_wz = get_parameter("min_follow_wz").as_double();
        tracker_config.gait.max_wz = get_parameter("max_follow_wz").as_double();
        tracker_config.apf_clearance = get_parameter("apf_clearance").as_double();
        tracker_config.slowdown_clearance = get_parameter("slowdown_clearance").as_double();
        tracker_config.emergency_clearance = get_parameter("emergency_clearance").as_double();
        tracking_timeout_ = get_parameter("tracking_timeout").as_double();
        if (!std::isfinite(tracking_timeout_) || tracking_timeout_ <= 0 || tracking_timeout_ > .5) {
            throw std::invalid_argument("tracking_timeout must be in (0, 0.5]");
        }
        shared_state_.direct_timeout = get_parameter("direct_timeout").as_double();
        for (double value : {tracker_config.follow_distance, tracker_config.corridor_width,
                tracker_config.frame_front, tracker_config.frame_back, tracker_config.frame_left,
                tracker_config.frame_right, shared_state_.direct_timeout}) {
            if (!std::isfinite(value) || value <= 0.0) {
                throw std::invalid_argument("geometry and timeout parameters must be positive and finite");
            }
        }
        if (!std::isfinite(tracker_config.scan_yaw)) {
            throw std::invalid_argument("scan_yaw must be finite");
        }
        lidar_tracker_.configure(tracker_config);
        shared_state_.updateTrackedTarget(tracker_config.follow_distance, 0.0);
        
        // 获取参数
        shared_state_.active.store(this->get_parameter("active").as_bool());
        bool enable_opencv = this->get_parameter("enable_opencv").as_bool();
        bool enable_web = this->get_parameter("enable_web").as_bool();
        bool enable_kalman = this->get_parameter("enable_kalman").as_bool();
        std::string web_root = this->get_parameter("web_root").as_string();
        if (gait_aware_ && (enable_web || get_parameter("enable_android").as_bool() ||
                           get_parameter("enable_actions").as_bool())) {
            throw std::invalid_argument("M20 gait-aware control requires timestamped ROS selection only");
        }
        
        RCLCPP_INFO(this->get_logger(), "节点启动 - 跟随: %s, OpenCV: %s, Web: %s, 卡尔曼: %s",
                    shared_state_.active.load() ? "开启" : "关闭",
                    enable_opencv ? "开启" : "关闭",
                    enable_web ? "开启" : "关闭",
                    enable_kalman ? "开启" : "关闭");
        
        // 初始化发布者和订阅者
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
        tracking_pub_ = create_publisher<std_msgs::msg::String>("~/tracking_status", 1);
        action_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/d1_cmd", 10);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&RobotNexusNode::scanCallback, this, std::placeholders::_1)
        );
        moving_service_ = create_service<std_srvs::srv::SetBool>(
            "~/set_moving", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                   std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
                if (request->data && gait_aware_) {
                    invalidateTracking("enable_rejected");
                    publishTracking();
                    response->success = false;
                    response->message = "Use epoch-bound enable_follow for M20; set_moving only disables";
                    return;
                }
                shared_state_.is_moving_enabled.store(request->data);
                shared_state_.setDirectCmd(0.0, 0.0, 0.0);
                if (!request->data) {
                    if (gait_aware_) ++fault_epoch_;  // Invalidate already queued enable requests.
                    shared_state_.setVelocity(0, 0, 0);
                    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
                }
                response->success = true;
                response->message = "Follow enable updated; platform arm is independent";
            });
        if (!gait_aware_) {
            target_sub_ = create_subscription<geometry_msgs::msg::Point>(
                "~/target", 1, [this](geometry_msgs::msg::Point::SharedPtr point) {
                if (std::isfinite(point->x) && std::isfinite(point->y)) {
                    shared_state_.setTarget(point->x, point->y);
                    fault_active_ = false;
                }
                publishTracking();
            });
        }
        selection_service_ = create_service<rcl_interfaces::srv::SetParametersAtomically>(
            "~/select_target", [this](
                const std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Request> request,
                std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Response> response) {
                selectTarget(*request, *response);
            });
        enable_service_ = create_service<rcl_interfaces::srv::SetParametersAtomically>(
            "~/enable_follow", [this](
                const std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Request> request,
                std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Response> response) {
                enableFollow(*request, *response);
            });
        
        // 设置日志回调
        auto log_cb = [this](const std::string& msg) {
            RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
        };
        
        // 设置速度发布回调
        auto vel_cb = [this](const geometry_msgs::msg::Twist& cmd) {
            cmd_vel_pub_->publish(cmd);
        };
        
        // 配置雷达追踪器
        lidar_tracker_.setOpenCVEnabled(enable_opencv);
        lidar_tracker_.setKalmanEnabled(enable_kalman);
        lidar_tracker_.setVelocityCallback(vel_cb);
        lidar_tracker_.setDataBroadcastCallback([this]() {
            android_comm_.sendScanData();
            web_comm_.broadcastData();
        });
        
        // 配置直接控制器
        direct_controller_.setVelocityCallback(vel_cb);
        
        // 配置Web通讯
        if (enable_web) {
            web_comm_.setLogCallback(log_cb);
            web_comm_.setWebRoot(web_root);
            web_comm_.autoDetectWebRoot({
                "./web",
                "../share/jie_deamon/web"
            });
            web_comm_.setDirectCmdCallback([this](double x, double y, double z) {
                direct_controller_.processDirectCmd(x, y, z);
                RCLCPP_INFO(this->get_logger(), "收到direct_cmd: vx=%.2f, vy=%.2f, wz=%.2f", x, y, z);
            });
            web_comm_.setActionCmdCallback([this](const std::string& action) {
                if (!get_parameter("enable_actions").as_bool()) {
                    RCLCPP_WARN(get_logger(), "Platform action rejected: %s", action.c_str());
                    return;
                }
                std_msgs::msg::String msg;
                msg.data = action;
                action_cmd_pub_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "发布动作指令: %s", action.c_str());
                
                if (action == "liedown") {
                    std::thread([this]() {
                        std::this_thread::sleep_for(std::chrono::seconds(5));
                        std_msgs::msg::String passive_msg;
                        passive_msg.data = "passive";
                        if (rclcpp::ok()) {
                            action_cmd_pub_->publish(passive_msg);
                            RCLCPP_INFO(this->get_logger(), "延迟发布动作指令: passive");
                        }
                    }).detach();
                }
            });
            web_comm_.start();
            
            std::string local_ip = web_comm_.getLocalIP();
            RCLCPP_INFO(this->get_logger(), "Web界面: http://%s:%d", local_ip.c_str(), HTTP_PORT);
        }
        
        // 配置Android通讯
        android_comm_.setLogCallback(log_cb);
        if (get_parameter("enable_android").as_bool()) {
            android_comm_.start();
        }
        
        // 创建直接控制定时器（10Hz）
        direct_control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DirectController::timerCallback, &direct_controller_)
        );
        tracking_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {
            if (gait_aware_ && !inputsFresh()) {
                invalidateTracking("scan_stale");
            }
            publishTracking();
        });
        parameter_guard_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>&) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = !gait_aware_;
            result.reason = "Restart the M20 profile to change configuration; use services for control";
            return result;
        });
        
        RCLCPP_INFO(this->get_logger(), "机器人中枢节点 'robot_nexus' 已启动");
    }
    
    ~RobotNexusNode()
    {
        lidar_tracker_.destroyWindows();
    }

private:
    // 共享状态
    SharedState shared_state_;
    
    // 功能模块
    LidarTracker lidar_tracker_;
    DirectController direct_controller_;
    WebCommManager web_comm_;
    AndroidCommManager android_comm_;
    
    // ROS2成员
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr direct_control_timer_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr moving_service_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
    rclcpp::Service<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr selection_service_;
    rclcpp::Service<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr enable_service_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tracking_pub_;
    rclcpp::TimerBase::SharedPtr tracking_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard_;
    std::chrono::steady_clock::time_point last_scan_{};
    bool gait_aware_ = false;
    double tracking_timeout_ = .3;
    double last_scan_stamp_ = 0;
    bool has_scan_ = false;
    bool fault_active_ = true;
    std::int64_t fault_epoch_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    std::int64_t last_selection_stamp_ = -1;
    std::int64_t last_enable_stamp_ = -1;

    void enableFollow(const rcl_interfaces::srv::SetParametersAtomically::Request& request,
                      rcl_interfaces::srv::SetParametersAtomically::Response& response) {
        auto reject = [&](const char* reason) {
            if (gait_aware_) invalidateTracking(reason);
            response.result.successful = false;
            response.result.reason = reason;
            publishTracking();
        };
        if (!gait_aware_ || !inputsFresh() || !lidar_tracker_.diagnostics().tracking_valid) {
            reject("enable_requires_fresh_selected_target"); return;
        }
        std::map<std::string, std::int64_t> values;
        for (const auto& p : request.parameters) {
            if (p.value.type != rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER ||
                !values.emplace(p.name, p.value.integer_value).second) {
                reject("invalid_enable_fields"); return;
            }
        }
        if (values.size() != 4 || !values.count("stamp_sec") || !values.count("stamp_nanosec") ||
            !values.count("selection_id") || !values.count("fault_epoch")) {
            reject("invalid_enable_fields"); return;
        }
        const auto sec = values.at("stamp_sec"), nsec = values.at("stamp_nanosec");
        if (sec < 0 || sec > std::numeric_limits<int32_t>::max() || nsec < 0 || nsec >= 1000000000) {
            reject("invalid_enable_stamp"); return;
        }
        const std::int64_t stamp = sec * 1000000000LL + nsec;
        const double age = (now().nanoseconds()-stamp)/1e9;
        double x, y;
        const auto selection = shared_state_.getTargetSelection(x, y);
        if (age < -.1 || age > tracking_timeout_ || stamp <= last_enable_stamp_ ||
            stamp < last_selection_stamp_ || values.at("fault_epoch") != fault_epoch_ ||
            values.at("selection_id") <= 0 ||
            static_cast<std::uint64_t>(values.at("selection_id")) != selection) {
            reject("stale_enable_request"); return;
        }
        shared_state_.setDirectCmd(0, 0, 0);
        shared_state_.is_moving_enabled.store(true);
        last_enable_stamp_ = stamp;
        response.result.successful = true;
        response.result.reason = "Follow computation enabled; platform arm remains separate";
        publishTracking();
    }

    void selectTarget(const rcl_interfaces::srv::SetParametersAtomically::Request& request,
                      rcl_interfaces::srv::SetParametersAtomically::Response& response) {
        auto reject = [&](const char* reason) {
            if (gait_aware_) invalidateTracking(reason);
            response.result.successful = false;
            response.result.reason = reason;
            publishTracking();
        };
        if (!gait_aware_) { reject("Use the legacy target topic for the legacy profile"); return; }
        if (!inputsFresh()) { reject("scan_stale"); return; }
        // Use an existing structured ROS service without mutating node parameters.
        std::map<std::string, rcl_interfaces::msg::ParameterValue> values;
        for (const auto& p : request.parameters) {
            if (!values.emplace(p.name, p.value).second) { reject("duplicate_selection_field"); return; }
        }
        using Type = rcl_interfaces::msg::ParameterType;
        auto typed = [&](const char* name, uint8_t type) {
            auto it = values.find(name);
            return it != values.end() && it->second.type == type;
        };
        if (values.size() != 6 || !typed("target_x", Type::PARAMETER_DOUBLE) ||
            !typed("target_y", Type::PARAMETER_DOUBLE) || !typed("frame_id", Type::PARAMETER_STRING) ||
            !typed("stamp_sec", Type::PARAMETER_INTEGER) || !typed("stamp_nanosec", Type::PARAMETER_INTEGER) ||
            !typed("fault_epoch", Type::PARAMETER_INTEGER)) {
            reject("invalid_selection_fields"); return;
        }
        const auto sec = values.at("stamp_sec").integer_value;
        const auto nsec = values.at("stamp_nanosec").integer_value;
        if (sec < 0 || sec > std::numeric_limits<int32_t>::max() || nsec < 0 || nsec >= 1000000000) {
            reject("invalid_selection_stamp"); return;
        }
        const std::int64_t stamp = sec * 1000000000LL + nsec;
        const auto age = (now().nanoseconds() - stamp) / 1e9;
        if (age < -.1 || age > tracking_timeout_ || stamp <= last_selection_stamp_ ||
            values.at("fault_epoch").integer_value != fault_epoch_) {
            reject("stale_selection_request"); return;
        }
        const auto frame = get_parameter("scan_frame").as_string();
        const double x = values.at("target_x").double_value;
        const double y = values.at("target_y").double_value;
        if (frame.empty() || values.at("frame_id").string_value != frame ||
            !std::isfinite(x) || !std::isfinite(y) || x <= get_parameter("frame_front").as_double() ||
            std::hypot(x, y) > 12) {
            reject("invalid_selection_geometry"); return;
        }
        lidar_tracker_.invalidateTarget("selection_changed");
        shared_state_.setTarget(x, y);
        last_selection_stamp_ = stamp;
        fault_active_ = false;
        response.result.successful = true;
        response.result.reason = "Selection accepted; verify tracking_valid before enabling";
        publishTracking();
    }

    bool inputsFresh() {
        const double age = now().seconds() - last_scan_stamp_;
        return has_scan_ && std::isfinite(age) && age >= -.1 && age <= tracking_timeout_ &&
            std::chrono::duration<double>(std::chrono::steady_clock::now()-last_scan_).count() <=
            tracking_timeout_;
    }

    void invalidateTracking(const char* reason) {
        if (!fault_active_) ++fault_epoch_;
        fault_active_ = true;
        lidar_tracker_.invalidateTarget(reason);
    }

    void publishTracking() {
        if (!tracking_pub_) return;
        const auto& d = lidar_tracker_.diagnostics();
        double x, y;
        const auto selection = shared_state_.getTargetSelection(x, y);
        std::ostringstream out;
        out << std::setprecision(15) << "{\"stamp\":" << last_scan_stamp_
            << ",\"selection_id\":" << selection
            << ",\"fault_epoch\":" << fault_epoch_
            << ",\"tracking_valid\":" << (d.tracking_valid ? "true" : "false")
            << ",\"state\":\"" << d.tracking_state << "\",\"controller\":\""
            << d.controller_state << "\",\"target_x\":" << x << ",\"target_y\":" << y
            << ",\"target_points\":" << d.target_points << ",\"corridor\":" << d.corridor
            << ",\"apf_x\":" << d.repulse_x << ",\"apf_y\":" << d.repulse_y << "}";
        std_msgs::msg::String msg;
        msg.data = out.str();
        tracking_pub_->publish(msg);
    }
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        // Validate message fields before constructing any ROS time object.
        if (scan_msg->header.stamp.sec < 0 || scan_msg->header.stamp.nanosec >= 1000000000u) {
            invalidateTracking("invalid_scan_header");
            publishTracking();
            return;
        }
        const double stamp = static_cast<double>(scan_msg->header.stamp.sec) +
            scan_msg->header.stamp.nanosec * 1e-9;
        if (gait_aware_) {
            if (has_scan_ && !inputsFresh()) invalidateTracking("scan_stale");
            const double age = now().seconds() - stamp;
            const auto frame = get_parameter("scan_frame").as_string();
            if (!std::isfinite(age) || age < -.1 || age > tracking_timeout_ ||
                (!frame.empty() && frame != scan_msg->header.frame_id)) {
                invalidateTracking("invalid_scan_header");
                publishTracking();
                return;
            }
        }
        last_scan_ = std::chrono::steady_clock::now();
        last_scan_stamp_ = stamp;
        has_scan_ = true;
        lidar_tracker_.processScan(scan_msg);
        if (gait_aware_ && !lidar_tracker_.diagnostics().tracking_valid && !fault_active_) {
            invalidateTracking(lidar_tracker_.diagnostics().tracking_state);
        }
        publishTracking();
    }
};

#ifndef JIE_DEAMON_NO_MAIN
int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<RobotNexusNode>();
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
#endif
