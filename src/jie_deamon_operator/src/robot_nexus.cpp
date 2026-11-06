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
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <thread>
#include <chrono>
#include <stdexcept>

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
public:
    RobotNexusNode()
        : Node("robot_nexus"),
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
        this->declare_parameter<bool>("operator_control", false);
        this->declare_parameter<double>("scan_yaw", 3.141592653589793);
        this->declare_parameter<double>("follow_distance", FOLLOW_DIST);
        this->declare_parameter<double>("corridor_width", RECTANGLE_WIDTH);
        this->declare_parameter<double>("frame_front", ROBOT_FRAME_FRONT);
        this->declare_parameter<double>("frame_back", ROBOT_FRAME_BACK);
        this->declare_parameter<double>("frame_left", ROBOT_FRAME_LEFT);
        this->declare_parameter<double>("frame_right", ROBOT_FRAME_RIGHT);
        this->declare_parameter<double>("direct_timeout", 0.3);
        this->declare_parameter<bool>("enable_lateral", false);
        this->declare_parameter<double>("lateral_gain", LATERAL_GAIN);
        this->declare_parameter<double>("lateral_max", LATERAL_MAX);
        this->declare_parameter<double>("lateral_deadband", LATERAL_DEADBAND);
        this->declare_parameter<double>("lateral_sign", 1.0);
        this->declare_parameter<double>("min_forward_speed", 0.0);
        LidarTracker::Config tracker_config;
        tracker_config.scan_yaw = get_parameter("scan_yaw").as_double();
        tracker_config.follow_distance = get_parameter("follow_distance").as_double();
        tracker_config.corridor_width = get_parameter("corridor_width").as_double();
        tracker_config.frame_front = get_parameter("frame_front").as_double();
        tracker_config.frame_back = get_parameter("frame_back").as_double();
        tracker_config.frame_left = get_parameter("frame_left").as_double();
        tracker_config.frame_right = get_parameter("frame_right").as_double();
        tracker_config.enable_lateral = get_parameter("enable_lateral").as_bool();
        tracker_config.lateral_gain = get_parameter("lateral_gain").as_double();
        tracker_config.lateral_max = get_parameter("lateral_max").as_double();
        tracker_config.lateral_deadband = get_parameter("lateral_deadband").as_double();
        tracker_config.lateral_sign = get_parameter("lateral_sign").as_double();
        tracker_config.min_forward_speed = get_parameter("min_forward_speed").as_double();
        shared_state_.direct_timeout = get_parameter("direct_timeout").as_double();
        for (double value : {tracker_config.follow_distance, tracker_config.corridor_width,
                tracker_config.frame_front, tracker_config.frame_back, tracker_config.frame_left,
                tracker_config.frame_right, shared_state_.direct_timeout,
                tracker_config.lateral_gain, tracker_config.lateral_max,
                tracker_config.lateral_deadband}) {
            if (!std::isfinite(value) || value <= 0.0) {
                throw std::invalid_argument("geometry and timeout parameters must be positive and finite");
            }
        }
        for (double value : {tracker_config.scan_yaw, tracker_config.lateral_sign,
                tracker_config.min_forward_speed}) {
            if (!std::isfinite(value)) {
                throw std::invalid_argument("scan_yaw, lateral_sign and min_forward_speed must be finite");
            }
        }
        lidar_tracker_.configure(tracker_config);
        shared_state_.setTarget(tracker_config.follow_distance, 0.0);

        // 获取参数
        shared_state_.active.store(this->get_parameter("active").as_bool());
        bool enable_opencv = this->get_parameter("enable_opencv").as_bool();
        bool enable_web = this->get_parameter("enable_web").as_bool();
        bool enable_kalman = this->get_parameter("enable_kalman").as_bool();
        std::string web_root = this->get_parameter("web_root").as_string();

        RCLCPP_INFO(this->get_logger(), "节点启动 - 跟随: %s, OpenCV: %s, Web: %s, 卡尔曼: %s",
                    shared_state_.active.load() ? "开启" : "关闭",
                    enable_opencv ? "开启" : "关闭",
                    enable_web ? "开启" : "关闭",
                    enable_kalman ? "开启" : "关闭");

        // 初始化发布者和订阅者
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
        operator_vel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/operator_velocity", 1);
        operator_target_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "~/operator_target", 1, [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (!get_parameter("operator_control").as_bool() || msg->data.size() != 3) return;
                for (double value : msg->data) if (!std::isfinite(value)) return;
                const double session = msg->data[0];
                if (session < 0 || session >= 9007199254740992.0 || std::floor(session) != session) return;
                operator_session_ = session;
                shared_state_.is_moving_enabled.store(session != 0);
                shared_state_.setDirectCmd(0, 0, 0);
                if (session != 0) {
                    shared_state_.control_mode.store(MODE_FOLLOW);
                    shared_state_.active.store(true);
                    shared_state_.setTarget(msg->data[1], msg->data[2]);
                }
            });
        action_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/d1_cmd", 10);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&RobotNexusNode::scanCallback, this, std::placeholders::_1)
        );
        moving_service_ = create_service<std_srvs::srv::SetBool>(
            "~/set_moving", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                   std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
                if (get_parameter("operator_control").as_bool() && request->data) {
                    response->success = false;
                    response->message = "Use the operator gateway";
                    return;
                }
                shared_state_.is_moving_enabled.store(request->data);
                shared_state_.setDirectCmd(0.0, 0.0, 0.0);
                response->success = true;
                response->message = "Follow enable updated; platform arm is independent";
            });
        target_sub_ = create_subscription<geometry_msgs::msg::Point>(
            "~/target", 1, [this](geometry_msgs::msg::Point::SharedPtr point) {
                if (!get_parameter("operator_control").as_bool() &&
                    std::isfinite(point->x) && std::isfinite(point->y)) {
                    shared_state_.setTarget(point->x, point->y);
                }
            });

        // 设置日志回调
        auto log_cb = [this](const std::string& msg) {
            RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
        };

        // 设置速度发布回调
        auto vel_cb = [this](const geometry_msgs::msg::Twist& cmd) {
            cmd_vel_pub_->publish(cmd);
            if (get_parameter("operator_control").as_bool()) {
                std_msgs::msg::Float64MultiArray tagged;
                tagged.data = {operator_session_, cmd.linear.x, cmd.linear.y, cmd.angular.z};
                operator_vel_pub_->publish(tagged);
            }
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
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr operator_target_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr operator_vel_pub_;
    double operator_session_{0};

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        lidar_tracker_.processScan(scan_msg);
    }
};

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RobotNexusNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
