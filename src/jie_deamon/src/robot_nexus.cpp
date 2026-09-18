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
 *
 * NEW-06 (M20适配): 新增 m20_preview 参数（默认false保持上游D1行为）。
 * m20_preview=true 时为"仅算法/NavCmd预览"模式：
 * - 完全禁用Android通讯、/d1_cmd动作发布与D1动作回调（不映射liedown为急停）
 * - 不创建直接控制定时器；Web直控指令与switch_mode≠FOLLOW一律拒绝
 * - /cmd_vel改为发布到预览话题 /new_chase/cmd_vel_raw（无真实底盘消费者）
 * - scan订阅SensorDataQoS，校验时间戳新鲜度与frame_id，配看门狗超时全零+失能
 * 本阶段物理运动功能不实现，机器不会运动。
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <chrono>
#include <atomic>
#include <cmath>
#include <stdexcept>

// NEW-06 P0: Foxy无 rclcpp::Node::resolve_topic_name / NodeTopicsInterface::resolve_topic_name
// （已grep核对 /opt/ros/foxy/include 全目录），使用既有API组合实现重映射感知的
// 创建前话题解析与创建后复核：
//   rclcpp/expand_topic_or_service_name.hpp  展开相对名
//   rcl/remap.h rcl_remap_topic_name         应用CLI重映射规则
//   rcl/publisher.h rcl_publisher_get_topic_name  创建后复核最终解析名
#include <rcl/node.h>
#include <rcl/remap.h>
#include <rcl/publisher.h>
#include <rclcpp/expand_topic_or_service_name.hpp>

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

        // NEW-06 M20预览参数
        this->declare_parameter<bool>("m20_preview", false);
        this->declare_parameter<std::string>("scan_topic", "/scan");
        this->declare_parameter<std::string>("raw_topic", "/new_chase/cmd_vel_raw");
        this->declare_parameter<std::string>("expected_scan_frame", "");  // 空=不校验frame
        this->declare_parameter<double>("rotation_angle", M_PI);          // 上游硬编码pi；M20=0
        this->declare_parameter<double>("follow_distance", FOLLOW_DIST);  // 上游0.4；M20=1.2
        this->declare_parameter<double>("rectangle_width", RECTANGLE_WIDTH); // 上游0.35；M20=0.7
        this->declare_parameter<double>("robot_frame_front", ROBOT_FRAME_FRONT); // M20初值0.46
        this->declare_parameter<double>("robot_frame_back", ROBOT_FRAME_BACK);   // M20初值0.46
        this->declare_parameter<double>("robot_frame_left", ROBOT_FRAME_LEFT);   // M20初值0.31
        this->declare_parameter<double>("robot_frame_right", ROBOT_FRAME_RIGHT); // M20初值0.31
        this->declare_parameter<double>("scan_stale_max", 0.5);  // scan过期阈值(秒)
        this->declare_parameter<double>("scan_future_max", 0.1); // scan未来戳容忍(秒)

        // 获取参数
        shared_state_.active.store(this->get_parameter("active").as_bool());
        bool enable_opencv = this->get_parameter("enable_opencv").as_bool();
        bool enable_web = this->get_parameter("enable_web").as_bool();
        bool enable_kalman = this->get_parameter("enable_kalman").as_bool();
        std::string web_root = this->get_parameter("web_root").as_string();

        // NEW-06: M20预览开关
        m20_preview_ = this->get_parameter("m20_preview").as_bool();
        shared_state_.m20_preview.store(m20_preview_);
        scan_stale_max_ = this->get_parameter("scan_stale_max").as_double();
        scan_future_max_ = this->get_parameter("scan_future_max").as_double();
        expected_scan_frame_ = this->get_parameter("expected_scan_frame").as_string();
        scan_topic_ = this->get_parameter("scan_topic").as_string();
        raw_topic_ = this->get_parameter("raw_topic").as_string();

        // 配置雷达追踪器可参数化几何（默认值=上游原版常量）
        lidar_tracker_.setRotationAngle(this->get_parameter("rotation_angle").as_double());
        lidar_tracker_.setFollowDistance(this->get_parameter("follow_distance").as_double());
        lidar_tracker_.setRectangleWidth(this->get_parameter("rectangle_width").as_double());
        lidar_tracker_.setRobotFrame(
            this->get_parameter("robot_frame_front").as_double(),
            this->get_parameter("robot_frame_back").as_double(),
            this->get_parameter("robot_frame_left").as_double(),
            this->get_parameter("robot_frame_right").as_double());

        RCLCPP_INFO(this->get_logger(), "节点启动 - 跟随: %s, OpenCV: %s, Web: %s, 卡尔曼: %s, M20预览: %s",
                    shared_state_.active.load() ? "开启" : "关闭",
                    enable_opencv ? "开启" : "关闭",
                    enable_web ? "开启" : "关闭",
                    enable_kalman ? "开启" : "关闭",
                    m20_preview_ ? "true(仅算法/NavCmd预览,机器不会运动)" : "false(D1原版)");

        // 初始化订阅者
        // NEW-06 P0修复: create_subscription——注意：第1轮报告中“上游遗漏”表述不实，
        // 这是本适配第1轮实现的遗漏（原仓库9403185无此段改动自然无订阅）。
        // 必须真正绑定scanCallback；
        // m20预览用SensorDataQoS(best-effort,与pointcloud_to_laserscan匹配)
        std::function<void(sensor_msgs::msg::LaserScan::SharedPtr)> scan_cb =
            std::bind(&RobotNexusNode::scanCallback, this, std::placeholders::_1);
        if (m20_preview_) {
            scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                scan_topic_, rclcpp::SensorDataQoS(), scan_cb);
        } else {
            // D1兼容: 与上游一致(默认QoS, depth 1, /scan)
            scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                scan_topic_, 1, scan_cb);
        }

        // 初始化发布者
        // NEW-06: m20预览下不创建 /cmd_vel 与 /d1_cmd 发布者（物理运动未实现）
        if (m20_preview_) {
            // NEW-06 P0修复: 创建前解析重映射后最终名，必须在 /new_chase/ 前缀下
            // 且不是真实控制话题名，非法配置直接抛错退出（exit 2）
            const std::string resolved_raw = resolvePreviewTopicName(raw_topic_);
            if (!isAllowedPreviewTopic(resolved_raw)) {
                throw std::runtime_error(
                    "raw_topic '" + raw_topic_ + "' 解析为 '" + resolved_raw +
                    "'：非法（必须严格位于 /new_chase/ 下且非真实控制话题）");
            }
            cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(raw_topic_, 1);
            // 创建后复核最终解析名（防未覆盖到的重映射路径）
            // 创建后复核最终解析名（Foxy: PublisherBase::get_publisher_handle() 返回shared_ptr）
            const char* actual = nullptr;
            {
                const auto rcl_pub_handle = cmd_vel_pub_->get_publisher_handle();
                if (rcl_pub_handle) actual = rcl_publisher_get_topic_name(rcl_pub_handle.get());
            }
            const std::string actual_str = actual ? std::string(actual) : std::string();
            if (!isAllowedPreviewTopic(actual_str)) {
                cmd_vel_pub_.reset();
                throw std::runtime_error(
                    "raw话题创建后复核失败: 期望'" + resolved_raw + "' 实际'" + actual_str +
                    "'——拒绝任何把预览raw重映射到真实控制话题的行为");
            }
            RCLCPP_INFO(this->get_logger(), "预览raw发布话题: %s (已通过解析与创建后复核)",
                        actual_str.c_str());
        } else {
            cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
            action_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/d1_cmd", 10);
        }

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
            // NEW-06: m20预览下禁用Android，仅Web广播
            if (m20_preview_) {
                web_comm_.broadcastData();
            } else {
                android_comm_.sendScanData();
                web_comm_.broadcastData();
            }
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
            // NEW-06: m20预览下不注册直控/动作回调——后端拒绝Web直控与D1动作
            if (!m20_preview_) {
                web_comm_.setDirectCmdCallback([this](double x, double y, double z) {
                    direct_controller_.processDirectCmd(x, y, z);
                    RCLCPP_INFO(this->get_logger(), "收到direct_cmd: vx=%.2f, vy=%.2f, wz=%.2f", x, y, z);
                });
                web_comm_.setActionCmdCallback([this](const std::string& action) {
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
            }
            web_comm_.start();

            std::string local_ip = web_comm_.getLocalIP();
            RCLCPP_INFO(this->get_logger(), "Web界面: http://%s:%d", local_ip.c_str(), HTTP_PORT);
        }

        // 配置Android通讯
        // NEW-06: m20预览下完全禁用Android启动
        if (!m20_preview_) {
            android_comm_.setLogCallback(log_cb);
            android_comm_.start();
        }

        // 创建直接控制定时器（10Hz）
        // NEW-06: m20预览下不创建直控定时器
        if (!m20_preview_) {
            direct_control_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&DirectController::timerCallback, &direct_controller_)
            );
        } else {
            // NEW-06: m20预览看门狗——0.5s无有效scan立即零速+失能+清跟踪
            watchdog_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&RobotNexusNode::watchdogCallback, this)
            );
        }

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
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    // NEW-06: M20预览状态
    bool m20_preview_ = false;
    std::string expected_scan_frame_;
    std::string scan_topic_;
    std::string raw_topic_;
    double scan_stale_max_ = 0.5;
    double scan_future_max_ = 0.1;
    // 最后一次有效scan时间（steady_clock 单调时钟, 纳秒; 0=从未收到）
    std::atomic<int64_t> last_valid_scan_ns_{0};
    // 超时告警节流
    std::atomic<int64_t> last_timeout_log_ns_{0};
    std::atomic<int64_t> last_stop_log_ns_{0};

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        if (m20_preview_) {
            // NEW-06 P1修复: 任何拒收分支当帧立即零速+失能+清跟踪(统一stopPreview)，而非静默return
            // NEW-06: 时间戳新鲜度校验——过期>scan_stale_max或未来>scan_future_max直接拒绝
            try {
                const rclcpp::Time stamp(scan_msg->header.stamp);
                const double age = (this->now() - stamp).seconds();
                if (age > scan_stale_max_) {
                    stopPreview("scan时间戳过期 " + std::to_string(age) + "s");
                    return;
                }
                if (age < -scan_future_max_) {
                    stopPreview("scan时间戳来自未来 " + std::to_string(age) + "s");
                    return;
                }
            } catch (const std::exception& e) {
                // NEW-06 P1: 时间戳异常不crash，当帧停预览
                stopPreview(std::string("scan时间戳解析异常: ") + e.what());
                return;
            }
            // NEW-06: frame标签校验——配置了期望frame且不匹配则拒绝
            if (!expected_scan_frame_.empty() && scan_msg->header.frame_id != expected_scan_frame_) {
                stopPreview("scan frame_id '" + scan_msg->header.frame_id +
                            "' 与配置 '" + expected_scan_frame_ + "' 不符");
                return;
            }
            // 元信息/内容合法性在LidarTracker内统一校验；其返回值指示本帧是否有合法点
            const bool data_valid = lidar_tracker_.processScan(scan_msg);
            if (data_valid) {
                // 数据流活跃（合法点存在；空/全非法帧不续期，由看门狗兜底）
                const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                last_valid_scan_ns_.store(now_ns);
            }
            return;
        }
        lidar_tracker_.processScan(scan_msg);
    }

    // NEW-06 P1: 统一的预览停止——立即零速发布+失能+清跟踪+广播同步Web状态
    void stopPreview(const std::string& reason)
    {
        shared_state_.setVelocity(0.0, 0.0, 0.0);
        shared_state_.clearTracking();
        geometry_msgs::msg::Twist zero_vel;
        if (cmd_vel_pub_) cmd_vel_pub_->publish(zero_vel);
        // 同步Web: clearTracking后广播，前端selected/valid立即刷新
        web_comm_.broadcastData();
        const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        const int64_t last_log = last_stop_log_ns_.load();
        if (last_log == 0 || (now_ns - last_log) > 2000000000LL) {
            last_stop_log_ns_.store(now_ns);
            RCLCPP_WARN(this->get_logger(), "预览停止(当帧): %s —— 已零速+失能+清除跟踪", reason.c_str());
        }
    }

    // NEW-06 P0: 重映射感知的话题解析（创建发布者之前调用）
    // Foxy无 resolve_topic_name API（已核对），组合既有API：
    //   1) rclcpp::expand_topic_or_service_name 展开为FQN
    //   2) rcl_remap_topic_name 应用节点CLI重映射规则(-r a:=b)
    std::string resolvePreviewTopicName(const std::string& topic)
    {
        const std::string fqn = rclcpp::expand_topic_or_service_name(
            topic, this->get_name(), this->get_namespace(), false);
        const rcl_node_t* node_handle = this->get_node_base_interface()->get_rcl_node_handle();
        const rcl_node_options_t* node_opts = rcl_node_get_options(node_handle);
        // NEW-06 第2轮P0修复: 必须同时应用context的全局重映射规则——大部分CLI -r
        // 由 rclcpp::init 收进 rcl_context_t.global_arguments（rcl/context.h:112）；
        // 仅当 node_opts.use_global_arguments=false 时忽略全局。
        // API核对（非猜测）: rclcpp/context.hpp:221-222 get_rcl_context() 返回
        // shared_ptr<rcl_context_t>; node_base_interface.hpp:70-71 get_context()
        // 返回 rclcpp::Context::SharedPtr。
        const rcl_arguments_t* global_args = nullptr;
        if (node_opts->use_global_arguments) {
            global_args =
                &this->get_node_base_interface()->get_context()->get_rcl_context()->global_arguments;
        }
        rcl_allocator_t allocator = rcl_get_default_allocator();
        char* remapped = nullptr;
        const rcl_ret_t ret = rcl_remap_topic_name(
            &node_opts->arguments, global_args, fqn.c_str(),
            this->get_name(), this->get_namespace(), allocator, &remapped);
        if (ret != RCL_RET_OK) {
            throw std::runtime_error("预览话题解析失败(remap规则应用出错): " + topic);
        }
        std::string resolved = remapped ? std::string(remapped) : fqn;
        if (remapped) allocator.deallocate(remapped, allocator.state);
        return resolved;
    }

    // NEW-06 P0: 预览话题白名单校验——严格 /new_chase/ 前缀，且不是真实控制话题
    static bool isAllowedPreviewTopic(const std::string& resolved)
    {
        static const char* kForbidden[] = {
            "/cmd_vel", "/d1_cmd", "/nav_cmd", "/motion_state", "/gait"};
        if (resolved.rfind("/new_chase/", 0) != 0) return false;
        for (const char* f : kForbidden) {
            if (resolved == f) return false;
        }
        return true;
    }

    // NEW-06: M20预览看门狗——monotonic时钟：
    // 1) 0.5s无有效scan：立即零速+失能+清跟踪+广播同步Web，而不仅是停止发布
    // 2) !active或!moving：持续零发布(与scan无关)，保证Web关闭后<=100ms零
    // 3) 从未收到scan：持续零发布兜底，不让"无扫描下使能"造成掩盖（但不清新选择）
    void watchdogCallback()
    {
        const int64_t last_ns = last_valid_scan_ns_.load();
        const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        const bool no_scan_ever = (last_ns == 0);
        const bool stale = !no_scan_ever &&
            (static_cast<double>(now_ns - last_ns) / 1e9 > scan_stale_max_);
        const bool should_zero = !shared_state_.active.load() ||
                                 !shared_state_.is_moving_enabled.load();
        if (should_zero || stale || no_scan_ever) {
            shared_state_.setVelocity(0.0, 0.0, 0.0);
            geometry_msgs::msg::Twist zero_vel;
            if (cmd_vel_pub_) cmd_vel_pub_->publish(zero_vel);
        }
        if (stale) {
            shared_state_.clearTracking();
            web_comm_.broadcastData();
            const int64_t last_log = last_timeout_log_ns_.load();
            if (last_log == 0 || (now_ns - last_log) > 2000000000LL) {
                last_timeout_log_ns_.store(now_ns);
                RCLCPP_WARN(this->get_logger(),
                            "看门狗超时: %.3fs无有效scan, 已零速+失能+清除跟踪(需重新选点+使能)",
                            static_cast<double>(now_ns - last_ns) / 1e9);
            }
        }
    }
};

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);

    int exit_code = 0;
    try {
        auto node = std::make_shared<RobotNexusNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        // NEW-06 P0: 非法配置（如预览话题被重映射到真实控制话题）直接退出码2
        RCLCPP_ERROR(rclcpp::get_logger("robot_nexus"),
                     "robot_nexus 退出: %s", e.what());
        exit_code = 2;
    }

    rclcpp::shutdown();
    return exit_code;
}
