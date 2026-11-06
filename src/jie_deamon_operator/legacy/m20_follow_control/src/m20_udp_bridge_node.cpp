#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "m20_follow_control/m20_protocol.hpp"

namespace m20::follow
{
namespace
{

std::string localTimestamp()
{
  const std::time_t now = std::time(nullptr);
  std::tm local{};
  localtime_r(&now, &local);
  char output[32]{};
  std::strftime(output, sizeof(output), "%Y-%m-%d %H:%M:%S", &local);
  return output;
}

enum class PendingAction
{
  NONE,
  STANDUP,
  LIEDOWN,
  SOFT_STOP,
  FLAT_GAIT,
  STAIR_GAIT,
};

const char *actionName(PendingAction action)
{
  switch (action)
  {
    case PendingAction::STANDUP:
      return "standup";
    case PendingAction::LIEDOWN:
      return "liedown";
    case PendingAction::SOFT_STOP:
      return "soft_stop";
    case PendingAction::FLAT_GAIT:
      return "flat_gait";
    case PendingAction::STAIR_GAIT:
      return "stair_gait";
    case PendingAction::NONE:
      return "none";
  }
  return "none";
}

}  // namespace

class M20UdpBridgeNode final : public rclcpp::Node
{
 public:
  M20UdpBridgeNode() : Node("m20_udp_bridge")
  {
    server_ip_ = declare_parameter<std::string>("server_ip", "10.21.31.103");
    server_port_ = declare_parameter("server_port", 30000);
    motion_output_enabled_ = declare_parameter("motion_output_enabled", false);
    armed_ = declare_parameter("arm_on_start", false) && motion_output_enabled_;
    cmd_timeout_ = std::chrono::milliseconds(
      std::max<std::int64_t>(50, declare_parameter("cmd_timeout_ms", 250)));
    status_timeout_ = std::chrono::milliseconds(
      std::max<std::int64_t>(100, declare_parameter("status_timeout_ms", 1200)));
    action_retry_ = std::chrono::milliseconds(
      std::max<std::int64_t>(100, declare_parameter("action_retry_ms", 500)));
    action_timeout_ = std::chrono::milliseconds(
      std::max<std::int64_t>(1000, declare_parameter("action_timeout_ms", 15000)));
    axis_parameters_.safe_max_x = declare_parameter("axis.safe_max_x", 0.25);
    axis_parameters_.safe_max_y = declare_parameter("axis.safe_max_y", 0.15);
    axis_parameters_.safe_max_yaw = declare_parameter("axis.safe_max_yaw", 0.3);
    axis_parameters_.full_scale_x = declare_parameter("axis.full_scale_x", 1.5);
    axis_parameters_.full_scale_y = declare_parameter("axis.full_scale_y", 0.6);
    axis_parameters_.full_scale_yaw = declare_parameter("axis.full_scale_yaw", 1.0);
    if (server_port_ <= 0 || server_port_ > 65535 || !axisParametersValid(axis_parameters_))
    {
      throw std::invalid_argument("invalid M20 UDP bridge server or axis parameters");
    }
    const double command_hz = std::max(1.0, declare_parameter("command_hz", 20.0));
    const double heartbeat_hz = std::max(1.0, declare_parameter("heartbeat_hz", 5.0));
    const std::string cmd_topic =
      declare_parameter<std::string>("cmd_vel_topic", "/m20_follow/cmd_vel");
    const std::string action_topic = declare_parameter<std::string>("action_topic", "/m20/action");
    const std::string arm_topic = declare_parameter<std::string>("arm_topic", "/m20/arm");
    const std::string status_topic =
      declare_parameter<std::string>("status_topic", "/m20/bridge_status");

    status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic, 10);
    cmd_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic, 10, std::bind(&M20UdpBridgeNode::cmdCallback, this, std::placeholders::_1));
    action_subscription_ = create_subscription<std_msgs::msg::String>(
      action_topic, 10, std::bind(&M20UdpBridgeNode::actionCallback, this, std::placeholders::_1));
    arm_subscription_ = create_subscription<std_msgs::msg::Bool>(
      arm_topic, 10, std::bind(&M20UdpBridgeNode::armCallback, this, std::placeholders::_1));

    openSocket();
    heartbeat_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                           std::chrono::duration<double>(1.0 / heartbeat_hz)),
                                         std::bind(&M20UdpBridgeNode::heartbeatTimer, this));
    command_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                         std::chrono::duration<double>(1.0 / command_hz)),
                                       std::bind(&M20UdpBridgeNode::commandTimer, this));
    status_timer_ = create_wall_timer(std::chrono::milliseconds(200),
                                      std::bind(&M20UdpBridgeNode::publishStatus, this));
    const auto now = std::chrono::steady_clock::now();
    last_command_time_ = now - cmd_timeout_ * 2;
    last_status_time_ = now - status_timeout_ * 2;
    action_started_ = now;
    last_action_send_ = now - action_retry_ * 2;

    RCLCPP_WARN(get_logger(), "M20 UDP bridge ready: server=%s:%d motion_output=%s armed=%s",
                server_ip_.c_str(), server_port_, motion_output_enabled_ ? "true" : "false",
                armed_ ? "true" : "false");
  }

  ~M20UdpBridgeNode() override
  {
    armed_ = false;
    if (motion_output_enabled_ && socket_fd_ >= 0)
    {
      for (int index = 0; index < 5; ++index)
      {
        sendFrame(M20Protocol::axis(nextMessageId(), {}, localTimestamp()));
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
    receiving_ = false;
    if (socket_fd_ >= 0)
    {
      ::shutdown(socket_fd_, SHUT_RDWR);
    }
    if (receive_thread_.joinable())
    {
      receive_thread_.join();
    }
    if (socket_fd_ >= 0)
    {
      ::close(socket_fd_);
      socket_fd_ = -1;
    }
  }

 private:
  void openSocket()
  {
    socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0)
    {
      RCLCPP_ERROR(get_logger(), "Cannot create M20 UDP socket: %s", std::strerror(errno));
      return;
    }
    sockaddr_in server{};
    server.sin_family = AF_INET;
    server.sin_port = htons(static_cast<std::uint16_t>(server_port_));
    if (::inet_pton(AF_INET, server_ip_.c_str(), &server.sin_addr) != 1 ||
        ::connect(socket_fd_, reinterpret_cast<sockaddr *>(&server), sizeof(server)) != 0)
    {
      RCLCPP_ERROR(get_logger(), "Cannot connect M20 UDP socket: %s", std::strerror(errno));
      ::close(socket_fd_);
      socket_fd_ = -1;
      return;
    }
    receiving_ = true;
    receive_thread_ = std::thread(&M20UdpBridgeNode::receiveLoop, this);
  }

  std::uint16_t nextMessageId()
  {
    return message_id_.fetch_add(1, std::memory_order_relaxed);
  }

  bool sendFrame(const std::vector<std::uint8_t> &frame)
  {
    if (socket_fd_ < 0 || frame.empty())
    {
      return false;
    }
    std::lock_guard<std::mutex> lock(send_mutex_);
    const ssize_t written = ::send(socket_fd_, frame.data(), frame.size(), MSG_NOSIGNAL);
    if (written != static_cast<ssize_t>(frame.size()))
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "M20 UDP send failed: %s",
                            std::strerror(errno));
      return false;
    }
    return true;
  }

  void receiveLoop()
  {
    std::vector<std::uint8_t> buffer(65551U);
    while (receiving_)
    {
      pollfd descriptor{socket_fd_, POLLIN, 0};
      const int ready = ::poll(&descriptor, 1, 200);
      if (ready <= 0 || !(descriptor.revents & POLLIN))
      {
        continue;
      }
      const ssize_t received = ::recv(socket_fd_, buffer.data(), buffer.size(), 0);
      if (received <= 0)
      {
        continue;
      }
      const ParsedDatagram parsed =
        M20Protocol::parse(std::vector<std::uint8_t>(buffer.begin(), buffer.begin() + received));
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!parsed.valid)
      {
        last_error_ = parsed.error;
        continue;
      }
      if (parsed.error_code && *parsed.error_code != 0)
      {
        last_error_ = "M20 response ErrorCode=" + std::to_string(*parsed.error_code);
      }
      if (parsed.status.received)
      {
        if (parsed.status.motion_state >= 0)
          robot_status_.motion_state = parsed.status.motion_state;
        if (parsed.status.gait >= 0)
          robot_status_.gait = parsed.status.gait;
        if (parsed.status.usage_mode >= 0)
          robot_status_.usage_mode = parsed.status.usage_mode;
        if (parsed.status.hard_estop >= 0)
          robot_status_.hard_estop = parsed.status.hard_estop;
        if (parsed.status.charge_state >= 0)
          robot_status_.charge_state = parsed.status.charge_state;
        if (parsed.status.direction >= 0)
          robot_status_.direction = parsed.status.direction;
        if (!parsed.status.version.empty())
          robot_status_.version = parsed.status.version;
        robot_status_.received = true;
        last_status_time_ = std::chrono::steady_clock::now();
      }
    }
  }

  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_command_.vx = std::isfinite(message->linear.x) ? message->linear.x : 0.0;
    last_command_.vy = std::isfinite(message->linear.y) ? message->linear.y : 0.0;
    last_command_.yaw_rate = std::isfinite(message->angular.z) ? message->angular.z : 0.0;
    last_command_time_ = std::chrono::steady_clock::now();
  }

  void armCallback(const std_msgs::msg::Bool::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (message->data && motion_output_enabled_ && pending_action_ == PendingAction::NONE &&
        statusFresh(std::chrono::steady_clock::now()) && statusAllowsMotion(robot_status_))
    {
      armed_ = true;
      last_error_.clear();
    }
    else
    {
      armed_ = false;
      last_command_ = {};
      if (message->data)
      {
        last_error_ = "Arm rejected: M20 BasicStatus safety conditions are not satisfied";
      }
    }
    RCLCPP_WARN(get_logger(), "M20 motion arm=%s", armed_ ? "true" : "false");
  }

  void actionCallback(const std_msgs::msg::String::SharedPtr message)
  {
    if (!motion_output_enabled_)
    {
      RCLCPP_WARN(get_logger(), "Ignoring action '%s': motion output is disabled",
                  message->data.c_str());
      return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (message->data == "standup")
      pending_action_ = PendingAction::STANDUP;
    else if (message->data == "liedown")
      pending_action_ = PendingAction::LIEDOWN;
    else if (message->data == "passive" || message->data == "soft_stop")
    {
      pending_action_ = PendingAction::SOFT_STOP;
    }
    else if (message->data == "flat_gait")
      pending_action_ = PendingAction::FLAT_GAIT;
    else if (message->data == "stair_gait")
      pending_action_ = PendingAction::STAIR_GAIT;
    else if (message->data == "disarm")
    {
      armed_ = false;
      pending_action_ = PendingAction::NONE;
      return;
    }
    else
    {
      RCLCPP_WARN(get_logger(), "Unknown M20 action '%s'", message->data.c_str());
      return;
    }
    armed_ = false;
    action_started_ = std::chrono::steady_clock::now();
    last_action_send_ = action_started_ - action_retry_ * 2;
  }

  void heartbeatTimer()
  {
    sendFrame(M20Protocol::heartbeat(nextMessageId(), localTimestamp()));
  }

  bool statusFresh(const std::chrono::steady_clock::time_point now) const
  {
    return robot_status_.received && now - last_status_time_ <= status_timeout_;
  }

  bool motionStateAllowsOutput(const std::chrono::steady_clock::time_point now) const
  {
    return statusFresh(now) && statusAllowsMotion(robot_status_);
  }

  void processPendingAction(const std::chrono::steady_clock::time_point now)
  {
    if (pending_action_ == PendingAction::NONE)
    {
      return;
    }
    if (now - action_started_ > action_timeout_)
    {
      last_error_ = std::string("M20 action timed out: ") + actionName(pending_action_);
      pending_action_ = PendingAction::NONE;
      return;
    }
    if (now - last_action_send_ < action_retry_)
    {
      return;
    }
    if (!statusFresh(now))
    {
      last_error_ = "Cannot execute action without fresh M20 BasicStatus";
      return;
    }

    std::vector<std::uint8_t> frame;
    switch (pending_action_)
    {
      case PendingAction::STANDUP:
        if (robot_status_.usage_mode != 0)
        {
          frame = M20Protocol::usageMode(nextMessageId(), 0, localTimestamp());
        }
        else if (robot_status_.motion_state == 17)
        {
          pending_action_ = PendingAction::NONE;
          return;
        }
        else if (robot_status_.motion_state == 1)
        {
          frame = M20Protocol::motionState(nextMessageId(), 17, localTimestamp());
        }
        else
        {
          frame = M20Protocol::motionState(nextMessageId(), 1, localTimestamp());
        }
        break;
      case PendingAction::LIEDOWN:
        if (robot_status_.motion_state == 4)
        {
          pending_action_ = PendingAction::NONE;
          return;
        }
        if (now - action_started_ >= std::chrono::milliseconds(500))
        {
          frame = M20Protocol::motionState(nextMessageId(), 4, localTimestamp());
        }
        break;
      case PendingAction::SOFT_STOP:
        if (robot_status_.motion_state == 2)
        {
          pending_action_ = PendingAction::NONE;
          return;
        }
        frame = M20Protocol::motionState(nextMessageId(), 2, localTimestamp());
        break;
      case PendingAction::FLAT_GAIT:
        if (robot_status_.motion_state != 17)
        {
          last_error_ = "Flat gait requires confirmed RL-control state";
          return;
        }
        if (robot_status_.gait == 0x3002)
        {
          pending_action_ = PendingAction::NONE;
          return;
        }
        frame = M20Protocol::gait(nextMessageId(), 0x3002, localTimestamp());
        break;
      case PendingAction::STAIR_GAIT:
        if (robot_status_.motion_state != 17)
        {
          last_error_ = "Stair gait requires confirmed RL-control state";
          return;
        }
        if (robot_status_.gait == 0x3003)
        {
          pending_action_ = PendingAction::NONE;
          return;
        }
        frame = M20Protocol::gait(nextMessageId(), 0x3003, localTimestamp());
        break;
      case PendingAction::NONE:
        return;
    }
    if (!frame.empty() && sendFrame(frame))
    {
      last_action_send_ = now;
    }
  }

  void commandTimer()
  {
    if (!motion_output_enabled_)
    {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    VelocityCommand command;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      processPendingAction(now);
      const bool fresh_command = now - last_command_time_ <= cmd_timeout_;
      if (armed_ && pending_action_ == PendingAction::NONE && fresh_command &&
          motionStateAllowsOutput(now))
      {
        command = last_command_;
      }
    }
    const auto axis = normalizeAxisCommand(command, axis_parameters_);
    sendFrame(M20Protocol::axis(nextMessageId(), axis, localTimestamp()));
  }

  void publishStatus()
  {
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(state_mutex_);
    std::ostringstream json;
    json << "{\"socket_ready\":" << (socket_fd_ >= 0 ? "true" : "false")
         << ",\"motion_output_enabled\":" << (motion_output_enabled_ ? "true" : "false")
         << ",\"armed\":" << (armed_ ? "true" : "false")
         << ",\"status_fresh\":" << (statusFresh(now) ? "true" : "false")
         << ",\"command_fresh\":" << (now - last_command_time_ <= cmd_timeout_ ? "true" : "false")
         << ",\"pending_action\":\"" << actionName(pending_action_)
         << "\",\"motion_state\":" << robot_status_.motion_state
         << ",\"gait\":" << robot_status_.gait << ",\"usage_mode\":" << robot_status_.usage_mode
         << ",\"hard_estop\":" << robot_status_.hard_estop
         << ",\"charge_state\":" << robot_status_.charge_state
         << ",\"direction\":" << robot_status_.direction << ",\"version\":\""
         << robot_status_.version << "\",\"last_error\":\"" << last_error_ << "\"}";
    std_msgs::msg::String status;
    status.data = json.str();
    status_publisher_->publish(status);
  }

  std::string server_ip_;
  int server_port_{30000};
  bool motion_output_enabled_{false};
  std::atomic<bool> armed_{false};
  AxisParameters axis_parameters_;
  std::chrono::milliseconds cmd_timeout_{250};
  std::chrono::milliseconds status_timeout_{1200};
  std::chrono::milliseconds action_retry_{500};
  std::chrono::milliseconds action_timeout_{15000};
  int socket_fd_{-1};
  std::atomic<bool> receiving_{false};
  std::thread receive_thread_;
  std::mutex send_mutex_;
  std::atomic<std::uint16_t> message_id_{0};
  std::mutex state_mutex_;
  VelocityCommand last_command_;
  RobotStatus robot_status_;
  std::string last_error_;
  PendingAction pending_action_{PendingAction::NONE};
  std::chrono::steady_clock::time_point last_command_time_;
  std::chrono::steady_clock::time_point last_status_time_;
  std::chrono::steady_clock::time_point action_started_;
  std::chrono::steady_clock::time_point last_action_send_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_subscription_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace m20::follow

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<m20::follow::M20UdpBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
