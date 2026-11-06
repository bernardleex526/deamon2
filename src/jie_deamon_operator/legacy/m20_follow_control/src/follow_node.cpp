#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sstream>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "m20_follow_control/follow_controller.hpp"
#include "m20_follow_control/pointcloud_wire.hpp"

namespace m20::follow
{
namespace
{

FollowParameters loadFollowParameters(rclcpp::Node &node)
{
  FollowParameters parameters;
  parameters.follow_distance = node.declare_parameter("follow.distance", 1.2);
  parameters.target_radius = node.declare_parameter("follow.target_radius", 0.4);
  parameters.min_target_points = static_cast<std::size_t>(
    std::max<std::int64_t>(1, node.declare_parameter("follow.min_target_points", 3)));
  parameters.target_smoothing = node.declare_parameter("follow.target_smoothing", 0.35);
  parameters.body_min_x = node.declare_parameter("body.min_x", -0.45);
  parameters.body_max_x = node.declare_parameter("body.max_x", 0.45);
  parameters.body_min_y = node.declare_parameter("body.min_y", -0.25);
  parameters.body_max_y = node.declare_parameter("body.max_y", 0.25);
  parameters.influence_distance = node.declare_parameter("obstacle.influence_distance", 1.0);
  parameters.slowdown_distance = node.declare_parameter("obstacle.slowdown_distance", 1.0);
  parameters.emergency_distance = node.declare_parameter("obstacle.emergency_distance", 0.7);
  parameters.repulsion_gain = node.declare_parameter("obstacle.repulsion_gain", 0.05);
  parameters.linear_gain = node.declare_parameter("control.linear_gain", 0.5);
  parameters.lateral_gain = node.declare_parameter("control.lateral_gain", 0.4);
  parameters.yaw_gain = node.declare_parameter("control.yaw_gain", 0.8);
  parameters.linear_deadband = node.declare_parameter("control.linear_deadband", 0.05);
  parameters.lateral_deadband = node.declare_parameter("control.lateral_deadband", 0.05);
  parameters.yaw_deadband = node.declare_parameter("control.yaw_deadband", 0.08);
  parameters.max_linear_x = node.declare_parameter("control.max_linear_x", 0.25);
  parameters.max_linear_y = node.declare_parameter("control.max_linear_y", 0.15);
  parameters.max_yaw_rate = node.declare_parameter("control.max_yaw_rate", 0.3);
  parameters.max_linear_acceleration =
    node.declare_parameter("control.max_linear_acceleration", 0.3);
  parameters.max_yaw_acceleration = node.declare_parameter("control.max_yaw_acceleration", 0.5);
  return parameters;
}

}  // namespace

class M20FollowNode final : public rclcpp::Node
{
 public:
  M20FollowNode() : Node("m20_follow_node"), controller_(loadFollowParameters(*this))
  {
    enabled_ = declare_parameter("enabled", false);
    input_mode_ = declare_parameter<std::string>("input_mode", "ros");
    input_topic_ = declare_parameter<std::string>("input_topic", "/LIDAR/POINTS");
    socket_path_ = declare_parameter<std::string>("socket_path", "/tmp/m20_follow_lidar.sock");
    output_topic_ = declare_parameter<std::string>("output_topic", "/m20_follow/cmd_vel");
    target_topic_ = declare_parameter<std::string>("target_topic", "/follow/target");
    enable_topic_ = declare_parameter<std::string>("enable_topic", "/follow/enable");
    status_topic_ = declare_parameter<std::string>("status_topic", "/follow/status");
    min_height_ = declare_parameter("cloud.min_height", -0.30);
    max_height_ = declare_parameter("cloud.max_height", 0.60);
    min_range_ = declare_parameter("cloud.min_range", 0.30);
    max_range_ = declare_parameter("cloud.max_range", 8.0);
    yaw_offset_ = declare_parameter("cloud.yaw_offset", 0.0);
    invert_x_ = declare_parameter("cloud.invert_x", false);
    invert_y_ = declare_parameter("cloud.invert_y", false);
    cloud_timeout_ = std::chrono::milliseconds(
      std::max<std::int64_t>(50, declare_parameter("cloud.timeout_ms", 350)));
    const double output_hz = std::max(1.0, declare_parameter("control.output_hz", 20.0));

    velocity_publisher_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);
    status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
    if (input_mode_ == "ros")
    {
      cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS().keep_last(5),
        std::bind(&M20FollowNode::cloudCallback, this, std::placeholders::_1));
    }
    else if (input_mode_ == "socket")
    {
      socket_thread_ = std::thread(&M20FollowNode::socketLoop, this);
    }
    else
    {
      throw std::invalid_argument("input_mode must be 'ros' or 'socket'");
    }
    target_subscription_ = create_subscription<geometry_msgs::msg::PointStamped>(
      target_topic_, 10, std::bind(&M20FollowNode::targetCallback, this, std::placeholders::_1));
    enable_subscription_ = create_subscription<std_msgs::msg::Bool>(
      enable_topic_, 10, std::bind(&M20FollowNode::enableCallback, this, std::placeholders::_1));
    output_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::duration<double>(1.0 / output_hz)),
                                      std::bind(&M20FollowNode::outputTimer, this));

    last_cloud_time_ = std::chrono::steady_clock::now() - cloud_timeout_ * 2;
    last_update_time_ = last_cloud_time_;
    RCLCPP_INFO(get_logger(), "M20 follow node ready: mode=%s input=%s output=%s enabled=%s",
                input_mode_.c_str(),
                input_mode_ == "ros" ? input_topic_.c_str() : socket_path_.c_str(),
                output_topic_.c_str(), enabled_ ? "true" : "false");
  }

  ~M20FollowNode() override
  {
    socket_stop_ = true;
    const int fd = socket_fd_.exchange(-1);
    if (fd >= 0)
    {
      ::shutdown(fd, SHUT_RDWR);
    }
    if (socket_thread_.joinable())
    {
      socket_thread_.join();
    }
  }

 private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    std::vector<Point3D> points;
    points.reserve(static_cast<std::size_t>(message->width) * message->height);
    try
    {
      sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*message, "z");
      for (; x != x.end(); ++x, ++y, ++z)
      {
        points.push_back(Point3D{*x, *y, *z});
      }
    }
    catch (const std::runtime_error &exception)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid M20 PointCloud2: %s",
                            exception.what());
      return;
    }

    processPointCloud(points);
  }

  void processPointCloud(const std::vector<Point3D> &source_points)
  {
    const auto now = std::chrono::steady_clock::now();
    std::vector<Point2D> points;
    points.reserve(source_points.size());
    const double cosine = std::cos(yaw_offset_);
    const double sine = std::sin(yaw_offset_);
    for (const auto &source : source_points)
    {
      if (!std::isfinite(source.x) || !std::isfinite(source.y) || !std::isfinite(source.z) ||
          source.z < min_height_ || source.z > max_height_)
      {
        continue;
      }
      const double source_x = invert_x_ ? -source.x : source.x;
      const double source_y = invert_y_ ? -source.y : source.y;
      Point2D point;
      point.x = cosine * source_x - sine * source_y;
      point.y = sine * source_x + cosine * source_y;
      const double range = std::hypot(point.x, point.y);
      if (range >= min_range_ && range <= max_range_)
      {
        points.push_back(point);
      }
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    const double dt = std::chrono::duration<double>(now - last_update_time_).count();
    last_cloud_time_ = now;
    last_update_time_ = now;
    filtered_point_count_ = points.size();
    last_result_ = enabled_ ? controller_.update(points, dt) : FollowResult{};
  }

  static bool readExact(int fd, void *output, std::size_t size)
  {
    auto *bytes = static_cast<std::uint8_t *>(output);
    std::size_t received = 0;
    while (received < size)
    {
      const ssize_t result = ::recv(fd, bytes + received, size - received, 0);
      if (result == 0)
      {
        return false;
      }
      if (result < 0)
      {
        if (errno == EINTR)
        {
          continue;
        }
        return false;
      }
      received += static_cast<std::size_t>(result);
    }
    return true;
  }

  void socketLoop()
  {
    while (!socket_stop_)
    {
      const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
      if (fd < 0)
      {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
      sockaddr_un address{};
      address.sun_family = AF_UNIX;
      if (socket_path_.size() >= sizeof(address.sun_path))
      {
        RCLCPP_ERROR(get_logger(), "Point-cloud socket path is too long: %s", socket_path_.c_str());
        ::close(fd);
        return;
      }
      std::strncpy(address.sun_path, socket_path_.c_str(), sizeof(address.sun_path) - 1);
      if (::connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0)
      {
        ::close(fd);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
      socket_fd_ = fd;
      RCLCPP_INFO(get_logger(), "Connected to M20 point-cloud socket %s", socket_path_.c_str());
      while (!socket_stop_)
      {
        std::uint32_t header[2]{};
        if (!readExact(fd, header, sizeof(header)) || header[0] != kPointCloudWireMagic ||
            header[1] == 0U || header[1] > kMaxPointCloudWireBytes)
        {
          break;
        }
        std::vector<std::uint8_t> payload(header[1]);
        if (!readExact(fd, payload.data(), payload.size()))
        {
          break;
        }
        WirePointCloud cloud;
        std::vector<Point3D> points;
        std::string error;
        if (deserializePointCloudWire(payload, cloud, error) &&
            extractXyzPoints(cloud, points, error))
        {
          processPointCloud(points);
        }
        else
        {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                                "Invalid M20 point-cloud wire packet: %s", error.c_str());
        }
      }
      int expected = fd;
      socket_fd_.compare_exchange_strong(expected, -1);
      ::close(fd);
      if (!socket_stop_)
      {
        RCLCPP_WARN(get_logger(), "M20 point-cloud socket disconnected; retrying");
      }
    }
  }

  void targetCallback(const geometry_msgs::msg::PointStamped::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    controller_.setTarget(message->point.x, message->point.y);
    last_result_ = {};
    RCLCPP_INFO(get_logger(), "Follow target search center set to x=%.3f y=%.3f", message->point.x,
                message->point.y);
  }

  void enableCallback(const std_msgs::msg::Bool::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    enabled_ = message->data;
    if (!enabled_)
    {
      controller_.reset();
      last_result_ = {};
    }
    RCLCPP_WARN(get_logger(), "Follow output %s", enabled_ ? "enabled" : "disabled");
  }

  void outputTimer()
  {
    FollowResult result;
    bool enabled = false;
    bool cloud_fresh = false;
    std::size_t point_count = 0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      enabled = enabled_;
      cloud_fresh = std::chrono::steady_clock::now() - last_cloud_time_ <= cloud_timeout_;
      if (!cloud_fresh)
      {
        controller_.reset();
        last_result_ = {};
      }
      point_count = filtered_point_count_;
      result = last_result_;
    }

    geometry_msgs::msg::Twist output;
    if (enabled && cloud_fresh && result.target_visible && !result.emergency_stop)
    {
      output.linear.x = result.command.vx;
      output.linear.y = result.command.vy;
      output.angular.z = result.command.yaw_rate;
    }
    velocity_publisher_->publish(output);

    if (++status_tick_ % 4U == 0U)
    {
      std::ostringstream json;
      json << "{\"enabled\":" << (enabled ? "true" : "false") << ",\"input_mode\":\"" << input_mode_
           << "\""
           << ",\"cloud_fresh\":" << (cloud_fresh ? "true" : "false")
           << ",\"target_visible\":" << (result.target_visible ? "true" : "false")
           << ",\"emergency_stop\":" << (result.emergency_stop ? "true" : "false")
           << ",\"filtered_points\":" << point_count
           << ",\"target_points\":" << result.target_points << ",\"target_x\":" << result.target_x
           << ",\"target_y\":" << result.target_y << "}";
      std_msgs::msg::String status;
      status.data = json.str();
      status_publisher_->publish(status);
    }
  }

  FollowController controller_;
  std::mutex state_mutex_;
  bool enabled_{false};
  FollowResult last_result_;
  std::size_t filtered_point_count_{0};
  std::chrono::steady_clock::time_point last_cloud_time_;
  std::chrono::steady_clock::time_point last_update_time_;
  std::chrono::milliseconds cloud_timeout_{350};
  std::size_t status_tick_{0};
  std::string input_mode_;
  std::string input_topic_;
  std::string socket_path_;
  std::string output_topic_;
  std::string target_topic_;
  std::string enable_topic_;
  std::string status_topic_;
  double min_height_{-0.30};
  double max_height_{0.60};
  double min_range_{0.30};
  double max_range_{8.0};
  double yaw_offset_{0.0};
  bool invert_x_{false};
  bool invert_y_{false};
  std::atomic<bool> socket_stop_{false};
  std::atomic<int> socket_fd_{-1};
  std::thread socket_thread_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_subscription_;
  rclcpp::TimerBase::SharedPtr output_timer_;
};

}  // namespace m20::follow

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<m20::follow::M20FollowNode>());
  rclcpp::shutdown();
  return 0;
}
