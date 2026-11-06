#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "m20_follow_control/follow_controller.hpp"

namespace m20::follow
{

struct AxisParameters
{
  double safe_max_x{0.25};
  double safe_max_y{0.15};
  double safe_max_yaw{0.3};
  double full_scale_x{1.5};
  double full_scale_y{0.6};
  double full_scale_yaw{1.0};
};

struct NormalizedAxis
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct RobotStatus
{
  bool received{false};
  int motion_state{-1};
  int gait{-1};
  int usage_mode{-1};
  int hard_estop{-1};
  int charge_state{-1};
  int direction{-1};
  std::string version;
};

struct ParsedDatagram
{
  bool valid{false};
  std::uint16_t message_id{0};
  int type{-1};
  int command{-1};
  std::optional<int> error_code;
  RobotStatus status;
  std::string json;
  std::string error;
};

NormalizedAxis normalizeAxisCommand(const VelocityCommand &command,
                                    const AxisParameters &parameters);
bool axisParametersValid(const AxisParameters &parameters);
bool statusAllowsMotion(const RobotStatus &status);

class M20Protocol
{
 public:
  static std::vector<std::uint8_t> heartbeat(std::uint16_t message_id,
                                             const std::string &timestamp);
  static std::vector<std::uint8_t> usageMode(std::uint16_t message_id, int mode,
                                             const std::string &timestamp);
  static std::vector<std::uint8_t> motionState(std::uint16_t message_id, int state,
                                               const std::string &timestamp);
  static std::vector<std::uint8_t> gait(std::uint16_t message_id, int gait_value,
                                        const std::string &timestamp);
  static std::vector<std::uint8_t> axis(std::uint16_t message_id, const NormalizedAxis &axis,
                                        const std::string &timestamp);

  static ParsedDatagram parse(const std::vector<std::uint8_t> &datagram);

 private:
  static std::vector<std::uint8_t> frame(std::uint16_t message_id, const std::string &json);
};

}  // namespace m20::follow
