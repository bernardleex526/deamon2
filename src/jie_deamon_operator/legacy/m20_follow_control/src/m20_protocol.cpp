#include "m20_follow_control/m20_protocol.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <sstream>

namespace m20::follow
{
namespace
{

std::optional<long long> extractInteger(const std::string &json, const std::string &key)
{
  const auto key_position = json.find("\"" + key + "\"");
  if (key_position == std::string::npos)
  {
    return std::nullopt;
  }
  const auto colon = json.find(':', key_position + key.size() + 2U);
  if (colon == std::string::npos)
  {
    return std::nullopt;
  }
  const char *begin = json.c_str() + colon + 1U;
  char *end = nullptr;
  errno = 0;
  const long long value = std::strtoll(begin, &end, 0);
  if (begin == end || errno == ERANGE)
  {
    return std::nullopt;
  }
  return value;
}

std::optional<std::string> extractString(const std::string &json, const std::string &key)
{
  const auto key_position = json.find("\"" + key + "\"");
  if (key_position == std::string::npos)
  {
    return std::nullopt;
  }
  const auto colon = json.find(':', key_position + key.size() + 2U);
  const auto quote = colon == std::string::npos ? std::string::npos : json.find('"', colon + 1U);
  if (quote == std::string::npos)
  {
    return std::nullopt;
  }
  const auto end = json.find('"', quote + 1U);
  if (end == std::string::npos)
  {
    return std::nullopt;
  }
  return json.substr(quote + 1U, end - quote - 1U);
}

std::string envelope(int type, int command, const std::string &timestamp, const std::string &items)
{
  std::ostringstream output;
  output << "{\"PatrolDevice\":{\"Type\":" << type << ",\"Command\":" << command << ",\"Time\":\""
         << timestamp << "\",\"Items\":" << items << "}}";
  return output.str();
}

double normalized(double value, double safe_limit, double full_scale)
{
  if (safe_limit <= 0.0 || full_scale <= 0.0)
  {
    return 0.0;
  }
  return std::clamp(value, -safe_limit, safe_limit) / full_scale;
}

}  // namespace

NormalizedAxis normalizeAxisCommand(const VelocityCommand &command,
                                    const AxisParameters &parameters)
{
  NormalizedAxis result;
  result.x =
    std::clamp(normalized(command.vx, parameters.safe_max_x, parameters.full_scale_x), -1.0, 1.0);
  result.y =
    std::clamp(normalized(command.vy, parameters.safe_max_y, parameters.full_scale_y), -1.0, 1.0);
  result.yaw = std::clamp(
    normalized(command.yaw_rate, parameters.safe_max_yaw, parameters.full_scale_yaw), -1.0, 1.0);
  return result;
}

bool axisParametersValid(const AxisParameters &parameters)
{
  return std::isfinite(parameters.safe_max_x) && std::isfinite(parameters.safe_max_y) &&
         std::isfinite(parameters.safe_max_yaw) && std::isfinite(parameters.full_scale_x) &&
         std::isfinite(parameters.full_scale_y) && std::isfinite(parameters.full_scale_yaw) &&
         parameters.safe_max_x >= 0.0 && parameters.safe_max_y >= 0.0 &&
         parameters.safe_max_yaw >= 0.0 && parameters.full_scale_x > 0.0 &&
         parameters.full_scale_y > 0.0 && parameters.full_scale_yaw > 0.0 &&
         parameters.safe_max_x <= parameters.full_scale_x &&
         parameters.safe_max_y <= parameters.full_scale_y &&
         parameters.safe_max_yaw <= parameters.full_scale_yaw;
}

bool statusAllowsMotion(const RobotStatus &status)
{
  return status.received && status.usage_mode == 0 && status.motion_state == 17 &&
         status.hard_estop == 0 && status.charge_state == 0 && status.direction == 0;
}

std::vector<std::uint8_t> M20Protocol::heartbeat(std::uint16_t message_id,
                                                 const std::string &timestamp)
{
  return frame(message_id, envelope(100, 100, timestamp, "{}"));
}

std::vector<std::uint8_t> M20Protocol::usageMode(std::uint16_t message_id, int mode,
                                                 const std::string &timestamp)
{
  return frame(message_id, envelope(1101, 5, timestamp, "{\"Mode\":" + std::to_string(mode) + "}"));
}

std::vector<std::uint8_t> M20Protocol::motionState(std::uint16_t message_id, int state,
                                                   const std::string &timestamp)
{
  return frame(message_id,
               envelope(2, 22, timestamp, "{\"MotionParam\":" + std::to_string(state) + "}"));
}

std::vector<std::uint8_t> M20Protocol::gait(std::uint16_t message_id, int gait_value,
                                            const std::string &timestamp)
{
  return frame(message_id,
               envelope(2, 23, timestamp, "{\"GaitParam\":" + std::to_string(gait_value) + "}"));
}

std::vector<std::uint8_t> M20Protocol::axis(std::uint16_t message_id,
                                            const NormalizedAxis &axis_command,
                                            const std::string &timestamp)
{
  std::ostringstream items;
  items << std::fixed << std::setprecision(6) << "{\"X\":" << std::clamp(axis_command.x, -1.0, 1.0)
        << ",\"Y\":" << std::clamp(axis_command.y, -1.0, 1.0)
        << ",\"Z\":0.000000,\"Roll\":0.000000,\"Pitch\":0.000000"
        << ",\"Yaw\":" << std::clamp(axis_command.yaw, -1.0, 1.0) << "}";
  return frame(message_id, envelope(2, 21, timestamp, items.str()));
}

std::vector<std::uint8_t> M20Protocol::frame(std::uint16_t message_id, const std::string &json)
{
  if (json.size() > std::numeric_limits<std::uint16_t>::max())
  {
    return {};
  }
  const auto length = static_cast<std::uint16_t>(json.size());
  std::vector<std::uint8_t> result(16U, 0U);
  result[0] = 0xeb;
  result[1] = 0x91;
  result[2] = 0xeb;
  result[3] = 0x90;
  result[4] = static_cast<std::uint8_t>(length & 0xffU);
  result[5] = static_cast<std::uint8_t>((length >> 8U) & 0xffU);
  result[6] = static_cast<std::uint8_t>(message_id & 0xffU);
  result[7] = static_cast<std::uint8_t>((message_id >> 8U) & 0xffU);
  result[8] = 0x01;
  result.insert(result.end(), json.begin(), json.end());
  return result;
}

ParsedDatagram M20Protocol::parse(const std::vector<std::uint8_t> &datagram)
{
  ParsedDatagram result;
  if (datagram.size() < 16U)
  {
    result.error = "datagram is shorter than the M20 APDU header";
    return result;
  }
  if (datagram[0] != 0xeb || datagram[1] != 0x91 || datagram[2] != 0xeb || datagram[3] != 0x90)
  {
    result.error = "invalid M20 APDU sync bytes";
    return result;
  }
  if (datagram[8] != 0x01)
  {
    result.error = "only JSON M20 APDU messages are supported";
    return result;
  }
  const std::size_t payload_size =
    static_cast<std::size_t>(datagram[4]) | (static_cast<std::size_t>(datagram[5]) << 8U);
  if (datagram.size() != payload_size + 16U)
  {
    result.error = "M20 APDU payload length does not match the datagram";
    return result;
  }

  result.message_id =
    static_cast<std::uint16_t>(datagram[6]) | (static_cast<std::uint16_t>(datagram[7]) << 8U);
  result.json.assign(datagram.begin() + 16, datagram.end());
  const auto type = extractInteger(result.json, "Type");
  const auto command = extractInteger(result.json, "Command");
  if (!type || !command)
  {
    result.error = "M20 JSON is missing Type or Command";
    return result;
  }
  result.type = static_cast<int>(*type);
  result.command = static_cast<int>(*command);
  if (const auto error_code = extractInteger(result.json, "ErrorCode"))
  {
    result.error_code = static_cast<int>(*error_code);
  }

  bool has_status = false;
  const auto copy_status = [&has_status](const auto &value, int &output)
  {
    if (value)
    {
      output = static_cast<int>(*value);
      has_status = true;
    }
  };
  copy_status(extractInteger(result.json, "MotionState"), result.status.motion_state);
  copy_status(extractInteger(result.json, "Gait"), result.status.gait);
  copy_status(extractInteger(result.json, "ControlUsageMode"), result.status.usage_mode);
  copy_status(extractInteger(result.json, "HES"), result.status.hard_estop);
  copy_status(extractInteger(result.json, "Charge"), result.status.charge_state);
  copy_status(extractInteger(result.json, "Direction"), result.status.direction);
  if (const auto version = extractString(result.json, "Version"))
  {
    result.status.version = *version;
    has_status = true;
  }
  result.status.received = has_status;
  result.valid = true;
  return result;
}

}  // namespace m20::follow
