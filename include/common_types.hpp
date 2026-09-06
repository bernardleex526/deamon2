/**
 * @file common_types.hpp
 * @brief 公共类型定义和常量
 */

#ifndef COMMON_TYPES_HPP
#define COMMON_TYPES_HPP

#include <mutex>
#include <atomic>
#include <string>
#include <vector>
#include <functional>
#include <chrono>

// ----- 常量定义 -----
constexpr double FOLLOW_DIST = 0.4;           // 机器人与目标的预设距离 (米)
constexpr double TARGET_RADIUS = 0.3;         // 目标搜索半径 (米)
constexpr double LINEAR_SCALE_FACTOR = 0.5;   // 前后运动速度比例系数
constexpr double ANGULAR_SCALE_FACTOR = 1.0;  // 旋转运动速度比例系数
constexpr double LINEAR_Y_SCALE_FACTOR = 1.0; // 左右运动速度比例系数
constexpr double RECTANGLE_WIDTH = 0.35;      // 矩形宽度 (米)

// 速度限制
constexpr double MAX_LINEAR_SPEED = 1.0;
constexpr double MAX_ANGULAR_SPEED = 1.0;

// 势场法避障参数
constexpr double APF_INFLUENCE_DIST = 0.25;   // 障碍物影响距离 (米)
constexpr double APF_REPULSE_GAIN = 0.01;      // 排斥力增益
constexpr double APF_EMERGENCY_DIST = 0.2;    // 紧急停止距离 (米)
constexpr double APF_SLOWDOWN_DIST = 0.25;    // 减速距离 (米)

// 机器人框架排除区域（雷达可能扫描到的内部支架）
constexpr double ROBOT_FRAME_FRONT = 0.15;    // 前方排除范围 (米)
constexpr double ROBOT_FRAME_BACK = 0.35;     // 后方排除范围 (米)
constexpr double ROBOT_FRAME_LEFT = 0.15;     // 左侧排除范围 (米)
constexpr double ROBOT_FRAME_RIGHT = 0.15;    // 右侧排除范围 (米)

// OpenCV可视化参数（调试模式）
constexpr int WINDOW_SIZE = 800;
constexpr int SPEED_DISPLAY_HEIGHT = 150;
constexpr int TOTAL_WINDOW_HEIGHT = WINDOW_SIZE + SPEED_DISPLAY_HEIGHT;
constexpr double DISPLAY_WORLD_SIZE_M = 4.0;
constexpr double METERS_TO_PIXELS = WINDOW_SIZE / DISPLAY_WORLD_SIZE_M;
constexpr double ROBOT_Y_OFFSET_M = 0.5;

constexpr int SPEED_BAR_LENGTH = 50;
constexpr int SPEED_ARC_RADIUS = 40;

// 网络端口
constexpr int UDP_SEND_PORT = 8888;
constexpr int UDP_RECV_PORT = 8889;
constexpr int HTTP_PORT = 8080;
constexpr int WS_PORT = 8890;

/**
 * @brief 控制模式枚举
 */
enum ControlMode {
    MODE_DIRECT = 0,  // 直接控制模式
    MODE_FOLLOW = 1,  // 跟随模式
    MODE_NAV = 2      // 导航模式
};

/**
 * @brief 速度指令结构体
 */
struct VelocityCmd {
    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;
};

/**
 * @brief 共享状态 - 各模块间共享的数据
 */
struct SharedState {
    // 目标位置 (需要保护)
    std::mutex target_mutex;
    double target_x = FOLLOW_DIST;
    double target_y = 0.0;
    
    // 速度指令缓存 (需要保护)
    std::mutex velocity_mutex;
    double cached_vx = 0.0;
    double cached_vy = 0.0;
    double cached_wz = 0.0;
    
    // 雷达点云缓存 (需要保护)
    std::mutex scan_data_mutex;
    std::vector<std::pair<double, double>> cached_points;
    
    // 直接控制指令 (需要保护)
    std::mutex direct_cmd_mutex;
    double direct_vx = 0.0;
    double direct_vy = 0.0;
    double direct_wz = 0.0;
    std::chrono::steady_clock::time_point direct_received{};
    double direct_timeout = 0.3;
    
    // 原子状态
    std::atomic<bool> active{false};
    std::atomic<bool> is_moving_enabled{false};
    std::atomic<int> control_mode{MODE_FOLLOW};
    
    // 获取目标位置
    void getTarget(double& x, double& y) {
        std::lock_guard<std::mutex> lock(target_mutex);
        x = target_x;
        y = target_y;
    }
    
    // 设置目标位置
    void setTarget(double x, double y) {
        std::lock_guard<std::mutex> lock(target_mutex);
        target_x = x;
        target_y = y;
    }
    
    // 获取速度缓存
    void getVelocity(double& vx, double& vy, double& wz) {
        std::lock_guard<std::mutex> lock(velocity_mutex);
        vx = cached_vx;
        vy = cached_vy;
        wz = cached_wz;
    }
    
    // 设置速度缓存
    void setVelocity(double vx, double vy, double wz) {
        std::lock_guard<std::mutex> lock(velocity_mutex);
        cached_vx = vx;
        cached_vy = vy;
        cached_wz = wz;
    }
    
    // 获取直接控制指令
    void getDirectCmd(double& vx, double& vy, double& wz) {
        std::lock_guard<std::mutex> lock(direct_cmd_mutex);
        vx = direct_vx;
        vy = direct_vy;
        wz = direct_wz;
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         direct_received).count() > direct_timeout) {
            vx = vy = wz = 0.0;
        }
    }
    
    // 设置直接控制指令
    void setDirectCmd(double vx, double vy, double wz) {
        std::lock_guard<std::mutex> lock(direct_cmd_mutex);
        direct_vx = vx;
        direct_vy = vy;
        direct_wz = wz;
        direct_received = std::chrono::steady_clock::now();
    }
    
    // 获取点云数据
    std::vector<std::pair<double, double>> getPoints() {
        std::lock_guard<std::mutex> lock(scan_data_mutex);
        return cached_points;
    }
    
    // 设置点云数据
    void setPoints(std::vector<std::pair<double, double>>&& points) {
        std::lock_guard<std::mutex> lock(scan_data_mutex);
        cached_points = std::move(points);
    }
};

/**
 * @brief 简易JSON解析工具
 */
class JsonParser {
public:
    // 从JSON中提取数值
    static double extractNumber(const std::string& str, const std::string& key) {
        size_t pos = str.find("\"" + key + "\"");
        if (pos == std::string::npos) return 0;
        pos = str.find(":", pos);
        if (pos == std::string::npos) return 0;
        pos++;
        while (pos < str.size() && (str[pos] == ' ' || str[pos] == '\t')) pos++;
        size_t end = pos;
        while (end < str.size() && (std::isdigit(str[end]) || str[end] == '.' || str[end] == '-')) end++;
        if (end > pos) {
            return std::stod(str.substr(pos, end - pos));
        }
        return 0;
    }
    
    // 解析x, y坐标
    static bool parseXY(const std::string& json, double& x, double& y) {
        x = extractNumber(json, "x");
        y = extractNumber(json, "y");
        return true;
    }
    
    // 检查消息类型
    static bool hasType(const std::string& json, const std::string& type) {
        return json.find("\"type\":\"" + type + "\"") != std::string::npos ||
               json.find("\"type\": \"" + type + "\"") != std::string::npos;
    }
    
    // 检查布尔值
    static bool getBool(const std::string& json, const std::string& key) {
        return json.find("\"" + key + "\":true") != std::string::npos ||
               json.find("\"" + key + "\": true") != std::string::npos;
    }
    // 提取字符串值
    static std::string extractString(const std::string& str, const std::string& key) {
        size_t pos = str.find("\"" + key + "\"");
        if (pos == std::string::npos) return "";
        pos = str.find(":", pos);
        if (pos == std::string::npos) return "";
        pos = str.find("\"", pos);
        if (pos == std::string::npos) return "";
        size_t start = pos + 1;
        size_t end = str.find("\"", start);
        if (end == std::string::npos) return "";
        return str.substr(start, end - start);
    }
};

#endif // COMMON_TYPES_HPP
