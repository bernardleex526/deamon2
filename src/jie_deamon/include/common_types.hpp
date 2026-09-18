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
#include <cmath>

// M20 预览限幅（NEW-06：仅算法预览，物理运动未实现）
constexpr double PREVIEW_MAX_LINEAR_X = 0.3;   // 预览 |vx| 上限 (m/s)
constexpr double PREVIEW_MAX_LINEAR_Y = 0.3;   // 预览 |vy| 上限 (m/s)
constexpr double PREVIEW_MAX_ANGULAR_Z = 0.6;  // 预览 |wz| 上限 (rad/s)

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

    // 原子状态
    std::atomic<bool> active{false};
    std::atomic<bool> is_moving_enabled{false};
    std::atomic<int> control_mode{MODE_FOLLOW};

    // M20预览模式（NEW-06）：true=仅算法/NavCmd预览，禁Android/动作/直控
    std::atomic<bool> m20_preview{false};
    // 目标是否已人工选定（仅selectTarget置位；算法跟踪不自动置位）
    std::atomic<bool> target_selected{false};
    // 当前帧目标跟踪是否有效（跟踪更新置位；丢失/超时清零）
    std::atomic<bool> tracking_valid{false};
    // 目标选择代数：selectTarget递增；算法跟踪写回前校验，防旧scan覆盖新人工选择
    // （注：同帧内"质心按旧目标计算"的窗口仍存在，已按单帧粒度documented，不做细粒度取消）
    std::atomic<uint64_t> target_generation{0};

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

    // 人工选定目标（Web/App选点路径专用）：置selected；
    // NEW-06 P1: 若此前moving=true必须关闭——换目标需重新使能，避免直接续追；
    // NEW-06 P1: 递增generation，使同帧旧scan的跟踪写回失效；selected/valid更新在target_mutex内一致完成
    void selectTarget(double x, double y) {
        std::lock_guard<std::mutex> lock(target_mutex);
        target_x = x;
        target_y = y;
        target_selected.store(true);
        tracking_valid.store(false);
        is_moving_enabled.store(false);
        target_generation.store(target_generation.load() + 1);
    }

    // 读取目标及其generation（同锁一致快照，供算法跟踪写回校验）
    uint64_t getTargetWithGen(double& x, double& y) {
        std::lock_guard<std::mutex> lock(target_mutex);
        x = target_x;
        y = target_y;
        return target_generation.load();
    }

    // 算法跟踪更新目标质心（不置selected，不改变人工选择语义）
    // NEW-06 P1: expected_gen与当前generation不一致（扫描期间人工换选点）则拒绝写回
    void updateTrackedTarget(double x, double y, uint64_t expected_gen) {
        std::lock_guard<std::mutex> lock(target_mutex);
        if (target_generation.load() != expected_gen) {
            return;
        }
        target_x = x;
        target_y = y;
        tracking_valid.store(true);
    }

    // 清除跟踪：目标丢失/超时后，需重新人工选点并重新使能（与目标读取同锁，避免竞态）
    void clearTracking() {
        std::lock_guard<std::mutex> lock(target_mutex);
        target_selected.store(false);
        tracking_valid.store(false);
        is_moving_enabled.store(false);
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
    }

    // 设置直接控制指令
    void setDirectCmd(double vx, double vy, double wz) {
        std::lock_guard<std::mutex> lock(direct_cmd_mutex);
        direct_vx = vx;
        direct_vy = vy;
        direct_wz = wz;
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

    // 严格数值提取：字段必须存在、完整token为合法JSON数字、且为有限值；
    // 失败返回false（不抛异常不崩进程）。
    // 注意：这是针对目标字段(x/y)的窄字段校验，不是完整JSON解析器。
    static bool extractNumberStrict(const std::string& str, const std::string& key, double& out) {
        size_t pos = str.find("\"" + key + "\"");
        if (pos == std::string::npos) return false;
        pos = str.find(":", pos);
        if (pos == std::string::npos) return false;
        pos++;
        // token前允许标准JSON空白
        while (pos < str.size() && (str[pos] == ' ' || str[pos] == '\t' ||
                                    str[pos] == '\r' || str[pos] == '\n')) pos++;
        size_t end = pos;
        while (end < str.size() &&
               (std::isdigit(static_cast<unsigned char>(str[end])) || str[end] == '.' ||
                str[end] == '-' || str[end] == '+' || str[end] == 'e' || str[end] == 'E')) end++;
        if (end == pos) return false;
        const std::string token = str.substr(pos, end - pos);
        // NEW-06 P1: 数字token必须完整合法——拒绝"1e"、"1..2"、"1junk"、"+1"、"01"、
        // "-01"、"NaN"等非法/截断解析输入
        if (!isCompleteNumericToken(token)) return false;
        // 合法JSON分隔: token后允许标准JSON空白(' ','\t','\r','\n')直到 ',' '}' ']'
        // （第2轮P1修复: {"x": 2 , "y": 0 } 是合法JSON，之前误拒）
        while (end < str.size() &&
               (str[end] == ' ' || str[end] == '\t' || str[end] == '\r' || str[end] == '\n')) end++;
        if (end < str.size()) {
            const char next = str[end];
            if (next != ',' && next != '}' && next != ']') return false;
        }
        try {
            const double v = std::stod(token);
            if (!std::isfinite(v)) return false;
            out = v;
            return true;
        } catch (...) {
            return false;
        }
    }

    // 完整JSON数字token校验: [ '-' ] int [ frac ] [ exp ]
    // 仅允许'-'符号（'+1'非法）; int至少1位且0后不可直接跟数字（'01'/'-01'非法）;
    // frac: '.' 后至少1位数字; exp: e/E [+-] 至少1位数字
    static bool isCompleteNumericToken(const std::string& t) {
        size_t i = 0;
        if (i < t.size() && t[i] == '-') i++;   // JSON数字仅允许'-'符号
        const size_t int_start = i;
        while (i < t.size() && std::isdigit(static_cast<unsigned char>(t[i]))) i++;
        if (i == int_start) return false;       // 整数部分至少1位数字
        // 前导零: '0'后不可直接跟数字（"01"、"-01"非法；"0.5"/"0e3"合法）
        if (t[int_start] == '0' && (i - int_start) > 1) return false;
        if (i < t.size() && t[i] == '.') {
            i++;
            const size_t frac_start = i;
            while (i < t.size() && std::isdigit(static_cast<unsigned char>(t[i]))) i++;
            if (i == frac_start) return false;  // "1." 非法（"1..2"同理在消耗检查被拒）
        }
        if (i < t.size() && (t[i] == 'e' || t[i] == 'E')) {
            i++;
            if (i < t.size() && (t[i] == '+' || t[i] == '-')) i++;
            const size_t exp_start = i;
            while (i < t.size() && std::isdigit(static_cast<unsigned char>(t[i]))) i++;
            if (i == exp_start) return false;   // "1e" 非法
        }
        return i == t.size();  // 整个token必须被消耗（"1junk"非法）
    }

    // 解析x,y坐标（严格版）：两者都必须明确存在且有限；非法输入返回false
    static bool parseXYStrict(const std::string& json, double& x, double& y) {
        double tx = 0, ty = 0;
        if (!extractNumberStrict(json, "x", tx)) return false;
        if (!extractNumberStrict(json, "y", ty)) return false;
        x = tx;
        y = ty;
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
