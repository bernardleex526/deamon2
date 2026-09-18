/**
 * @file web_comm.cpp
 * @brief Web通讯模块实现
 */

#include "web_comm.hpp"

std::string WebCommManager::getLocalIP() {
    struct ifaddrs *ifAddrStruct = NULL;
    void *tmpAddrPtr = NULL;
    std::string ip = "localhost";

    getifaddrs(&ifAddrStruct);
    for (auto ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr) continue;
        if (ifa->ifa_addr->sa_family == AF_INET) {
            tmpAddrPtr = &((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
            char addressBuffer[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
            std::string ipStr(addressBuffer);
            std::string ifName(ifa->ifa_name);
            if (ipStr != "127.0.0.1" && !ipStr.empty()) {
                ip = ipStr;
                if (ifName.find("w") == 0) break;
            }
        }
    }
    if (ifAddrStruct != NULL) freeifaddrs(ifAddrStruct);
    return ip;
}

void WebCommManager::start() {
    if (running_) return;
    running_ = true;

    http_server_ = std::make_unique<httplib::Server>();
    if (!web_root_.empty()) http_server_->set_mount_point("/", web_root_);

    // API: 获取状态
    http_server_->Get("/api/status", [this](const httplib::Request&, httplib::Response& res) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3);
        oss << "{\"is_moving_enabled\":" << (state_.is_moving_enabled.load() ? "true" : "false");
        oss << ",\"is_active\":" << (state_.active.load() ? "true" : "false");
        // NEW-06: 预览模式与目标选点/跟踪状态
        oss << ",\"m20_preview\":" << (state_.m20_preview.load() ? "true" : "false");
        oss << ",\"is_selected\":" << (state_.target_selected.load() ? "true" : "false");
        oss << ",\"is_tracking_valid\":" << (state_.tracking_valid.load() ? "true" : "false");
        double tx, ty; state_.getTarget(tx, ty);
        oss << ",\"target\":{\"x\":" << tx << ",\"y\":" << ty << "}}";
        res.set_content(oss.str(), "application/json");
    });

    // API: 设置目标
    // NEW-06: 严格校验——x/y必须明确存在且为有限数字，非法输入返回HTTP 400并拒绝
    // （不将缺失/非法字段静默置0，不抛异常）
    http_server_->Post("/api/set_target", [this](const httplib::Request& req, httplib::Response& res) {
        double x = 0, y = 0;
        if (JsonParser::parseXYStrict(req.body, x, y)) {
            state_.selectTarget(x, y);
            log("Web设置目标: (" + std::to_string(x) + ", " + std::to_string(y) + ")");
            res.set_content("{\"ok\":true}", "application/json");
        } else {
            res.status = 400;
            res.set_content("{\"ok\":false,\"error\":\"invalid target: need finite x and y\"}", "application/json");
        }
    });

    // API: 设置运动使能
    http_server_->Post("/api/set_moving", [this](const httplib::Request& req, httplib::Response& res) {
        bool enabled = req.body.find("true") != std::string::npos;
        // NEW-06 P1: m20预览下未人工选点时拒绝使能（避免反序授权：先使能后选点直接续追）
        if (state_.m20_preview.load() && enabled && !state_.target_selected.load()) {
            log("Web运动使能拒绝: 未人工选点，需先双击雷达图选目标");
            res.status = 400;
            res.set_content("{\"ok\":false,\"error\":\"select target before enabling motion\"}", "application/json");
            return;
        }
        state_.is_moving_enabled.store(enabled);
        log(std::string("Web运动使能: ") + (enabled ? "开启" : "关闭"));
        res.set_content("{\"ok\":true}", "application/json");
    });

    // API: 设置激活
    http_server_->Post("/api/set_active", [this](const httplib::Request& req, httplib::Response& res) {
        bool active = req.body.find("true") != std::string::npos;
        state_.active.store(active);
        log(std::string("Web跟随功能: ") + (active ? "开启" : "关闭"));
        res.set_content("{\"ok\":true}", "application/json");
    });

    http_thread_ = std::thread([this]() {
        log("HTTP服务器启动在端口 " + std::to_string(HTTP_PORT));
        http_server_->listen("0.0.0.0", HTTP_PORT);
    });

    startWebSocketServer();
}

void WebCommManager::stop() {
    running_ = false;
    if (http_server_) http_server_->stop();
    if (ws_server_socket_ >= 0) { close(ws_server_socket_); ws_server_socket_ = -1; }
    {
        std::lock_guard<std::mutex> lock(ws_clients_mutex_);
        for (int fd : ws_clients_) close(fd);
        ws_clients_.clear();
    }
    if (http_thread_.joinable()) http_thread_.join();
    if (ws_thread_.joinable()) ws_thread_.join();
}

void WebCommManager::broadcastData() {
    if (!running_) return;
    double tx, ty; state_.getTarget(tx, ty);
    double vx, vy, wz; state_.getVelocity(vx, vy, wz);
    auto points = state_.getPoints();

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "{\"type\":\"scan_data\",";
    oss << "\"target\":{\"x\":" << tx << ",\"y\":" << ty << "},";
    oss << "\"is_moving_enabled\":" << (state_.is_moving_enabled.load() ? "true" : "false") << ",";
    oss << "\"is_active\":" << (state_.active.load() ? "true" : "false") << ",";
    oss << "\"mode\":" << state_.control_mode.load() << ",";
    // NEW-06: 预览模式与目标选点/跟踪状态（target stale在Web可见）
    oss << "\"m20_preview\":" << (state_.m20_preview.load() ? "true" : "false") << ",";
    oss << "\"is_selected\":" << (state_.target_selected.load() ? "true" : "false") << ",";
    oss << "\"is_tracking_valid\":" << (state_.tracking_valid.load() ? "true" : "false") << ",";
    oss << "\"velocity\":{\"vx\":" << vx << ",\"vy\":" << vy << ",\"wz\":" << wz << "},";
    oss << "\"rectangle_width\":" << RECTANGLE_WIDTH << ",\"points\":[";

    size_t max_points = 180;
    size_t step = points.size() > max_points ? points.size() / max_points : 1;
    bool first = true;
    for (size_t i = 0; i < points.size(); i += step) {
        if (!first) oss << ",";
        oss << "{\"x\":" << points[i].first << ",\"y\":" << points[i].second << "}";
        first = false;
    }
    oss << "]}";
    broadcastToWebSockets(oss.str());
}

void WebCommManager::startWebSocketServer() {
    ws_server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
    if (ws_server_socket_ < 0) { log("创建WebSocket服务器socket失败"); return; }

    int opt = 1;
    setsockopt(ws_server_socket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(WS_PORT);

    if (bind(ws_server_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        log("绑定WebSocket端口失败"); close(ws_server_socket_); ws_server_socket_ = -1; return;
    }
    listen(ws_server_socket_, 10);
    log("WebSocket服务器启动在端口 " + std::to_string(WS_PORT));
    ws_thread_ = std::thread(&WebCommManager::wsAcceptLoop, this);
}

void WebCommManager::wsAcceptLoop() {
    while (running_ && ws_server_socket_ >= 0) {
        struct pollfd pfd = {ws_server_socket_, POLLIN, 0};
        if (poll(&pfd, 1, 100) <= 0) continue;
        struct sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);
        int client_fd = accept(ws_server_socket_, (struct sockaddr*)&client_addr, &client_len);
        if (client_fd < 0) continue;
        std::thread(&WebCommManager::handleWebSocketClient, this, client_fd).detach();
    }
}

void WebCommManager::handleWebSocketClient(int fd) {
    char buffer[4096];
    int n = recv(fd, buffer, sizeof(buffer) - 1, 0);
    if (n <= 0) { close(fd); return; }
    buffer[n] = '\0';

    std::string request(buffer);
    std::string ws_key;
    size_t key_pos = request.find("Sec-WebSocket-Key:");
    if (key_pos != std::string::npos) {
        key_pos += 18;
        while (key_pos < request.size() && request[key_pos] == ' ') key_pos++;
        size_t end = request.find("\r\n", key_pos);
        if (end != std::string::npos) ws_key = request.substr(key_pos, end - key_pos);
    }
    if (ws_key.empty()) { close(fd); return; }

    // NEW-06 安全修复: 严格校验Sec-WebSocket-Key（24字符base64, 末尾"==", 解码16字节），
    // 任何非base64字符（含shell元字符）直接拒绝——杜绝进入popen命令拼接（注入风险）
    if (!isValidWebSocketKey(ws_key)) {
        log("WebSocket握手拒绝: 非法Sec-WebSocket-Key");
        close(fd);
        return;
    }

    std::string accept_key = computeWebSocketAcceptKey(ws_key);
    std::ostringstream oss;
    oss << "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: " << accept_key << "\r\n\r\n";
    send(fd, oss.str().c_str(), oss.str().size(), 0);

    { std::lock_guard<std::mutex> lock(ws_clients_mutex_); ws_clients_.insert(fd); }
    log("WebSocket客户端已连接");

    while (running_) {
        struct pollfd pfd = {fd, POLLIN, 0};
        if (poll(&pfd, 1, 100) <= 0) continue;
        std::string message;
        if (!recvWebSocketMessage(fd, message)) break;
        if (!message.empty()) handleWebSocketMessage(message);
    }

    { std::lock_guard<std::mutex> lock(ws_clients_mutex_); ws_clients_.erase(fd); }
    close(fd);
    log("WebSocket客户端已断开");
}

void WebCommManager::handleWebSocketMessage(const std::string& msg) {
    // NEW-06: 预览模式下禁用直控与D1动作（双保险：robot_nexus侧也未注册回调）
    const bool m20 = state_.m20_preview.load();
    if (JsonParser::hasType(msg, "set_target")) {
        double x = 0, y = 0;
        // NEW-06: 严格校验，非法输入忽略（不置0，不抛异常）
        if (JsonParser::parseXYStrict(msg, x, y)) {
            state_.selectTarget(x, y);
        }
    } else if (JsonParser::hasType(msg, "set_moving")) {
        const bool enabled = JsonParser::getBool(msg, "enabled");
        // NEW-06 P1: m20预览下未人工选点时保持false/拒绝true（避免反序授权）
        if (m20 && enabled && !state_.target_selected.load()) {
            log("set_moving忽略: 未人工选点，需先选目标再使能");
            return;
        }
        state_.is_moving_enabled.store(enabled);
    } else if (JsonParser::hasType(msg, "set_active")) {
        state_.active.store(JsonParser::getBool(msg, "active"));
    } else if (JsonParser::hasType(msg, "switch_mode")) {
        size_t pos = msg.find("\"mode\"");
        if (pos != std::string::npos) {
            pos = msg.find(":", pos);
            if (pos != std::string::npos) {
                // NEW-06: 捕获解析异常，坏输入不崩溃
                int mode = 0;
                try {
                    mode = std::stoi(msg.substr(pos + 1));
                } catch (...) {
                    log("switch_mode忽略: 非法mode值");
                    return;
                }
                if (m20 && mode != MODE_FOLLOW) {
                    // NEW-06: 预览模式只允许FOLLOW，拒绝直控/导航切换
                    log("switch_mode拒绝: M20预览模式仅支持跟随模式");
                    return;
                }
                state_.control_mode.store(mode);
                if (mode == MODE_FOLLOW) state_.is_moving_enabled.store(false);
            }
        }
    } else if (JsonParser::hasType(msg, "direct_cmd")) {
        // NEW-06: 预览模式拒绝Web直控
        if (m20) return;
        double x = JsonParser::extractNumber(msg, "x");
        double y = JsonParser::extractNumber(msg, "y");
        double z = JsonParser::extractNumber(msg, "z");
        if (direct_cmd_callback_) direct_cmd_callback_(x, y, z);
    } else if (JsonParser::hasType(msg, "action_cmd")) {
        // NEW-06: 预览模式拒绝D1动作指令（不映射liedown为急停）
        if (m20) return;
        std::string action = JsonParser::extractString(msg, "action");
        if (action_cmd_callback_ && !action.empty()) action_cmd_callback_(action);
    }
}

// NEW-06: 严格校验Sec-WebSocket-Key
// RFC6455: 16随机字节的base64 → 24字符、末尾"=="、字符集仅[A-Za-z0-9+/]
// 该校验保证进入popen的key不含任何shell元字符
bool WebCommManager::isValidWebSocketKey(const std::string& key) {
    if (key.size() != 24) return false;
    if (key[22] != '=' || key[23] != '=') return false;
    static const std::string b64chars =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    for (size_t i = 0; i < 22; ++i) {
        if (b64chars.find(key[i]) == std::string::npos) return false;
    }
    // 22个数据字符 + "==" 恰好解码为16字节（5组x3 + 1）
    return true;
}

std::string WebCommManager::computeWebSocketAcceptKey(const std::string& key) {
    std::string magic = key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    std::string cmd = "echo -n '" + magic + "' | openssl sha1 -binary | base64";
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) return "";
    char result[128];
    if (fgets(result, sizeof(result), pipe) != nullptr) {
        pclose(pipe);
        std::string ret(result);
        while (!ret.empty() && (ret.back() == '\n' || ret.back() == '\r')) ret.pop_back();
        return ret;
    }
    pclose(pipe);
    return "";
}

bool WebCommManager::recvWebSocketMessage(int fd, std::string& message) {
    unsigned char header[2];
    if (recv(fd, header, 2, 0) != 2) return false;
    int opcode = header[0] & 0x0F;
    if (opcode == 0x08) return false;
    bool masked = header[1] & 0x80;
    uint64_t payload_len = header[1] & 0x7F;

    if (payload_len == 126) {
        unsigned char ext[2];
        if (recv(fd, ext, 2, 0) != 2) return false;
        payload_len = (ext[0] << 8) | ext[1];
    } else if (payload_len == 127) {
        unsigned char ext[8];
        if (recv(fd, ext, 8, 0) != 8) return false;
        payload_len = 0;
        for (int i = 0; i < 8; i++) payload_len = (payload_len << 8) | ext[i];
    }

    unsigned char mask[4] = {0};
    if (masked) { if (recv(fd, mask, 4, 0) != 4) return false; }

    if (payload_len > 0 && payload_len < 65536) {
        std::vector<char> data(payload_len);
        size_t received = 0;
        while (received < payload_len) {
            int n = recv(fd, data.data() + received, payload_len - received, 0);
            if (n <= 0) return false;
            received += n;
        }
        if (masked) for (size_t i = 0; i < payload_len; i++) data[i] ^= mask[i % 4];
        message.assign(data.begin(), data.end());
    }
    return true;
}

bool WebCommManager::sendWebSocketMessage(int fd, const std::string& message) {
    std::vector<unsigned char> frame;
    frame.push_back(0x81);
    size_t len = message.size();
    if (len < 126) frame.push_back(static_cast<unsigned char>(len));
    else if (len < 65536) { frame.push_back(126); frame.push_back((len >> 8) & 0xFF); frame.push_back(len & 0xFF); }
    else { frame.push_back(127); for (int i = 7; i >= 0; i--) frame.push_back((len >> (8 * i)) & 0xFF); }
    frame.insert(frame.end(), message.begin(), message.end());
    return send(fd, frame.data(), frame.size(), MSG_NOSIGNAL) == (ssize_t)frame.size();
}

void WebCommManager::broadcastToWebSockets(const std::string& message) {
    std::lock_guard<std::mutex> lock(ws_clients_mutex_);
    std::vector<int> dead;
    for (int fd : ws_clients_) if (!sendWebSocketMessage(fd, message)) dead.push_back(fd);
    for (int fd : dead) { ws_clients_.erase(fd); close(fd); }
}
