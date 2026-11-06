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
        double tx, ty; state_.getTarget(tx, ty);
        oss << ",\"target\":{\"x\":" << tx << ",\"y\":" << ty << "}}";
        res.set_content(oss.str(), "application/json");
    });

    // API: 设置目标
    http_server_->Post("/api/set_target", [this](const httplib::Request& req, httplib::Response& res) {
        double x = 0, y = 0;
        if (JsonParser::parseXY(req.body, x, y)) {
            state_.setTarget(x, y);
            log("Web设置目标: (" + std::to_string(x) + ", " + std::to_string(y) + ")");
        }
        res.set_content("{\"ok\":true}", "application/json");
    });

    // API: 设置运动使能
    http_server_->Post("/api/set_moving", [this](const httplib::Request& req, httplib::Response& res) {
        bool enabled = req.body.find("true") != std::string::npos;
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
    if (JsonParser::hasType(msg, "set_target")) {
        double x, y; JsonParser::parseXY(msg, x, y);
        state_.setTarget(x, y);
    } else if (JsonParser::hasType(msg, "set_moving")) {
        state_.is_moving_enabled.store(JsonParser::getBool(msg, "enabled"));
    } else if (JsonParser::hasType(msg, "set_active")) {
        state_.active.store(JsonParser::getBool(msg, "active"));
    } else if (JsonParser::hasType(msg, "switch_mode")) {
        size_t pos = msg.find("\"mode\"");
        if (pos != std::string::npos) {
            pos = msg.find(":", pos);
            if (pos != std::string::npos) {
                int mode = std::stoi(msg.substr(pos + 1));
                state_.control_mode.store(mode);
                if (mode == MODE_FOLLOW) state_.is_moving_enabled.store(false);
            }
        }
    } else if (JsonParser::hasType(msg, "direct_cmd")) {
        double x = JsonParser::extractNumber(msg, "x");
        double y = JsonParser::extractNumber(msg, "y");
        double z = JsonParser::extractNumber(msg, "z");
        if (direct_cmd_callback_) direct_cmd_callback_(x, y, z);
    } else if (JsonParser::hasType(msg, "action_cmd")) {
        std::string action = JsonParser::extractString(msg, "action");
        if (action_cmd_callback_ && !action.empty()) action_cmd_callback_(action);
    }
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
