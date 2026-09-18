/**
 * @file android_comm.cpp
 * @brief Android UDP通讯模块实现
 */

#include "android_comm.hpp"

void AndroidCommManager::start() {
    if (running_) return;
    
    // 创建发送socket
    udp_send_socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_send_socket_ == INVALID_SOCKET) {
        log("创建发送socket失败");
    }
    
    // 创建接收socket
    udp_recv_socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_recv_socket_ == INVALID_SOCKET) {
        log("创建接收socket失败");
        return;
    }
    
    // 绑定接收端口
    struct sockaddr_in recv_addr{};
    recv_addr.sin_family = AF_INET;
    recv_addr.sin_addr.s_addr = INADDR_ANY;
    recv_addr.sin_port = htons(UDP_RECV_PORT);
    
    if (bind(udp_recv_socket_, (struct sockaddr*)&recv_addr, sizeof(recv_addr)) == SOCKET_ERROR) {
        log("绑定UDP接收端口失败");
    } else {
        log("UDP接收端口 " + std::to_string(UDP_RECV_PORT) + " 已绑定");
    }
    
    // 设置接收超时
    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(udp_recv_socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    
    running_ = true;
    recv_thread_ = std::thread(&AndroidCommManager::receiveLoop, this);
}

void AndroidCommManager::stop() {
    running_ = false;
    if (recv_thread_.joinable()) recv_thread_.join();
    if (udp_send_socket_ != INVALID_SOCKET) { closesocket(udp_send_socket_); udp_send_socket_ = INVALID_SOCKET; }
    if (udp_recv_socket_ != INVALID_SOCKET) { closesocket(udp_recv_socket_); udp_recv_socket_ = INVALID_SOCKET; }
}

void AndroidCommManager::receiveLoop() {
    char buffer[4096];
    struct sockaddr_in client_addr{};
    socklen_t client_len = sizeof(client_addr);
    
    while (running_) {
        int recv_len = recvfrom(udp_recv_socket_, buffer, sizeof(buffer) - 1, 0,
                                (struct sockaddr*)&client_addr, &client_len);
        
        if (recv_len > 0) {
            buffer[recv_len] = '\0';
            
            char ip_str[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &(client_addr.sin_addr), ip_str, INET_ADDRSTRLEN);
            
            {
                std::lock_guard<std::mutex> lock(client_mutex_);
                client_ip_ = ip_str;
                client_connected_ = true;
            }
            
            parseCommand(std::string(buffer));
        }
    }
}

void AndroidCommManager::parseCommand(const std::string& json) {
    if (JsonParser::hasType(json, "heartbeat")) {
        // 心跳包，忽略
    }
    else if (JsonParser::hasType(json, "set_target")) {
        double x, y;
        if (JsonParser::parseXY(json, x, y)) {
            state_.setTarget(x, y);
            log("设置目标位置: (" + std::to_string(x) + ", " + std::to_string(y) + ")");
        }
    }
    else if (JsonParser::hasType(json, "set_moving")) {
        bool enabled = JsonParser::getBool(json, "enabled");
        state_.is_moving_enabled.store(enabled);
        log(std::string("运动使能: ") + (enabled ? "开启" : "关闭"));
    }
    else if (JsonParser::hasType(json, "set_active")) {
        bool active = JsonParser::getBool(json, "active");
        state_.active.store(active);
        log(std::string("跟随功能: ") + (active ? "开启" : "关闭"));
    }
}

void AndroidCommManager::sendScanData() {
    std::string client_ip;
    {
        std::lock_guard<std::mutex> lock(client_mutex_);
        if (!client_connected_) return;
        client_ip = client_ip_;
    }
    
    double tx, ty;
    state_.getTarget(tx, ty);
    auto points = state_.getPoints();
    
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "{\"type\":\"scan_data\",";
    oss << "\"target\":{\"x\":" << tx << ",\"y\":" << ty << "},";
    oss << "\"is_moving_enabled\":" << (state_.is_moving_enabled.load() ? "true" : "false") << ",";
    oss << "\"is_active\":" << (state_.active.load() ? "true" : "false") << ",";
    oss << "\"points\":[";
    
    size_t max_points = 180;
    size_t step = points.size() > max_points ? points.size() / max_points : 1;
    bool first = true;
    for (size_t i = 0; i < points.size(); i += step) {
        if (!first) oss << ",";
        oss << "{\"x\":" << points[i].first << ",\"y\":" << points[i].second << "}";
        first = false;
    }
    oss << "]}";
    
    std::string json = oss.str();
    
    struct sockaddr_in dest_addr{};
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_SEND_PORT);
    inet_pton(AF_INET, client_ip.c_str(), &dest_addr.sin_addr);
    
    sendto(udp_send_socket_, json.c_str(), json.size(), 0,
           (struct sockaddr*)&dest_addr, sizeof(dest_addr));
}
